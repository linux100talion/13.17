# ============================================================================
# scene_descriptor.py — ядро Нейросети №2 (топологическая карта, семантика).
#
# Назначение (итог): релокализация VINS после потери трекинга. Текущий кадр
# сжимается в один вектор (DINOv2) → ближайшее место в карте облёта (FAISS) →
# его сохранённая поза (GPS/ENU + кватернион IMU) → этим переинициализируем
# абсолютную позицию. Карта потому и хранит позу, а не текстовую метку.
#
# Это ТА ЖЕ ретривал-машина (DINOv2/FAISS), что нужна NN1 как префильтр
# (см. nn1_anchor_howto.txt, «КРИТИЧНЫЕ МЕСТА»): сначала NN2 сужает кандидатов,
# потом LightGlue матчит только их. Поэтому encoder/index вынесены сюда отдельно.
#
# Формат карты (data/scene_map/, см. tools/nn2_scene/nn2_scene_howto.txt) — ДВА
# синхронизированных файла (FAISS хранит только векторы+ID, метаданные — рядом):
#   map.index      — FAISS-индекс (IndexFlatIP по L2-нормированным векторам =
#                    косинусная близость); строка i ↔ entries[i]
#   metadata.json  — {model, dim, metric, origin, entries:[{id, stamp, x,y,z,
#                    qx,qy,qz,qw, label}]}
#
# Следующий инкремент — метрический MLP-«топограф» поверх DINOv2 (Triplet Loss
# на дельтах VINS): «выпрямляет» эмбеддинг в изометрию (L2 ∝ метры). Встанет
# пост-DINOv2 головой прямо в SceneEncoder.encode() — обвязка не меняется.
# Пока глобальный дескриптор = CLS-токен DINOv2 (просто, робастно, без словаря).
#
# Тяжёлые импорты (torch/faiss) — на уровне модуля: грузятся ТОЛЬКО при запуске
# ноды nn2_scene, не при colcon build (ament_python модули при сборке не
# импортирует). DINOv2 тянется через torch.hub (кэш ~/.cache/torch/hub).
# ============================================================================
import json
from pathlib import Path

import cv2
import faiss
import numpy as np
import torch

from nav_pkg.nn2.metric_decode import knn_decode_guard   # kNN-декод + страж (numpy)

# DINOv2 ImageNet-нормировка (модель обучена с ней).
_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
_STD = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)

# embed_dim бэкбонов DINOv2 (CLS-токен) — для сверки с metadata["dim"].
_DINOV2_DIM = {
    "dinov2_vits14": 384,
    "dinov2_vitb14": 768,
    "dinov2_vitl14": 1024,
    "dinov2_vitg14": 1536,
}


class SceneMatch:
    """Результат топологической локализации по одному кадру."""
    def __init__(self, scene_id, label, score, entry):
        self.scene_id = scene_id        # int, ID места в карте (= строка FAISS)
        self.label = label              # str, метка места (баннер оператору)
        self.score = score              # float, косинусная близость [0..1]
        self.entry = entry              # dict метаданных места (поза, кватернион)


class MetricFix:
    """kNN-засечка со стражем (XVIII): top-1 для баннера/позы + метрическая
    позиция/ковариация из knn_decode_guard."""
    def __init__(self, top1, dec):
        self.top1 = top1                # SceneMatch (баннер, базовая поза/кватернион)
        self.x, self.y = dec["x"], dec["y"]     # метрическая позиция (ENU-метры)
        self.cov = dec["cov"]           # 2×2 ковариация (м²), анизотропная
        self.std = dec["std"]           # скалярный σ-эквивалент (совместимость)
        self.conf = dec["conf"]         # уверенность 0..1 (сбита при стражe)
        self.source = dec["source"]     # "knn" | "top1-fallback"
        self.spread = dec["spread"]     # RMS-разброс соседей, м (сигнал алиасинга)


class MetricHead(torch.nn.Module):
    """MLP-«топограф»: DINOv2 CLS -> метрическое пространство (L2 ≈ метры).

    «Выпрямляет» гладкий, но неравномерный эмбеддинг DINOv2 в (локально) метрический:
    L2-расстояние между выходами ~ физическому перемещению. Обучается ОФЛАЙН на
    дельтах одометрии VINS (Triplet/дистанц-регрессия, см. nn2_scene_howto.txt).
    Веса .pt самоописательны (config + state_dict) — SceneEncoder грузит их, чтобы
    карта и онлайн считались ОДНОЙ связкой (DINOv2+MLP).
    """
    def __init__(self, in_dim, hidden=256, out_dim=64):
        super().__init__()
        self.in_dim, self.hidden, self.out_dim = int(in_dim), int(hidden), int(out_dim)
        self.net = torch.nn.Sequential(
            torch.nn.Linear(self.in_dim, self.hidden),
            torch.nn.LayerNorm(self.hidden),
            torch.nn.GELU(),
            torch.nn.Linear(self.hidden, self.hidden),
            torch.nn.GELU(),
            torch.nn.Linear(self.hidden, self.out_dim),
        )

    def forward(self, x):
        return self.net(x)

    def save(self, path):
        torch.save({"in_dim": self.in_dim, "hidden": self.hidden,
                    "out_dim": self.out_dim, "state_dict": self.state_dict()},
                   str(path))

    @classmethod
    def load(cls, path, map_location=None):
        ckpt = torch.load(str(path), map_location=map_location)
        head = cls(ckpt["in_dim"], ckpt["hidden"], ckpt["out_dim"])
        head.load_state_dict(ckpt["state_dict"])
        return head


class SceneEncoder:
    """DINOv2 (+ опц. MLP-«топограф») → один вектор на кадр (глобальный дескриптор).

    Общий для ноды (через SceneMatcher) и сборщика карты (build_scene_map.py),
    чтобы карта и онлайн-запрос считались ОДНОЙ моделью (иначе векторы несравнимы).
    Без MLP: сырой/нормированный CLS DINOv2 (метрика косинус/IP). С MLP: метрический
    выход (метрика L2 ≈ метры). .dim и .metric отражают активную связку.
    """
    # Сторона входа DINOv2 должна быть кратна патчу (14). 224 = 16*14.
    INPUT_SIZE = 224

    def __init__(self, model_name="dinov2_vits14", device="cuda",
                 mlp_path=None, logger=None):
        self.log = logger
        self.model_name = model_name

        # CUDA обязательна для real-time на Orin; на CPU инференс — сотни мс.
        if device == "cuda" and not torch.cuda.is_available():
            self._warn("CUDA недоступна — падаю на CPU (инференс будет медленным).")
            device = "cpu"
        self.device = device

        # trust_repo: первый запуск тянет код+веса DINOv2 из torch.hub в кэш.
        self.model = torch.hub.load(
            "facebookresearch/dinov2", model_name, trust_repo=True).eval().to(device)
        self.backbone_dim = _DINOV2_DIM.get(model_name)
        self._mean = _MEAN.to(device)
        self._std = _STD.to(device)

        # MLP-«топограф» опционален. Есть веса -> выход метрический (L2 ≈ метры),
        # нет -> сырой DINOv2 (косинус/IP). Путь к весам приходит из metadata карты
        # (SceneMatcher), чтобы карта и онлайн грузили одну связку.
        self.head = None
        if mlp_path:
            p = Path(mlp_path)
            if p.exists():
                self.head = MetricHead.load(p, map_location=device).eval().to(device)
                self._info(f"MLP-топограф '{p.name}': out_dim={self.head.out_dim}, "
                           f"метрика L2 (метры)")
            else:
                self._warn(f"MLP-топограф не найден ({p}) — сырой DINOv2 (косинус).")
        self.dim = self.head.out_dim if self.head else self.backbone_dim
        self.metric = "l2" if self.head else "ip"
        self._info(f"DINOv2 '{model_name}' (backbone {self.backbone_dim}) на "
                   f"{device}; выход dim={self.dim}, метрика {self.metric}")

    @torch.no_grad()
    def encode(self, frame_bgr, normalize=True):
        """Кадр (BGR np.ndarray) -> вектор (dim,) float32.

        С MLP-«топографом»: выход метрический (L2 ≈ метры), normalize ИГНОРИРУЕТСЯ
        (нормировка убила бы масштаб). Без MLP: normalize=True — L2-норм под косинус
        (IP), так считают карту и онлайн-запрос; normalize=False — «сырой» DINOv2
        (нужен оценке изометрии tools/nn2_scene/eval_isometry.py, где важна L2-величина).
        """
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.INPUT_SIZE, self.INPUT_SIZE))
        t = torch.from_numpy(rgb).float().permute(2, 0, 1).unsqueeze(0) / 255.0
        t = (t.to(self.device) - self._mean) / self._std
        vec = self.model(t)                                # (1, backbone) — CLS-токен
        if self.head is not None:
            vec = self.head(vec)                           # (1, out_dim) метрика L2
        elif normalize:
            vec = torch.nn.functional.normalize(vec, dim=1)   # под косинус (IP)
        return vec.squeeze(0).cpu().numpy().astype(np.float32)

    def _info(self, msg):
        if self.log:
            self.log.info(msg)

    def _warn(self, msg):
        if self.log:
            self.log.warn(msg)


class SceneMatcher:
    """Загрузка карты (FAISS + метаданные) + поиск ближайшего места по кадру."""

    def __init__(self, map_path, device="cuda", model_name="dinov2_vits14",
                 min_score=0.5, max_dist=10.0, logger=None):
        self.log = logger
        self.min_score = float(min_score)    # порог косинуса (IP-карта, больше=лучше)
        self.max_dist = float(max_dist)      # порог расстояния, м (L2-карта с MLP)

        self.index = None
        self.origin = None
        self.entries = []
        self.mlp_path = None
        meta = self._load_meta(Path(map_path))

        # Модель карты главнее параметра ноды: запрос обязан считаться той же
        # связкой (DINOv2[+MLP]), что и карта, — иначе векторы из разных пространств.
        if meta and meta.get("model"):
            model_name = meta["model"]
        self.encoder = SceneEncoder(model_name=model_name, device=device,
                                    mlp_path=self.mlp_path, logger=logger)

        if self.index is not None and self.encoder.dim and \
                self.index.d != self.encoder.dim:
            self._warn(f"размерность индекса ({self.index.d}) != модели "
                       f"({self.encoder.dim}) — карта собрана другой сетью, "
                       f"поиск отключён.")
            self.index = None

    # --- загрузка карты ------------------------------------------------------
    def _load_meta(self, map_dir):
        meta_path = map_dir / "metadata.json"
        if not meta_path.exists():
            self._warn(f"metadata.json не найден ({meta_path}) — работаю вхолостую "
                       f"(локализации не будет, пока карта не собрана: "
                       f"tools/nn2_scene/build_scene_map.py).")
            return None

        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        self.origin = meta.get("origin")
        self.entries = meta.get("entries", [])
        # Веса MLP-«топографа» лежат рядом с картой (имя — в metadata.mlp).
        if meta.get("mlp"):
            self.mlp_path = str(map_dir / meta["mlp"])

        index_path = map_dir / "map.index"
        if not index_path.exists():
            self._warn(f"map.index отсутствует ({index_path}) — есть метаданные, "
                       f"но нет векторов; поиск невозможен (пересобери карту).")
            return meta

        self.index = faiss.read_index(str(index_path))
        if self.index.ntotal != len(self.entries):
            self._warn(f"рассинхрон карты: векторов {self.index.ntotal}, "
                       f"метаданных {len(self.entries)} — поиск отключён.")
            self.index = None
        else:
            self._info(f"карта загружена: мест {self.index.ntotal}, "
                       f"метрика {meta.get('metric', 'ip')}")
        return meta

    # --- инференс ------------------------------------------------------------
    def query(self, frame_bgr):
        """Кадр (BGR np.ndarray) -> SceneMatch или None (ниже порога/пустая карта).

        score в SceneMatch: для IP-карты (сырой DINOv2) — косинус [0..1], больше=
        лучше; для L2-карты (с MLP) — расстояние В МЕТРАХ, меньше=лучше.
        """
        if self.index is None or self.index.ntotal == 0:
            return None

        vec = self.encoder.encode(frame_bgr).reshape(1, -1)
        vals, ids = self.index.search(vec, 1)       # top-1
        val = float(vals[0, 0])
        idx = int(ids[0, 0])
        if idx < 0:
            return None

        if self.index.metric_type == faiss.METRIC_INNER_PRODUCT:
            if val < self.min_score:                # косинус: больше — лучше
                return None
            score = val
        else:                                       # L2: faiss отдаёт КВАДРАТ дистанции
            if val > self.max_dist * self.max_dist:
                return None
            score = float(np.sqrt(val))             # расстояние в метрах

        entry = self.entries[idx]
        label = entry.get("label") or f"place_{entry.get('id', idx)}"
        return SceneMatch(entry.get("id", idx), label, score, entry)

    def search(self, frame_bgr, k):
        """Кодирует кадр ОДИН раз -> k ближайших мест: (ids (m,), dists (m,))
        или (None, None). dists: МЕТРЫ (L2-карта) или КОСИНУС (IP-карта), сырые
        (без гейта), nearest-first."""
        if self.index is None or self.index.ntotal == 0:
            return None, None
        vec = self.encoder.encode(frame_bgr).reshape(1, -1)
        kk = int(min(k, self.index.ntotal))
        vals, ids = self.index.search(vec, kk)
        ids, vals = ids[0], vals[0]
        keep = ids >= 0
        ids, vals = ids[keep], vals[keep]
        if self.index.metric_type == faiss.METRIC_INNER_PRODUCT:
            dists = vals                            # косинус (больше — лучше)
        else:
            dists = np.sqrt(np.maximum(vals, 0.0))  # L2: КВАДРАТ -> метры
        return ids, dists

    def metric_fix(self, frame_bgr, k=6, guard_radius=8.0, sigma_floor=1.0,
                   conf_scale=5.0, guard_penalty=0.3):
        """kNN-локализация СО СТРАЖЕМ алиасинга (XVIII) вместо top-1. Кодирует кадр
        раз, берёт k соседей, гейтит ЛУЧШЕГО (min_score/max_dist), декодирует
        позицию+ковариацию (knn_decode_guard; фолбэк top-1 при разбегании соседей).
        -> MetricFix или None (пусто/ниже порога)."""
        ids, dists = self.search(frame_bgr, k)
        if ids is None or len(ids) == 0:
            return None
        metric = "ip" if self.index.metric_type == faiss.METRIC_INNER_PRODUCT else "l2"
        best = float(dists[0])                       # nearest-first -> [0] лучший
        if metric == "ip":
            if best < self.min_score:
                return None
            score = best
        else:
            if best > self.max_dist:                 # dists уже в метрах
                return None
            score = best
        P = np.array([[self.entries[i].get("x"), self.entries[i].get("y")]
                      for i in ids], dtype=np.float64)
        dec = knn_decode_guard(P, dists, metric, k=k, guard_radius=guard_radius,
                               sigma_floor=sigma_floor, conf_scale=conf_scale,
                               min_score=self.min_score, guard_penalty=guard_penalty)
        i0 = int(ids[0])
        entry = self.entries[i0]
        label = entry.get("label") or f"place_{entry.get('id', i0)}"
        top1 = SceneMatch(entry.get("id", i0), label, score, entry)
        return MetricFix(top1, dec)

    # --- логирование (logger опционален, чтобы класс был тестируем вне ROS) ---
    def _info(self, msg):
        if self.log:
            self.log.info(msg)

    def _warn(self, msg):
        if self.log:
            self.log.warn(msg)
