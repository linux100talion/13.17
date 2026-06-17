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
# Формат карты (data/scene_map/, см. tools/nn2_scene_howto.txt) — ДВА
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


class SceneEncoder:
    """DINOv2 → один L2-нормированный вектор на кадр (глобальный дескриптор).

    Общий для ноды (через SceneMatcher) и сборщика карты (build_scene_map.py),
    чтобы карта и онлайн-запрос считались ОДНОЙ моделью (иначе векторы несравнимы).
    """
    # Сторона входа DINOv2 должна быть кратна патчу (14). 224 = 16*14.
    INPUT_SIZE = 224

    def __init__(self, model_name="dinov2_vits14", device="cuda", logger=None):
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
        self.dim = _DINOV2_DIM.get(model_name)
        self._mean = _MEAN.to(device)
        self._std = _STD.to(device)
        self._info(f"DINOv2 '{model_name}' (dim={self.dim}) на {device}")

    @torch.no_grad()
    def encode(self, frame_bgr, normalize=True):
        """Кадр (BGR np.ndarray) -> вектор (dim,) float32.

        normalize=True (по умолчанию) — L2-норм под косинус (IP), так считают
        карту и онлайн-запрос. normalize=False — «сырой» дескриптор: нужен
        оценке изометрии (tools/eval_isometry.py), где важна L2-величина.
        """
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.INPUT_SIZE, self.INPUT_SIZE))
        t = torch.from_numpy(rgb).float().permute(2, 0, 1).unsqueeze(0) / 255.0
        t = (t.to(self.device) - self._mean) / self._std
        vec = self.model(t)                                # (1, dim) — CLS-токен
        if normalize:
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
                 min_score=0.5, logger=None):
        self.log = logger
        self.min_score = float(min_score)

        self.index = None
        self.origin = None
        self.entries = []
        meta = self._load_meta(Path(map_path))

        # Модель карты главнее параметра ноды: запрос обязан считаться той же
        # сетью, что и карта, — иначе векторы из разных пространств.
        if meta and meta.get("model"):
            model_name = meta["model"]
        self.encoder = SceneEncoder(model_name=model_name, device=device, logger=logger)

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
                       f"tools/build_scene_map.py).")
            return None

        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        self.origin = meta.get("origin")
        self.entries = meta.get("entries", [])

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
        """Кадр (BGR np.ndarray) -> SceneMatch или None (ниже порога/пустая карта)."""
        if self.index is None or self.index.ntotal == 0:
            return None

        vec = self.encoder.encode(frame_bgr).reshape(1, -1)
        scores, ids = self.index.search(vec, 1)     # top-1 по косинусу
        score = float(scores[0, 0])
        idx = int(ids[0, 0])
        if idx < 0 or score < self.min_score:
            return None

        entry = self.entries[idx]
        label = entry.get("label") or f"place_{entry.get('id', idx)}"
        return SceneMatch(entry.get("id", idx), label, score, entry)

    # --- логирование (logger опционален, чтобы класс был тестируем вне ROS) ---
    def _info(self, msg):
        if self.log:
            self.log.info(msg)

    def _warn(self, msg):
        if self.log:
            self.log.warn(msg)
