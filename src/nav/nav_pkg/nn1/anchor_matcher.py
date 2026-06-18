# ============================================================================
# anchor_matcher.py — ядро Нейросети №1 (якорная локализация).
#
# Инкремент 1: SuperPoint (фичи) + LightGlue (матчинг) текущего кадра против
# георефернс-базы облёта. На выход — найденный ориентир (id + координаты из БД)
# и bbox совпавших точек. Ray tracing и сброс дрейфа VINS — Инкремент 2.
#
# Формат базы (data/reference_db/, см. tools/nn1/nn1_anchor_howto.txt):
#   database.json:
#     origin      — точка взлёта = ориентир №0 (GPS+высота, датум VINS<->гео)
#     landmarks   — id -> {lat, lon, alt}  (реальные гео-координаты)
#     references  — имя_файла.npz -> id_ориентира
#   *.npz (выхлоп SuperPoint по кадрам облёта):
#     keypoints (N,2) float, descriptors (N,256) float, image_size (2,) float
#
# Тяжёлые импорты (torch/lightglue) — на уровне модуля: грузится ТОЛЬКО при
# запуске ноды nn1_anchor, не при colcon build (ament_python модули не
# импортирует при сборке).
# ============================================================================
import json
from pathlib import Path

import cv2
import numpy as np
import torch
from lightglue import LightGlue, SuperPoint
from lightglue.utils import rbd


class AnchorMatch:
    """Результат локализации по одному кадру."""
    def __init__(self, landmark_id, coords, bbox, num_matches, ref_name):
        self.landmark_id = landmark_id          # str, ключ из landmarks
        self.coords = coords                    # {lat, lon, alt} или None
        self.bbox = bbox                        # (x1, y1, x2, y2) в пикселях кадра
        self.num_matches = num_matches          # сколько точек совпало
        self.ref_name = ref_name                # какой эталон сработал


class AnchorMatcher:
    def __init__(self, db_path, device="cuda", max_keypoints=1024,
                 min_matches=15, logger=None):
        self.log = logger
        self.min_matches = int(min_matches)

        # CUDA обязательна для real-time на Orin; на CPU инференс — секунды.
        if device == "cuda" and not torch.cuda.is_available():
            self._warn("CUDA недоступна — падаю на CPU (инференс будет медленным).")
            device = "cpu"
        self.device = device

        self.extractor = SuperPoint(max_num_keypoints=int(max_keypoints)).eval().to(device)
        self.matcher = LightGlue(features="superpoint").eval().to(device)

        self.origin = None
        self.landmarks = {}
        self.references = []   # список (ref_name, landmark_id, feats_dict)
        self._load_db(db_path)

    # --- загрузка базы -------------------------------------------------------
    def _load_db(self, db_path):
        db_dir = Path(db_path)
        meta = db_dir / "database.json"
        if not meta.exists():
            self._warn(f"database.json не найден ({meta}) — работаю вхолостую "
                       f"(детекций не будет, пока база не собрана).")
            return

        db = json.loads(meta.read_text(encoding="utf-8"))
        self.origin = db.get("origin")
        self.landmarks = db.get("landmarks", {})

        for ref_name, lm_id in db.get("references", {}).items():
            npz = db_dir / ref_name
            if not npz.exists():
                self._warn(f"эталон {ref_name} отсутствует — пропускаю "
                           f"(сгенерируй базу: tools/nn1/build_reference_db.py).")
                continue
            data = np.load(npz)
            feats = {
                "keypoints":   torch.from_numpy(data["keypoints"]).float(),
                "descriptors": torch.from_numpy(data["descriptors"]).float(),
                "image_size":  torch.from_numpy(data["image_size"]).float(),
            }
            # batch-измерение + на устройство
            feats = {k: v.unsqueeze(0).to(self.device) for k, v in feats.items()}
            self.references.append((ref_name, lm_id, feats))

        self._info(f"загружено эталонов: {len(self.references)}, "
                   f"ориентиров: {len(self.landmarks)}")

    # --- инференс ------------------------------------------------------------
    @torch.no_grad()
    def _extract(self, frame_bgr):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        t = torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0  # (3,H,W) в [0,1]
        return self.extractor.extract(t.to(self.device))            # batched dict

    @torch.no_grad()
    def query(self, frame_bgr):
        """Кадр (BGR np.ndarray) -> AnchorMatch или None."""
        if not self.references:
            return None

        live = self._extract(frame_bgr)
        live_kpts = rbd(live)["keypoints"]   # (N,2)

        best = None
        # TODO Инкремент 1.x: ретривал-префильтр (DINOv2/AnyLoc + FAISS) выбирает
        # top-K кандидатов; сейчас — брутфорс по всей базе (ок для крошечной БД,
        # но НЕ масштабируется на тысячи эталонов).
        for ref_name, lm_id, ref in self.references:
            out = rbd(self.matcher({"image0": ref, "image1": live}))
            matches = out["matches"]         # (M,2): [idx_в_эталоне, idx_в_кадре]
            m = int(matches.shape[0])
            if m >= self.min_matches and (best is None or m > best[0]):
                pts = live_kpts[matches[:, 1]].cpu().numpy()
                x1, y1 = pts.min(axis=0)
                x2, y2 = pts.max(axis=0)
                best = (m, lm_id, (float(x1), float(y1), float(x2), float(y2)), ref_name)

        if best is None:
            return None
        m, lm_id, bbox, ref_name = best
        return AnchorMatch(lm_id, self.landmarks.get(lm_id), bbox, m, ref_name)

    # --- логирование (logger опционален, чтобы класс был тестируем вне ROS) ---
    def _info(self, msg):
        if self.log:
            self.log.info(msg)

    def _warn(self, msg):
        if self.log:
            self.log.warn(msg)
