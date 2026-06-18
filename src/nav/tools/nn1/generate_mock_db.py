#!/usr/bin/env python3
# ============================================================================
# generate_mock_db.py — МОК георефернс-базы NN1 для проверки ОБВЯЗКИ.
#
# Пишет database.json (схема origin/landmarks/references) + *.npz со СЛУЧАЙНЫМИ
# дескрипторами в формате, который ждёт anchor_matcher.py:
#   keypoints (N,2) float, descriptors (N,256) float, image_size (2,) float
#
# ⚠️ Дескрипторы случайные — LightGlue реальных совпадений НЕ найдёт. Годится
# только для проверки ROS-обвязки, путей и форм тензоров, НЕ алгоритма.
# Для настоящей базы по кадрам облёта — tools/nn1/build_reference_db.py.
#
# Пишет рядом с боевой базой: src/nav/data/reference_db/.
# ============================================================================
import json
from pathlib import Path

import numpy as np

DB_DIR = Path(__file__).resolve().parents[2] / "data" / "reference_db"
DB_DIR.mkdir(parents=True, exist_ok=True)

IMAGE_SIZE = (1280.0, 720.0)   # как у /image_color
NUM_KEYPOINTS = 500

# origin = точка взлёта (ориентир №0): VINS (0,0,0) <-> реальные GPS+высота.
database = {
    "origin": {
        "id": "takeoff", "lat": 47.090000, "lon": 37.100000, "alt": 140.0,
        "comment": "Точка взлёта = датум VINS<->гео (см. nn1_anchor_howto.txt)."
    },
    "landmarks": {
        "crossroad_1": {"lat": 47.111000, "lon": 37.222000, "alt": 150.0},
        "bunker_1":    {"lat": 47.333000, "lon": 37.444000, "alt": 145.5},
        "bush_1":      {"lat": 47.555000, "lon": 37.666000, "alt": 160.2},
    },
    "references": {
        "crossroad_view_001.npz": "crossroad_1",
        "crossroad_view_002.npz": "crossroad_1",
        "crossroad_view_003.npz": "crossroad_1",
        "bunker_view_001.npz": "bunker_1",
        "bunker_view_002.npz": "bunker_1",
        "bunker_view_003.npz": "bunker_1",
        "bush1_view_001.npz": "bush_1",
        "bush1_view_002.npz": "bush_1",
        "bush1_view_003.npz": "bush_1",
    },
}

(DB_DIR / "database.json").write_text(
    json.dumps(database, indent=2, ensure_ascii=False), encoding="utf-8")

for ref_name in database["references"]:
    keypoints = (np.random.rand(NUM_KEYPOINTS, 2) * IMAGE_SIZE).astype(np.float32)
    descriptors = np.random.rand(NUM_KEYPOINTS, 256).astype(np.float32)   # (N,256)!
    image_size = np.array(IMAGE_SIZE, dtype=np.float32)
    np.savez(DB_DIR / ref_name,
             keypoints=keypoints, descriptors=descriptors, image_size=image_size)

print(f"✅ Мок-база записана в {DB_DIR}")
print(f"   эталонов .npz: {len(database['references'])} "
      f"(дескрипторы СЛУЧАЙНЫЕ — только для проверки обвязки)")
