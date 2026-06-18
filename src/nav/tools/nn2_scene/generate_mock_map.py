#!/usr/bin/env python3
# ============================================================================
# generate_mock_map.py — МОК топологической карты NN2 для проверки ОБВЯЗКИ.
#
# Пишет map.index (FAISS IndexFlatIP) со СЛУЧАЙНЫМИ L2-нормированными векторами
# + metadata.json (схема model/dim/metric/origin/entries) с фиктивной траекторией.
#
# ⚠️ Векторы случайные — реальной локализации НЕ будет, top-1 случаен. Годится
# только для проверки ROS-обвязки (nn2_scene -> relocalizer), путей и форм.
# Для настоящей карты по облёту — tools/nn2_scene/build_scene_map.py.
#
# Пишет рядом с боевой картой: src/nav/data/scene_map/. dim=384 = dinov2_vits14
# (модель, которой будет считать нода) — иначе SceneMatcher отбракует по размеру.
# ============================================================================
import json
from pathlib import Path

import faiss
import numpy as np

OUT_DIR = Path(__file__).resolve().parents[2] / "data" / "scene_map"
OUT_DIR.mkdir(parents=True, exist_ok=True)

MODEL = "dinov2_vits14"
DIM = 384
N = 12

rng = np.random.default_rng(0)
vecs = rng.standard_normal((N, DIM)).astype(np.float32)
vecs /= np.linalg.norm(vecs, axis=1, keepdims=True)   # L2-норм -> косинус через IP

index = faiss.IndexFlatIP(DIM)
index.add(vecs)
faiss.write_index(index, str(OUT_DIR / "map.index"))

# Фиктивная траектория облёта (круг радиуса 50 м) + единичный кватернион.
entries = []
for i in range(N):
    ang = 2.0 * np.pi * i / N
    entries.append({
        "id": i,
        "stamp": float(i) * 0.5,
        "x": round(50.0 * float(np.cos(ang)), 3),
        "y": round(50.0 * float(np.sin(ang)), 3),
        "z": 30.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
        "label": f"sector_{chr(ord('a') + i)}",
    })

metadata = {
    "model": MODEL,
    "dim": DIM,
    "metric": "ip",
    "origin": {
        "id": "takeoff", "lat": 47.090000, "lon": 37.100000, "alt": 140.0,
        "comment": "Датум VINS<->гео (см. nn1_anchor_howto.txt). Мок-значения."
    },
    "entries": entries,
}
(OUT_DIR / "metadata.json").write_text(
    json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")

print(f"✅ Мок-карта записана в {OUT_DIR}: мест {N}, dim {DIM} "
      f"(векторы СЛУЧАЙНЫЕ — только для проверки обвязки)")
