#!/usr/bin/env python3
# ============================================================================
# build_reference_db.py — сборка БОЕВОЙ георефернс-базы NN1 из кадров облёта.
#
# Прогоняет каждый кадр через SuperPoint и сохраняет дескрипторы в формате,
# который ждёт anchor_matcher.py: keypoints (N,2), descriptors (N,256),
# image_size (2,). Строит database.json (origin/landmarks/references).
#
# Раскладка входа (имя подкаталога = id ориентира):
#   images_root/
#     crossroad_1/ view_001.png view_002.png ...
#     bunker_1/    ...
#   coords.json:
#     {"origin": {"id":"takeoff","lat":..,"lon":..,"alt":..},
#      "landmarks": {"crossroad_1": {"lat":..,"lon":..,"alt":..}, ...}}
#
# Пример:
#   python3 build_reference_db.py --images ~/overflight --coords coords.json
# ============================================================================
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import torch
from lightglue import SuperPoint
from lightglue.utils import rbd

EXTS = {".png", ".jpg", ".jpeg", ".bmp"}
DEFAULT_OUT = Path(__file__).resolve().parents[2] / "data" / "reference_db"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True,
                    help="папка: подкаталог на ориентир (имя=id), внутри кадры облёта")
    ap.add_argument("--coords", required=True,
                    help="JSON с origin + landmarks{id:{lat,lon,alt}}")
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--max-keypoints", type=int, default=2048)
    args = ap.parse_args()

    device = args.device
    if device == "cuda" and not torch.cuda.is_available():
        print("⚠ CUDA недоступна — считаю на CPU (медленно).")
        device = "cpu"

    extractor = SuperPoint(max_num_keypoints=args.max_keypoints).eval().to(device)

    coords = json.loads(Path(args.coords).read_text(encoding="utf-8"))
    landmarks = coords.get("landmarks", {})
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    images_root = Path(args.images)

    references = {}
    for lm_dir in sorted(p for p in images_root.iterdir() if p.is_dir()):
        lm_id = lm_dir.name
        if lm_id not in landmarks:
            print(f"⚠ {lm_id}: нет координат в --coords, пропускаю")
            continue
        n = 0
        for img_path in sorted(lm_dir.iterdir()):
            if img_path.suffix.lower() not in EXTS:
                continue
            bgr = cv2.imread(str(img_path))
            if bgr is None:
                print(f"⚠ не читается {img_path}")
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            t = torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0
            with torch.no_grad():
                feats = rbd(extractor.extract(t.to(device)))
            ref_name = f"{lm_id}__{img_path.stem}.npz"
            np.savez(
                out_dir / ref_name,
                keypoints=feats["keypoints"].cpu().numpy().astype(np.float32),
                descriptors=feats["descriptors"].cpu().numpy().astype(np.float32),
                image_size=feats["image_size"].cpu().numpy().astype(np.float32).reshape(2),
            )
            references[ref_name] = lm_id
            n += 1
        print(f"{lm_id}: {n} эталонов")

    database = {
        "origin": coords.get("origin"),
        "landmarks": landmarks,
        "references": references,
    }
    (out_dir / "database.json").write_text(
        json.dumps(database, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"✅ База собрана в {out_dir}: эталонов {len(references)}")


if __name__ == "__main__":
    main()
