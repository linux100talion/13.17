#!/usr/bin/env python3
# ============================================================================
# build_scene_map.py — сборка топологической карты NN2 из ROS bag облёта.
#
# «Обучение с учителем» (см. nn2_scene_idea.txt): идём по таймлайну bag'а,
# ~N раз в секунду выхватываем кадр, ассоциируем его с БЛИЖАЙШЕЙ по времени
# позой VINS (одометрия) и кватернионом IMU, гоним кадр через DINOv2 -> вектор
# в FAISS (map.index), а физический контекст (поза+кватернион) -> metadata.json.
#
# message_filters синхронизирует ЖИВЫЕ подписки; для офлайн-bag берём ту же
# идею проще и воспроизводимее — ближайшее по времени последнее значение
# (odom/imu кэшируются, на момент кадра берётся последнее виденное).
#
# Раскладка выхода (data/scene_map/, формат ждёт scene_descriptor.py):
#   map.index      — FAISS IndexFlatIP (L2-норм. векторы = косинус)
#   metadata.json  — {model, dim, metric, origin, entries:[...]}
#
# Пример:
#   python3 build_scene_map.py --bag ~/flight_bag --rate 2.0
# ============================================================================
import argparse
import json
import sys
from pathlib import Path

import numpy as np

# Делаем nav_pkg импортируемым (общий энкодер с нодой -> карта и онлайн считаются
# ОДНОЙ моделью). faiss/torch тянутся транзитивно из scene_descriptor.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import faiss                                              # noqa: E402
from cv_bridge import CvBridge                            # noqa: E402
from rclpy.serialization import deserialize_message       # noqa: E402
import rosbag2_py                                          # noqa: E402
from rosidl_runtime_py.utilities import get_message        # noqa: E402

from nav_pkg.scene_descriptor import SceneEncoder          # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parents[1] / "data" / "scene_map"


def open_bag(path, storage_id):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="каталог rosbag2 (sqlite3/mcap)")
    ap.add_argument("--image-topic", default="/image_color")
    ap.add_argument("--odom-topic", default="/vins_estimator/odometry",
                    help="источник позы (nav_msgs/Odometry): X,Y,Z места")
    ap.add_argument("--imu-topic", default="/mavros/imu/data",
                    help="источник кватерниона (sensor_msgs/Imu): ориентация")
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--rate", type=float, default=2.0,
                    help="кадров в секунду выхватывать с таймлайна")
    ap.add_argument("--storage-id", default="sqlite3", help="sqlite3 | mcap")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--model", default="dinov2_vits14")
    ap.add_argument("--origin", default=None,
                    help="JSON с датумом облёта (origin) -> в metadata; опционально")
    args = ap.parse_args()

    reader, type_map = open_bag(args.bag, args.storage_id)
    for need in (args.image_topic, args.odom_topic, args.imu_topic):
        if need not in type_map:
            print(f"⚠ топик {need} нет в bag (есть: {sorted(type_map)})")

    bridge = CvBridge()
    encoder = SceneEncoder(model_name=args.model, device=args.device)

    sample_dt_ns = int(1e9 / args.rate)
    last_sample_ns = None
    last_odom = None
    last_imu = None

    vectors = []
    entries = []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, get_message(type_map[topic]))

        if topic == args.odom_topic:
            last_odom = msg
        elif topic == args.imu_topic:
            last_imu = msg
        elif topic == args.image_topic:
            if last_sample_ns is not None and (t_ns - last_sample_ns) < sample_dt_ns:
                continue
            last_sample_ns = t_ns

            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            vectors.append(encoder.encode(frame))

            pos = last_odom.pose.pose.position if last_odom else None
            q = last_imu.orientation if last_imu else None
            i = len(entries)
            entries.append({
                "id": i,
                "stamp": t_ns * 1e-9,
                "x": pos.x if pos else None,   # ENU/VINS X (или Lon, если GPS)
                "y": pos.y if pos else None,   # ENU/VINS Y (или Lat)
                "z": pos.z if pos else None,   # ENU/VINS Z (или Alt)
                "qx": q.x if q else None, "qy": q.y if q else None,
                "qz": q.z if q else None, "qw": q.w if q else None,
                "label": f"place_{i}",
            })

    if not vectors:
        print("⚠ ни одного кадра не выбрано — карта не записана.")
        return

    mat = np.vstack(vectors).astype(np.float32)
    index = faiss.IndexFlatIP(mat.shape[1])     # косинус (векторы L2-нормированы)
    index.add(mat)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    faiss.write_index(index, str(out_dir / "map.index"))

    origin = json.loads(Path(args.origin).read_text(encoding="utf-8")) \
        if args.origin else None
    metadata = {
        "model": args.model,
        "dim": int(mat.shape[1]),
        "metric": "ip",
        "origin": origin,
        "entries": entries,
    }
    (out_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"✅ Карта собрана в {out_dir}: мест {len(entries)} (модель {args.model})")


if __name__ == "__main__":
    main()
