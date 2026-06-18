#!/usr/bin/env python3
# ============================================================================
# visualize_fiber.py — увидеть «позиционный fiber» в пространстве дескрипторов.
#
# Теория — tools/nn2_topograph_theory.txt: видимость зависит от позиции + нуисансов
# (ракурс/свет/время). Позиционный fiber — это тонкая «нить», которую дескрипторы
# чертят, КОГДА дрон движется; нуисансы — разброс ВОКРУГ нити. Скрипт делает это
# наглядным: проецирует дескрипторы кадров (сырой DINOv2 или, с --mlp, выход
# топографа) в 2D/3D (PCA) и кладёт рядом физическую траекторию VINS, крася обе
# ОДНОЙ величиной (длина пути / время). Совпадает форма -> fiber восстановим;
# каша/толстый разброс -> нуисансы доминируют (топограф ещё не выпрямил).
#
# Зависит только от того, что в образе nav (+ matplotlib). PCA — через numpy SVD
# (без sklearn).
#
# Пример:
#   python3 visualize_fiber.py --bag ~/f1 ~/f2 --color arclen --out /tmp/fiber.png
#   python3 visualize_fiber.py --bag ~/f1 --mlp topograph.pt --out /tmp/fiber.png
# ============================================================================
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from cv_bridge import CvBridge                             # noqa: E402
from rclpy.serialization import deserialize_message        # noqa: E402
import rosbag2_py                                          # noqa: E402
from rosidl_runtime_py.utilities import get_message        # noqa: E402

from nav_pkg.nn2.scene_descriptor import SceneEncoder     # noqa: E402


def read_bag(path, storage_id, image_topic, odom_topic, rate, encoder, stride):
    """bag -> (feats (n,D), poses (n,3), stamps (n,)) с выборкой ~rate/stride Гц."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in (image_topic, odom_topic):
        if need not in type_map:
            print(f"⚠ топик {need} нет в bag {path}")

    bridge = CvBridge()
    sample_dt_ns = int(1e9 / rate)
    last_ns = None
    last_odom = None
    seen = 0
    feats, poses, stamps = [], [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, get_message(type_map[topic]))
        if topic == odom_topic:
            last_odom = msg
        elif topic == image_topic:
            if last_ns is not None and (t_ns - last_ns) < sample_dt_ns:
                continue
            if last_odom is None:
                continue
            last_ns = t_ns
            seen += 1
            if (seen - 1) % stride != 0:        # прореживание с сохранением порядка
                continue
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            feats.append(encoder.encode(frame, normalize=False))
            p = last_odom.pose.pose.position
            poses.append([p.x, p.y, p.z])
            stamps.append(t_ns * 1e-9)
    return (np.asarray(feats, np.float32), np.asarray(poses, np.float64),
            np.asarray(stamps, np.float64))


def pca(feats, k):
    """Топ-k главных компонент (numpy SVD). -> (proj (n,k), explained_ratio (k,))."""
    x = feats - feats.mean(axis=0, keepdims=True)
    _, s, vt = np.linalg.svd(x, full_matrices=False)
    proj = x @ vt[:k].T
    ratio = (s[:k] ** 2) / float(np.sum(s ** 2))
    return proj, ratio


def arclen(poses):
    """Накопленная длина пути вдоль траектории, м (для окраски «прогресса»)."""
    d = np.linalg.norm(np.diff(poses, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", nargs="+", required=True)
    ap.add_argument("--out", default="/tmp/fiber.png")
    ap.add_argument("--image-topic", default="/image_color")
    ap.add_argument("--odom-topic", default="/vins_estimator/odometry")
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--stride", type=int, default=1, help="брать каждый stride-й кадр")
    ap.add_argument("--storage-id", default="sqlite3")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--model", default="dinov2_vits14")
    ap.add_argument("--mlp", default=None, help="веса топографа .pt — fiber ПОСЛЕ MLP")
    ap.add_argument("--dims", type=int, default=2, choices=(2, 3),
                    help="размерность проекции дескрипторов")
    ap.add_argument("--color", default="arclen",
                    choices=("arclen", "time", "x", "y", "z", "pass"),
                    help="чем красить точки (общая величина для обеих панелей)")
    args = ap.parse_args()

    encoder = SceneEncoder(model_name=args.model, device=args.device, mlp_path=args.mlp)

    feats_all, poses_all, stamps_all, seg = [], [], [], []
    off = 0
    for b, bag in enumerate(args.bag):
        print(f"== bag: {bag}")
        f, p, s = read_bag(bag, args.storage_id, args.image_topic,
                           args.odom_topic, args.rate, encoder, args.stride)
        if len(f) == 0:
            print("   ⚠ пусто, пропускаю")
            continue
        feats_all.append(f); poses_all.append(p); stamps_all.append(s)
        seg.append((off, off + len(f), b))
        off += len(f)
        print(f"   кадров: {len(f)}")
    if not feats_all:
        raise SystemExit("⚠ ни одного кадра — нечего рисовать.")

    feats = np.vstack(feats_all)
    poses = np.vstack(poses_all)
    proj, ratio = pca(feats, args.dims)
    print(f"== PCA дескрипторов: explained по компонентам {np.round(ratio, 3)} "
          f"(сумма топ-{args.dims}: {ratio.sum():.3f}) — чем выше, тем «тоньше» fiber")

    # величина окраски (одна для обеих панелей)
    if args.color == "arclen":
        c = np.concatenate([arclen(poses[s:e]) for s, e, _ in seg])
        clabel = "длина пути, м"
    elif args.color == "time":
        c = np.concatenate([stamps_all[i] - stamps_all[i][0] for i in range(len(seg))])
        clabel = "время, с"
    elif args.color == "pass":
        c = np.concatenate([np.full(e - s, bi) for s, e, bi in seg])
        clabel = "пролёт #"
    else:
        c = poses[:, {"x": 0, "y": 1, "z": 2}[args.color]]
        clabel = f"{args.color}, м"

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit("⚠ matplotlib нет — поставь его в образе nav для PNG.")

    fig = plt.figure(figsize=(12, 5.5))
    # панель 1: физическая траектория (x-y)
    ax1 = fig.add_subplot(1, 2, 1)
    for s, e, _ in seg:
        ax1.plot(poses[s:e, 0], poses[s:e, 1], "-", lw=0.5, color="0.7", zorder=1)
    sc1 = ax1.scatter(poses[:, 0], poses[:, 1], c=c, s=8, cmap="viridis", zorder=2)
    ax1.set_title("Физическая траектория (VINS), x-y")
    ax1.set_xlabel("x, м"); ax1.set_ylabel("y, м"); ax1.set_aspect("equal", "datalim")

    # панель 2: проекция дескрипторов (fiber-«нить»)
    space = "DINOv2+MLP (метрика)" if encoder.head is not None else "DINOv2 (сырой)"
    if args.dims == 3:
        ax2 = fig.add_subplot(1, 2, 2, projection="3d")
        for s, e, _ in seg:
            ax2.plot(proj[s:e, 0], proj[s:e, 1], proj[s:e, 2], "-", lw=0.5,
                     color="0.7", zorder=1)
        sc2 = ax2.scatter(proj[:, 0], proj[:, 1], proj[:, 2], c=c, s=8, cmap="viridis")
        ax2.set_zlabel("PC3")
    else:
        ax2 = fig.add_subplot(1, 2, 2)
        for s, e, _ in seg:
            ax2.plot(proj[s:e, 0], proj[s:e, 1], "-", lw=0.5, color="0.7", zorder=1)
        sc2 = ax2.scatter(proj[:, 0], proj[:, 1], c=c, s=8, cmap="viridis", zorder=2)
    ax2.set_title(f"Пространство дескрипторов: {space}, PCA-{args.dims}D")
    ax2.set_xlabel("PC1"); ax2.set_ylabel("PC2")

    fig.colorbar(sc2, ax=ax2, label=clabel)
    fig.colorbar(sc1, ax=ax1, label=clabel)
    fig.suptitle("Позиционный fiber: совпадает форма обеих панелей -> fiber "
                 "восстановим; толстый разброс вокруг нити -> нуисансы")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"✅ Fiber-визуализация -> {args.out}")


if __name__ == "__main__":
    main()
