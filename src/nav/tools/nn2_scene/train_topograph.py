#!/usr/bin/env python3
# ============================================================================
# train_topograph.py — ОФЛАЙН-обучение MLP-«топографа» (MetricHead) для NN2.
#
# Цель: «выпрямить» эмбеддинг DINOv2 в (локально) метрический — L2-расстояние
# между выходами головы ≈ физическому перемещению (метры). Бэкбон DINOv2
# ЗАМОРОЖЕН; учим только маленькую голову (scene_descriptor.MetricHead).
#
# Супервизия — дельты одометрии VINS из bag'а облёта (см. nn2_scene_howto.txt).
# Лосс = дистанц-регрессия (||Δhead|| -> метры; задаёт МАСШТАБ) + Triplet Margin
# (порядок/устойчивость). Пары/тройки берём по ВРЕМЕННЫМ офсетам внутри одного
# bag (как в идее): positive ≈ t+pos_dt, negative ≈ t+neg_dt. Кросс-bag пары не
# строим — позы разных пролётов не сведены в один кадр (co-register — отдельно).
#
# Выход: веса .pt (MetricHead.save). Дальше:
#   build_scene_map.py --mlp topograph.pt   (метрическая карта, IndexFlatL2)
#   eval_isometry.py   --mlp topograph.pt   (замер до/после)
#
# Пример:
#   python3 train_topograph.py --bag ~/f1 ~/f2 --out topograph.pt --steps 4000
# ============================================================================
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import torch                                               # noqa: E402
from cv_bridge import CvBridge                             # noqa: E402
from rclpy.serialization import deserialize_message        # noqa: E402
import rosbag2_py                                          # noqa: E402
from rosidl_runtime_py.utilities import get_message        # noqa: E402

from nav_pkg.nn2.scene_descriptor import MetricHead, SceneEncoder   # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parents[2] / "data" / "scene_map" / "topograph.pt"


def read_bag(path, storage_id, image_topic, odom_topic, rate, encoder):
    """bag -> (feats (n,backbone) сырой CLS DINOv2, poses (n,3)) с выборкой ~rate Гц."""
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
    feats, poses = [], []
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
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            feats.append(encoder.encode(frame, normalize=False))   # сырой CLS (бэкбон)
            p = last_odom.pose.pose.position
            poses.append([p.x, p.y, p.z])
    return np.asarray(feats, np.float32), np.asarray(poses, np.float64)


def load_dataset(args, encoder):
    """Считывает (или берёт из кэша) фичи+позы всех bag'ов. Возвращает массивы и
    границы пролётов [(start,end), ...] (пары/тройки — только внутри границ)."""
    if args.cache and Path(args.cache).exists():
        d = np.load(args.cache)
        print(f"== кэш фич: {args.cache}")
        return d["feats"], d["poses"], d["bounds"]

    feats_all, poses_all, bounds = [], [], []
    off = 0
    for bag in args.bag:
        print(f"== bag: {bag}")
        f, p = read_bag(bag, args.storage_id, args.image_topic,
                        args.odom_topic, args.rate, encoder)
        if len(f) == 0:
            print("   ⚠ пусто, пропускаю")
            continue
        feats_all.append(f); poses_all.append(p)
        bounds.append((off, off + len(f)))
        off += len(f)
        print(f"   кадров: {len(f)}")
    if not feats_all:
        raise SystemExit("⚠ ни одного кадра — нечего обучать.")
    feats = np.vstack(feats_all).astype(np.float32)
    poses = np.vstack(poses_all).astype(np.float64)
    bounds = np.asarray(bounds, np.int64)
    if args.cache:
        np.savez(args.cache, feats=feats, poses=poses, bounds=bounds)
        print(f"   фичи -> кэш {args.cache}")
    return feats, poses, bounds


def valid_anchors(bounds, k_neg):
    """Глобальные индексы, у которых anchor+k_neg остаётся в том же пролёте."""
    a = []
    for s, e in bounds:
        if e - s > k_neg + 1:
            a.append(np.arange(s, e - k_neg))
    if not a:
        raise SystemExit(f"⚠ все пролёты короче neg_dt — увеличь bag или уменьши "
                         f"--neg-dt (k_neg={k_neg} кадров).")
    return np.concatenate(a)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", nargs="+", required=True, help="один+ каталог rosbag2")
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--cache", default=None, help=".npz кэш фич (ускоряет повтор)")
    ap.add_argument("--image-topic", default="/image_color")
    ap.add_argument("--odom-topic", default="/vins_estimator/odometry")
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--storage-id", default="sqlite3")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--model", default="dinov2_vits14")
    ap.add_argument("--hidden", type=int, default=256)
    ap.add_argument("--out-dim", type=int, default=64)
    ap.add_argument("--steps", type=int, default=4000)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--margin", type=float, default=5.0, help="triplet margin, м")
    ap.add_argument("--w-reg", type=float, default=1.0, help="вес дистанц-регрессии")
    ap.add_argument("--w-tri", type=float, default=0.5, help="вес triplet")
    ap.add_argument("--pos-dt", type=float, default=0.5, help="positive ~ t+pos_dt, с")
    ap.add_argument("--neg-dt", type=float, default=10.0, help="negative ~ t+neg_dt, с")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)
    dev = args.device if (args.device != "cuda" or torch.cuda.is_available()) else "cpu"
    if dev != args.device:
        print("⚠ CUDA недоступна — учу на CPU.")

    # Бэкбон без головы -> encode(normalize=False) даёт сырой CLS для обучения.
    encoder = SceneEncoder(model_name=args.model, device=dev)
    feats_np, poses_np, bounds = load_dataset(args, encoder)
    print(f"== всего кадров: {len(feats_np)}, пролётов: {len(bounds)}, "
          f"backbone_dim: {feats_np.shape[1]}")

    F = torch.from_numpy(feats_np).to(dev)
    P = torch.from_numpy(poses_np).float().to(dev)

    k_pos = max(1, round(args.pos_dt * args.rate))
    k_neg = max(k_pos + 1, round(args.neg_dt * args.rate))
    anchors = valid_anchors(bounds, k_neg)
    print(f"== офсеты: k_pos={k_pos}, k_neg={k_neg} кадров; валидных якорей: "
          f"{len(anchors)}")

    head = MetricHead(in_dim=feats_np.shape[1], hidden=args.hidden,
                      out_dim=args.out_dim).to(dev).train()
    opt = torch.optim.Adam(head.parameters(), lr=args.lr)

    for step in range(1, args.steps + 1):
        sel = rng.choice(anchors, size=args.batch)
        pos = sel + k_pos
        neg = sel + k_neg
        r = rng.integers(1, k_neg + 1, size=args.batch)      # офсет для регрессии
        par = sel + r                                        # в том же пролёте

        si = torch.from_numpy(sel).to(dev)
        pi = torch.from_numpy(pos).to(dev)
        ni = torch.from_numpy(neg).to(dev)
        ri = torch.from_numpy(par).to(dev)

        ha, hp, hn, hr = head(F[si]), head(F[pi]), head(F[ni]), head(F[ri])
        d_ap = torch.norm(ha - hp, dim=1)
        d_an = torch.norm(ha - hn, dim=1)
        loss_tri = torch.relu(d_ap - d_an + args.margin).mean()

        d_reg = torch.norm(ha - hr, dim=1)
        tgt = torch.norm(P[si] - P[ri], dim=1)              # метры по VINS
        loss_reg = torch.nn.functional.smooth_l1_loss(d_reg, tgt)

        loss = args.w_reg * loss_reg + args.w_tri * loss_tri
        opt.zero_grad(); loss.backward(); opt.step()

        if step % max(1, args.steps // 20) == 0 or step == 1:
            print(f"  step {step:5d}/{args.steps}  loss={loss.item():.3f} "
                  f"(reg={loss_reg.item():.3f} м, tri={loss_tri.item():.3f})")

    head.eval()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    head.save(out)
    print(f"✅ Топограф обучен -> {out} (in={feats_np.shape[1]}, hidden={args.hidden}, "
          f"out={args.out_dim}). Дальше: build_scene_map.py --mlp {out.name}")


if __name__ == "__main__":
    main()
