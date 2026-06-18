#!/usr/bin/env python3
# ============================================================================
# train_route_coords.py — ОФЛАЙН-обучение route-координат NN2: s(f) и e(f).
#
# Близнец train_topograph.py, но вместо метрики учит ДВЕ скалярные головы:
#   s(descriptor) ∈ [0,1] — прогресс вдоль маршрута (∂/∂s = «вперёд»);
#   e(descriptor) ∈ ℝ (м) — знаковое смещение от нити (∂/∂e = «к маршруту»).
# Вместе они — Frenet-рамка маршрута; из них собирается поле f = −∇V и засечка
# p̂ = R(σ)+e·n (см. nn2_navigation_dream.txt, XIV–XVI).
#
# Цели s*, e* — ОДНОЙ проекцией поз VINS на центрлинию (route_geometry):
#   построили центрлинию (опорный пролёт или центр пучка) -> project_many -> s*,e*.
# Бэкбон DINOv2 заморожен, головы — MetricHead(out_dim=1) (для s сверху сигмоида).
#
# ── ДВЕ СТАДИИ (расцепление тяжёлого извлечения и быстрого обучения) ─────────
#   1) ИЗВЛЕЧЕНИЕ фич (ROS + DINOv2, медленно, раз; на ноуте/Orin с bag'ами):
#        train_route_coords.py --bag ~/f1 ~/f2 --save-feats feats.npz --extract-only
#      -> feats.npz {feats(N,backbone), poses(N,3), bounds(P,2), model, топики, rate}
#   2) ОБУЧЕНИЕ голов (только torch, БЕЗ ROS/DINOv2/bag'ов; быстро, повторяемо —
#      перебор центрлинии/lr/весов не требует пере-извлечения):
#        train_route_coords.py --npz feats.npz --multipass --out route_coords.pt
#   (Можно и одной командой: --bag ... [--save-feats feats.npz] — извлечёт и обучит.)
#
# ⚠ Для e нужен ПОПЕРЕЧНЫЙ разброс данных (один проход -> e*≈0): сдвинутые проходы
#   или мультипролёт (--multipass). См. раздел XVI.
#
# Импорты torch/scene_descriptor — ЛЕНИВЫЕ (внутри функций), чтобы npz-I/O и
# геометрию можно было импортировать/тестировать без torch (как demo_synth).
# ============================================================================
import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))                 # route_geometry (сосед по tools/nn2_route/)
sys.path.insert(0, str(HERE.parents[1]))      # src/nav -> nav_pkg (для ленивых импортов)

# ROS нужен только для read_bag (стадия извлечения). Обучение из --npz его не зовёт.
try:
    from cv_bridge import CvBridge
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message
    _HAVE_ROS = True
except ImportError:
    _HAVE_ROS = False
from route_geometry import build_centerline, centerline_from_passes  # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parents[2] / "data" / "scene_map" / "route_coords.pt"


# ============================================================================
# npz-расцепление: фичи+позы+пролёты (стенд между извлечением и обучением)
# ============================================================================
def save_feats_npz(path, feats, poses, bounds, model, image_topic, odom_topic, rate):
    """Сохранить извлечённые фичи/позы/границы пролётов + контекст. Чистый numpy."""
    np.savez(str(path),
             feats=np.asarray(feats, np.float32),
             poses=np.asarray(poses, np.float64),
             bounds=np.asarray(bounds, np.int64),
             model=np.array(str(model)),
             image_topic=np.array(str(image_topic)),
             odom_topic=np.array(str(odom_topic)),
             rate=np.array(float(rate)))


def load_feats_npz(path):
    """-> (feats (N,D) f32, poses (N,3) f64, bounds (P,2) i64, model str|None)."""
    d = np.load(str(path), allow_pickle=False)
    feats = d["feats"].astype(np.float32)
    poses = d["poses"].astype(np.float64)
    bounds = d["bounds"].astype(np.int64)
    model = str(d["model"]) if "model" in d.files else None
    return feats, poses, bounds, model


def read_bag(path, storage_id, image_topic, odom_topic, rate, encoder):
    """bag -> (feats (n,backbone) сырой CLS, poses (n,3)) с выборкой ~rate Гц."""
    if not _HAVE_ROS:
        raise SystemExit("read_bag требует ROS (cv_bridge/rosbag2_py).")
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
    last_ns, last_odom = None, None
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
            feats.append(encoder.encode(frame, normalize=False))
            p = last_odom.pose.pose.position
            poses.append([p.x, p.y, p.z])
    return np.asarray(feats, np.float32), np.asarray(poses, np.float64)


def extract_bags(bags, storage_id, image_topic, odom_topic, rate, model, device):
    """Стадия 1: bag'и -> (feats, poses, bounds) через DINOv2 (ленивый torch+энкодер)."""
    import torch                                              # noqa: E402
    from nav_pkg.nn2.scene_descriptor import SceneEncoder     # noqa: E402
    dev = device if (device != "cuda" or torch.cuda.is_available()) else "cpu"
    if dev != device:
        print("⚠ CUDA недоступна — считаю фичи на CPU (медленно).")
    encoder = SceneEncoder(model_name=model, device=dev)      # без головы -> сырой CLS

    feats, poses, bounds, off = [], [], [], 0
    for bag in bags:
        print(f"== bag: {bag}")
        f, p = read_bag(bag, storage_id, image_topic, odom_topic, rate, encoder)
        if len(f) == 0:
            print("   ⚠ пусто, пропускаю"); continue
        feats.append(f); poses.append(p)
        bounds.append((off, off + len(f))); off += len(f)
        print(f"   кадров: {len(f)}")
    if not feats:
        raise SystemExit("⚠ ни одного кадра — нечего извлекать.")
    return (np.vstack(feats).astype(np.float32), np.vstack(poses),
            np.asarray(bounds, np.int64))


def build_route_targets(poses, bounds, multipass, reference, smooth_window, resample_ds):
    """Центрлиния + цели s*,e* одной проекцией (чистый numpy, route_geometry)."""
    passes_xy = [poses[s:e, :2] for s, e in bounds]
    ref = min(reference, len(passes_xy) - 1)
    if multipass and len(passes_xy) > 1:
        cl = centerline_from_passes(passes_xy, reference=ref,
                                    smooth_window=smooth_window, resample_ds=resample_ds)
    else:
        cl = build_centerline(passes_xy[ref],
                              smooth_window=smooth_window, resample_ds=resample_ds)
    s_star, e_star, _ = cl.project_many(poses[:, :2])
    return cl, s_star, e_star


def main():
    ap = argparse.ArgumentParser(
        description="Обучение route-координат s,e (двухстадийно: извлечение -> обучение).")
    ap.add_argument("--bag", nargs="+", default=None,
                    help="ROS bag'и облёта (стадия ИЗВЛЕЧЕНИЯ; нужны ROS+DINOv2)")
    ap.add_argument("--npz", default=None,
                    help="готовые фичи .npz (стадия ОБУЧЕНИЯ; ROS/DINOv2 НЕ нужны)")
    ap.add_argument("--save-feats", default=None,
                    help="куда сохранить извлечённые фичи .npz (для повторного обучения)")
    ap.add_argument("--extract-only", action="store_true",
                    help="только извлечь фичи в --save-feats и выйти (без обучения)")
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--image-topic", default="/image_color")
    ap.add_argument("--odom-topic", default="/vins_estimator/odometry")
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--storage-id", default="sqlite3")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--model", default="dinov2_vits14")
    ap.add_argument("--hidden", type=int, default=256)
    # центрлиния
    ap.add_argument("--multipass", action="store_true",
                    help="центрлиния по СЕРЕДИНЕ пучка пролётов (иначе опорный)")
    ap.add_argument("--reference", type=int, default=0, help="индекс опорного пролёта")
    ap.add_argument("--smooth-window", type=int, default=9)
    ap.add_argument("--resample-ds", type=float, default=2.0, help="шаг центрлинии, м")
    # обучение
    ap.add_argument("--steps", type=int, default=4000)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--w-s", type=float, default=1.0, help="вес регрессии s")
    ap.add_argument("--w-e", type=float, default=1.0, help="вес регрессии e")
    ap.add_argument("--w-rank", type=float, default=0.2, help="вес ranking s (порядок)")
    ap.add_argument("--dump", default=None, help=".npz с s*,e* на кадр (для крошек)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if not args.bag and not args.npz:
        raise SystemExit("нужен либо --bag (извлечь из bag'ов), либо --npz (обучить из фич).")
    if args.extract_only and not args.save_feats:
        raise SystemExit("--extract-only требует --save-feats (куда писать фичи).")

    model = args.model

    # === СТАДИЯ 1: фичи — из npz (готовые) или извлечением из bag'ов ===
    if args.npz:
        feats, poses, bounds, npz_model = load_feats_npz(args.npz)
        if npz_model:
            model = npz_model       # чекпойнт обязан помнить, какой сетью считаны фичи
        print(f"== фичи из npz {args.npz}: кадров {len(feats)}, dim {feats.shape[1]}, "
              f"пролётов {len(bounds)}, модель {model}")
    else:
        feats, poses, bounds = extract_bags(
            args.bag, args.storage_id, args.image_topic, args.odom_topic,
            args.rate, model, args.device)
        if args.save_feats:
            save_feats_npz(args.save_feats, feats, poses, bounds, model,
                           args.image_topic, args.odom_topic, args.rate)
            print(f"== фичи сохранены -> {args.save_feats} "
                  f"(переобучай головы из них через --npz, без bag'ов/DINOv2)")
        if args.extract_only:
            print("✅ извлечение завершено (--extract-only).")
            return

    # === центрлиния + цели s*,e* (чистый numpy) ===
    cl, s_star, e_star = build_route_targets(
        poses, bounds, args.multipass, args.reference,
        args.smooth_window, args.resample_ds)
    print(f"== центрлиния: L={cl.L:.1f} м; s*∈[{s_star.min():.2f},{s_star.max():.2f}], "
          f"e*∈[{e_star.min():.1f},{e_star.max():.1f}] м "
          f"(узкий e -> поперечного разброса мало, см. XVI)")
    if args.dump:
        np.savez(args.dump, s=s_star, e=e_star, poses=poses)
        print(f"   s*,e* -> {args.dump}")

    # === СТАДИЯ 2: обучение голов (ленивый torch) ===
    import torch                                              # noqa: E402
    torch.manual_seed(args.seed)
    dev = args.device if (args.device != "cuda" or torch.cuda.is_available()) else "cpu"
    if dev != args.device:
        print("⚠ CUDA недоступна — учу головы на CPU (быстро: MLP на готовых фичах).")
    s_head, e_head = fit_route_heads(
        feats, s_star, e_star, bounds, device=dev, hidden=args.hidden,
        steps=args.steps, batch=args.batch, lr=args.lr, w_s=args.w_s,
        w_e=args.w_e, w_rank=args.w_rank, seed=args.seed)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    torch.save({
        "model": model,
        "backbone_dim": int(feats.shape[1]),
        "hidden": args.hidden,
        "s_head": s_head.state_dict(),
        "e_head": e_head.state_dict(),
        "centerline_verts": cl.V,      # R(σ): нить для p̂ = R(σ)+e·n при засечке
        "centerline_L": cl.L,
    }, out)
    print(f"✅ Route-координаты обучены -> {out} (s,e головы + центрлиния). "
          f"Дальше: поле −∇V и засечка p̂=R(σ)+e·n из (s,e).")


def fit_route_heads(feats, s_star, e_star, bounds, device="cpu", hidden=256,
                    steps=4000, batch=512, lr=1e-3, w_s=1.0, w_e=1.0, w_rank=0.2,
                    seed=0, log=print):
    """Учит головы s(f),e(f) на готовых фичах+целях. Вынесено из main, чтобы код
    обучения вызывали и не-ROS потребители (напр. demo_synth.py). Возвращает
    (s_head, e_head) — torch-модули MetricHead(out_dim=1). torch/MetricHead —
    ленивым импортом (модуль импортируется и без torch)."""
    import torch                                              # noqa: E402
    from nav_pkg.nn2.scene_descriptor import MetricHead       # noqa: E402
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    F = torch.from_numpy(np.asarray(feats, np.float32)).to(device)
    s_t = torch.from_numpy(np.asarray(s_star, np.float32)).to(device)
    e_t = torch.from_numpy(np.asarray(e_star, np.float32)).to(device)

    s_head = MetricHead(in_dim=F.shape[1], hidden=hidden, out_dim=1).to(device).train()
    e_head = MetricHead(in_dim=F.shape[1], hidden=hidden, out_dim=1).to(device).train()
    opt = torch.optim.Adam(list(s_head.parameters()) + list(e_head.parameters()), lr=lr)
    huber = torch.nn.functional.smooth_l1_loss
    n = len(feats)

    for step in range(1, steps + 1):
        bi = torch.from_numpy(rng.integers(0, n, size=batch)).to(device)
        loss_s = huber(torch.sigmoid(s_head(F[bi]).squeeze(1)), s_t[bi])
        loss_e = huber(e_head(F[bi]).squeeze(1), e_t[bi])

        a = rng.integers(0, n, size=batch)
        b = rng.integers(0, n, size=batch)
        same = np.array([_same_pass(x, y, bounds) for x, y in zip(a, b)])
        a, b = a[same], b[same]
        if len(a) > 0:
            lo = np.where(s_star[a] <= s_star[b], a, b)   # «раньше» по маршруту
            hi = np.where(s_star[a] <= s_star[b], b, a)
            slo = torch.sigmoid(s_head(F[torch.from_numpy(lo).to(device)]).squeeze(1))
            shi = torch.sigmoid(s_head(F[torch.from_numpy(hi).to(device)]).squeeze(1))
            loss_rank = torch.relu(slo - shi).mean()       # хотим slo < shi
        else:
            loss_rank = torch.zeros((), device=device)

        loss = w_s * loss_s + w_e * loss_e + w_rank * loss_rank
        opt.zero_grad(); loss.backward(); opt.step()
        if step % max(1, steps // 20) == 0 or step == 1:
            log(f"  step {step:5d}/{steps}  loss={loss.item():.3f} "
                f"(s={loss_s.item():.3f}, e={loss_e.item():.3f} м, "
                f"rank={float(loss_rank):.3f})")
    s_head.eval(); e_head.eval()
    return s_head, e_head


def _same_pass(i, j, bounds):
    for s, e in bounds:
        if s <= i < e and s <= j < e:
            return True
    return False


if __name__ == "__main__":
    main()
