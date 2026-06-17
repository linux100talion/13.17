#!/usr/bin/env python3
# ============================================================================
# eval_isometry.py — измеряет изометрию эмбеддинга NN2 на bag'ах облёта.
#
# Реализует метод из tools/nn2_isometry_eval.txt: насколько латентное расстояние
# пропорционально метрам (DINOv2, а когда появится MLP-«топограф» в SceneEncoder
# — то же мерит DINOv2+MLP). Печатает числа + (опц.) scatter PNG.
#
# Метрики: Spearman/Pearson (монотонность/линейность), линейный фит α (единиц
# вектора на метр) + СКО остатка в метрах, разброс α по карте (глобальная
# изометрия <=> α≈const), Kruskal stress-1, диагноз алиасинга (малое d_lat при
# большом d_phys), сквозная ошибка локализации (recall@Xм) и разбивка по yaw.
#
# Эмбеддинг берётся СЫРОЙ (encode(normalize=False)) — для изометрии важна
# L2-величина. Несколько пролётов: позы должны быть в ОДНОМ кадре (co-register).
#
# Пример:
#   python3 eval_isometry.py --bag ~/flight_map --query-bag ~/flight_test \
#       --window 50 --plot /tmp/iso.png
# ============================================================================
import argparse
import sys
from pathlib import Path

import numpy as np

# nav_pkg импортируемым (общий энкодер с нодой/картой). faiss/torch тянутся
# транзитивно из scene_descriptor; NN-поиск здесь — брутфорс numpy.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cv_bridge import CvBridge                            # noqa: E402
from rclpy.serialization import deserialize_message        # noqa: E402
import rosbag2_py                                          # noqa: E402
from rosidl_runtime_py.utilities import get_message        # noqa: E402

from nav_pkg.scene_descriptor import SceneEncoder          # noqa: E402


# --- чтение bag ------------------------------------------------------------
def read_bag(path, storage_id, image_topic, odom_topic, imu_topic, rate, encoder):
    """bag -> (feats (N,D), poses (N,3), yaws (N,), stamps (N,)) с выборкой ~rate Гц."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in (image_topic, odom_topic, imu_topic):
        if need not in type_map:
            print(f"⚠ топик {need} нет в bag {path} (есть: {sorted(type_map)})")

    bridge = CvBridge()
    sample_dt_ns = int(1e9 / rate)
    last_sample_ns = None
    last_odom = None
    last_imu = None

    feats, poses, yaws, stamps = [], [], [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, get_message(type_map[topic]))
        if topic == odom_topic:
            last_odom = msg
        elif topic == imu_topic:
            last_imu = msg
        elif topic == image_topic:
            if last_sample_ns is not None and (t_ns - last_sample_ns) < sample_dt_ns:
                continue
            if last_odom is None:
                continue   # без позы пара бессмысленна
            last_sample_ns = t_ns
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            feats.append(encoder.encode(frame, normalize=False))
            p = last_odom.pose.pose.position
            poses.append([p.x, p.y, p.z])
            yaws.append(_yaw(last_imu.orientation) if last_imu else 0.0)
            stamps.append(t_ns * 1e-9)

    if not feats:
        raise SystemExit(f"⚠ из bag {path} не выбрано ни одного кадра.")
    return (np.asarray(feats, np.float32), np.asarray(poses, np.float64),
            np.asarray(yaws, np.float64), np.asarray(stamps, np.float64))


def _yaw(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


# --- метрики (numpy-only, без scipy) ---------------------------------------
def _pearson(a, b):
    return float(np.corrcoef(a, b)[0, 1])


def _spearman(a, b):
    ra = np.argsort(np.argsort(a))
    rb = np.argsort(np.argsort(b))
    return _pearson(ra.astype(float), rb.astype(float))


def _fit_alpha0(d_phys, d_lat):
    """α через 0: d_lat ≈ α·d_phys (МНК без свободного члена)."""
    denom = float(np.dot(d_phys, d_phys))
    return float(np.dot(d_phys, d_lat) / denom) if denom > 0 else float("nan")


def _stress1(d_phys, d_lat, alpha0):
    num = float(np.sum((d_lat - alpha0 * d_phys) ** 2))
    den = float(np.sum(d_lat ** 2))
    return float(np.sqrt(num / den)) if den > 0 else float("nan")


def sample_pairs(feats, poses, yaws, window, max_pairs, rng):
    """Случайные пары (i,j) с d_phys<=window -> (d_lat, d_phys, d_yaw, anchor_idx)."""
    n = len(feats)
    want = max_pairs
    # с запасом: часть пар отсеется окном
    cand = min(n * (n - 1) // 2, want * 4) if n > 1 else 0
    i = rng.integers(0, n, size=cand)
    j = rng.integers(0, n, size=cand)
    keep = i != j
    i, j = i[keep], j[keep]
    dphys = np.linalg.norm(poses[i] - poses[j], axis=1)
    m = dphys <= window
    i, j, dphys = i[m][:want], j[m][:want], dphys[m][:want]
    dlat = np.linalg.norm(feats[i] - feats[j], axis=1)
    dyaw = np.abs(np.arctan2(np.sin(yaws[i] - yaws[j]), np.cos(yaws[i] - yaws[j])))
    return dlat, dphys, dyaw, i


def local_alpha_cv(poses, anchor_idx, dphys, dlat, grid, min_pairs):
    """Разброс α по ячейкам сетки (по anchor). Возвращает (mean, std, CV, n_cells)."""
    cells = np.floor(poses[anchor_idx][:, :2] / grid).astype(np.int64)
    keys = cells[:, 0].astype(np.int64) * 100003 + cells[:, 1]
    alphas = []
    for k in np.unique(keys):
        sel = keys == k
        if int(np.count_nonzero(sel)) < min_pairs:
            continue
        alphas.append(_fit_alpha0(dphys[sel], dlat[sel]))
    if not alphas:
        return float("nan"), float("nan"), float("nan"), 0
    a = np.asarray(alphas)
    mean, std = float(a.mean()), float(a.std())
    return mean, std, (std / mean if mean else float("nan")), len(a)


def localization_error(qf, qp, qs, mf, mp, metric, same_bag, guard_s):
    """Для каждого query — NN в карте (брутфорс) -> ошибка позы в метрах + idx NN.

    metric='ip' (сырой DINOv2) — NN по косинусу; 'l2' (с MLP) — по евклиду.
    """
    if metric == "ip":
        qn = qf / (np.linalg.norm(qf, axis=1, keepdims=True) + 1e-9)
        mn = mf / (np.linalg.norm(mf, axis=1, keepdims=True) + 1e-9)
        sim = qn @ mn.T                          # больше — лучше
        if same_bag:
            dt = np.abs(qs[:, None] - qs[None, :])
            sim[dt < guard_s] = -2.0             # глушим временных соседей (и себя)
        nn = np.argmax(sim, axis=1)
    else:                                        # l2: квадрат евклида, меньше — лучше
        d2 = (np.sum(qf ** 2, 1)[:, None] + np.sum(mf ** 2, 1)[None, :]
              - 2.0 * qf @ mf.T)
        if same_bag:
            dt = np.abs(qs[:, None] - qs[None, :])
            d2[dt < guard_s] = np.inf
        nn = np.argmin(d2, axis=1)
    err = np.linalg.norm(qp - mp[nn], axis=1)
    return err, nn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="карта-bag (rosbag2)")
    ap.add_argument("--query-bag", default=None,
                    help="отдельный query-bag (честнее: иные свет/время). Позы "
                         "обязаны быть в ОДНОМ кадре с картой (co-register).")
    ap.add_argument("--image-topic", default="/image_color")
    ap.add_argument("--odom-topic", default="/vins_estimator/odometry")
    ap.add_argument("--imu-topic", default="/mavros/imu/data")
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--storage-id", default="sqlite3")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--model", default="dinov2_vits14")
    ap.add_argument("--mlp", default=None,
                    help="веса MLP-«топографа» (.pt) — мерить DINOv2+MLP (метрика L2)")
    ap.add_argument("--window", type=float, default=50.0, help="окно d_phys, м")
    ap.add_argument("--max-pairs", type=int, default=200000)
    ap.add_argument("--grid", type=float, default=25.0, help="ячейка α-карты, м")
    ap.add_argument("--cell-min-pairs", type=int, default=50)
    ap.add_argument("--alias-pct", type=float, default=5.0,
                    help="порог d_lat = этот перцентиль (малое сходство)")
    ap.add_argument("--alias-dphys", type=float, default=30.0, help="далеко, м")
    ap.add_argument("--recall", default="5,10,20", help="пороги recall, м")
    ap.add_argument("--guard-s", type=float, default=2.0,
                    help="LOO: глушить временных соседей в пределах, с")
    ap.add_argument("--plot", default=None, help="путь к scatter PNG (опц.)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    encoder = SceneEncoder(model_name=args.model, device=args.device, mlp_path=args.mlp)

    print(f"== карта-bag: {args.bag}")
    mf, mp, my, ms = read_bag(args.bag, args.storage_id, args.image_topic,
                              args.odom_topic, args.imu_topic, args.rate, encoder)
    print(f"   кадров: {len(mf)}, dim: {mf.shape[1]}")

    # --- изометрия (пары внутри карты) ---
    dlat, dphys, _dyaw, anc = sample_pairs(mf, mp, my, args.window, args.max_pairs, rng)
    print(f"\n== изометрия: пар в окне 0..{args.window:g} м: {len(dlat)}")
    if len(dlat) < 10:
        print("   ⚠ слишком мало пар — проверь окно/выборку.")
    else:
        alpha0 = _fit_alpha0(dphys, dlat)
        res_m = float(np.std(dlat - alpha0 * dphys)) / alpha0 if alpha0 else float("nan")
        mean, std, cv, ncells = local_alpha_cv(
            mp, anc, dphys, dlat, args.grid, args.cell_min_pairs)
        print(f"   Spearman ρ (монотонность): {_spearman(dphys, dlat):+.3f}")
        print(f"   Pearson  r (линейность):   {_pearson(dphys, dlat):+.3f}")
        print(f"   α (ед. вектора / м):       {alpha0:.4g}")
        print(f"   СКО остатка:               {res_m:.2f} м")
        print(f"   Kruskal stress-1:          {_stress1(dphys, dlat, alpha0):.3f}  (0=идеал)")
        print(f"   α по карте: mean={mean:.4g} std={std:.4g} CV={cv:.3f} "
              f"(ячеек {ncells}, чем меньше CV — тем глобальнее изометрия)")

        # --- алиасинг ---
        eps = float(np.percentile(dlat, args.alias_pct))
        mask = (dlat < eps) & (dphys > args.alias_dphys)
        frac = float(np.mean(mask))
        print(f"\n== алиасинг: d_lat<{eps:.3g} (p{args.alias_pct:g}) и "
              f"d_phys>{args.alias_dphys:g} м -> {frac*100:.2f}% пар "
              f"({int(mask.sum())} шт.); это зоны под верификацию NN1")

    # --- сквозная локализация ---
    same_bag = args.query_bag is None
    if same_bag:
        qf, qp, qs = mf, mp, ms
        print(f"\n== локализация (LOO в карте, guard {args.guard_s:g} с)")
    else:
        print(f"== query-bag: {args.query_bag}")
        qf, qp, qy, qs = read_bag(args.query_bag, args.storage_id, args.image_topic,
                                  args.odom_topic, args.imu_topic, args.rate, encoder)
        print(f"   кадров: {len(qf)}")
        print(f"\n== локализация (query-bag vs карта; позы должны быть в одном кадре)")
    err, nn = localization_error(qf, qp, qs, mf, mp, encoder.metric, same_bag,
                                 args.guard_s)
    thr = [float(x) for x in args.recall.split(",") if x.strip()]
    print(f"   медиана: {np.median(err):.2f} м | 90-й перц.: "
          f"{np.percentile(err, 90):.2f} м")
    for x in thr:
        print(f"   recall@{x:g}м: {np.mean(err < x)*100:.1f}%")

    # --- по ракурсу: |Δyaw| между query и его NN ---
    q_yaw = qy if not same_bag else my
    dyaw_q = np.abs(np.arctan2(np.sin(q_yaw - my[nn]), np.cos(q_yaw - my[nn])))
    print(f"\n== по ракурсу (|Δyaw| query<->NN):")
    edges = np.deg2rad([0, 30, 90, 180])
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (dyaw_q >= lo) & (dyaw_q < hi)
        if int(sel.sum()) == 0:
            continue
        print(f"   {np.rad2deg(lo):3.0f}–{np.rad2deg(hi):3.0f}°: "
              f"медиана {np.median(err[sel]):.2f} м (n={int(sel.sum())})")

    # --- scatter PNG ---
    if args.plot and len(dlat) >= 10:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            plt.figure(figsize=(5, 5))
            plt.scatter(dphys, dlat, s=2, alpha=0.2)
            xs = np.array([0, args.window])
            plt.plot(xs, alpha0 * xs, "r-", lw=1, label=f"α·d_phys (α={alpha0:.3g})")
            plt.xlabel("d_phys, м"); plt.ylabel("d_lat (L2)")
            plt.title("Изометрия: латент vs метры"); plt.legend()
            plt.tight_layout(); plt.savefig(args.plot, dpi=120)
            print(f"\n   scatter -> {args.plot}")
        except ImportError:
            print("\n   ⚠ matplotlib нет — PNG пропущен.")


if __name__ == "__main__":
    main()
