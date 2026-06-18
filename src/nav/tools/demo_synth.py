#!/usr/bin/env python3
# ============================================================================
# demo_synth.py — END-TO-END мини-демо route-координат БЕЗ ROS, на синтетике.
#
# Прогоняет всю цепочку числом: маршрут -> позиции «облёта» -> цели (s*,e*)
# проекцией (route_geometry) -> синтетические дескрипторы g(s,e)+шум -> ОБУЧЕНИЕ
# descriptor->(s,e) -> поле −∇V и засечка p̂ (route_field) -> метрики на тесте.
#
# Обучение: если есть torch -> зовёт НАСТОЯЩУЮ fit_route_heads из
# train_route_coords (головы MetricHead); иначе numpy-fallback (ридж-регрессия) —
# чтобы демо считалось и без torch/ROS. Дескрипторы кодируют (s,e) ЛИНЕЙНО (+
# гармоники s как «нелинейный» фон), поэтому и головы, и ридж их восстанавливают;
# сигнал e ослаблен -> видно, что e грубее s (раздел XVI).
#
# Запуск:  python3 demo_synth.py
# ============================================================================
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from route_geometry import build_centerline                # noqa: E402
from route_field import RouteField                          # noqa: E402

try:
    import torch                                            # noqa: F401
    from train_route_coords import fit_route_heads
    HAVE_TORCH = True
except Exception:
    HAVE_TORCH = False


def synth_route(n=400):
    """Кривой маршрут (синус) -> центрлиния."""
    t = np.linspace(0, 1, n)
    xy = np.stack([200 * t, 40 * np.sin(2 * np.pi * t)], axis=1)
    return build_centerline(xy, smooth_window=1, resample_ds=2.0)


def synth_flight(cl, offsets=(-15.0, 0.0, 15.0), per_pass=300, rng=None):
    """Позиции «облёта»: проходы со сдвигом ±offset от нити (даёт разброс e)."""
    rng = rng or np.random.default_rng(0)
    pos, bounds = [], []
    off = 0
    for eo in offsets:
        sig = np.sort(rng.uniform(0, cl.L, per_pass))
        pts = np.array([cl.frame_at(s)[0] + eo * cl.frame_at(s)[2] for s in sig])
        pts += rng.normal(0, 0.5, pts.shape)         # лёгкий джиттер траектории
        pos.append(pts); bounds.append((off, off + len(pts))); off += len(pts)
    return np.vstack(pos), np.asarray(bounds, np.int64)


def synth_descriptors(s, e, dim=64, e_signal=0.3, noise=0.4, rng=None):
    """g(s,e): (s,e) ЛИНЕЙНО + гармоники s (фон) -> dim-вектор + шум. Столбцы
    НОРМИРОВАНЫ, затем вес e ослаблен (e_signal) -> при равном шуме e восстановим
    хуже s (как в реальности, XVI)."""
    rng = rng or np.random.default_rng(1)
    cols = np.stack([s, e,
                     np.sin(2 * np.pi * s), np.cos(2 * np.pi * s),
                     np.sin(4 * np.pi * s), np.cos(4 * np.pi * s)], axis=1)
    cols = (cols - cols.mean(0)) / (cols.std(0) + 1e-9)    # нормировка столбцов
    w = np.array([1.0, e_signal, 0.6, 0.6, 0.4, 0.4])      # сигнал e ослаблен
    phi = np.concatenate([cols * w, np.ones((len(s), 1))], axis=1)   # (N,7)
    A = rng.normal(0, 1, (phi.shape[1], dim))
    return (phi @ A + rng.normal(0, noise, (len(s), dim))).astype(np.float32)


def ridge_fit(X, Y, lam=1e-1):
    """numpy-fallback «головы»: стандартизация + гребневая регрессия -> predict."""
    mu, sd = X.mean(0), X.std(0) + 1e-9
    Xs = np.concatenate([(X - mu) / sd, np.ones((len(X), 1))], axis=1)
    W = np.linalg.solve(Xs.T @ Xs + lam * np.eye(Xs.shape[1]), Xs.T @ Y)

    def predict(Xt):
        Xts = np.concatenate([(Xt - mu) / sd, np.ones((len(Xt), 1))], axis=1)
        return Xts @ W
    return predict


def main():
    rng = np.random.default_rng(0)
    cl = synth_route()
    pos, bounds = synth_flight(cl, rng=rng)
    s_star, e_star, _ = cl.project_many(pos)
    feats = synth_descriptors(s_star, e_star, rng=rng)
    print(f"== синтетика: маршрут L={cl.L:.0f} м, кадров {len(feats)}, dim {feats.shape[1]}; "
          f"s*∈[{s_star.min():.2f},{s_star.max():.2f}], e*∈[{e_star.min():.0f},{e_star.max():.0f}] м")

    # train/test split
    idx = rng.permutation(len(feats))
    ntr = int(0.8 * len(idx))
    tr, te = idx[:ntr], idx[ntr:]

    if HAVE_TORCH:
        print("== обучение: НАСТОЯЩИЕ головы MetricHead (fit_route_heads, torch)")
        sh, eh = fit_route_heads(feats[tr], s_star[tr], e_star[tr],
                                 np.array([[0, len(tr)]]), steps=1500, batch=256)
        with torch.no_grad():
            ft = torch.from_numpy(feats[te])
            s_hat = torch.sigmoid(sh(ft).squeeze(1)).numpy()
            e_hat = eh(ft).squeeze(1).numpy()
    else:
        print("== обучение: torch нет -> numpy-fallback (ридж-регрессия)")
        pred = ridge_fit(feats[tr], np.stack([s_star[tr], e_star[tr]], axis=1))
        out = pred(feats[te])
        s_hat = np.clip(out[:, 0], 0, 1)
        e_hat = out[:, 1]

    # --- метрики цепочки ---
    s_mae = float(np.mean(np.abs(s_hat - s_star[te])))
    e_mae = float(np.mean(np.abs(e_hat - e_star[te])))

    fld = RouteField(cl, alpha=1.0, beta=0.3, speed=None)
    cos, perr = [], []
    for i, k in enumerate(te):
        vt = fld.velocity_route(s_star[k], e_star[k])     # поле из истины
        vp = fld.velocity_route(s_hat[i], e_hat[i])       # поле из предсказания
        cos.append(float(vt @ vp / (np.linalg.norm(vt) * np.linalg.norm(vp) + 1e-9)))
        perr.append(float(np.linalg.norm(fld.position(s_hat[i], e_hat[i]) - pos[k])))

    print("\n== РЕЗУЛЬТАТ цепочки (на тесте):")
    print(f"  s MAE: {s_mae:.4f} (доля маршрута) ~ {s_mae * cl.L:.1f} м вдоль")
    print(f"  e MAE: {e_mae:.2f} м вбок   (хуже s — сигнал e ослаблен, XVI)")
    print(f"  поле −∇V: средний косинус направления {np.mean(cos):.3f} (1=идеал)")
    print(f"  засечка p̂: медиана ошибки {np.median(perr):.1f} м, "
          f"90-й перц. {np.percentile(perr, 90):.1f} м")
    print("\n  Цепочка проекция->цели->обучение->поле/засечка прошла end-to-end.")


if __name__ == "__main__":
    main()
