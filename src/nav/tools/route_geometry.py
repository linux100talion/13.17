#!/usr/bin/env python3
# ============================================================================
# route_geometry.py — центрлиния маршрута + проекция позиции -> (s, e).
#
# Переиспользуемая ГЕО-утилита для route-координат NN2 (см.
# nn2_navigation_dream.txt, разделы XIV–XVI). Из траектории(ий) VINS строит
# полилинию-центрлинию R(σ), а затем для любой позиции выдаёт Frenet-разложение:
#   s = нормированная длина дуги вдоль маршрута [0,1]   (∂/∂s = касательная T)
#   e = знаковое расстояние до нити, метры              (∂/∂e = нормаль n)
# Одна проекция -> ОБЕ координаты (s* и e* для обучения s(f)/e(f) и для крошек).
#
# Только numpy (без ROS/torch) — утилита самодостаточна и тестируема локально.
# Координаты ГОРИЗОНТАЛЬНЫЕ (xy): на 200–300 м маршрут в плоскости, z ведёт баро.
# ============================================================================
import numpy as np


def _moving_average(xy, w):
    """Сглаживание полилинии скользящим средним по столбцам (reflect-края)."""
    if w is None or w < 2:
        return xy
    pad = w // 2
    kernel = np.ones(w) / w
    out = np.empty_like(xy)
    for c in range(xy.shape[1]):
        padded = np.pad(xy[:, c], pad, mode="reflect")
        out[:, c] = np.convolve(padded, kernel, mode="valid")[:xy.shape[0]]
    return out


def _resample_by_arclen(xy, ds):
    """Перевыборка полилинии к РАВНОМЕРНОМУ шагу ds (м) по длине дуги."""
    if ds is None:
        return xy
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    L = float(cum[-1])
    if L <= 0:
        return xy
    targets = np.append(np.arange(0.0, L, ds), L)
    return np.stack([np.interp(targets, cum, xy[:, 0]),
                     np.interp(targets, cum, xy[:, 1])], axis=1)


class Centerline:
    """Полилиния маршрута с арк-длиной и проекцией точек в (s, e)."""

    def __init__(self, verts):
        v = np.asarray(verts, dtype=float)[:, :2]
        # выкидываем нулевые сегменты (дубли точек) — иначе деление на 0
        keep = np.concatenate([[True],
                               np.linalg.norm(np.diff(v, axis=0), axis=1) > 1e-9])
        v = v[keep]
        if len(v) < 2:
            raise ValueError("центрлиния: нужно >= 2 различных точек")
        self.V = v                                   # (M,2)
        self.seg = np.diff(v, axis=0)                # (M-1,2)
        self.seglen = np.linalg.norm(self.seg, axis=1)        # (M-1,)
        self.dir = self.seg / self.seglen[:, None]            # единичные
        self.cumlen = np.concatenate([[0.0], np.cumsum(self.seglen)])  # (M,)
        self.L = float(self.cumlen[-1])

    # --- проекция ------------------------------------------------------------
    def project_many(self, P):
        """P (N,>=2) -> (s (N,), e (N,), sigma (N,)). Ближайший сегмент брутфорсом."""
        p = np.asarray(P, dtype=float)[:, :2]
        a = p[:, None, :] - self.V[None, :-1, :]          # (N,S,2) от начала сег.
        t = np.clip((a * self.dir[None]).sum(2) / self.seglen[None], 0.0, 1.0)
        foot = self.V[None, :-1, :] + t[:, :, None] * self.seg[None]   # (N,S,2)
        d = np.linalg.norm(p[:, None, :] - foot, axis=2)  # (N,S) расст. до сегментов
        k = np.argmin(d, axis=1)                          # (N,) лучший сегмент
        rows = np.arange(len(p))

        sigma = self.cumlen[k] + t[rows, k] * self.seglen[k]
        s = sigma / self.L if self.L > 0 else np.zeros_like(sigma)
        # знаковая нормаль выбранного сегмента: n_left = (-dy, dx)
        dk = self.dir[k]
        n_left = np.stack([-dk[:, 1], dk[:, 0]], axis=1)
        e = ((p - foot[rows, k]) * n_left).sum(1)         # знаковое расст. до нити
        return s, e, sigma

    def project(self, p):
        """Одна точка p (>=2,) -> (s, e, sigma) скалярами."""
        s, e, sigma = self.project_many(np.asarray(p, float)[None, :])
        return float(s[0]), float(e[0]), float(sigma[0])

    # --- прямое отображение (нужно полю −∇V и засечке p̂ = R(σ)+e·n) -----------
    def frame_at(self, sigma):
        """Арк-длина σ -> (R(σ), T̂, n̂): точка нити, касательная, левая нормаль."""
        sig = float(np.clip(sigma, 0.0, self.L))
        k = int(np.searchsorted(self.cumlen, sig, side="right") - 1)
        k = min(max(k, 0), len(self.seg) - 1)
        T = self.dir[k].copy()                       # единичная касательная
        n = np.array([-T[1], T[0]])                  # единичная левая нормаль
        R = self.V[k] + (sig - self.cumlen[k]) * T   # точка на нити
        return R, T, n


def build_centerline(points, smooth_window=9, resample_ds=2.0):
    """Центрлиния из ОДНОЙ траектории (напр. опорный пролёт VINS): сглаживание
    скользящим средним -> перевыборка по длине дуги -> Centerline."""
    xy = np.asarray(points, dtype=float)[:, :2]
    xy = _moving_average(xy, smooth_window)
    xy = _resample_by_arclen(xy, resample_ds)
    return Centerline(xy)


def centerline_from_passes(passes, reference=0, smooth_window=9,
                           resample_ds=2.0, n_bins=200):
    """Центральная кривая сквозь ПУЧОК пролётов (co-registered в одной рамке).

    1) строим опорную центрлинию из reference-пролёта;
    2) проецируем точки ВСЕХ пролётов -> их s;
    3) бинуем по s, усредняем РЕАЛЬНЫЕ xy в каждом бине -> центроид сечения пучка
       = центральная кривая;
    4) сглаживаем/ресэмплим -> Centerline.
    Так центрлиния идёт по «середине» облётов, а не по одному из них.
    """
    passes = [np.asarray(p, float)[:, :2] for p in passes]
    ref = build_centerline(passes[reference], smooth_window, resample_ds)

    allp = np.vstack(passes)
    s, _, _ = ref.project_many(allp)
    edges = np.linspace(0.0, 1.0, n_bins + 1)
    idx = np.clip(np.digitize(s, edges) - 1, 0, n_bins - 1)

    means = []
    for b in range(n_bins):
        sel = idx == b
        if np.any(sel):
            means.append(allp[sel].mean(axis=0))
    central = np.asarray(means)                       # по возрастанию s (бины упорядочены)
    central = _moving_average(central, smooth_window)
    central = _resample_by_arclen(central, resample_ds)
    return Centerline(central)


# --- самопроверка (только numpy, без ROS) -----------------------------------
def _selftest():
    # прямой маршрут вдоль +x на [0,100]
    route = np.stack([np.linspace(0, 100, 50), np.zeros(50)], axis=1)
    cl = build_centerline(route, smooth_window=1, resample_ds=5.0)
    assert abs(cl.L - 100.0) < 1e-6, cl.L

    s, e, sig = cl.project(np.array([50.0, 7.0]))     # слева (+y) от +x
    assert abs(s - 0.5) < 1e-3 and abs(e - 7.0) < 1e-3, (s, e)
    s2, e2, _ = cl.project(np.array([25.0, -3.0]))    # справа
    assert abs(s2 - 0.25) < 1e-3 and abs(e2 + 3.0) < 1e-3, (s2, e2)

    # батч
    P = np.array([[50.0, 7.0], [25.0, -3.0], [0.0, 0.0], [100.0, 0.0]])
    S, E, _ = cl.project_many(P)
    assert np.allclose(S, [0.5, 0.25, 0.0, 1.0], atol=1e-3), S
    assert np.allclose(E, [7.0, -3.0, 0.0, 0.0], atol=1e-3), E

    # frame_at: на нити вдоль +x середина ~ (50,0), T≈(1,0), n≈(0,1)
    R, T, nrm = cl.frame_at(50.0)
    assert np.allclose(R, [50, 0], atol=1e-3) and np.allclose(T, [1, 0], atol=1e-3) \
        and np.allclose(nrm, [0, 1], atol=1e-3), (R, T, nrm)

    # пучок: два сдвинутых на ±10 прохода вдоль +x -> центрлиния по середине (y≈0)
    p_up = np.stack([np.linspace(0, 100, 60), np.full(60, 10.0)], axis=1)
    p_dn = np.stack([np.linspace(0, 100, 60), np.full(60, -10.0)], axis=1)
    clb = centerline_from_passes([p_up, p_dn], reference=0,
                                 smooth_window=1, resample_ds=5.0)
    _, e_mid, _ = clb.project(np.array([50.0, 0.0]))
    assert abs(e_mid) < 1.0, e_mid       # середина пучка близко к нити
    print("route_geometry selftest: OK "
          f"(L={cl.L:.1f}; проверены s,e, батч, пучок)")


if __name__ == "__main__":
    _selftest()
