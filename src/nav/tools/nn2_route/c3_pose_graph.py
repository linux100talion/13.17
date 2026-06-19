#!/usr/bin/env python3
# ============================================================================
# c3_pose_graph.py — ШАГ 6 (c)-основного: POSE-GRAPH (стадия 5, тонкая дооптимизация).
#
# Зачем (XXI: «бутылочное горло — РЕГИСТРАЦИЯ»; register_flights стадия 5): грубая
# регистрация (Umeyama, ШАГ 1/2) сводит облёты в одну рамку ЖЁСТКИМ T на облёт, но
# НЕ давит дрейф ВНУТРИ облёта (VINS накапливает ошибку вдоль траектории). Pose-graph
# уточняет позы ВСЕХ кейфреймов совместно: одометрия держит локальную форму, loop-
# closure и якоря стягивают глобально -> дрейф выдавлен, метки топографа согласованы.
#
# Узлы:  позы кейфреймов всех облётов в общей рамке (SE2: x,y,θ).
# Рёбра: (1) ОДОМЕТРИЯ внутри облёта (из VINS-дельт) — последовательные;
#        (2) LOOP-CLOSURE внутри/между облётами (кросс-связи ШАГ 2) — «это одно место»;
#        (3) ЯКОРЬ NN1 — unary-приор абсолютной позы (георефернс);
#        (4) ДАТУМ — unary-приор, фиксирует gauge (иначе решение плавает).
# Решаем Gauss-Newton'ом на многообразии SE2 (Grisetti). Грубая регистрация (ШАГ 1/2)
# = ХОРОШАЯ начальная точка (иначе нелинейный МНК уйдёт в локальный минимум).
#
# Здесь — чистый numpy SE2-солвер + сборка графа + selftest (дрейф давится).
# На GCP можно подменить на GTSAM/g2o/ceres ради масштаба/робастных ядер — интерфейс
# графа (узлы+рёбра) тот же. Чистый numpy — тестируется.
# ============================================================================
import numpy as np


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def v2t(v):
    """(x,y,θ) -> 3×3 однородная SE2."""
    x, y, th = float(v[0]), float(v[1]), float(v[2])
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def t2v(T):
    """SE2 3×3 -> (x,y,θ)."""
    return np.array([T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])])


class BinaryEdge:
    """Относительное измерение i->j (одометрия / loop-closure): z=(dx,dy,dθ) в рамке i,
    Omega — информационная матрица (обратная ковариация)."""
    def __init__(self, i, j, z, omega):
        self.i, self.j = int(i), int(j)
        self.z = np.asarray(z, float)
        self.omega = np.asarray(omega, float)


class UnaryEdge:
    """Абсолютный приор на узел i (якорь NN1 / датум): z=(x,y,θ) в общей рамке."""
    def __init__(self, i, z, omega):
        self.i = int(i)
        self.z = np.asarray(z, float)
        self.omega = np.asarray(omega, float)


def _binary_err_jac(xi, xj, z):
    """Ошибка и якобианы относительного ребра (Grisetti, SE2).
    e = t2v(Z^{-1} · Xi^{-1} · Xj); A=de/dxi, B=de/dxj."""
    Zt, Xi, Xj = v2t(z), v2t(xi), v2t(xj)
    e = t2v(np.linalg.inv(Zt) @ np.linalg.inv(Xi) @ Xj)
    e[2] = _wrap(e[2])
    ti, tj = xi[:2], xj[:2]
    thi = xi[2]
    c, s = np.cos(thi), np.sin(thi)
    Ri_T = np.array([[c, s], [-s, c]])                 # R_i^T
    dRi_T = np.array([[-s, c], [-c, -s]])              # d R_i^T / dθ_i
    cz, sz = np.cos(z[2]), np.sin(z[2])
    Rij_T = np.array([[cz, sz], [-sz, cz]])            # R_ij^T
    A = np.zeros((3, 3)); B = np.zeros((3, 3))
    A[:2, :2] = -Rij_T @ Ri_T
    A[:2, 2] = Rij_T @ (dRi_T @ (tj - ti))
    A[2, 2] = -1.0
    B[:2, :2] = Rij_T @ Ri_T
    B[2, 2] = 1.0
    return e, A, B


def optimize_pose_graph(nodes_init, binary_edges, unary_edges, iters=30, tol=1e-6,
                        damping=1e-6, log=None):
    """Gauss-Newton на SE2. nodes_init (n,3) — начальные позы (из грубой регистрации).
    -> (nodes_opt (n,3), chi2_history). Gauge фиксируют unary-приоры (датум/якоря);
    если их нет — слабый приор на узел 0 (иначе H вырождена)."""
    x = np.array(nodes_init, float).copy()
    n = len(x)
    chi2_hist = []
    have_unary = len(unary_edges) > 0
    for it in range(iters):
        H = np.zeros((3 * n, 3 * n))
        b = np.zeros(3 * n)
        chi2 = 0.0
        for ed in binary_edges:
            e, A, B = _binary_err_jac(x[ed.i], x[ed.j], ed.z)
            Om = ed.omega
            ii, jj = slice(3 * ed.i, 3 * ed.i + 3), slice(3 * ed.j, 3 * ed.j + 3)
            H[ii, ii] += A.T @ Om @ A
            H[ii, jj] += A.T @ Om @ B
            H[jj, ii] += B.T @ Om @ A
            H[jj, jj] += B.T @ Om @ B
            b[ii] += A.T @ Om @ e
            b[jj] += B.T @ Om @ e
            chi2 += float(e @ Om @ e)
        for ue in unary_edges:                         # абсолютный приор (якорь/датум)
            e = x[ue.i] - ue.z; e[2] = _wrap(e[2])     # глобальная параметризация, J=I
            ii = slice(3 * ue.i, 3 * ue.i + 3)
            H[ii, ii] += ue.omega
            b[ii] += ue.omega @ e
            chi2 += float(e @ ue.omega @ e)
        if not have_unary:
            H[0:3, 0:3] += np.eye(3) * 1e6             # gauge: пришпиливаем узел 0
        H += np.eye(3 * n) * damping
        chi2_hist.append(chi2)
        if log:
            log(f"  iter {it:2d}  chi2={chi2:.4f}")
        dx = np.linalg.solve(H, -b)
        x += dx.reshape(n, 3)
        x[:, 2] = _wrap(x[:, 2])
        if np.linalg.norm(dx) < tol:
            break
    return x, chi2_hist


# ── сборка графа из облётов (одометрия + loop + якоря + датум) ────────────────
def odometry_edges(node_offset, local_poses_xyth, sigma_xy=0.1, sigma_th=0.02):
    """Рёбра одометрии внутри ОДНОГО облёта из последовательных ЛОКАЛЬНЫХ поз VINS
    (x,y,θ): z = поза_i^{-1} · поза_{i+1}. node_offset — индекс 1-го узла облёта."""
    edges = []
    Om = np.diag([1.0 / sigma_xy ** 2, 1.0 / sigma_xy ** 2, 1.0 / sigma_th ** 2])
    P = np.asarray(local_poses_xyth, float)
    for k in range(len(P) - 1):
        z = t2v(np.linalg.inv(v2t(P[k])) @ v2t(P[k + 1]))
        edges.append(BinaryEdge(node_offset + k, node_offset + k + 1, z, Om))
    return edges


def loop_edges(pairs, sigma_xy=0.3, sigma_th=0.05):
    """Loop-closure рёбра: pairs — list (gi, gj, z(3,)) глобальных индексов узлов и
    относительного измерения «эти кейфреймы = одно место» (из кросс-связей ШАГ 2)."""
    Om = np.diag([1.0 / sigma_xy ** 2, 1.0 / sigma_xy ** 2, 1.0 / sigma_th ** 2])
    return [BinaryEdge(gi, gj, np.asarray(z, float), Om) for gi, gj, z in pairs]


def anchor_priors(items, sigma_xy=0.5, sigma_th=0.1):
    """Unary-приоры якорей NN1 / датума: items — list (gi, z(3,) абс. поза)."""
    Om = np.diag([1.0 / sigma_xy ** 2, 1.0 / sigma_xy ** 2, 1.0 / sigma_th ** 2])
    return [UnaryEdge(gi, np.asarray(z, float), Om) for gi, z in items]


# --- самопроверка (numpy): дрейф одометрии давится loop+якорями ---------------
def _selftest():
    rng = np.random.default_rng(0)

    def gt_traj(n, start, step, turn):
        """Истинная траектория: позы (x,y,θ) с поворотом."""
        P = [np.array([start[0], start[1], start[2]])]
        for _ in range(n - 1):
            th = P[-1][2] + turn
            P.append(np.array([P[-1][0] + step * np.cos(th),
                               P[-1][1] + step * np.sin(th), th]))
        return np.array(P)

    # два облёта, частично по одним местам (для loop-closure)
    A = gt_traj(20, (0, 0, 0.0), 4.0, 0.05)
    B = gt_traj(20, (2, 3, 0.3), 4.0, 0.04)
    GT = np.vstack([A, B]); nA = len(A)

    # ОДОМЕТРИЯ с шумом -> интегрируем -> ДРЕЙФ (начальная точка графа)
    def integrate_noisy(P):
        out = [P[0].copy()]
        for k in range(len(P) - 1):
            z = t2v(np.linalg.inv(v2t(P[k])) @ v2t(P[k + 1]))
            z = z + rng.normal(0, [0.15, 0.15, 0.03])      # шум одометрии
            out.append(t2v(v2t(out[-1]) @ v2t(z)))
        return np.array(out)
    initA = integrate_noisy(A)
    initB = integrate_noisy(B)
    init = np.vstack([initA, initB])

    # РЁБРА: одометрия (из ИСТИННЫХ дельт + шум, как реальный VINS даёт),
    edges = []
    edges += odometry_edges(0, A) + odometry_edges(nA, B)
    # LOOP-CLOSURE: 4 пары «одно место» между облётами (истинная относит. поза + шум)
    loops = []
    for ka, kb in [(5, 2), (9, 6), (13, 10), (17, 14)]:
        z = t2v(np.linalg.inv(v2t(GT[ka])) @ v2t(GT[nA + kb])) + rng.normal(0, [0.05, 0.05, 0.01])
        loops.append((ka, nA + kb, z))
    edges += loop_edges(loops)
    # ЯКОРЯ NN1 + ДАТУМ: 3 абсолютных приора (истина + малый шум)
    anchors = anchor_priors([(0, GT[0] + rng.normal(0, [0.05, 0.05, 0.01])),       # датум
                             (nA - 1, GT[nA - 1] + rng.normal(0, [0.1, 0.1, 0.02])),
                             (nA + 18, GT[nA + 18] + rng.normal(0, [0.1, 0.1, 0.02]))])

    opt, chi2 = optimize_pose_graph(init, edges, anchors, iters=30)

    def ate(X):                                            # средняя позиц. ошибка vs истина
        return float(np.linalg.norm(X[:, :2] - GT[:, :2], axis=1).mean())
    ate_before, ate_after = ate(init), ate(opt)

    assert ate_after < ate_before * 0.5, (ate_before, ate_after)
    assert chi2[-1] < chi2[0], (chi2[0], chi2[-1])
    assert ate_after < 0.6, ate_after

    print("c3_pose_graph selftest: OK")
    print(f"  узлов {len(GT)} (2 облёта), рёбер: одометрия {2*(nA-1)}, loop {len(loops)}, "
          f"якорей/датум {len(anchors)}")
    print(f"  ATE (ср. ошибка vs истина): ДО {ate_before:.2f} м -> ПОСЛЕ {ate_after:.2f} м "
          f"({ate_before/max(ate_after,1e-9):.1f}× лучше)")
    print(f"  chi2: {chi2[0]:.1f} -> {chi2[-1]:.3f} за {len(chi2)} итераций")
    print("  -> дрейф одометрии выдавлен loop-closure+якорями; на GCP — GTSAM/g2o (тот же граф)")


if __name__ == "__main__":
    _selftest()
