#!/usr/bin/env python3
# ============================================================================
# route_fusion.py — НАБРОСОК объединения метрики и (s,e): иерархия + фьюзинг.
#
# Теория — nn2_navigation_dream.txt, XVIII. Идея в трёх шагах:
#   1) φ (метрический ствол, L2≈метры) -> p̂_metric (карта/FAISS), route-AGNOSTIC;
#   2) p̂_metric + центрлиния -> s,e ГЕОМЕТРИЕЙ (project) — e БЕЗ обучения,
#      чинит боль XVI (нет поперечных данных); ИЛИ s,e из голов (по виду);
#   3) слияние двух оценок позы по АНИЗОТРОПНОЙ ковариации (инф. форма):
#        route-поза p̂_r=R(σ)+e·n (узкая поперёк, широкая вдоль) ⊗ метрика p̂_m.
#   Вне маршрута (e велик / метрика не уверена) -> только метрика (восстановление).
#
# Чистый numpy в ядре (тестируем без torch/ROS). Метрический декодер здесь —
# kNN-регрессия в пространстве φ (СТЕНД вместо настоящего MetricHead+FAISS): так
# демо считается на Termux. Боевая нода подменяет MetricMap.decode на
# scene_descriptor.SceneEncoder(mlp=...) + FAISS-карту (data/scene_map), а s,e —
# на головы из train_route_coords.pt. Слияние/гейт остаются как есть.
#
# Куда встаёт: это шаг «позы» для relocalizer_field — вместо одиночного
# field.position(s,e) даёт фьюз-позу p̂+cov (-> Калман/ray_tracer) и команду поля.
# ============================================================================
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route_geometry import Centerline                      # noqa: E402
from route_field import RouteField                         # noqa: E402


# ============================================================================
# 1. МЕТРИЧЕСКИЙ СТВОЛ φ -> позиция (СТЕНД: kNN-регрессия; бой: MetricHead+FAISS)
# ============================================================================
class MetricMap:
    """Метрическая карта φ: хранит (embeddings E (N,d), positions P (N,2)).
    decode(q) -> (p̂ (2,), cov (2,2), conf): k ближайших мест в пространстве φ
    (L2≈метры), позиция = взвешенное среднее, ковариация = разброс соседей + пол,
    conf∈(0,1] по близости ближайшего соседа (метры).

    Карта покрывает ПЛОЩАДЬ (10 облётов = область, не только нить) — поэтому p̂
    определяется и ВНЕ маршрута, откуда e падает геометрией (XVIII)."""
    def __init__(self, embeddings, positions, k=6, sigma_floor=1.0, conf_scale=5.0):
        self.E = np.asarray(embeddings, np.float64)
        self.P = np.asarray(positions, np.float64)
        self.k = int(k)
        self.sigma_floor = float(sigma_floor)
        self.conf_scale = float(conf_scale)

    def decode(self, q):
        q = np.asarray(q, np.float64)
        d = np.linalg.norm(self.E - q, axis=1)             # L2 в φ ≈ метры
        idx = np.argsort(d)[:self.k]
        dk, Pk = d[idx], self.P[idx]
        w = np.exp(-(dk / (dk.mean() + 1e-9)) ** 2)        # мягкие веса по близости
        w = w / (w.sum() + 1e-12)
        p = (w[:, None] * Pk).sum(0)
        diff = Pk - p
        cov = (w[:, None, None] * np.einsum("ni,nj->nij", diff, diff)).sum(0)
        cov = cov + self.sigma_floor ** 2 * np.eye(2)      # пол неопределённости, м²
        conf = float(np.exp(-dk.min() / self.conf_scale))  # ближе сосед -> увереннее
        return p, cov, conf


# ============================================================================
# 2. ROUTE-ПОЗА из (s,e) с АНИЗОТРОПНОЙ ковариацией + слияние гауссиан
# ============================================================================
def route_pose(field: RouteField, s, e, s_std=0.02, e_std=2.0):
    """(s,e) -> p̂=R(σ)+e·n и ковариация: широкая вдоль T (неопр. s), узкая вдоль n
    (неопр. e). Это сильная сторона route-подхода: поперёк маршрута он ТОЧЕН."""
    R, T, n = field.cl.frame_at(s * field.cl.L)
    p = R + float(e) * n
    sa = float(s_std) * field.cl.L                         # неопр. вдоль маршрута, м
    sc = float(e_std)                                      # неопр. поперёк, м
    cov = sa ** 2 * np.outer(T, T) + sc ** 2 * np.outer(n, n)
    return p, cov


def gaussian_fuse(pa, Ca, pb, Cb):
    """Произведение двух гауссиан (информационная форма Калмана):
       I = Ca⁻¹+Cb⁻¹;  C = I⁻¹;  p = C(Ca⁻¹pa + Cb⁻¹pb).
    Ковариация результата ВСЕГДА не больше каждой из входных -> фьюз не вредит."""
    Ia, Ib = np.linalg.inv(Ca), np.linalg.inv(Cb)
    C = np.linalg.inv(Ia + Ib)
    p = C @ (Ia @ np.asarray(pa, float) + Ib @ np.asarray(pb, float))
    return p, C


# ============================================================================
# 3. ФЬЮЗ-ЛОКАЛИЗАТОР: метрика снизу, (s,e) сверху (XVIII)
# ============================================================================
class FusedLocalizer:
    """Одна точка входа XVIII. По дескриптору:
      - метрика -> p̂_m (route-agnostic) + cov + conf;
      - проекция p̂_m на центрлинию -> (s,e) ГЕОМЕТРИЕЙ (e без обучения);
      - если переданы se_app (головы по виду) — берём их (точнее у нити);
      - НА маршруте (|e|≤gate и метрика уверена): поза = фьюз(метрика, route-поза);
        ВНЕ маршрута: поза = только метрика (восстановление, домен NN1/потеря VINS);
      - команда поля v=−∇V из выбранных (s,e).
    Возвращает dict {p, cov, s, e, v, on_route, conf, source}."""
    def __init__(self, metric_map: MetricMap, field: RouteField,
                 e_gate=8.0, min_metric_conf=0.3, s_std=0.02, e_std=2.0):
        self.mm = metric_map
        self.field = field
        self.e_gate = float(e_gate)
        self.min_metric_conf = float(min_metric_conf)
        self.s_std, self.e_std = float(s_std), float(e_std)

    def localize(self, descriptor, se_app=None):
        p_m, C_m, conf = self.mm.decode(descriptor)
        s_g, e_g, _ = self.field.cl.project_many(p_m[None])       # геометрия из p̂_m
        s_g, e_g = float(s_g[0]), float(e_g[0])

        if se_app is not None:                                    # головы по виду
            s, e, source = float(se_app[0]), float(se_app[1]), "appearance+metric"
        else:
            s, e, source = s_g, e_g, "geometry(metric)"

        on_route = abs(e) <= self.e_gate and conf >= self.min_metric_conf
        p_r, C_r = route_pose(self.field, s, e, self.s_std, self.e_std)
        if on_route:
            p, C = gaussian_fuse(p_m, C_m, p_r, C_r)              # ⊗ обе оценки
        else:
            p, C, source = p_m, C_m, "metric-only(off-route)"    # только метрика

        v = self.field.velocity_route(s, e)                      # рулёжка −∇V
        return {"p": p, "cov": C, "s": s, "e": e, "v": v,
                "on_route": on_route, "conf": conf, "source": source}


# --- самопроверка (только numpy) --------------------------------------------
def _selftest():
    from route_geometry import build_centerline
    rng = np.random.default_rng(0)

    # маршрут вдоль +x; поле тянет вперёд (T) и к нити (−e·n)
    cl = build_centerline(np.stack([np.linspace(0, 100, 50), np.zeros(50)], 1),
                          smooth_window=1, resample_ds=5.0)
    field = RouteField(cl, alpha=1.0, beta=0.5, speed=None)

    # метрическая карта покрывает ПЛОЩАДЬ (сетка ±20 м от нити = «10 облётов»).
    # φ = P·Qᵀ, Q (d,2) с ортонормальными столбцами -> ||φ(a)−φ(b)||=||a−b|| (изометрия).
    gx, gy = np.meshgrid(np.linspace(0, 100, 21), np.linspace(-20, 20, 11))
    P = np.stack([gx.ravel(), gy.ravel()], 1).astype(float)
    Q, _ = np.linalg.qr(rng.normal(0, 1, (16, 2)))           # (16,2), QᵀQ=I
    E = P @ Q.T + rng.normal(0, 0.3, (len(P), 16))           # φ + лёгкий шум
    mmap = MetricMap(E, P, k=6, sigma_floor=1.0, conf_scale=5.0)
    loc = FusedLocalizer(mmap, field, e_gate=8.0)

    # ЗАПРОС ВНЕ нити: истинная позиция (50, 7) -> e_true=7. Сети e мы НЕ учили.
    p_true = np.array([50.0, 7.0])
    q = p_true @ Q.T
    out = loc.localize(q)
    assert out["source"].startswith("geometry"), out["source"]
    assert abs(out["e"] - 7.0) < 2.0, out["e"]               # e ПОЛУЧЕН ГЕОМЕТРИЕЙ
    assert out["v"][0] > 0 and out["v"][1] < 0, out["v"]     # вперёд + тянет на −y

    # фьюз сжимает ковариацию относительно КАЖДОЙ из входных
    p_m, C_m, _ = mmap.decode(q)
    p_r, C_r = route_pose(field, out["s"], out["e"], 0.02, 2.0)
    p_f, C_f = gaussian_fuse(p_m, C_m, p_r, C_r)
    assert np.trace(C_f) < min(np.trace(C_m), np.trace(C_r)), (
        np.trace(C_m), np.trace(C_r), np.trace(C_f))
    assert np.allclose(out["p"], p_f, atol=1e-6)             # localize == фьюз на маршруте

    # ВНЕ зоны маршрута (e за гейтом) -> только метрика
    p_far = np.array([50.0, 18.0])
    far = loc.localize(p_far @ Q.T)
    assert not far["on_route"] and "metric-only" in far["source"], far["source"]

    print("route_fusion selftest: OK")
    print(f"  decode p̂={p_m.round(1)} ≈ {p_true} (карта покрывает площадь)")
    print(f"  e_geom={out['e']:.1f} ≈ 7  — БЕЗ обучения e (XVIII чинит XVI)")
    print(f"  tr(cov): метрика {np.trace(C_m):.1f} / route {np.trace(C_r):.1f} "
          f"-> фьюз {np.trace(C_f):.1f}")
    print(f"  off-route(|e|=18) -> source='{far['source']}'")


if __name__ == "__main__":
    _selftest()
