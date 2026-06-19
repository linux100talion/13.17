#!/usr/bin/env python3
# ============================================================================
# cross_flight_correspond.py — ШАГ 2 (c)-основного: КРОСС-ОБЛЁТНЫЕ СВЯЗИ (стадия 3b).
#
# Зачем (c3_pipeline_howto, пробел П1): регистрация (ШАГ 1) висела ТОЛЬКО на якорях
# NN1 (редких). Если у облёта якорей мало/нет — его нечем свести в общую рамку.
# Выход — связи МЕЖДУ облётами: где два облёта видели ОДНО место, его φ-дескрипторы
# близки. Такая пара даёт связь local_новый <-> global (позиция места в уже-сведённой
# рамке). Так облёт без якорей цепляется к карте через ПЕРЕКРЫТИЕ с зарегистриро-
# ванными.
#
# φ-совпадение ШУМИТ (перцептуальный алиасинг -> ложные пары), поэтому ГЕОМ-ПРОВЕРКА:
#   1. взаимный kNN + Lowe-ratio -> кандидаты-пары (грубо отсеять неоднозначные);
#   2. RANSAC по SE2 (umeyama на 2 точках) -> инлаеры, согласные с ОДНИМ жёстким
#      переносом (ложные пары не лягут на ту же геометрию -> выброшены).
# Инлаеры = надёжные связи; кормим ими ОРКЕСТРАТОР (ШАГ 1) тем же интерфейсом
# (corr_idx/corr_xy у Flight). build_correspondences = инкрементальная сборка:
# сидим на якорях -> распространяем регистрацию через кросс-связи.
#
# Ядро SE2 — из register_flights (стадия 4). Чистый numpy — тестируется.
# ============================================================================
import sys
import pathlib

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "nn2_scene"))
from register_flights import umeyama_2d, apply_se2, register_to_global  # noqa: E402
from build_global_dataset import Flight  # noqa: E402


def _pairwise_d2(q, r):
    """(Nq,D),(Nr,D) -> (Nq,Nr) квадраты L2-расстояний."""
    q = np.asarray(q, np.float64); r = np.asarray(r, np.float64)
    return (q ** 2).sum(1)[:, None] + (r ** 2).sum(1)[None, :] - 2.0 * q @ r.T


def mutual_knn_matches(qf, rf, max_ratio=0.8, max_dist=None):
    """φ-кандидаты: для запроса qf ищем ближайший в rf И наоборот; оставляем ВЗАИМНЫЕ
    (i<->j лучшие друг для друга) + Lowe-ratio (d1/d2<max_ratio — отсев неоднозначных,
    типичный алиасинг). -> array (K,3): (qi, rj, dist). Непараметрика, синтетич.размеры."""
    d2 = _pairwise_d2(qf, rf)
    q_nn = np.argmin(d2, axis=1)                              # для каждого q — лучший r
    r_nn = np.argmin(d2, axis=0)                              # для каждого r — лучший q
    out = []
    for qi, rj in enumerate(q_nn):
        if r_nn[rj] != qi:                                    # не взаимные -> мимо
            continue
        row = d2[qi]
        d1 = row[rj]
        if rf.shape[0] >= 2:                                  # 2-й ближайший для ratio
            second = np.partition(row, 1)[1]
            if d1 > (max_ratio ** 2) * max(second, 1e-12):
                continue
        if max_dist is not None and d1 > max_dist ** 2:
            continue
        out.append((qi, rj, float(np.sqrt(max(d1, 0.0)))))
    return np.array(out, np.float64) if out else np.zeros((0, 3))


def ransac_se2(src, dst, n_iter=200, thresh=2.0, min_inliers=3, seed=0):
    """Геом-проверка: ищет жёсткое T=(R,t), под которое МАКСИМУМ пар src->dst лягут в
    пределах thresh (м). src,dst — (M,2) кандидат-пары (могут содержать ложные).
    -> (R,t, inlier_mask) или (None,None,None) если согласного T нет.
    2D-rigid -> минимальная выборка 2 пары."""
    src = np.asarray(src, np.float64); dst = np.asarray(dst, np.float64)
    M = len(src)
    if M < 2:
        return None, None, None
    rng = np.random.default_rng(seed)
    best_mask, best_n = None, 0
    for _ in range(n_iter):
        s = rng.choice(M, 2, replace=False)
        if np.linalg.norm(src[s[0]] - src[s[1]]) < 1e-6:      # вырожденная выборка
            continue
        R, t, _ = umeyama_2d(src[s], dst[s], with_scale=False)
        err = np.linalg.norm(apply_se2(R, t, src) - dst, axis=1)
        mask = err < thresh
        if mask.sum() > best_n:
            best_n, best_mask = int(mask.sum()), mask
    if best_mask is None or best_n < min_inliers:
        return None, None, None
    R, t, _ = umeyama_2d(src[best_mask], dst[best_mask], with_scale=False)  # рефит на инлаерах
    err = np.linalg.norm(apply_se2(R, t, src) - dst, axis=1)
    return R, t, err < thresh


def correspond_to_pool(flight, pool_feats, pool_xy, max_ratio=0.8, ransac_thresh=2.0,
                       min_inliers=3):
    """Связи ОДНОГО (ещё не сведённого) облёта к ПУЛУ кадров уже-сведённых облётов
    (pool_feats <-> pool_xy в ГЛОБАЛЬНОЙ рамке). φ-кандидаты -> RANSAC-инлаеры.
    -> (corr_idx (m,) лок. индексы кадров облёта, corr_xy (m,2) ГЛОБ. позиции). Пусто,
    если согласного перекрытия нет."""
    m = mutual_knn_matches(flight.feats, pool_feats, max_ratio=max_ratio)
    if len(m) < 2:
        return np.zeros(0, np.int64), np.zeros((0, 2))
    qi = m[:, 0].astype(int); rj = m[:, 1].astype(int)
    src = flight.local_xy[qi]                                 # лок. позы кадров облёта
    dst = pool_xy[rj]                                         # их глоб. позиции из пула
    R, t, mask = ransac_se2(src, dst, thresh=ransac_thresh, min_inliers=min_inliers)
    if mask is None:
        return np.zeros(0, np.int64), np.zeros((0, 2))
    return qi[mask], dst[mask]


def build_correspondences(flights, anchors, min_inliers=3, max_passes=5,
                          max_ratio=0.8, ransac_thresh=2.0):
    """ИНКРЕМЕНТАЛЬНАЯ сборка связей (стадия 3a+3b). flights — list[Flight] (corr пока
    пустые); anchors — {flight_id: (idx (Ai,), global_xy (Ai,2))} для облётов, видевших
    якоря NN1. Алгоритм: (1) СИД — облёты с >=2 якорями регистрируем по якорям, их позы
    в пул; (2) РАСПРОСТРАНЕНИЕ — каждый незарегистрир. облёт цепляем к пулу через
    correspond_to_pool, при >=min_inliers инлаерах регистрируем и доливаем в пул;
    повторяем, пока есть прогресс.
    Возвращает (enriched_flights, registered_ids, unreached_ids): у enriched заполнены
    corr_idx/corr_xy (якоря у сидов / кросс-связи у прочих) -> отдать в
    build_global_dataset, который сведёт ВСЕХ единым кодом регистрации."""
    by_id = {f.flight_id: f for f in flights}
    global_xy = {}                                            # fid -> (Ni,2) в общей рамке
    corr = {}                                                 # fid -> (idx, xy) использованные связи
    pool_f, pool_xy = [], []

    # (1) СИД по якорям
    for fid, fl in by_id.items():
        a = anchors.get(fid)
        if a is None or len(a[0]) < 2:
            continue
        idx, gxy = np.asarray(a[0], np.int64), np.asarray(a[1], np.float64)
        R, t, c = register_to_global(fl.local_xy[idx], gxy)
        global_xy[fid] = apply_se2(R, t, fl.local_xy, c)
        corr[fid] = (idx, gxy)
        pool_f.append(fl.feats); pool_xy.append(global_xy[fid])

    # (2) РАСПРОСТРАНЕНИЕ по кросс-связям
    for _ in range(max_passes):
        progressed = False
        if not pool_f:
            break
        PF, PX = np.vstack(pool_f), np.vstack(pool_xy)
        for fid, fl in by_id.items():
            if fid in global_xy:
                continue
            ci, cx = correspond_to_pool(fl, PF, PX, max_ratio=max_ratio,
                                        ransac_thresh=ransac_thresh, min_inliers=min_inliers)
            if len(ci) < min_inliers:
                continue
            R, t, c = register_to_global(fl.local_xy[ci], cx)
            global_xy[fid] = apply_se2(R, t, fl.local_xy, c)
            corr[fid] = (ci, cx)
            pool_f.append(fl.feats); pool_xy.append(global_xy[fid])
            progressed = True
        if not progressed:
            break

    enriched = []
    for fid, fl in by_id.items():
        ci, cx = corr.get(fid, (np.zeros(0, np.int64), np.zeros((0, 2))))
        enriched.append(Flight(fl.feats, fl.local_xy, ci, cx, fid, place_ids=fl.place_ids))
    reached = set(global_xy)
    unreached = [fid for fid in by_id if fid not in reached]
    return enriched, sorted(reached), sorted(unreached)


# --- самопроверка: облёты БЕЗ якорей цепляются через перекрытие (numpy) --------
def _rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _selftest():
    rng = np.random.default_rng(1)
    n_places, D = 80, 24
    G = rng.uniform([0, 0], [220, 130], (n_places, 2))
    place_desc = rng.normal(0, 1, (n_places, D)).astype(np.float32)
    anchor_places = rng.choice(n_places, 5, replace=False)

    def make_flight(seed, fid, places, give_anchors):
        r = np.random.default_rng(seed)
        idx = np.asarray(places, np.int64)
        Rf, tf = _rot(r.uniform(-np.pi, np.pi)), r.uniform([-50, -50], [50, 50])
        local = (G[idx] - tf) @ Rf + r.normal(0, 0.3, (len(idx), 2))
        feats = place_desc[idx] + r.normal(0, 0.04, (len(idx), D)).astype(np.float32)
        fl = Flight(feats, local, np.zeros(0, np.int64), np.zeros((0, 2)), fid, place_ids=idx)
        anc = None
        if give_anchors:
            seen = np.isin(idx, anchor_places)
            if seen.sum() >= 2:
                anc = (np.nonzero(seen)[0], G[idx[seen]])
        return fl, anc

    # Облёты с перекрытием по местам. F1,F2 видят якоря (сиды). F3 якорей НЕ видит,
    # но делит места с F1; F4 делит с F3 (цепочка); F5 — ИЗОЛИРОВАН (свой регион).
    common = np.setdiff1d(np.arange(n_places - 10), anchor_places)   # общий пул мест
    common = np.union1d(common, anchor_places)               # (якоря — в общем пуле)
    private5 = np.arange(n_places - 10, n_places)             # места ТОЛЬКО для F5
    p1 = np.union1d(rng.choice(common, 30, replace=False), anchor_places[:3])
    p2 = np.union1d(rng.choice(common, 30, replace=False), anchor_places[2:5])
    p3 = np.union1d(rng.choice(p1, 18, replace=False), rng.choice(common, 12, replace=False))
    p4 = np.union1d(rng.choice(p3, 18, replace=False), rng.choice(common, 10, replace=False))
    p5 = rng.choice(private5, 8, replace=False)               # без перекрытия с пулом

    specs = [(11, 1, p1, True), (12, 2, p2, True), (13, 3, p3, False),
             (14, 4, p4, False), (15, 5, p5, False)]
    flights, anchors = [], {}
    for seed, fid, places, ga in specs:
        fl, anc = make_flight(seed, fid, places, ga)
        flights.append(fl)
        if anc is not None:
            anchors[fid] = anc

    enriched, reached, unreached = build_correspondences(flights, anchors, min_inliers=4)

    # F1,F2 — по якорям; F3,F4 — по кросс-связям; F5 — изолирован, не достижим
    assert set(reached) >= {1, 2, 3, 4}, reached
    assert 5 in unreached, (reached, unreached)
    # связи F3 — кросс (НЕ якоря): проверим, что они приводят к ВЕРНОЙ глоб. рамке
    from build_global_dataset import build_global_dataset
    ds, transforms, qc = build_global_dataset(enriched, drop_residual=5.0)
    err = np.linalg.norm(ds.xy - G[ds.place_ids], axis=1)
    assert err.mean() < 2.0, err.mean()
    assert {1, 2, 3, 4} <= set(np.unique(ds.flight_id).tolist())

    f3 = next(f for f in enriched if f.flight_id == 3)
    print("cross_flight_correspond selftest: OK")
    print(f"  сведено по ЯКОРЯМ: F1,F2; по КРОСС-СВЯЗЯМ: F3,F4; недостижим: F5")
    print(f"  reached={reached}  unreached={unreached}")
    print(f"  F3 (без якорей) кросс-связей-инлаеров: {len(f3.corr_idx)}")
    print(f"  итог-набор: {len(ds)} кадров, метки vs глоб. истина {err.mean():.2f} м (макс {err.max():.2f})")
    print(f"  {qc}")
    print("  -> облёты без якорей цепляются к карте через перекрытие; изолированный отсеян")


if __name__ == "__main__":
    _selftest()
