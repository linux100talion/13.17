#!/usr/bin/env python3
# ============================================================================
# c3_route_pipeline.py — ШАГ 3 (c)-основного: XXVI end-to-end (синтез геометрия->C).
#
# Зачем (XXVI «геометрия учит, C рулит»): сводим стадии 4->6 (регистрация, ШАГ 1/2)
# с разметкой маршрута и обучением голов в ОДНУ офлайн-цепочку:
#   1. глобальный набор (φ, ГЛОБ.xy) из build_global_dataset/cross_flight_correspond;
#   2. РИСУЕМ полилинию маршрута в глобальной рамке (не обязательно чей-то пролёт);
#   3. ПРОЕЦИРУЕМ глоб. позы ВСЕХ облётов на полилинию -> s*,e* (геометрия!);
#   4. учим дешёвые головы s(φ),e(φ) на (φ, s*,e*);             ← вариант C
#   5. рантайм: φ -> головы -> s,e -> поле −∇V. БЕЗ FAISS, один forward-pass.
# Геометрия ушла в РАЗМЕТКУ, не в рантайм. Это ОБОБЩЕНИЕ train_route_coords:
# центрлиния — ЛЮБАЯ полилиния, проекция — поз ВСЕХ облётов (а не одного пролёта).
#
# Стадия 4 (обучение голов) = train_route_coords.fit_route_heads (torch, MLP) — на
# машине с torch. Здесь: разметка (route_targets_on_polyline), ПОКРЫТИЕ (XXVI-
# оговорка) и numpy-проверка УЧЕБНОСТИ меток (kNN в φ как честный аналог MLP) +
# экспорт набора под fit_route_heads. Чистый numpy — тестируется.
# ============================================================================
import numpy as np

from route_geometry import Centerline, build_centerline
from build_global_dataset import GlobalDataset


def bounds_from_flight_id(flight_id):
    """Границы (P,2) непрерывных блоков одного облёта в стэке (для ranking-лосса
    fit_route_heads: порядок s осмыслен ВНУТРИ облёта, не между). Требует, чтобы
    кадры облёта лежали подряд (так делает build_global_dataset)."""
    fid = np.asarray(flight_id)
    if len(fid) == 0:
        return np.zeros((0, 2), np.int64)
    cut = np.nonzero(np.diff(fid) != 0)[0] + 1
    starts = np.concatenate([[0], cut])
    ends = np.concatenate([cut, [len(fid)]])
    return np.stack([starts, ends], axis=1).astype(np.int64)


def draw_polyline(verts, smooth_window=1, resample_ds=2.0):
    """Маршрут = НАРИСОВАННАЯ полилиния waypoint'ов (глоб. рамка). Сглаживаем/
    ресэмплим в нить. Источник verts: рука на карте / список GPS / демо-пролёт."""
    v = np.asarray(verts, float)[:, :2]
    if smooth_window > 1 or resample_ds:
        return build_centerline(v, smooth_window=smooth_window, resample_ds=resample_ds)
    return Centerline(v)


def route_targets_on_polyline(global_xy, polyline):
    """XXVI стадия 3: проекция ГЛОБ. поз ВСЕХ облётов на НАРИСОВАННУЮ полилинию ->
    s*,e* на каждый кадр. ОБОБЩЕНИЕ build_route_targets: центрлиния задана, а не
    строится из опорного пролёта. polyline — Centerline или verts (N,2).
    -> (Centerline, s_star (N,), e_star (N,))."""
    cl = polyline if isinstance(polyline, Centerline) else draw_polyline(polyline)
    s_star, e_star, _ = cl.project_many(np.asarray(global_xy, float)[:, :2])
    return cl, s_star, e_star


def coverage(s_star, e_star, e_tol=8.0, nbins=50):
    """XXVI-оговорка про ПОКРЫТИЕ: полилиния получает осмысленные метки лишь там, где
    облёты летали РЯДОМ (малый |e|). Бинуем по s, бин «покрыт», если есть кадр с
    |e|<e_tol. -> (covered_frac, per_bin_bool, gap_bins). Низкое покрытие = маршрут
    выходит за конверт налётанного."""
    s = np.asarray(s_star); e = np.asarray(e_star)
    edges = np.linspace(0.0, 1.0, nbins + 1)
    idx = np.clip(np.digitize(s, edges) - 1, 0, nbins - 1)
    near = np.abs(e) < e_tol
    per_bin = np.zeros(nbins, bool)
    for b in range(nbins):
        sel = idx == b
        per_bin[b] = bool(np.any(sel & near))
    return float(per_bin.mean()), per_bin, int((~per_bin).sum())


def save_route_dataset(path, feats, s_star, e_star, flight_id, centerline, model=None):
    """Набор стадии 6+разметка -> npz под fit_route_heads (torch-машина):
    feats, s*, e*, bounds (из flight_id), центрлиния (для p̂=R(σ)+e·n при засечке)."""
    bounds = bounds_from_flight_id(flight_id)
    np.savez(str(path),
             feats=np.asarray(feats, np.float32),
             s_star=np.asarray(s_star, np.float64),
             e_star=np.asarray(e_star, np.float64),
             bounds=bounds,
             flight_id=np.asarray(flight_id, np.int64),
             centerline_verts=np.asarray(centerline.V, np.float64),
             centerline_L=np.array(float(centerline.L)),
             model=np.array(str(model)))


# --- numpy-проверка УЧЕБНОСТИ меток (стенд-ин вместо torch-MLP) ---------------
def knn_predict_se(train_feats, train_s, train_e, query_feats, k=5):
    """Стенд-ин головы C: предсказать s,e для query по k ближайшим в φ среди train.
    kNN — честный аналог «выучит ли MLP»: если метки СОГЛАСОВАННАЯ функция φ (одно
    место -> один φ -> одни s,e), kNN их вернёт. Реальная голова = MLP (fit_route_
    heads), но эта проверка не требует torch. Веса ∝ 1/d (мягкое усреднение)."""
    tf = np.asarray(train_feats, np.float64); qf = np.asarray(query_feats, np.float64)
    d2 = (qf ** 2).sum(1)[:, None] + (tf ** 2).sum(1)[None, :] - 2.0 * qf @ tf.T
    k = min(k, tf.shape[0])
    nn = np.argpartition(d2, k - 1, axis=1)[:, :k]            # k ближайших
    s_pred, e_pred = np.zeros(len(qf)), np.zeros(len(qf))
    ts, te = np.asarray(train_s), np.asarray(train_e)
    for i in range(len(qf)):
        w = 1.0 / (np.sqrt(np.maximum(d2[i, nn[i]], 0.0)) + 1e-6)
        w /= w.sum()
        s_pred[i] = (w * ts[nn[i]]).sum()
        e_pred[i] = (w * te[nn[i]]).sum()
    return s_pred, e_pred


# --- самопроверка: глоб. набор -> полилиния -> метки -> учебность (numpy) ------
def _rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _selftest():
    from build_global_dataset import Flight, build_global_dataset

    rng = np.random.default_rng(2)
    n_places, D = 60, 24                                      # компактный пул -> высокое перекрытие облётов
    G = rng.uniform([0, 0], [200, 120], (n_places, 2))
    place_desc = rng.normal(0, 1, (n_places, D)).astype(np.float32)
    anchors = rng.choice(n_places, 6, replace=False)

    def make_flight(seed, fid):
        r = np.random.default_rng(seed)
        # высокое перекрытие: облёты тянут из ОБЩЕГО пула -> held-out места видны в train
        idx = np.unique(np.concatenate([r.choice(n_places, 45, replace=False), anchors[:4]]))
        Rf, tf = _rot(r.uniform(-np.pi, np.pi)), r.uniform([-40, -40], [40, 40])
        local = (G[idx] - tf) @ Rf + r.normal(0, 0.3, (len(idx), 2))
        feats = place_desc[idx] + r.normal(0, 0.05, (len(idx), D)).astype(np.float32)
        seen = np.isin(idx, anchors)
        return Flight(feats, local, np.nonzero(seen)[0], G[idx[seen]], fid, place_ids=idx)

    flights = [make_flight(s, fid=s) for s in (1, 2, 3, 4)]
    ds, _, qc = build_global_dataset(flights, drop_residual=5.0)        # стадии 4->6

    # (XXVI.2) РИСУЕМ полилинию через карту (диагональ — часть пройдёт сквозь конверт)
    route_verts = np.array([[10.0, 20.0], [70.0, 50.0], [130.0, 70.0], [190.0, 100.0]])
    cl = draw_polyline(route_verts, smooth_window=1, resample_ds=3.0)

    # (XXVI.3) проекция ВСЕХ облётов -> метки s*,e*
    cl, s_star, e_star = route_targets_on_polyline(ds.xy, cl)
    cov_frac, _, gaps = coverage(s_star, e_star, e_tol=10.0, nbins=40)

    # (XXVI.4 проверка УЧЕБНОСТИ) держим облёт 4, учим на 1-3, предсказываем s,e
    test = ds.flight_id == 4
    train = ~test
    s_pred, e_pred = knn_predict_se(ds.feats[train], s_star[train], e_star[train],
                                    ds.feats[test], k=5)
    s_mae = float(np.mean(np.abs(s_pred - s_star[test])))
    e_mae = float(np.mean(np.abs(e_pred - e_star[test])))

    # метки = СОГЛАСОВАННАЯ функция φ -> kNN на удержанном облёте бьёт геом-истину
    assert s_mae < 0.05, s_mae
    assert e_mae < 5.0, e_mae
    assert ds.feats[train].shape[0] > 0 and test.sum() > 0
    # экспорт под fit_route_heads (torch-машина)
    import tempfile, os
    tmp = os.path.join(tempfile.gettempdir(), "c3_route_set.npz")
    save_route_dataset(tmp, ds.feats, s_star, e_star, ds.flight_id, cl, model="dinov2_vits14")
    chk = np.load(tmp)
    assert chk["feats"].shape[0] == len(ds) and chk["bounds"].shape[1] == 2

    print("c3_route_pipeline selftest: OK")
    print(f"  глоб. набор: {len(ds)} кадров, {len(set(ds.flight_id.tolist()))} облётов; {qc}")
    print(f"  полилиния: L={cl.L:.1f} м; s*∈[{s_star.min():.2f},{s_star.max():.2f}], "
          f"|e*| до {np.abs(e_star).max():.0f} м")
    print(f"  ПОКРЫТИЕ (|e|<10м): {cov_frac*100:.0f}% длины ({gaps}/40 бинов пусты — конверт налётанного, XXVI)")
    print(f"  УЧЕБНОСТЬ (held-out облёт 4, kNN-стенд-ин): s MAE {s_mae:.3f}, e MAE {e_mae:.2f} м")
    print(f"  набор -> {tmp} (feats+s*+e*+bounds+центрлиния) под fit_route_heads (torch)")
    print("  -> геометрия разметила метки (офлайн), они УЧЕБНЫ как функция φ; рантайм = чистый C")


if __name__ == "__main__":
    _selftest()
