#!/usr/bin/env python3
# ============================================================================
# build_global_dataset.py — ШАГ 1 (c)-основного: ОРКЕСТРАТОР стадий 4->6.
#
# Зачем (XXI/XXVI, c3_pipeline_howto «главный пробел»): куски стадий есть, но нет
# того, кто проводит N облётов СКВОЗЬ регистрацию в ОДИН согласованный обучающий
# набор. Этот файл — связка: N×(феатуры+ЛОКАЛЬНЫЕ позы VINS) + связи (якоря NN1 /
# кросс-облётные) -> на каждый облёт жёсткое T_i (Umeyama) -> ВСЕ позы в ОДНУ
# глобальную рамку -> стек (φ, ГЛОБАЛЬНАЯ xy, flight_id, frame_idx) = обучающий
# набор стадии 6 + отчёт QC (стадия 8). На нём дальше build_route_targets рисует
# полилинию и проецирует -> s*,e* (ШАГ 3, XXVI).
#
# Ядро регистрации переиспользуем из nn2_scene/register_flights.py (стадия 4).
# Этот шаг — на СИНТЕТИКЕ (реальные npz из train_route_coords.extract_bags позже,
# на машине с torch). Чистый numpy — тестируется здесь.
#
# ── ВХОД (на облёт) ──────────────────────────────────────────────────────────
#   feats     (Ni,D) f32  — дескрипторы φ кадров (DINOv2/топограф);
#   local_xy  (Ni,2) f64  — позы VINS В ЛОКАЛЬНОЙ рамке облёта (свой origin+курс);
#   corr_idx  (Mi,)  i64   — индексы кадров-СВЯЗЕЙ (видели якорь / кросс-совпадение);
#   corr_xy   (Mi,2) f64   — ГЛОБАЛЬНЫЕ позиции этих связей (георефернс якорей /
#                            позиция в уже-сведённой рамке).
# Якорей в npz сейчас нет -> связи передаём отдельно (стадия 3a). Кросс-облётные
# связи (стадия 3b) добавит ШАГ 2 (cross_flight_correspond) тем же интерфейсом.
#
# ── ВЫХОД ────────────────────────────────────────────────────────────────────
#   GlobalDataset(feats (M,D), xy (M,2) ГЛОБ., flight_id (M,), frame_idx (M,))
#   + transforms[i]=(R,t,c), + QCReport (пер-облётная и кросс-облётная невязка,
#     список облётов на дроп/переснять).
# ============================================================================
import sys
import pathlib

import numpy as np

# ядро регистрации (стадия 4) — из соседнего nn2_scene/
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "nn2_scene"))
from register_flights import register_to_global, apply_se2, cross_flight_residual  # noqa: E402


class Flight:
    """Один облёт: φ кадров + ЛОКАЛЬНЫЕ позы VINS + связи (индексы кадров и их
    ГЛОБАЛЬНЫЕ позиции). place_ids (опц.) — id мест на кадр, для кросс-облётного QC
    (в проде получают из кросс-связей; в selftest — известная истина)."""
    def __init__(self, feats, local_xy, corr_idx, corr_xy, flight_id, place_ids=None):
        self.feats = np.asarray(feats, np.float32)
        self.local_xy = np.asarray(local_xy, np.float64)
        self.corr_idx = np.asarray(corr_idx, np.int64)
        self.corr_xy = np.asarray(corr_xy, np.float64)
        self.flight_id = int(flight_id)
        self.place_ids = None if place_ids is None else np.asarray(place_ids, np.int64)
        n = len(self.local_xy)
        assert len(self.feats) == n, "feats и local_xy разной длины"

    @classmethod
    def from_npz(cls, path, corr_idx, corr_xy, flight_id, place_ids=None):
        """Стадия 2-выход (train_route_coords.save_feats_npz) + связи отдельно.
        poses (N,3) -> берём xy. Если облётов в npz несколько (bounds) — тут
        считаем npz = ОДИН облёт (мульти-bounds расплющиваем)."""
        d = np.load(str(path), allow_pickle=False)
        feats = d["feats"].astype(np.float32)
        local_xy = d["poses"].astype(np.float64)[:, :2]
        return cls(feats, local_xy, corr_idx, corr_xy, flight_id, place_ids)


class QCReport:
    """Стадия 8. per_flight_res[i] — макс. невязка переноса связей облёта i (как
    жёсткая регистрация села на свои же связи). cross_res — невязка копий ОДНОГО
    места из разных облётов (нужны place_ids). dropped — облёты выше порога."""
    def __init__(self, per_flight_res, cross_mean, cross_max, dropped, n_flights):
        self.per_flight_res = per_flight_res
        self.cross_mean = cross_mean
        self.cross_max = cross_max
        self.dropped = dropped
        self.n_flights = n_flights

    def __str__(self):
        worst = max(self.per_flight_res.values()) if self.per_flight_res else 0.0
        return (f"QC: облётов {self.n_flights}, дропнуто {len(self.dropped)} "
                f"{self.dropped if self.dropped else ''}; "
                f"пер-облётная невязка макс {worst:.2f} м; "
                f"кросс-облётная среднее {self.cross_mean:.2f} м / макс {self.cross_max:.2f} м")


class GlobalDataset:
    """Стек стадии 6: все облёты в ОДНОЙ глобальной рамке."""
    def __init__(self, feats, xy, flight_id, frame_idx, place_ids=None):
        self.feats = np.asarray(feats, np.float32)
        self.xy = np.asarray(xy, np.float64)
        self.flight_id = np.asarray(flight_id, np.int64)
        self.frame_idx = np.asarray(frame_idx, np.int64)
        self.place_ids = None if place_ids is None else np.asarray(place_ids, np.int64)

    def __len__(self):
        return len(self.xy)

    def save(self, path):
        """-> npz (feats, xy ГЛОБ., flight_id, frame_idx[, place_ids]) для ШАГ 3."""
        kw = dict(feats=self.feats, xy=self.xy,
                  flight_id=self.flight_id, frame_idx=self.frame_idx)
        if self.place_ids is not None:
            kw["place_ids"] = self.place_ids
        np.savez(str(path), **kw)


def _register_one(flight, with_scale):
    """Стадия 4 на один облёт: связи (local<->global) -> T -> ВСЕ позы в мир.
    Возвращает (global_xy, T=(R,t,c), per_flight_residual). residual = макс.
    ошибка переноса СОБСТВЕННЫХ связей (sanity: правда ли жёсткое T их совмещает;
    большая = плохие/коллинеарные связи или нежёсткая деформация = битый VINS)."""
    local_corr = flight.local_xy[flight.corr_idx]
    R, t, c = register_to_global(local_corr, flight.corr_xy, with_scale=with_scale)
    global_xy = apply_se2(R, t, flight.local_xy, c)
    proj_corr = apply_se2(R, t, local_corr, c)
    res = float(np.linalg.norm(proj_corr - flight.corr_xy, axis=1).max()) if len(local_corr) else 0.0
    return global_xy, (R, t, c), res


def build_global_dataset(flights, with_scale=False, drop_residual=5.0, qc_place_ids=None):
    """ОРКЕСТРАТОР (стадии 4->6+8). flights — list[Flight] с уже заполненными
    связями (corr_idx/corr_xy). drop_residual — порог пер-облётной невязки (м):
    выше -> облёт помечается dropped и НЕ идёт в набор (битый VINS / мало связей).
    qc_place_ids — какие места проверять кросс-облётно (нужны Flight.place_ids).
    -> (GlobalDataset, transforms{fid:(R,t,c)}, QCReport)."""
    feats_all, xy_all, fid_all, fidx_all, pid_all = [], [], [], [], []
    transforms, per_res, dropped = {}, {}, []
    registered_for_qc = []
    have_pid = all(f.place_ids is not None for f in flights)

    for fl in flights:
        if len(fl.corr_idx) < 2:
            dropped.append(fl.flight_id)                      # нечем регистрировать
            per_res[fl.flight_id] = float("inf")
            continue
        gxy, T, res = _register_one(fl, with_scale)
        transforms[fl.flight_id] = T
        per_res[fl.flight_id] = res
        if res > drop_residual:
            dropped.append(fl.flight_id)                      # не сошёлся -> вон
            continue
        feats_all.append(fl.feats)
        xy_all.append(gxy)
        fid_all.append(np.full(len(gxy), fl.flight_id, np.int64))
        fidx_all.append(np.arange(len(gxy), dtype=np.int64))
        if have_pid:
            pid_all.append(fl.place_ids)
            registered_for_qc.append((gxy, fl.place_ids))

    if feats_all:
        ds = GlobalDataset(np.vstack(feats_all), np.vstack(xy_all),
                           np.concatenate(fid_all), np.concatenate(fidx_all),
                           np.concatenate(pid_all) if have_pid else None)
    else:
        ds = GlobalDataset(np.zeros((0, 1), np.float32), np.zeros((0, 2)),
                           np.zeros(0, np.int64), np.zeros(0, np.int64))

    if have_pid and registered_for_qc:
        ids = qc_place_ids if qc_place_ids is not None else \
            np.unique(np.concatenate([p for _, p in registered_for_qc]))
        cmean, cmax = cross_flight_residual(registered_for_qc, ids)
    else:
        cmean, cmax = 0.0, 0.0
    qc = QCReport(per_res, cmean, cmax, dropped, len(flights))
    return ds, transforms, qc


# --- самопроверка: N синтетич. облётов -> ОДИН согласованный набор (numpy) -----
def _rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _selftest():
    rng = np.random.default_rng(0)
    n_places, D = 70, 16
    G = rng.uniform([0, 0], [200, 120], (n_places, 2))        # глоб. истина мест
    place_desc = rng.normal(0, 1, (n_places, D)).astype(np.float32)   # латентный φ места
    anchors = rng.choice(n_places, 6, replace=False)          # 6 георефернс-якорей NN1

    def make_flight(seed, fid, m=24, broken=False):
        r = np.random.default_rng(seed)
        idx = np.unique(np.concatenate([r.choice(n_places, m, replace=False), anchors[:3]]))
        Rf, tf = _rot(r.uniform(-np.pi, np.pi)), r.uniform([-40, -40], [40, 40])
        local = (G[idx] - tf) @ Rf + r.normal(0, 0.4, (len(idx), 2))   # VINS лок. рамка
        if broken:
            local *= 1.0 + r.normal(0, 0.25, local.shape)     # нежёсткая деформация = битый VINS
        # φ кадра = дескриптор места + помеха облёта (свет/ракурс)
        feats = place_desc[idx] + r.normal(0, 0.05, (len(idx), D)).astype(np.float32)
        seen = np.isin(idx, anchors)                          # кадры, видевшие якоря
        corr_idx = np.nonzero(seen)[0]
        corr_xy = G[idx[seen]]                                # АБС. георефернс якорей
        return Flight(feats, local, corr_idx, corr_xy, fid, place_ids=idx)

    flights = [make_flight(s, fid=s) for s in (1, 2, 3, 4)]
    flights.append(make_flight(99, fid=99, broken=True))      # 5-й — битый VINS

    ds, transforms, qc = build_global_dataset(flights, drop_residual=5.0)

    # битый облёт отсеян; здоровые сведены и кросс-невязка мала
    assert 99 in qc.dropped, qc.dropped
    assert all(f in transforms for f in (1, 2, 3, 4))
    assert qc.cross_max < 4.0, qc.cross_max
    # метки набора = ГЛОБАЛЬНАЯ истина (по месту): сверим xy с G[place_id]
    err = np.linalg.norm(ds.xy - G[ds.place_ids], axis=1)
    assert err.mean() < 1.5, err.mean()
    # формы согласованы
    assert ds.feats.shape == (len(ds), D) and ds.xy.shape == (len(ds), 2)
    assert set(np.unique(ds.flight_id)) == {1, 2, 3, 4}

    print("build_global_dataset selftest: OK")
    print(f"  {qc}")
    print(f"  набор: {len(ds)} кадров из {len(set(ds.flight_id.tolist()))} облётов, φ-dim {ds.feats.shape[1]}")
    print(f"  метки vs глоб. истина: среднее {err.mean():.2f} м, макс {err.max():.2f} м")
    print(f"  пер-облётная невязка: " +
          ", ".join(f"f{k}={v:.2f}м" if np.isfinite(v) else f"f{k}=inf" for k, v in qc.per_flight_res.items()))
    print("  -> 4 здоровых облёта в ОДНОЙ рамке, битый дропнут; набор готов под build_route_targets (ШАГ 3)")


if __name__ == "__main__":
    _selftest()
