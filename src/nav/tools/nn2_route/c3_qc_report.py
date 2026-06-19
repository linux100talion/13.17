#!/usr/bin/env python3
# ============================================================================
# c3_qc_report.py — ШАГ 4 (c)-основного: единый QC-отчёт по офлайн-пайплайну (стадия 8).
#
# Зачем (XXI: «бутылочное горло — РЕГИСТРАЦИЯ», XXVI: «покрытие»): стройка (ШАГ 1-3)
# раскидала проверки по модулям (QCReport регистрации, coverage маршрута). Перед
# обучением голов C нужен ОДИН отчёт-светофор: можно ли учить на этих данных или
# часть облётов/участков карты гнилые. Сводит:
#   1. ЗДОРОВЬЕ РЕГИСТРАЦИИ — пер-облётная невязка, дропнутые/недостижимые облёты
#      (битый VINS / мало связей) -> кого ПЕРЕСНЯТЬ;
#   2. ПОКРЫТИЕ КАРТЫ — сетка по площади: где облётов нет, там NN2 не локализует
#      (карта дырявая) -> куда ДОЛЕТЕТЬ;
#   3. ПОКРЫТИЕ МАРШРУТА (XXVI) — вдоль полилинии: участки s без близких кадров
#      (|e| велик) -> метки слабые, рулёжка слепа -> где ДОСНЯТЬ вдоль маршрута;
#   4. РАЗНООБРАЗИЕ (XXI) — сколько РАЗНЫХ облётов видело каждый участок: бин,
#      виденный 1 облётом, НЕ инвариантен к свету/сезону -> доснять в др. условиях.
# Выдаёт вердикт (списки на переснять/долетать/доснять). Чистый numpy — тестируется.
# ============================================================================
import numpy as np


def _gap_segments(per_bin, nbins):
    """Непрерывные пробелы (рантайны False) -> список (s_start, s_end) в долях s."""
    segs, b = [], 0
    while b < nbins:
        if per_bin[b]:
            b += 1; continue
        a = b
        while b < nbins and not per_bin[b]:
            b += 1
        segs.append((a / nbins, b / nbins))
    return segs


def map_coverage(global_xy, cell=10.0, bbox=None):
    """ПОКРЫТИЕ КАРТЫ: сетка cell×cell м по площади облётов. -> (filled_frac,
    n_filled, n_total, empty_fraction). Дырки = где NN2 не локализует."""
    p = np.asarray(global_xy, float)[:, :2]
    if len(p) == 0:
        return 0.0, 0, 0, 1.0
    lo = p.min(0) if bbox is None else np.array(bbox[:2], float)
    hi = p.max(0) if bbox is None else np.array(bbox[2:], float)
    span = np.maximum(hi - lo, 1e-6)
    nx, ny = int(np.ceil(span[0] / cell)), int(np.ceil(span[1] / cell))
    nx, ny = max(nx, 1), max(ny, 1)
    ix = np.clip(((p[:, 0] - lo[0]) / cell).astype(int), 0, nx - 1)
    iy = np.clip(((p[:, 1] - lo[1]) / cell).astype(int), 0, ny - 1)
    occ = np.zeros((nx, ny), bool)
    occ[ix, iy] = True
    total = nx * ny
    filled = int(occ.sum())
    return filled / total, filled, total, 1.0 - filled / total


def route_coverage(s_star, e_star, flight_id, e_tol=8.0, nbins=50, min_flights=2):
    """ПОКРЫТИЕ + РАЗНООБРАЗИЕ вдоль полилинии. Бин по s «покрыт», если есть кадр с
    |e|<e_tol; «разнообразен», если таких кадров от >=min_flights РАЗНЫХ облётов.
    -> dict: covered_frac, gap_segments, diverse_frac, thin_segments (покрыты, но
    < min_flights облётов -> не инвариантны к условиям, XXI)."""
    s = np.asarray(s_star); e = np.asarray(e_star); fid = np.asarray(flight_id)
    near = np.abs(e) < e_tol
    edges = np.linspace(0.0, 1.0, nbins + 1)
    idx = np.clip(np.digitize(s, edges) - 1, 0, nbins - 1)
    covered = np.zeros(nbins, bool)
    n_flights_bin = np.zeros(nbins, int)
    for b in range(nbins):
        sel = (idx == b) & near
        covered[b] = bool(np.any(sel))
        n_flights_bin[b] = int(len(np.unique(fid[sel]))) if np.any(sel) else 0
    diverse = n_flights_bin >= min_flights
    thin = covered & ~diverse                                # покрыт, но мало облётов
    return {
        "covered_frac": float(covered.mean()),
        "gap_segments": _gap_segments(covered, nbins),       # вовсе без близких кадров
        "diverse_frac": float(diverse.mean()),
        "thin_segments": _gap_segments(~thin, nbins),        # покрыт, но < min_flights облётов
        "thin_bins": int(thin.sum()),
        "n_flights_bin": n_flights_bin,
        "min_flights": min_flights,
    }


class QCSummary:
    """Светофор + вердикт по всему пайплайну."""
    def __init__(self, reg, mapc, routec, drop_residual):
        self.reg = reg                    # QCReport из build_global_dataset
        self.mapc = mapc                  # map_coverage(...)
        self.routec = routec              # route_coverage(...)
        self.drop_residual = drop_residual

    def reshoot(self):
        """Облёты ПЕРЕСНЯТЬ: дропнутые (битый VINS / мало связей / недостижимы)."""
        return sorted(self.reg.dropped)

    def verdict(self):
        """Грубый светофор: GREEN/YELLOW/RED по покрытию+разнообразию+дропам."""
        filled = self.mapc[0]; cov = self.routec["covered_frac"]; div = self.routec["diverse_frac"]
        if self.reshoot() or cov < 0.5 or filled < 0.4:
            return "RED"
        if div < 0.6 or cov < 0.8:
            return "YELLOW"
        return "GREEN"

    def __str__(self):
        rc = self.routec
        worst = max((v for v in self.reg.per_flight_res.values() if np.isfinite(v)), default=0.0)
        lines = [
            f"==== QC (c)-основной: {self.verdict()} ====",
            f"РЕГИСТРАЦИЯ: облётов {self.reg.n_flights}, дропнуто {len(self.reg.dropped)} "
            f"{self.reshoot() if self.reshoot() else ''}; "
            f"пер-облётная невязка макс {worst:.2f} м (порог {self.drop_residual}); "
            f"кросс-облётная {self.reg.cross_mean:.2f}/{self.reg.cross_max:.2f} м",
            f"КАРТА: занято {self.mapc[0]*100:.0f}% ячеек ({self.mapc[1]}/{self.mapc[2]}), "
            f"дыр {self.mapc[3]*100:.0f}%",
            f"МАРШРУТ: покрыто {rc['covered_frac']*100:.0f}% длины; разнообразно "
            f"(>={rc['min_flights']} облётов) {rc['diverse_frac']*100:.0f}%; "
            f"тонких бинов {rc['thin_bins']}",
        ]
        if rc["gap_segments"]:
            g = ", ".join(f"[{a:.2f}-{b:.2f}]" for a, b in rc["gap_segments"])
            lines.append(f"  ДОСНЯТЬ вдоль маршрута (нет близких кадров), s: {g}")
        if self.reshoot():
            lines.append(f"  ПЕРЕСНЯТЬ облёты: {self.reshoot()}")
        return "\n".join(lines)


def build_qc_report(ds, reg_qc, s_star, e_star, drop_residual=5.0,
                    cell=10.0, e_tol=8.0, nbins=50, min_flights=2):
    """Свести всё в QCSummary. ds — GlobalDataset; reg_qc — QCReport регистрации;
    s_star,e_star — метки из route_targets_on_polyline."""
    mapc = map_coverage(ds.xy, cell=cell)
    routec = route_coverage(s_star, e_star, ds.flight_id, e_tol=e_tol,
                            nbins=nbins, min_flights=min_flights)
    return QCSummary(reg_qc, mapc, routec, drop_residual)


# --- самопроверка (numpy): полный пайплайн -> отчёт ловит проблемы -------------
def _rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _selftest():
    from build_global_dataset import Flight, build_global_dataset
    from c3_route_pipeline import draw_polyline, route_targets_on_polyline

    rng = np.random.default_rng(3)
    n_places, D = 60, 16
    G = rng.uniform([0, 0], [200, 120], (n_places, 2))
    place_desc = rng.normal(0, 1, (n_places, D)).astype(np.float32)
    anchors = rng.choice(n_places, 6, replace=False)

    def make_flight(seed, fid, broken=False):
        r = np.random.default_rng(seed)
        idx = np.unique(np.concatenate([r.choice(n_places, 40, replace=False), anchors[:4]]))
        Rf, tf = _rot(r.uniform(-np.pi, np.pi)), r.uniform([-40, -40], [40, 40])
        local = (G[idx] - tf) @ Rf + r.normal(0, 0.3, (len(idx), 2))
        if broken:
            local *= 1.0 + r.normal(0, 0.25, local.shape)
        feats = place_desc[idx] + r.normal(0, 0.05, (len(idx), D)).astype(np.float32)
        seen = np.isin(idx, anchors)
        return Flight(feats, local, np.nonzero(seen)[0], G[idx[seen]], fid, place_ids=idx)

    flights = [make_flight(s, fid=s) for s in (1, 2, 3)]
    flights.append(make_flight(99, fid=99, broken=True))     # битый -> дроп
    ds, _, reg_qc = build_global_dataset(flights, drop_residual=5.0)

    route = np.array([[10.0, 20.0], [70.0, 50.0], [130.0, 70.0], [190.0, 100.0]])
    cl = draw_polyline(route, smooth_window=1, resample_ds=3.0)
    _, s_star, e_star = route_targets_on_polyline(ds.xy, cl)

    rep = build_qc_report(ds, reg_qc, s_star, e_star, drop_residual=5.0,
                          cell=15.0, e_tol=12.0, nbins=40, min_flights=2)

    assert 99 in rep.reshoot(), rep.reshoot()                 # битый -> переснять
    assert rep.verdict() in ("RED", "YELLOW", "GREEN")
    assert rep.verdict() == "RED"                             # дроп есть -> RED
    assert 0.0 <= rep.routec["covered_frac"] <= 1.0
    assert rep.mapc[2] > 0                                    # сетка карты непуста

    print("c3_qc_report selftest: OK")
    print(str(rep))
    print(f"  (диверсити-бинов по числу облётов: "
          f"max {rep.routec['n_flights_bin'].max()}, нулевых {(rep.routec['n_flights_bin']==0).sum()}/40)")
    print("  -> единый светофор: RED из-за дропнутого облёта; видны дыры карты и пробелы маршрута")


if __name__ == "__main__":
    _selftest()
