# ============================================================================
# metric_decode.py — kNN-декод метрической позиции СО СТРАЖЕМ алиасинга (XVIII).
#
# Общий «мозг» метрической засечки для ДВУХ потребителей (чтобы не разъезжались):
#   - боевой SceneMatcher.metric_fix (FAISS-карта, nav_pkg) — на проводе;
#   - офлайн route_fusion.MetricMap.decode (in-memory φ, tools) — для прогона/анализа.
#
# Почему kNN, а не top-1 (см. разбор XVIII): top-1 возвращает одно реальное место
# (квантованно, ковариация фейковая); kNN даёт суб-сэмпловую позицию + ЧЕСТНУЮ
# ковариацию из разброса соседей. Но у kNN свой провал — АЛИАСИНГ: если k соседей
# из двух похожих, но разных мест, среднее ляжет МЕЖДУ ними (фантом). Поэтому СТРАЖ:
# меряем пространственный разброс соседей; компактны -> доверяем kNN; разбежались ->
# фолбэк на top-1 (одно реальное место) с широкой ковариацией и сбитой уверенностью.
# Разброс соседей заодно — бесплатный детектор алиасинга (стыкуется с фильтром по s).
#
# Чистый numpy — без torch/faiss/ROS, тестируется где угодно.
# ============================================================================
import math

import numpy as np


def knn_decode_guard(P, dists, metric, k=6, guard_radius=8.0, sigma_floor=1.0,
                     conf_scale=5.0, min_score=0.5, guard_penalty=0.3):
    """k соседей (позиции P (k,2), расстояния dists (k,), nearest-first) -> засечка.

    metric: "l2" — dists в МЕТРАХ (топограф), меньше=лучше; "ip" — КОСИНУС, больше=
    лучше. Возвращает dict:
      x,y    — позиция (kNN-среднее; при срабатывании стража — top-1);
      cov    — 2×2 ковариация (м²): анизотропная из разброса соседей (+ пол), либо
               широкая изотропная при стражe;
      std    — скалярный σ-эквивалент (√(tr(cov)/2)), м — для обратной совместимости;
      conf   — уверенность (0..1]; гасится при стражe;
      source — "knn" | "top1-fallback";
      spread — RMS-разброс соседей, м (= сигнал алиасинга).
    """
    P = np.asarray(P, np.float64)
    dists = np.asarray(dists, np.float64)
    n = len(P)

    # веса по близости + базовая уверенность из ЛУЧШЕГО соседа
    if metric == "l2":
        d = np.maximum(dists, 0.0)                          # метры
        w = np.exp(-(d / (d.mean() + 1e-9)) ** 2)
        base_conf = math.exp(-float(d.min()) / conf_scale) if conf_scale > 0 else 0.0
    else:                                                   # ip (косинус)
        cos = dists
        w = np.clip((cos - min_score) / (1.0 - min_score + 1e-9), 0.0, None)
        best = float(cos.max())
        base_conf = max(0.0, min(1.0, (best - min_score) / (1.0 - min_score + 1e-9)))
    if w.sum() <= 1e-9:
        w = np.ones(n)
    w = w / w.sum()

    # kNN-позиция + анизотропная ковариация из разброса соседей
    p = (w[:, None] * P).sum(0)
    diff = P - p
    cov = (w[:, None, None] * np.einsum("ni,nj->nij", diff, diff)).sum(0)
    spread = float(np.sqrt(max(np.trace(cov), 0.0)))        # RMS-разброс соседей, м

    if spread <= guard_radius:                              # СОСЕДИ КОМПАКТНЫ -> kNN
        x, y = float(p[0]), float(p[1])
        C = cov + sigma_floor ** 2 * np.eye(2)
        conf = base_conf
        source = "knn"
    else:                                                   # АЛИАСИНГ -> фолбэк top-1
        x, y = float(P[0, 0]), float(P[0, 1])               # одно РЕАЛЬНОЕ место
        big = guard_radius + spread
        C = big ** 2 * np.eye(2)                            # широко, изотропно (форме не верим)
        conf = base_conf * guard_penalty
        source = "top1-fallback"

    std = float(np.sqrt(max(np.trace(C) / 2.0, 0.0)))
    return {"x": x, "y": y, "cov": C, "std": std, "conf": float(conf),
            "source": source, "spread": spread}


def _selftest():
    # 1) КОМПАКТНЫЕ соседи около (50,7) -> kNN, узкая cov, source=knn
    P = np.array([[50, 7], [51, 7], [49, 6.5], [50, 8], [48, 7], [52, 7.5]], float)
    d = np.array([0.2, 1.0, 1.2, 1.1, 2.0, 2.1])
    r = knn_decode_guard(P, d, "l2", guard_radius=8.0)
    assert r["source"] == "knn", r["source"]
    assert abs(r["x"] - 50) < 2 and abs(r["y"] - 7) < 2, r
    assert r["spread"] < 8 and r["std"] < 5, r

    # 2) РАЗБЕЖАВШИЕСЯ соседи (два кластера: (10,0) и (90,0)) -> алиасинг -> фолбэк top-1
    P2 = np.array([[10, 0], [11, 0], [9, 0], [90, 0], [91, 0], [89, 0]], float)
    d2 = np.array([0.3, 0.4, 0.5, 0.6, 0.7, 0.8])
    r2 = knn_decode_guard(P2, d2, "l2", guard_radius=8.0, guard_penalty=0.3)
    assert r2["source"] == "top1-fallback", r2["source"]
    assert (r2["x"], r2["y"]) == (10.0, 0.0), r2          # одно реальное место (top-1)
    assert r2["spread"] > 8 and r2["std"] > 8, r2          # широкая cov
    assert r2["conf"] < knn_decode_guard(P, d, "l2")["conf"], "страж не сбил conf"

    # 3) IP-карта (косинус): conf по запасу над порогом
    r3 = knn_decode_guard(P, np.array([0.9, 0.8, 0.75, 0.7, 0.65, 0.6]), "ip",
                          min_score=0.5)
    assert r3["source"] == "knn" and 0.0 < r3["conf"] <= 1.0, r3
    print("metric_decode selftest: OK "
          f"(knn x={r['x']:.1f},y={r['y']:.1f} std={r['std']:.1f}; "
          f"страж spread={r2['spread']:.0f}м -> top1 conf={r2['conf']:.2f})")


if __name__ == "__main__":
    _selftest()
