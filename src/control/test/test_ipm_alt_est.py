#!/usr/bin/env python3
"""Юнит-тест ПОПРАВКИ ВЫСОТЫ ПО ЗУМУ ЗЕМЛИ (perception/alt_est.py, BS_IPM_ALT_EST).

1. Сам оценщик на точных ε (ε = ln((h1+δ)/(h0+δ)) − ln(h1/h0)): сходится к δ, на висении
   (h не меняется) не трогается, выбросы отбрасывает.
2. В канале на честном рендере (test_ipm_lever.render): камера на истинной высоте, каналу
   дана высота на δ меньше/больше, борт набирает — оценка ≈ δ (синтетика: ±0.03 м).
   Геометрию канала оценка НЕ меняет (только наблюдение).

Запуск:  python3 src/control/test/test_ipm_alt_est.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.argv = sys.argv[:1]
_src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_ipm_lever.py'),
            encoding='utf-8').read().split("print('A.")[0]
exec(_src)          # FX..DT, render, make, FlowEstimator, np, check, results

from control_pkg.perception.alt_est import GroundOffsetEstimator   # noqa: E402

# 1. оценщик на точных данных
for d_true in (0.2, -0.15, 0.0):
    g = GroundOffsetEstimator()
    for k in range(60):
        h0, h1 = 0.6 + 0.015 * k, 0.6 + 0.015 * (k + 1)
        g.update(k * DT, math.log((h1 + d_true) / (h0 + d_true)) - math.log(h1 / h0), h0, h1)
    check(f'точные ε, δ {d_true:+.2f}: оценка {g.delta:+.4f}', abs(g.delta - d_true) < 1e-3)
g = GroundOffsetEstimator()
for k in range(60):
    h0, h1 = 0.6 + 0.015 * k, 0.6 + 0.015 * (k + 1)
    g.update(k * DT, math.log((h1 + 0.2) / (h0 + 0.2)) - math.log(h1 / h0), h0, h1)
d0 = g.delta
for k in range(60):                                   # висение: ε ≈ 0, h не меняется
    g.update(2.0 + k * DT, 0.0, 1.5, 1.5)
check(f'висение не сдвигает оценку ({d0:+.4f} → {g.delta:+.4f})', abs(g.delta - d0) < 1e-6)
g.update(5.0, 5.0, 1.5, 1.52)                         # выброс |ε| > eps_max
check('выброс отброшен', abs(g.delta - d0) < 1e-6)
g = GroundOffsetEstimator()
for k in range(120):                                  # только висение с шумом — оценки нет
    g.update(k * DT, 0.002 * ((-1) ** k), 1.5, 1.5 + 1e-4 * ((-1) ** k))
check(f'одно висение: оценки нет (не уходит в бесконечность) — {g.delta}', g.delta is None)
g = GroundOffsetEstimator()
for k in range(60):                                   # у земли (< h_min) кадры не берутся
    g.update(k * DT, 0.05, 0.2 + 0.005 * k, 0.2 + 0.005 * (k + 1))
check('ниже h_min кадры не берутся', g.n == 0 and g.delta is None)


# 2. в канале на рендере
def run(delta, h0, h1, n=60):
    e = FlowEstimator(FX, FY, CX, CY, np.eye(3), cam_tilt=0.0, ipm_model='exact', ipm_derot=1.0,
                      ipm_scale_ref=3.0, ipm_adapt=1.05, ipm_scale_exact=True, ipm_alt_est=True)
    r = make((0.0, 0.0, 0.0))
    for k in range(n + 1):
        h = h0 + (h1 - h0) * k / n                      # истинная высота камеры
        e._ipm_update(render(r, 0.0, 0.0, 0.0, h, t=(0.0, 0.0, 0.0)), k * DT, h - delta,
                      0.0, 0.0, 0.0)
    return e


for d_true in (0.2, -0.15):
    e = run(d_true, 0.6, 1.5)
    print(f'  рендер: δ {d_true:+.2f}, набор 0.6→1.5 м: оценка {e.ipm_alt_delta:+.3f} ± {e.ipm_alt_sigma:.3f}')
    check(f'рендер δ {d_true:+.2f}: |ошибка| < 0.03 м', abs(e.ipm_alt_delta - d_true) < 0.03)
    check(f'рендер δ {d_true:+.2f}: геометрия на принятой высоте (не на оценке)',
          abs(e.ipm_geom_h - (1.5 - d_true)) < 1e-9)

bad = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(bad)}/{len(results)} OK')
sys.exit(1 if bad else 0)
