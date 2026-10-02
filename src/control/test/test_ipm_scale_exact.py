#!/usr/bin/env python3
"""Юнит-тест ФАНТОМА НАБОРА канала вида сверху (`ipm_scale_exact`, BS_IPM_SCALE_EXACT).

Полоса ∝ высоте (ipm_scale_ref) и адаптивное окно меняют геометрию сетки между кадрами.
Прежний учёт вычитал лишь сдвиг начала окна, и неподвижная земля при наборе читалась как
ход вперёд: (X − x0)·Δh/h. Здесь борт СТОИТ, высота известна точно, меняется только она —
ложного пути быть не должно. Сцена — честный рендер (test_ipm_lever.render).

Запуск:  python3 src/control/test/test_ipm_scale_exact.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.argv = sys.argv[:1]
_src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_ipm_lever.py'),
            encoding='utf-8').read().split("print('A.")[0]
exec(_src)          # FX..DT, render, make, FlowEstimator, np, check, results

STATE = dict(ipm_scale_ref=3.0, ipm_adapt=1.05)       # штатная геометрия полосы


def run(exact, frames):
    e = FlowEstimator(FX, FY, CX, CY, np.eye(3), cam_tilt=0.0, ipm_model='exact', ipm_derot=1.0,
                      ipm_scale_exact=exact, **STATE)
    r = make((0.0, 0.0, 0.0))
    for i, (xb, psi, h, wz) in enumerate(frames):
        e._ipm_update(render(r, xb, 0.0, psi, h, t=(0.0, 0.0, 0.0)), i * DT, h, 0.0, 0.0, wz)
    return e


for h0, h1 in ((1.0, 1.5), (2.0, 2.5), (1.5, 1.0)):
    fr = [(0.0, 0.0, h0 + (h1 - h0) * i / 30, 0.0) for i in range(31)]
    old, new = run(False, fr).ipm_fwd, run(True, fr).ipm_fwd
    print(f'  набор {h0}→{h1} м, борт стоит: ложный путь прежний {old:+.3f} м, exact {new:+.3f} м')
    if h1 > h0:
        check(f'{h0}→{h1}: прежний учёт даёт фантом > 0.15 м', old > 0.15)
    check(f'{h0}→{h1}: exact — |ложный путь| < 0.03 м', abs(new) < 0.03)

fwd = [(0.5 * i / 30, 0.0, 1.0, 0.0) for i in range(31)]
spin = [(0.0, 0.5 * i * DT, 1.0, 0.5) for i in range(25)]
a, b = run(False, fwd), run(True, fwd)
check(f'неизменная высота, ход 0.5 м: exact = прежний ({b.ipm_fwd:.4f} / {a.ipm_fwd:.4f})',
      abs(a.ipm_fwd - b.ipm_fwd) < 1e-6 and abs(a.ipm_lat - b.ipm_lat) < 1e-6)
a, b = run(False, spin), run(True, spin)
check(f'неизменная высота, разворот: exact = прежний (вбок {b.ipm_lat:+.4f} / {a.ipm_lat:+.4f})',
      abs(a.ipm_fwd - b.ipm_fwd) < 1e-6 and abs(a.ipm_lat - b.ipm_lat) < 1e-6)

bad = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(bad)}/{len(results)} OK')
sys.exit(1 if bad else 0)
