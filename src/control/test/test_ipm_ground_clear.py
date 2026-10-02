#!/usr/bin/env python3
"""Юнит-тест КЛИРЕНСА КОРПУСА в геометрии канала вида сверху (`ipm_ground_clear`).

Высота перцепции отсчитывается от позы «стою на земле», геометрии нужна высота камеры над
землёй. Сцена — честный рендер камеры на истинной высоте над землёй; оценщику даётся высота
«от позы стоя» = истинная − клиренс. Без клиренса в геометрии постоянная ошибка высоты при
наборе даёт фантом хода вперёд; с клиренсом — нет. Плюс: _ipm_geom_h добавляет клиренс,
а гейт «на земле» смотрит сырую высоту.

Запуск:  python3 src/control/test/test_ipm_ground_clear.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.argv = sys.argv[:1]
_src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_ipm_lever.py'),
            encoding='utf-8').read().split("print('A.")[0]
exec(_src)          # FX..DT, render, make, FlowEstimator, np, check, results

CLEAR = 0.195
GEOM = dict(ipm_scale_ref=3.0, ipm_adapt=1.05, ipm_scale_exact=True)


def climb(clear, h0, h1, n=30):
    e = FlowEstimator(FX, FY, CX, CY, np.eye(3), cam_tilt=0.0, ipm_model='exact', ipm_derot=1.0,
                      ipm_ground_clear=clear, **GEOM)
    r = make((0.0, 0.0, 0.0))
    for i in range(n + 1):
        h = h0 + (h1 - h0) * i / n                     # истинная высота камеры над землёй
        e._ipm_update(render(r, 0.0, 0.0, 0.0, h, t=(0.0, 0.0, 0.0)), i * DT, h - CLEAR,
                      0.0, 0.0, 0.0)
    return e.ipm_fwd


for h0, h1 in ((1.0, 1.5), (0.8, 1.2)):
    old, new = climb(0.0, h0, h1), climb(CLEAR, h0, h1)
    print(f'  набор {h0}→{h1} м (высота перцепции −{CLEAR}): ложный путь без клиренса {old:+.3f}, '
          f'с клиренсом {new:+.3f} м')
    check(f'{h0}→{h1}: без клиренса фантом заметен (|путь| > 0.03 м)', abs(old) > 0.03)
    check(f'{h0}→{h1}: с клиренсом |путь| < 0.03 м', abs(new) < 0.03)

e = FlowEstimator(FX, FY, CX, CY, np.eye(3), ipm_ground_clear=CLEAR)
check('_ipm_geom_h добавляет клиренс', abs(e._ipm_geom_h(1.0, 0.0, 0.0) - (1.0 + CLEAR)) < 1e-12)
e._ipm_update(np.zeros((720, 1280), np.uint8), 0.0, 0.05, 0.0, 0.0, 0.0)
check('гейт земли судит сырую высоту (0.05 < 0.5 → код 1)', e.ipm_fail == 1)

bad = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(bad)}/{len(results)} OK')
sys.exit(1 if bad else 0)
