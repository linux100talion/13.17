#!/usr/bin/env python3
"""Юнит-тест ФВЧ ускорения прогноза в ОСЯХ КУРСА (`ipm_acc_world`, BS_IPM_ACC_WORLD).

Борт висит против ветра: наклон, балансирующий ветер, неподвижен В МИРЕ, истинная скорость
ноль. ФВЧ (`ipm_acc_tau`) обязан вычесть этот наклон из прогноза. На развороте наклон в
осях тела вращается с −ω_z. ФВЧ в осях тела отстаёт от вращения, и прогноз набирает ложную
скорость ∝ ω_z — та самая ошибка, найденная реплеем серии cmd/ipm_lever (ipm_yaw_err.py).
ФВЧ в осях курса (оценка поворачивается на −ω_z·dt) держит ноль.

Без кадров: зовём `_vel_predict` напрямую, измерений нет (фильтр — чистый прогноз).

Запуск:  python3 src/control/test/test_ipm_acc_world.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                                    # noqa: E402

from control_pkg.perception.flow_estimator import FlowEstimator       # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


TILT = math.radians(4.0)      # наклон против ветра (≈ ветер 5 м/с)
DT = 1.0 / 30.0


def run(acc_world, wz, sec=12.0, hover=20.0):
    """Висение `hover` с (ФВЧ сходится), потом разворот с ω_z = wz `sec` с.
    Возвращает максимальную |скорость прогноза| за разворот."""
    e = FlowEstimator(640.0, 640.0, 640.0, 360.0, np.eye(3), ipm_vel_tau=0.4,
                      ipm_acc_tau=5.0, ipm_acc_world=acc_world)
    psi, t, vmax = 0.0, 0.0, 0.0
    for i in range(int((hover + sec) / DT)):
        w = wz if t >= hover else 0.0
        psi += w * DT
        # мировой наклон «нос против ветра на восток» в осях тела, повернувшегося на psi:
        # тангаж (нос вниз +) = TILT·cos psi, крен (правое вниз +) = TILT·sin psi
        pitch, roll = TILT * math.cos(psi), TILT * math.sin(psi)
        e._vel_predict(t, pitch, roll, w)
        if t >= hover:
            vmax = max(vmax, math.hypot(*e._ipm_v))
        t += DT
    return vmax


for wz in (0.2, 0.4):
    body, world = run(False, wz), run(True, wz)
    print(f'  ω_z {wz} рад/с: ложная скорость прогноза — ФВЧ в теле {body:.3f} м/с, '
          f'в осях курса {world:.3f} м/с')
    check(f'ω {wz}: в осях тела ложная скорость заметна (> 0.1 м/с)', body > 0.1)
    check(f'ω {wz}: в осях курса — меньше 0.02 м/с', world < 0.02)

h_body, h_world = run(False, 0.0), run(True, 0.0)
check(f'без разворота обе оценки держат ноль ({h_body:.4f} / {h_world:.4f})',
      h_body < 0.005 and h_world < 0.005)

# выключенная ручка = бит-в-бит прежнее поведение
a = FlowEstimator(640.0, 640.0, 640.0, 360.0, np.eye(3), ipm_vel_tau=0.4, ipm_acc_tau=5.0)
b = FlowEstimator(640.0, 640.0, 640.0, 360.0, np.eye(3), ipm_vel_tau=0.4, ipm_acc_tau=5.0,
                  ipm_acc_world=False)
for i in range(300):
    for e in (a, b):
        e._vel_predict(i * DT, 0.07 * math.sin(i * 0.05), 0.05 * math.cos(i * 0.03), 0.3)
check('ipm_acc_world=False — бит-в-бит прежний прогноз', a._ipm_v == b._ipm_v)

bad = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(bad)}/{len(results)} OK')
sys.exit(1 if bad else 0)
