#!/usr/bin/env python3
"""Юнит-тест PROFILE-ONLY модели движения (чистый python, без ROS):
- позиц-холдер (GzHold) ИНТЕГРИРУЕТ стик-профиль в движущуюся уставку → отклик растёт;
- нулевой профиль + дрон в опоре → держит (центр);
- симметричный челнок (стик-профиль ±level) → уставка возвращается к опоре.

Заменяет устаревший test_gz_shuttle_equiv (тот проверял метрическую d_*-модель монолита,
которой больше нет — движение теперь только стик-профили).

Запуск:  python3 src/control/test/test_profile_motion.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.control_stack import ControlStack       # noqa: E402
from control_pkg.domain.control.excitation import NoExcitation       # noqa: E402
from control_pkg.domain.control.stabilization import GzHold          # noqa: E402
from control_pkg.domain.control.trajectory import (                  # noqa: E402
    ProfileTrajectory, Shuttle, StaticSetpoint)
from control_pkg.domain.state import DroneState                      # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def gt(x=0.0, y=0.0, now=0.0):
    return DroneState(gt_valid=True, gt_x=x, gt_y=y, gt_yaw=0.0, now_sim=now)


# --- 1. Постоянный форвард-профиль → уставка едет → pitch монотонно уходит от центра ---
stack = ControlStack([GzHold()], ProfileTrajectory([(100.0, 1500, 1900, 1500)]),
                     NoExcitation())
stack.enter(gt(now=0.0))
pitches = []
for k in range(1, 6):
    rc = stack.update(gt(now=k * 0.1))    # дрон стоит в опоре, уставка уезжает вперёд
    pitches.append(rc.pitch)
mono = all(pitches[i] < pitches[i - 1] for i in range(1, len(pitches)))
check("постоянный форвард-профиль → pitch уходит от центра", pitches[-1] != 1500)
check("отклик растёт монотонно (уставка интегрируется)", mono)

# --- 2. Нулевой профиль + дрон в опоре → держит (центр) ---
stack2 = ControlStack([GzHold()], StaticSetpoint(), NoExcitation())
stack2.enter(gt(now=0.0))
rc = stack2.update(gt(now=0.1))
check("нулевой профиль + дрон в опоре → центр (держит)", rc.roll == 1500 and rc.pitch == 1500)

# --- 3. Симметричный челнок → интеграл уставки возвращается к опоре ---
gz = GzHold()
stack3 = ControlStack([gz], Shuttle(level=0.3, leg=3.0, pause=2.0, forward=False),
                      NoExcitation())
stack3.enter(gt(now=0.0))
t = 0.0
while t < 9.0:                            # total = 2*3+2 = 8с + запас
    t += 0.05
    stack3.update(gt(now=t))
check("симметричный челнок → уставка вернулась к опоре (|Δ|<0.05)", abs(gz._spy) < 0.05)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ PROFILE-ONLY МОДЕЛЬ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
