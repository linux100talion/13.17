#!/usr/bin/env python3
"""Юнит-тест per-axis композиции ControlStack (срез 3+, чистый python):
- БАЗА = намерение траектории (оператор), c_*→PWM; незанятая ось = открытый контур;
- частичные стабилизаторы (DpRollHold roll + DpYawHold yaw) перезаписывают свои оси,
  НЕЗАНЯТАЯ (pitch) = профиль-оператор (c_fwd→PWM), НЕ живой пилот;
- живой пилот входит ТОЛЬКО через RcTransmitter (как траектория), не как отдельная база.

Запуск:  python3 src/control/test/test_multiaxis_stack.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.control_stack import ControlStack           # noqa: E402
from control_pkg.domain.control.excitation import NoExcitation           # noqa: E402
from control_pkg.domain.control.stabilization import DpRollHold, DpYawHold  # noqa: E402
from control_pkg.domain.control.trajectory import (                       # noqa: E402
    ConstProfile, RcTransmitter, StaticSetpoint)
from control_pkg.domain.state import DroneState                          # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def state(**kw):
    d = dict(gt_valid=True, now_sim=0.05)
    d.update(kw)
    return DroneState(**d)


# --- 1. Пустой список + профиль → ВСЕ оси = намерение профиля (c_*→PWM, span 400) ---
# c_fwd=0.5→pitch 1700; c_right=-0.25→roll 1400; c_yaw=0.1→yaw 1540.
stack = ControlStack([], ConstProfile(10, c_fwd=0.5, c_right=-0.25, c_yaw=0.1), NoExcitation())
s = state()
stack.enter(s)
rc = stack.update(s)
check("[]+профиль → pitch = c_fwd·span (1700)", rc.pitch == 1700)
check("[]+профиль → roll = c_right·span (1400)", rc.roll == 1400)
check("[]+профиль → yaw = c_yaw·span (1540)", rc.yaw == 1540)

# --- 2. Частичные: DpRollHold(roll)+DpYawHold(yaw), pitch НЕ занят → профиль-оператор ---
stack = ControlStack([DpRollHold(), DpYawHold()],
                     ConstProfile(10, c_fwd=0.5), NoExcitation())
s = state(flow_seq=1, flow_lateral=5.0, flow_yaw=3.0, flow_conf=0.5, flow_dt=0.05)
stack.enter(s)
rc = stack.update(s)
check("частичные: pitch НЕЗАНЯТ → профиль-оператор (c_fwd·span=1700)", rc.pitch == 1700)
check("частичные: roll занят DpRollHold (перезаписан)", rc.roll == 1540)
check("частичные: yaw занят DpYawHold (перезаписан)", rc.yaw == 1518)

# --- 3. StaticSetpoint (нули) → незанятые оси = центр (профиль=0) ---
stack = ControlStack([DpRollHold()], StaticSetpoint(), NoExcitation())
s = state(flow_seq=1, flow_lateral=5.0, flow_conf=0.5, flow_dt=0.05)
stack.enter(s)
rc = stack.update(s)
check("StaticSetpoint: roll регулируется, pitch/yaw = центр",
      rc.roll == 1540 and rc.pitch == 1500 and rc.yaw == 1500)

# --- 4. Живой пилот входит через RcTransmitter (траектория), направленно ---
stack = ControlStack([], RcTransmitter(), NoExcitation())
s = state(pilot_roll=1400, pilot_pitch=1580, pilot_yaw=1450)
stack.enter(s)
rc = stack.update(s)
check("RcTransmitter: pitch вперёд (>центр)", rc.pitch > 1500)
check("RcTransmitter: roll влево (<центр)", rc.roll < 1500)
check("RcTransmitter: yaw (<центр)", rc.yaw < 1500)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ PER-AXIS КОМПОЗИЦИЯ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
