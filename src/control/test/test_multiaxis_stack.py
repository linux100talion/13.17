#!/usr/bin/env python3
"""Юнит-тест per-axis композиции ControlStack (срез 3, чистый python):
- пустой список стабилизаторов → ВСЕ оси = сырые стики пилота (manual);
- частичные стабилизаторы (DpRollHold roll + DpYawHold yaw) → эти оси регулируются,
  а НЕЗАНЯТАЯ (pitch) = сырой стик пилота;
- одиночный стабилизатор (список из одного) — обратная совместимость.

Запуск:  python3 src/control/test/test_multiaxis_stack.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.control_stack import ControlStack           # noqa: E402
from control_pkg.domain.control.excitation import NoExcitation           # noqa: E402
from control_pkg.domain.control.stabilization import DpRollHold, DpYawHold  # noqa: E402
from control_pkg.domain.control.trajectory import RcTransmitter, StaticSetpoint  # noqa: E402
from control_pkg.domain.state import DroneState                          # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def state(**kw):
    d = dict(gt_valid=True, now_sim=0.05)
    d.update(kw)
    return DroneState(**d)


# --- 1. Пустой список стабилизаторов = всё пилоту (manual) ---
stack = ControlStack([], StaticSetpoint(), NoExcitation())
s = state(pilot_roll=1400, pilot_pitch=1580, pilot_yaw=1450)
stack.enter(s)
rc = stack.update(s)
check("[] → roll = стик пилота", rc.roll == 1400)
check("[] → pitch = стик пилота", rc.pitch == 1580)
check("[] → yaw = стик пилота", rc.yaw == 1450)

# --- 2. Частичные: DpRollHold(roll) + DpYawHold(yaw), pitch НЕ занят → пилот ---
stack = ControlStack([DpRollHold(), DpYawHold()], RcTransmitter(), NoExcitation())
s = state(pilot_roll=1400, pilot_pitch=1580, pilot_yaw=1450,
          flow_seq=1, flow_lateral=5.0, flow_yaw=3.0, flow_conf=0.5, flow_dt=0.05)
stack.enter(s)
rc = stack.update(s)
check("частичные: pitch НЕЗАНЯТ → стик пилота (1580)", rc.pitch == 1580)
check("частичные: roll занят DpRollHold (≠ стик 1400)", rc.roll != 1400 and rc.roll == 1540)
check("частичные: yaw занят DpYawHold (≠ стик 1450)", rc.yaw != 1450 and rc.yaw == 1518)

# --- 3. Одиночный стабилизатор как список из одного (обратная совместимость) ---
stack = ControlStack([DpRollHold()], StaticSetpoint(), NoExcitation())
s = state(pilot_roll=1400, pilot_pitch=1580, pilot_yaw=1450,
          flow_seq=1, flow_lateral=5.0, flow_conf=0.5, flow_dt=0.05)
stack.enter(s)
rc = stack.update(s)
check("одиночный: roll регулируется, pitch/yaw = пилот", rc.roll == 1540 and rc.pitch == 1580 and rc.yaw == 1450)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ PER-AXIS КОМПОЗИЦИЯ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
