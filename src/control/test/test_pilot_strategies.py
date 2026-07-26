#!/usr/bin/env python3
"""Юнит-тесты пилот-стратегий среза 2 (чистый python, без ROS):
- RcTransmitter: стик=скорость → интеграл смещения уставки; мёртвая зона; центр → стоп.
- PilotPassthrough: сырые стики → RC 1:1.
- Arbiter: тумблер MANUAL → сырые стики (incl throttle); AUTO → автономная команда без изменений.

Запуск:  python3 src/control/test/test_pilot_strategies.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.arbiter import Arbiter                    # noqa: E402
from control_pkg.domain.control.stabilization import PilotPassthrough  # noqa: E402
from control_pkg.domain.control.trajectory import RcTransmitter        # noqa: E402
from control_pkg.domain.rc import RcCommand                            # noqa: E402
from control_pkg.domain.setpoint import Setpoint                       # noqa: E402
from control_pkg.domain.state import DroneState                        # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# --- RcTransmitter: интеграл стика ---
rt = RcTransmitter(vel_gain=0.8, deadzone=30, full_pwm=400)
# полный вперёд (pitch=1900 → axis=+1.0), 10 шагов по 0.1с = 1.0с
s = DroneState(pilot_pitch=1900, pilot_roll=1500)
for k in range(11):
    intent = rt.intent(s, k * 0.1)
# d_fwd ≈ vel_gain * axis * t = 0.8 * 1.0 * 1.0 = 0.8
check("RcTransmitter интегрирует вперёд (~0.8м)", approx(intent.d_fwd, 0.8, 1e-3))
check("RcTransmitter боковое не двинулось", approx(intent.d_right, 0.0))
# центр → интеграл СТОИТ
s.pilot_pitch = 1500
d_before = intent.d_fwd
for k in range(11, 22):
    intent = rt.intent(s, k * 0.1)
check("RcTransmitter центр → уставка стоит", approx(intent.d_fwd, d_before))
# мёртвая зона: стик в пределах deadzone → 0
rt2 = RcTransmitter(vel_gain=1.0, deadzone=30, full_pwm=400)
s2 = DroneState(pilot_pitch=1520)   # +20 < deadzone 30
for k in range(5):
    i2 = rt2.intent(s2, k * 0.1)
check("RcTransmitter мёртвая зона держит 0", approx(i2.d_fwd, 0.0))

# --- PilotPassthrough: 1:1 ---
pp = PilotPassthrough()
s3 = DroneState(pilot_roll=1400, pilot_pitch=1600, pilot_yaw=1550, pilot_throttle=1700)
rc = pp.update(s3, Setpoint(), 0.05)
check("PilotPassthrough roll 1:1", rc.roll == 1400)
check("PilotPassthrough pitch 1:1", rc.pitch == 1600)
check("PilotPassthrough yaw 1:1", rc.yaw == 1550)
check("PilotPassthrough throttle=центр (миссия держит)", rc.throttle == 1500)

# --- Arbiter ---
arb = Arbiter()
auto_cmd = RcCommand(roll=1600, pitch=1400, throttle=1500, yaw=1520)
# AUTO (switch=0): автономная команда без изменений
sA = DroneState(pilot_switch=0, pilot_roll=1111, pilot_pitch=1222,
                pilot_throttle=1333, pilot_yaw=1444)
out = arb.resolve(sA, auto_cmd)
check("Arbiter AUTO → автономная команда", out == auto_cmd and not arb.last_manual)
# MANUAL (switch=1): сырые стики пилота, включая throttle
sM = DroneState(pilot_switch=1, pilot_roll=1111, pilot_pitch=1222,
                pilot_throttle=1333, pilot_yaw=1444)
out = arb.resolve(sM, auto_cmd)
check("Arbiter MANUAL → сырые стики (roll)", out.roll == 1111)
check("Arbiter MANUAL → пилоту throttle", out.throttle == 1333)
check("Arbiter MANUAL → last_manual=True", arb.last_manual)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ПИЛОТ-СТРАТЕГИИ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
