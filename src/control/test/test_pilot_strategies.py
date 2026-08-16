#!/usr/bin/env python3
"""Юнит-тесты пилот-стратегий среза 2 (чистый python, без ROS):
- RcTransmitter: стик=скорость → интеграл смещения уставки; мёртвая зона; центр → стоп.
- PilotPassthrough: сырые стики → RC 1:1.
- Arbiter: тумблер MANUAL → сырые стики (incl throttle); AUTO → автономная команда без изменений.
- joy_sticks (ядро JoyPilot): оси /joy [-1..1] → PWM 1500±400 + тумблер по порогу.

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
from control_pkg.infrastructure.ros_pilot import joy_sticks            # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# --- RcTransmitter: нормированный стик → c_* (profile-only, БЕЗ интеграла) ---
# Знак: вход СЫРОЙ PWM пульта (RC2 выше центра = стик на себя = НАЗАД), выход НАМЕРЕНИЕ
# (c_fwd=+1 = вперёд) → psign=−1. Парный знак в ControlStack даёт сквозной pass-through.
rt = RcTransmitter(deadzone=30, full_pwm=400)
# стик полностью НА СЕБЯ (pitch=1900 → нормировка +1.0) = намерение НАЗАД
intent = rt.intent(DroneState(pilot_pitch=1900, pilot_roll=1500, pilot_yaw=1500), 0.5)
check("RcTransmitter стик на себя (1900) → c_fwd=−1.0 (назад)", approx(intent.c_fwd, -1.0))
# стик ОТ СЕБЯ (pitch=1100) = намерение ВПЕРЁД
intent = rt.intent(DroneState(pilot_pitch=1100), 0.5)
check("RcTransmitter стик от себя (1100) → c_fwd=+1.0 (вперёд)", approx(intent.c_fwd, 1.0))
check("RcTransmitter боковой в центре → c_right=0", approx(intent.c_right, 0.0))
# центр → c_*=0 (не «стоит интеграл», а НЕТ команды)
intent = rt.intent(DroneState(pilot_pitch=1500), 1.0)
check("RcTransmitter центр → c_fwd=0", approx(intent.c_fwd, 0.0))
# половина хода: -(1700-1500-30)/(400-30) ≈ -0.459
intent = rt.intent(DroneState(pilot_pitch=1700), 0.0)
check("RcTransmitter полстика на себя → c_fwd≈−0.46", approx(intent.c_fwd, -170.0 / 370.0, 1e-3))
# мёртвая зона: стик в пределах deadzone → 0
intent = rt.intent(DroneState(pilot_pitch=1520), 0.0)   # +20 < deadzone 30
check("RcTransmitter мёртвая зона держит 0", approx(intent.c_fwd, 0.0))

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

# --- joy_sticks: ядро JoyPilot (оси /joy → PWM, тумблер) ---
# полный ход осей: +1 → 1900, −1 → 1100; тумблер CH6 (axes[5]) > 0.5 → MANUAL
r, p, t, y, sw = joy_sticks([1.0, -1.0, 0.0, 0.5, 0.0, 1.0])
check("joy_sticks roll +1 → 1900", r == 1900)
check("joy_sticks pitch −1 → 1100", p == 1100)
check("joy_sticks throttle 0 → центр", t == 1500)
check("joy_sticks yaw +0.5 → 1700", y == 1700)
check("joy_sticks тумблер >0.5 → MANUAL", sw == 1)
# тумблер ниже порога → AUTO; ровно на пороге (0.5) — тоже AUTO (строгое >)
_, _, _, _, sw = joy_sticks([0.0, 0.0, 0.0, 0.0, 0.0, 0.5])
check("joy_sticks тумблер на пороге 0.5 → AUTO", sw == 0)
# знак оси инвертируется параметром (наземная сверка знаков — по одному стику)
r, _, _, _, _ = joy_sticks([1.0, 0.0, 0.0, 0.0], signs=(-1.0, 1.0, 1.0, 1.0))
check("joy_sticks sign=−1 зеркалит ось", r == 1100)
# выход за [-1..1] (драйвер/калибровка) клампится в 1100..1900
r, _, _, _, _ = joy_sticks([1.7, 0.0, 0.0, 0.0])
check("joy_sticks ось >1 клампится в 1900", r == 1900)
# короткий axes (нет CH6): стики без тумблера → центр не трогаем, sw=0 (AUTO)
r, p, t, y, sw = joy_sticks([0.25, 0.0])
check("joy_sticks короткий axes → отсутствующие оси = центр", t == 1500 and y == 1500)
check("joy_sticks без оси тумблера → AUTO", sw == 0)
check("joy_sticks присутствующая ось при коротком axes работает", r == 1600)
# боевые знаки TX12 (выверены первым живым полётом): roll/yaw зеркальны, pitch/thr прямые
from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT   # noqa: E402
check("JOY_SIGNS_DEFAULT: roll/yaw зеркальны, pitch/thr прямые",
      JOY_SIGNS_DEFAULT == (-1.0, 1.0, 1.0, -1.0))
r, p, t, y, _ = joy_sticks([1.0, 1.0, 1.0, 1.0], signs=JOY_SIGNS_DEFAULT)
check("joy_sticks с боевыми знаками: +1 по всем осям → 1100/1900/1900/1100",
      (r, p, t, y) == (1100, 1900, 1900, 1100))

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ПИЛОТ-СТРАТЕГИИ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
