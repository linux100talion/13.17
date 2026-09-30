#!/usr/bin/env python3
"""att_convert — PWM → углы/набор/thrust против формул ArduCopter (без ROS).

Параметры — SITL проекта (2026-09-30: ATC_ANGLE_MAX 20, RC 1000/1500/2000, RC3_DZ 30,
THR_DZ 100, PILOT_SPD_UP 2.5, PILOT_SPD_DN 0, PILOT_Y_RATE 202.5, WP_SPD 2.5/1.5) и
реального борта (stellar_cld.txt: 988/1500/2011, RC3_TRIM 1522, THR_DZ 200, SPD 0.5/0.5).

Запуск:  python3 src/control/test/test_att_convert.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control_pkg.domain.rc import RcCommand                          # noqa: E402
from control_pkg.domain.attitude import AttitudeCommand             # noqa: E402
from control_pkg.infrastructure.att_convert import (                 # noqa: E402
    PARAMS, FcuStickParams, climb_from_throttle, limit_tilt, rc_to_attitude,
    thrust_from_climb)

ok = True


def check(name, cond, detail=""):
    global ok
    print(f"  [{'OK ' if cond else 'FAIL'}] {name}" + (f" — {detail}" if detail and not cond else ""))
    ok &= bool(cond)


SIM = {'ATC_ANGLE_MAX': 20, 'PILOT_Y_RATE': 202.5,
       'RC1_MIN': 1000, 'RC1_MAX': 2000, 'RC1_TRIM': 1500,
       'RC2_MIN': 1000, 'RC2_MAX': 2000, 'RC2_TRIM': 1500,
       'RC3_MIN': 1000, 'RC3_MAX': 2000, 'RC3_DZ': 30,
       'RC4_MIN': 1000, 'RC4_MAX': 2000, 'RC4_TRIM': 1500,
       'THR_DZ': 100, 'PILOT_SPD_UP': 2.5, 'PILOT_SPD_DN': 0.0,
       'WP_SPD_UP': 2.5, 'WP_SPD_DN': 1.5}
BOARD = dict(SIM, **{'RC1_MIN': 988, 'RC1_MAX': 2011, 'RC2_MIN': 988, 'RC2_MAX': 2011,
                     'RC3_MIN': 988, 'RC3_MAX': 2011, 'RC4_MIN': 988, 'RC4_MAX': 2011,
                     'THR_DZ': 200, 'PILOT_SPD_UP': 0.5, 'PILOT_SPD_DN': 0.5, 'PILOT_Y_RATE': 90})
deg = math.degrees

print("кэш параметров")
c = FcuStickParams()
check("пустой — не готов", not c.ready() and set(c.missing()) == set(PARAMS))
for k, v in SIM.items():
    c.on_param(k, v)
check("все прочитаны — готов", c.ready())
check("чужой параметр игнорируется", c.on_param('FOO', 1) is False)
check("повтор значения — без изменения", c.on_param('THR_DZ', 100) is False)

print("крен/тангаж (rc_input_to_roll_pitch_rad)")
a = rc_to_attitude(RcCommand(), SIM)
check("центр → 0 / 0 / 0 / 0", a.roll == 0 and a.pitch == 0 and a.yaw_rate == 0 and a.climb == 0)
a = rc_to_attitude(RcCommand(pitch=1100), SIM)
check("стик вперёд 1100 → нос вниз −16°", abs(deg(a.pitch) + 16.0) < 1e-9, f"{deg(a.pitch):.3f}")
a = rc_to_attitude(RcCommand(roll=1750), SIM)
check("крен вправо 1750 → +10°", abs(deg(a.roll) - 10.0) < 1e-9, f"{deg(a.roll):.3f}")
a = rc_to_attitude(RcCommand(roll=2000, pitch=1000), SIM)
tilt = deg(math.atan(math.hypot(math.tan(a.pitch), math.tan(a.roll) / math.cos(a.pitch))))
check("полный крен+тангаж: наклон вектора ≤ 20° (потолок держит ядро)", tilt <= 20.0 + 1e-6, f"{tilt:.2f}")
a = rc_to_attitude(RcCommand(pitch=1510), BOARD)
check("борт: мёртвой зоны нет — 10 µs уже дают угол", abs(a.pitch) > 0)
check("борт: 2011 (за пределом 500 µs) → кламп 20°",
      abs(deg(rc_to_attitude(RcCommand(pitch=2011), BOARD).pitch) - 20) < 1e-9)
check("реверс не применяется: вперёд всегда нос вниз и на борту",
      rc_to_attitude(RcCommand(pitch=1300), BOARD).pitch < 0)

print("курс")
check("вправо в упор (2000) = +90 °/с — фиксированный масштаб, не PILOT_Y_RATE",
      abs(deg(rc_to_attitude(RcCommand(yaw=2000), SIM).yaw_rate) - 90.0) < 1e-9)
check("влево 1250 → −45 °/с",
      abs(deg(rc_to_attitude(RcCommand(yaw=1250), SIM).yaw_rate) + 45.0) < 1e-9)

print("единицы не зависят от борта (вариант A 2026-09-30)")
for rc in (RcCommand(pitch=1400), RcCommand(roll=1620, pitch=1450), RcCommand(yaw=1700)):
    a_sim, a_brd = rc_to_attitude(rc, SIM), rc_to_attitude(rc, BOARD)
    check(f"{rc}: сим и борт дают тот же угол и темп",
          abs(a_sim.roll - a_brd.roll) < 1e-12 and abs(a_sim.pitch - a_brd.pitch) < 1e-12
          and abs(a_sim.yaw_rate - a_brd.yaw_rate) < 1e-12)
check("0.040 °/µs: 100 µs → 4°", abs(deg(rc_to_attitude(RcCommand(pitch=1600), SIM).pitch) - 4.0) < 1e-9)
tight = dict(SIM, ATC_ANGLE_MAX=10)
check("потолок — ATC_ANGLE_MAX полётника (10° режет 16°)",
      abs(deg(rc_to_attitude(RcCommand(pitch=1100), tight).pitch) + 10.0) < 1e-6)

print("газ (get_pilot_desired_climb_rate_ms)")
check("1500 = центр → 0", climb_from_throttle(1500, SIM) == 0.0)
check("в мёртвой зоне THR_DZ (1540) → 0", climb_from_throttle(1540, SIM) == 0.0)
check("в упор 2000 → +PILOT_SPD_UP", abs(climb_from_throttle(2000, SIM) - 2.5) < 1e-9)
check("в пол 1000 → −PILOT_SPD_UP (SPD_DN 0 = как вверх)", abs(climb_from_throttle(1000, SIM) + 2.5) < 1e-9)
v = climb_from_throttle(1620, SIM)
check("газ 0.3 реплея (1620) → вверх, меньше полного", 0 < v < 2.5, f"{v:.3f}")
check("борт: 1620 внутри THR_DZ 200 → 0 (газ реплея там не взлетит)",
      climb_from_throttle(1620, BOARD) == 0.0)

print("thrust (GUID_OPTIONS бит 3 = 0)")
check("0 м/с → 0.5", thrust_from_climb(0.0, SIM) == 0.5)
check("+WP_SPD_UP → 1.0", thrust_from_climb(2.5, SIM) == 1.0)
check("−WP_SPD_DN → 0.0", thrust_from_climb(-1.5, SIM) == 0.0)
check("за пределами — кламп", thrust_from_climb(9, SIM) == 1.0 and thrust_from_climb(-9, SIM) == 0.0)
check("+1.25 м/с → 0.75", abs(thrust_from_climb(1.25, SIM) - 0.75) < 1e-12)

print("потолок наклона для любой команды углом (полётник его не ставит)")
big = limit_tilt(AttitudeCommand(pitch=math.radians(-40), yaw_rate=0.3, climb=0.7), SIM)
check("тангаж −40° → −20°, темп и набор не тронуты",
      abs(deg(big.pitch) + 20) < 1e-6 and big.yaw_rate == 0.3 and big.climb == 0.7, f"{deg(big.pitch):.3f}")
small = AttitudeCommand(roll=math.radians(5), pitch=math.radians(-5))
check("в пределах — как есть", limit_tilt(small, SIM) is small)
def thrust_xy(c):
    return -math.tan(c.pitch), math.tan(c.roll) / math.cos(c.pitch)


src = AttitudeCommand(roll=math.radians(30), pitch=math.radians(-30))
diag = limit_tilt(src, SIM)
(x0, y0), (x1, y1) = thrust_xy(src), thrust_xy(diag)
check("диагональ: направление вектора тяги сохраняется, наклон = 20°",
      abs(math.atan2(y0, x0) - math.atan2(y1, x1)) < 1e-9
      and abs(deg(math.atan(math.hypot(x1, y1))) - 20) < 1e-6)

print("ИТОГ:", "✅ ATT CONVERT OK" if ok else "❌ ПРОВАЛ")
sys.exit(0 if ok else 1)
