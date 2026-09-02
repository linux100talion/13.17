#!/usr/bin/env python3
"""Оффлайн-тест VinsHold kd_err (D-член на ОШИБКЕ скорости, чистый python).

Фикс серии eagle (2026-09-02): со старым законом kd умножает АБСОЛЮТНУЮ
скорость — в движении 4 м/с это 480 PWM постоянного «тормоза» при потолке 150,
борт летит на позиционном долге 9–12 м и звенит ~1 Гц. Проверяем:
- kd_err=False = старый закон бит-в-бит (движение и висение);
- висение / отпущенный стик: kd_err ничего не меняет (v_уставки=0);
- установившееся слежение (v == v_уставки): D-член = 0, долга нет;
- знак: быстрее уставки — тормозим, медленнее — подгоняем;
- геометрия: при vins_yaw=90° продольная уставка уходит в мировую Y;
- уставка позиции продолжает бежать с cmd_gain (сам интегратор не тронут).

Запуск:  python3 src/control/test/test_vins_kd_err.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.vins_hold import VinsHold                # noqa: E402
from control_pkg.domain.rc import RC_CENTER                              # noqa: E402
from control_pkg.domain.setpoint import Setpoint                         # noqa: E402
from control_pkg.domain.state import DroneState                          # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


DT = 0.05
GAINS = dict(kp=40.0, kd=120.0, ki=0.0, imax=100.0, max_pwm=150.0,
             psign=1.0, rsign=1.0, cmd_gain=4.0)


def make(kd_err, yaw=0.0):
    vh = VinsHold(kd_err=kd_err, **GAINS)
    vh.enter(DroneState(now_sim=100.0, vins_x=0.0, vins_y=0.0, vins_yaw=yaw))
    return vh


def st(x=0.0, y=0.0, vx=0.0, vy=0.0, yaw=0.0, t=100.05):
    return DroneState(now_sim=t, vins_x=x, vins_y=y, vins_yaw=yaw,
                      vins_vx=vx, vins_vy=vy, vins_valid=True)


# --- 1. висение, стик в центре: оба закона идентичны (v_уставки = 0) ---
rc_a = make(False).update(st(vx=0.3), Setpoint(), DT)
rc_b = make(True).update(st(vx=0.3), Setpoint(), DT)
check("висение: kd_err не меняет выход (бит-в-бит)",
      (rc_a.pitch, rc_a.roll) == (rc_b.pitch, rc_b.roll))

# --- 2. старый закон в движении: kd·v пробивает потолок ---
# стик вперёд, борт уже летит 4 м/с в ноль ошибки: kd·4 = 480 → кламп 150
rc = make(False).update(st(vx=4.0), Setpoint(c_fwd=1.0), DT)
check("старый закон: в полёте 4 м/с D-член в упоре (+150)",
      rc.pitch == RC_CENTER + 150)

# --- 3. kd_err: установившееся слежение (v == v_уставки) → D = 0 ---
# уставка сдвинулась на vsp·dt = 0.2 м, борт в 0 → kp·e = 40·(−0.2) = −8
rc = make(True).update(st(vx=4.0), Setpoint(c_fwd=1.0), DT)
check("kd_err: v = v_уставки → остаётся только kp по свежему сдвигу (−8)",
      rc.pitch == RC_CENTER - 8)

# --- 4. знак D вокруг движущегося равновесия ---
rc_fast = make(True).update(st(vx=5.0), Setpoint(c_fwd=1.0), DT)
rc_slow = make(True).update(st(vx=3.0), Setpoint(c_fwd=1.0), DT)
check("быстрее уставки (+1 м/с) → тормоз (kd·1 − 8 = +112)",
      rc_fast.pitch == RC_CENTER + 112)
check("медленнее уставки (−1 м/с) → подгон (−kd·1 − 8 = −128)",
      rc_slow.pitch == RC_CENTER - 128)

# --- 5. отпустили стик на ходу: оба закона снова идентичны ---
rc_a = make(False).update(st(vx=4.0), Setpoint(), DT)
rc_b = make(True).update(st(vx=4.0), Setpoint(), DT)
check("стик отпущен на ходу: торможение не изменилось (бит-в-бит)",
      (rc_a.pitch, rc_a.roll) == (rc_b.pitch, rc_b.roll))

# --- 6. геометрия: курс 90° — «вперёд» = мировая Y ---
yaw = math.pi / 2
rc = make(True, yaw=yaw).update(st(vy=4.0, yaw=yaw), Setpoint(c_fwd=1.0), DT)
check("yaw=90°: v_y = v_уставки → тот же чистый kp (−8)",
      rc.pitch == RC_CENTER - 8)

# --- 7. интегратор уставки не тронут: долг позиции копится как раньше ---
vh = make(True)
for i in range(20):                      # 1 с полного стика, борт стоит
    rc = vh.update(st(t=100.05 + i * DT), Setpoint(c_fwd=1.0), DT)
# уставка убежала на 4 м/с × 1 с = 4 м; kp·4 = 160 → кламп 150
check("борт стоит, уставка бежит: через 1 с kp-долг в упоре (−150)",
      rc.pitch == RC_CENTER - 150)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ VINS KD_ERR OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
