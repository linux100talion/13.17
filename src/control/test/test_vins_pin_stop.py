#!/usr/bin/env python3
"""Оффлайн-тест VinsHold pin_stop (гвоздь по остановке, чистый python).

Пункт 2б: на отпускании стика, как только борт встал (|v_vins| < 0.3),
уставка перевязывается на точку остановки — один раз на отпускание.
Гейны kp=40, kd=0, ki=0 — в выходе живёт только позиционная ошибка.
Проверяем:
- без гвоздя: после стопа за уставкой kp тянет назад (возврат);
- с гвоздём: в момент остановки уставка = точка стопа → выход центр;
- гвоздь одноразовый: последующий дрейф при малой v НЕ перевязывает
  уставку (ошибка держится — иначе гвоздь ездил бы за сносом);
- во время торможения (v > 0.3) уставка ещё старая (kp помогает тормозить);
- без предшествующего стика гвоздя нет (висение с ошибкой — держим);
- enter() сбрасывает заказ гвоздя;
- совместимость с i_latch: гвоздь и разморозка трима в один момент.

Запуск:  python3 src/control/test/test_vins_pin_stop.py
"""
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
GAINS = dict(kp=40.0, kd=0.0, ki=0.0, imax=100.0, max_pwm=150.0,
             psign=1.0, rsign=1.0, cmd_gain=4.0)


def make(pin_stop, **kw):
    vh = VinsHold(pin_stop=pin_stop, **{**GAINS, **kw})
    vh.enter(DroneState(now_sim=100.0, vins_x=0.0, vins_y=0.0))
    return vh


def st(x, vx, t):
    return DroneState(now_sim=t, vins_x=x, vins_y=0.0, vins_vx=vx,
                      vins_valid=True)


# сценарий «разгон → отпустили → перелёт → стоп»: стик 1 с (уставка убегает
# на 4 м), борт долетел до x=6 (перелёт +2 за уставку) и встал
def fly(vh):
    t = 100.0
    for i in range(20):                      # 1 с полного стика
        t += DT
        vh.update(st(x=0.0, vx=0.0, t=t), Setpoint(c_fwd=1.0), DT)
    t += DT                                  # отпустили, борт ещё летит (v=3)
    rc_brake = vh.update(st(x=5.0, vx=3.0, t=t), Setpoint(), DT)
    t += DT                                  # встал в x=6 (за уставкой 4)
    rc_stop = vh.update(st(x=6.0, vx=0.1, t=t), Setpoint(), DT)
    t += DT                                  # секунда спустя всё ещё в x=6
    rc_hold = vh.update(st(x=6.0, vx=0.0, t=t), Setpoint(), DT)
    t += DT                                  # снесло на 6.5 при малой v
    rc_drift = vh.update(st(x=6.5, vx=0.1, t=t), Setpoint(), DT)
    return rc_brake, rc_stop, rc_hold, rc_drift


# --- 1. без гвоздя: стоп за уставкой → kp тянет назад ---
rb, rs, rh, rd = fly(make(False))
check("без гвоздя: после стопа тянет назад (выход > центра)",
      rs.pitch > RC_CENTER + 30)

# --- 2. с гвоздём: в момент остановки уставка = точка стопа ---
rb2, rs2, rh2, rd2 = fly(make(True))
check("торможение (v=3 > 0.3): уставка ещё старая, kp помогает тормозить",
      rb2.pitch == rb.pitch)
check("стоп: уставка перевязана — выход центр (возврата нет)",
      rs2.pitch == RC_CENTER)
check("держим точку стопа (x не изменился → центр)", rh2.pitch == RC_CENTER)

# --- 3. гвоздь одноразовый: снос при малой v — ошибка ДЕРЖИТСЯ ---
check("снос 0.5 м после гвоздя: НЕ перевязано (kp·0.5=20 назад)",
      rd2.pitch == RC_CENTER + 20)

# --- 4. без предшествующего стика гвоздя нет ---
vh = make(True)
rc = vh.update(st(x=-0.5, vx=0.1, t=100.05), Setpoint(), DT)
check("висение с ошибкой без стика: держим (гвоздь не заказан)",
      rc.pitch < RC_CENTER)

# --- 5. enter() сбрасывает заказ гвоздя ---
vh = make(True)
vh.update(st(x=0.0, vx=0.0, t=100.05), Setpoint(c_fwd=1.0), DT)
vh.enter(DroneState(now_sim=200.0, vins_x=0.0, vins_y=0.0))
rc = vh.update(st(x=-0.5, vx=0.1, t=200.05), Setpoint(), DT)
check("после enter(): заказ снят, ошибка держится", rc.pitch < RC_CENTER)

# --- 6. с i_latch: гвоздь и разморозка трима в один момент ---
vh = make(True, ki=8.0, i_latch=True)
rb3, rs3, rh3, rd3 = fly(vh)
check("i_latch+pin_stop: в момент стопа выход центр (трим 0, ошибка 0)",
      rs3.pitch == RC_CENTER)
rc = None
for i in range(40):                          # 2 с сноса 0.5 м: И-член копится
    rc = vh.update(st(x=6.5, vx=0.1, t=101.3 + i * DT), Setpoint(), DT)
check("i_latch+pin_stop: снос после гвоздя — И-член снова учится "
      "(через 2 с выход дальше kp·0.5)", rc.pitch > RC_CENTER + 20)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ VINS PIN_STOP OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
