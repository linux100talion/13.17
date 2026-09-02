#!/usr/bin/env python3
"""Оффлайн-тест VinsHold i_latch (защёлка трима, чистый python).

Аналог _TRIM_LATCH станции: И-член заморожен от живого стика до «гвоздя»
(борт встал после отпускания, |v_vins| < 0.3). Гейны kp=0, kd=0, ki=8 —
в выходе ЖИВЁТ ТОЛЬКО И-член, закон изолирован. Проверяем:
- без защёлки И-член копится под стиком (старое поведение не тронуто);
- с защёлкой под стиком И-член заморожен (выход не растёт);
- выученный ДО стика трим держится замороженным весь ход (не сброшен);
- после отпускания на ходу (v > 0.3) И-член ещё спит (выбег — не ветер);
- борт встал (v < 0.3) — трим снова учится;
- стик в мёртвой зоне (0.01) защёлку не взводит;
- enter() сбрасывает защёлку.

Запуск:  python3 src/control/test/test_vins_i_latch.py
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
GAINS = dict(kp=0.0, kd=0.0, ki=8.0, imax=100.0, max_pwm=150.0,
             psign=1.0, rsign=1.0, cmd_gain=4.0)


def make(i_latch):
    vh = VinsHold(i_latch=i_latch, **GAINS)
    vh.enter(DroneState(now_sim=100.0, vins_x=0.0, vins_y=0.0))
    return vh


def run(vh, n, sp, x=0.0, vx=0.0, t0=100.0):
    rc = None
    for i in range(n):
        s = DroneState(now_sim=t0 + (i + 1) * DT, vins_x=x, vins_y=0.0,
                       vins_vx=vx, vins_valid=True)
        rc = vh.update(s, sp, DT)
    return rc


# --- 1. без защёлки: под стиком И-член копится (старый закон) ---
vh = make(False)
rc = run(vh, 40, Setpoint(c_fwd=1.0))    # 2 с полного стика, борт стоит в 0
# уставка убежала вперёд, ошибка отрицательная растёт → И-член наматывается
check("без защёлки: под стиком И-член намотан (выход < центра)",
      rc.pitch < RC_CENTER - 20)

# --- 2. с защёлкой: тот же ход — И-член заморожен в нуле ---
vh = make(True)
rc = run(vh, 40, Setpoint(c_fwd=1.0))
check("с защёлкой: под стиком И-член заморожен (выход = центр)",
      rc.pitch == RC_CENTER)

# --- 3. выученный до стика трим держится замороженным ---
vh = make(True)
run(vh, 40, Setpoint(), x=-0.5)          # висение с ошибкой −0.5 м: трим учится
rc0 = run(vh, 1, Setpoint(), x=-0.5, t0=102.0)
learned = rc0.pitch - RC_CENTER
check("висение: трим выучен (выход ≠ центр)", learned != 0)
rc = run(vh, 40, Setpoint(c_fwd=1.0), x=-0.5, t0=102.05)
check("стик жил 2 с: трим ТОТ ЖЕ (заморожен, не сброшен и не рос)",
      rc.pitch - RC_CENTER == learned)

# --- 4. отпустили на ходу: выбег (v=2 > 0.3) — И-член ещё спит ---
rc = run(vh, 40, Setpoint(), x=-0.5, vx=2.0, t0=104.05)
check("выбег после отпускания: И-член ещё спит (трим тот же)",
      rc.pitch - RC_CENTER == learned)

# --- 5. борт встал (v=0.1 < 0.3): трим снова учится ---
rc = run(vh, 40, Setpoint(), x=-0.5, vx=0.1, t0=106.05)
check("гвоздь (v<0.3): И-член снова копится (выход ушёл от learned)",
      rc.pitch - RC_CENTER != learned)

# --- 6. стик в мёртвой зоне защёлку не взводит ---
vh = make(True)
rc = run(vh, 40, Setpoint(c_fwd=0.01), x=-0.5)
check("стик 0.01 (< dz): И-член живёт как без стика", rc.pitch < RC_CENTER)

# --- 7. enter() сбрасывает защёлку ---
vh = make(True)
run(vh, 5, Setpoint(c_fwd=1.0))
vh.enter(DroneState(now_sim=200.0, vins_x=0.0, vins_y=0.0))
rc = run(vh, 40, Setpoint(), x=-0.5, t0=200.0)
check("после enter(): защёлка снята, трим учится", rc.pitch < RC_CENTER)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ VINS I_LATCH OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
