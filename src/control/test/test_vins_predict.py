#!/usr/bin/env python3
"""Оффлайн-тест VinsHold predict (предиктор позы, чистый python).

Репро пилы: контроллер 20 Гц (dt=0.05), уставка бежит vsp=4 м/с, поза VINS
обновляется 10 Гц (замирает на 2 тика). Борт идеально летит по уставке.
Гейны kp=40, kd=0, ki=0 — в выходе живёт только kp·e. Проверяем:
- без предиктора выход ПИЛИТ (чередование ±kp·vsp·dt = ±8 PWM);
- с предиктором выход ровный (ошибка нулевая на каждом тике);
- свежий отсчёт (возраст 0) — предиктор ничего не меняет;
- кап экстраполяции: возраст 1 с → продвигаем только на _PRED_MAX;
- kd_err поверх предиктора: слежение по-прежнему даёт D=0;
- predict=False — бит-в-бит старый закон.

Запуск:  python3 src/control/test/test_vins_predict.py
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
VSP = 4.0
GAINS = dict(kp=40.0, kd=0.0, ki=0.0, imax=100.0, max_pwm=150.0,
             psign=1.0, rsign=1.0, cmd_gain=VSP)


def saw_run(predict, n=20, **kw):
    """Полёт точно по уставке; VINS шагает 10 Гц. Возвращает список pitch-PWM."""
    vh = VinsHold(predict=predict, **{**GAINS, **kw})
    vh.enter(DroneState(now_sim=100.0, vins_x=0.0, vins_y=0.0))
    out = []
    for i in range(1, n + 1):
        t = 100.0 + i * DT
        # последний отсчёт VINS — на сетке 0.1 с; поза заморожена на нём
        t_smp = 100.0 + int(i * DT / 0.10) * 0.10
        x_smp = (t_smp - 100.0) * VSP        # истинная поза в момент отсчёта
        rc = vh.update(DroneState(now_sim=t, vins_x=x_smp, vins_y=0.0,
                                  vins_vx=VSP, vins_valid=True,
                                  vins_last_sim=t_smp),
                       Setpoint(c_fwd=1.0), DT)
        out.append(rc.pitch - RC_CENTER)
    return out


# --- 1. без предиктора: пила ±kp·vsp·dt = ±8 PWM ---
saw = saw_run(False)
steps = [abs(saw[i + 1] - saw[i]) for i in range(len(saw) - 1)]
check("без предиктора: команда пилит (макс шаг ≥ 8 PWM)", max(steps) >= 8)
check("без предиктора: реверсы знака шага есть",
      any((saw[i + 1] - saw[i]) * (saw[i] - saw[i - 1]) < 0
          for i in range(1, len(saw) - 1)))

# --- 2. с предиктором: выход ровный ---
flat = saw_run(True)
fsteps = [abs(flat[i + 1] - flat[i]) for i in range(len(flat) - 1)]
check("с предиктором: команда ровная (макс шаг ≤ 1 PWM)", max(fsteps) <= 1)
check("с предиктором: ошибка нулевая (выход = центр ±1)",
      all(abs(v) <= 1 for v in flat))

# --- 3. свежий отсчёт (возраст 0): предиктор ничего не меняет ---
def one(predict, age):
    vh = VinsHold(predict=predict, **GAINS)
    vh.enter(DroneState(now_sim=100.0, vins_x=0.0, vins_y=0.0))
    t = 100.0 + DT
    return vh.update(DroneState(now_sim=t, vins_x=1.0, vins_y=0.0,
                                vins_vx=2.0, vins_valid=True,
                                vins_last_sim=t - age),
                     Setpoint(), DT).pitch

check("возраст 0: predict on == off", one(True, 0.0) == one(False, 0.0))

# --- 4. кап экстраполяции: возраст 1 с → только _PRED_MAX ---
# poza 1.0 + 2.0·0.3 = 1.6; уставка 0 → kp·1.6 = 64
rc = one(True, 1.0)
check("возраст 1 с: экстраполяция капится 0.3 с (kp·1.6 = 64)",
      rc == RC_CENTER + 64)

# --- 5. kd_err поверх предиктора: слежение по уставке → D = 0 ---
flat2 = saw_run(True, kd=80.0, kd_err=True)
f2steps = [abs(flat2[i + 1] - flat2[i]) for i in range(len(flat2) - 1)]
check("kd_err + предиктор: слежение ровное (макс шаг ≤ 1 PWM)",
      max(f2steps) <= 1)

# --- 6. predict=False — закон бит-в-бит прежний (то же, что пила из п.1) ---
saw2 = saw_run(False)
check("выкл: воспроизводимо бит-в-бит", saw2 == saw)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ VINS PREDICT OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
