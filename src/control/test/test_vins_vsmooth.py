#!/usr/bin/env python3
"""Оффлайн-тест VinsHold vsmooth (ФНЧ vins-скорости для D-члена, чистый python).

Источник пилы команды — kd·(сырая VINS-скорость). Подаём в D-член скорость с
шумом ±0.4 м/с вокруг 4 м/с (10 Гц шаг позы → шумная конечная разность) и
смотрим PWM. Проверяем:
- без сглаживания kd·шум пилит выход (крупный шаг);
- со сглаживанием τ=0.3 выход ровнее (шаг меньше);
- сглаживание — на VINS-скорость (kd_err вычитает уставку ПОСЛЕ);
- vsmooth=0 бит-в-бит прежний закон;
- enter() сбрасывает фильтр (нет переноса между сегментами);
- установившееся слежение (v=vsp, без шума): выход тот же (ФНЧ не сдвигает
  постоянный сигнал).

Запуск:  python3 src/control/test/test_vins_vsmooth.py
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
GAINS = dict(kp=0.0, kd=80.0, ki=0.0, imax=100.0, max_pwm=150.0,
             psign=1.0, rsign=1.0, cmd_gain=VSP)


def noisy_run(vsmooth, seed=1, n=60):
    """Борт идёт по уставке (e=0), скорость VINS зашумлена ±0.4 м/с 10 Гц."""
    import random
    rng = random.Random(seed)
    vh = VinsHold(vsmooth=vsmooth, kd_err=True, **GAINS)
    vh.enter(DroneState(now_sim=100.0, vins_x=0.0, vins_y=0.0))
    out = []
    vnoise = 0.0
    for i in range(1, n + 1):
        t = 100.0 + i * DT
        # обновляем шум скорости раз в 2 тика (10 Гц)
        if i % 2 == 1:
            vnoise = rng.uniform(-0.4, 0.4)
        x = (t - 100.0) * VSP          # позиция точно по уставке
        rc = vh.update(DroneState(now_sim=t, vins_x=x, vins_y=0.0,
                                  vins_vx=VSP + vnoise, vins_vy=0.0,
                                  vins_valid=True, vins_last_sim=t),
                       Setpoint(c_fwd=1.0), DT)
        out.append(rc.pitch - RC_CENTER)
    return out


def maxstep(xs):
    return max(abs(xs[i + 1] - xs[i]) for i in range(len(xs) - 1))


# --- 1. без сглаживания: kd·шум пилит ---
raw = noisy_run(0.0)
check("без сглаживания: выход пилит (макс шаг ≥ 20 PWM)", maxstep(raw) >= 20)

# --- 2. со сглаживанием τ=0.3: выход заметно ровнее ---
sm = noisy_run(0.3)
check("τ=0.3: макс шаг вдвое+ меньше сырого",
      maxstep(sm) <= 0.5 * maxstep(raw))

# --- 3. vsmooth=0 — бит-в-бит прежний закон ---
check("vsmooth=0 воспроизводимо бит-в-бит", noisy_run(0.0) == raw)

# --- 4. установившееся слежение без шума: ФНЧ не сдвигает постоянную ---
def steady(vsmooth):
    vh = VinsHold(vsmooth=vsmooth, kd_err=True, **GAINS)
    vh.enter(DroneState(now_sim=100.0, vins_x=0.0, vins_y=0.0))
    rc = None
    for i in range(1, 40):
        t = 100.0 + i * DT
        x = (t - 100.0) * VSP
        rc = vh.update(DroneState(now_sim=t, vins_x=x, vins_y=0.0,
                                  vins_vx=VSP, vins_vy=0.0, vins_valid=True,
                                  vins_last_sim=t), Setpoint(c_fwd=1.0), DT)
    return rc.pitch

check("слежение v=vsp: сглаж. и без — один выход (центр)",
      steady(0.3) == steady(0.0) == RC_CENTER)

# --- 5. enter() сбрасывает фильтр (нет переноса) ---
vh = VinsHold(vsmooth=0.3, kd_err=True, **GAINS)
vh.enter(DroneState(now_sim=100.0, vins_x=0.0, vins_y=0.0))
vh.update(DroneState(now_sim=100.05, vins_x=0.2, vins_y=0.0, vins_vx=9.0,
                     vins_vy=0.0, vins_valid=True, vins_last_sim=100.05),
          Setpoint(c_fwd=1.0), DT)                       # накачали фильтр 9 м/с
vh.enter(DroneState(now_sim=200.0, vins_x=0.0, vins_y=0.0))
# первый апдейт после enter: фильтр стартует с текущей v (5), не с 9
rc = vh.update(DroneState(now_sim=200.05, vins_x=0.2, vins_y=0.0, vins_vx=VSP,
                          vins_vy=0.0, vins_valid=True, vins_last_sim=200.05),
               Setpoint(c_fwd=1.0), DT)
check("после enter(): фильтр сброшен (v=vsp → около центра)",
      abs(rc.pitch - RC_CENTER) <= 8)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ VINS VSMOOTH OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
