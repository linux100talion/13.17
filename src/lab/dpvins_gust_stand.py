#!/usr/bin/env python3
"""dpvins_gust_stand.py — СТЕНД DpVins на висении под порывами: свип ki (и kp) на планте
проекта без полётов. Ответ на вопрос серии dphold_vs_dpvins 2026-09-05: DpVins пропускал
порыв 8 м/с на 6.3–9.5 м против 2.4–2.8 у DpHold — сколько из этого лечится тримом (ki),
не трогая kp (kp понижен до 40/32 ради σθ 0.5°, кампания ab_dpv_*).

Плант — как в test_dpvins §13: v̇ = α·(ветер − наклон), α = 0.01 м/с² на PWM (100 PWM =
1 м/с²; ветер 10 м/с ≈ 100 PWM-экв., 5 ≈ 44), наклон исполняется апериодикой τ 0.26 с
(идентифицированный лаг FCU, модель BS_GZ_KD), скорость измеряется с лагом 0.15 с (EMA
VinsTrack a=0.4 @10 Гц). Порыв — огибающая wind_gust.py (1−cos фронт 2 с, плато 5, спад 4,
период 20) с силой GUST PWM-экв.; калибровка: при ki 6 пик за цикл должен выйти 6–9 м и
vmax 1.4–1.9, как в полётах → GUST 75 (порыв 8 м/с). До порывов борт получает толчок,
чтобы связался первый гвоздь (в полёте — посев трима от демпфера + первый стоп).

Метрики по циклам без первого (установившийся режим): пик смещения от точки на фронте,
остаток в конце цикла, vmax; плюс первый цикл отдельно (обучение трима под порывом).
Итог 2026-09-05 (kp 40/32, vsmooth 0.3, imax 120, GUST 75):
  ki   6: пик 9.26 м, остаток 0.20   — эталон, совпал с полётами
  ki  15: пик 4.93, остаток 0.02
  ki  20: пик 4.09, остаток 0.13
  ki  30: пик 3.64, остаток 0.22, vmax 1.80   — кандидат dpvins/ki30.txt
  ki  45: пик 7.23, остаток 1.60, vmax 2.69   — раскачка трима (перелёт)
  ki  60: пик 6.19, остаток 1.44, vmax 3.01
  для масштаба: kp 90 + ki 30 (демпфер) — пик 1.62; kp 60/48 + ki 30 — 2.69.
Стенд не знает шума VINS и предельного цикла kp против контура ориентации FCU (Tθ 1.6 с) —
σθ он не предскажет, это только полёт. Запуск с хоста (ROS не нужен):
  python3 src/lab/dpvins_gust_stand.py [--gust 75] [--ki 6,15,20,30,45,60] [--kp 40,32]
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "control"))
from control_pkg.domain.control.vins_axes import DpVins      # noqa: E402
from control_pkg.domain.rc import RC_CENTER                  # noqa: E402
from control_pkg.domain.setpoint import Setpoint             # noqa: E402
from control_pkg.domain.state import DroneState              # noqa: E402

DT = 0.05
ALPHA = 0.01          # м/с² на PWM (100 PWM = 1 м/с², test_dpvins §13)
TAU_ACT = 0.26        # лаг исполнения наклона FCU, с
TAU_MEAS = 0.15       # лаг измерения скорости (EMA VinsTrack), с
BASE_PWM = 8.0        # база 1 м/с ≈ 8 PWM-экв.
GUST = dict(at=30.0, rise=2.0, hold=5.0, fall=4.0, every=20.0)


def gust_env(t):
    if t < GUST['at']:
        return 0.0
    ph = (t - GUST['at']) % GUST['every']
    if ph < GUST['rise']:
        return 0.5 * (1 - math.cos(math.pi * ph / GUST['rise']))
    ph -= GUST['rise']
    if ph < GUST['hold']:
        return 1.0
    ph -= GUST['hold']
    if ph < GUST['fall']:
        return 0.5 * (1 + math.cos(math.pi * ph / GUST['fall']))
    return 0.0


def run(ki, gust_pwm, kp_fwd=40.0, kp_lat=32.0, vsmooth=0.3, imax=120.0, cycles=6):
    vh = DpVins(kp_fwd=kp_fwd, kp_lat=kp_lat, ki=ki, ki_trim=60.0, imax=imax, max_pwm=150.0,
                cmd_gain=4.0, pos_kp=0.3, pos_vmax=0.3, pos_acc=0.15, vsmooth=vsmooth,
                i_latch=True)
    t = 0.0
    vh.enter(DroneState(now_sim=t))
    x = v = v_meas = f_act = 0.0
    kicked = False
    peaks, cyc, cyc_x0, cyc_pk, vmax_c = [], None, 0.0, 0.0, 0.0
    t_end = GUST['at'] + GUST['every'] * cycles
    while t < t_end:
        t += DT
        s = DroneState(now_sim=t, vins_valid=True, vins_x=x, vins_y=0.0, vins_yaw=0.0,
                       vins_vx=v_meas, vins_vy=0.0, pilot_roll=RC_CENTER, pilot_pitch=RC_CENTER)
        rc = vh.update(s, Setpoint(), DT)
        cmd = rc.pitch - RC_CENTER
        f_act += (cmd - f_act) * (1 - math.exp(-DT / TAU_ACT))
        wind = BASE_PWM + gust_pwm * gust_env(t)
        v += (wind - f_act) * ALPHA * DT
        x += v * DT
        v_meas += (v - v_meas) * (1 - math.exp(-DT / TAU_MEAS))
        if not kicked and t > 15.0:          # толчок → первый гвоздь до порывов
            v += 0.5
            kicked = True
        if t >= GUST['at']:
            k = int((t - GUST['at']) // GUST['every'])
            if k != cyc:
                if cyc is not None:
                    peaks.append((cyc_pk, abs(x - cyc_x0), vmax_c))
                cyc, cyc_x0, cyc_pk, vmax_c = k, x, 0.0, 0.0
            cyc_pk = max(cyc_pk, abs(x - cyc_x0))
            vmax_c = max(vmax_c, abs(v))
    if cyc is not None and cyc < cycles:       # незакрытый последний цикл (граница)
        peaks.append((cyc_pk, abs(x - cyc_x0), vmax_c))
    return peaks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gust', type=float, default=75.0, help='сила порыва, PWM-экв. (75 ≈ 8 м/с)')
    ap.add_argument('--ki', default='6,15,20,30,45,60')
    ap.add_argument('--kp', default='40,32', help='kp_fwd,kp_lat')
    ap.add_argument('--vsmooth', type=float, default=0.3)
    ap.add_argument('--imax', type=float, default=120.0)
    a = ap.parse_args()
    kp_fwd, kp_lat = (float(x) for x in a.kp.split(','))
    print(f"плант: α {ALPHA} м/с²/PWM, τ_act {TAU_ACT} с, τ_meas {TAU_MEAS} с; порыв {a.gust:g} PWM-экв. "
          f"({GUST['rise']:g}/{GUST['hold']:g}/{GUST['fall']:g} с каждые {GUST['every']:g}); "
          f"kp {kp_fwd:g}/{kp_lat:g} vsmooth {a.vsmooth:g} imax {a.imax:g}")
    print(f"{'ki':>5} {'пик,м':>6} {'остаток,м':>9} {'vmax':>5} | {'1-й цикл пик':>12}")
    for ki in (float(x) for x in a.ki.split(',')):
        p = run(ki, a.gust, kp_fwd, kp_lat, a.vsmooth, a.imax)
        ss = p[1:]
        print(f"{ki:5g} {sum(q[0] for q in ss) / len(ss):6.2f} {sum(q[1] for q in ss) / len(ss):9.2f} "
              f"{max(q[2] for q in ss):5.2f} | {p[0][0]:12.2f}")


if __name__ == '__main__':
    main()
