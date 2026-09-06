#!/usr/bin/env python3
"""dpvins_gust_stand.py — СТЕНД DpVins на висении под порывами: свип ki (и kp) на планте
проекта без полётов. Ответ на вопрос серии dphold_vs_dpvins 2026-09-05: DpVins пропускал
порыв 8 м/с на 6.3–9.5 м против 2.4–2.8 у DpHold — сколько из этого лечится тримом (ki),
не трогая kp (kp понижен до 40/32 ради σθ 0.5°, кампания ab_dpv_*).

Плант — как в test_dpvins §13: v̇ = α·(ветер − наклон), α = 0.01 м/с² на PWM (100 PWM =
1 м/с²; ветер 10 м/с ≈ 100 PWM-экв., 5 ≈ 44), наклон исполняется апериодикой τ 0.26 с
(идентифицированный лаг FCU, модель BS_GZ_KD), скорость — с лагом 0.35 с (ИЗМЕРЕНО по bag lv2_joy_20260905_184557: spd= статуса
отстаёт от истины на 0.35 с — VinsTrack EMA a=0.4 на 10 Гц + приход; до полёта стенд брал
0.15 и льстил: обещал запас до ki 45, а ki 30 раскачался в полёте). Порыв — огибающая wind_gust.py (1−cos фронт 2 с, плато 5, спад 4,
период 20) с силой GUST PWM-экв.; калибровка: при ki 6 пик за цикл должен выйти 6–9 м и
vmax 1.4–1.9, как в полётах → GUST 75 (порыв 8 м/с). До порывов борт получает толчок,
чтобы связался первый гвоздь (в полёте — посев трима от демпфера + первый стоп).

Метрики по циклам без первого (установившийся режим): пик смещения от точки на фронте,
остаток в конце цикла, vmax; плюс первый цикл отдельно (обучение трима под порывом).
Итог 2026-09-05 с τ_meas 0.35 (kp 40/32, vsmooth 0.3, imax 120, GUST 75):
  ki   6: пик 9.61 м, остаток 0.22, vmax 1.84   — эталон, совпал с полётами (6.3–9.5)
  ki  10: пик 8.60, остаток 0.07
  ki  15: пик 4.26, остаток 0.05, vmax 1.77     — устойчив и при τ 0.5 (4.43 / 0.11)
  ki  20: пик 4.81, остаток 0.16                — на грани (при τ 0.5: 5.63 / 0.21)
  ki  30: пик 5.52, остаток 1.87, vmax 2.33     — РАСКАЧКА; полёт 184557 (cmd/1): пик 5.15,
                                                  остаток 2.65, vmax 2.5, T 8–10 с — совпало
  (при старом τ 0.15: ki 30 → 3.64 / 0.22, раскачка только с ki 45 — стенд льстил.)
ФАЗА BRAKE (DpVins.brake, закон станции демпфера; τ 0.35, ki 15, kp 40/32):
  brake_vmax 1.0 (кап демпфера): brake 3/5/8 → пик 5.4–5.5, остаток 1.5–1.6 — кап режет
      вклад тормоза до kp·1.0 = 40 PWM, равновесие сноса 75 PWM-экв. остаётся ~0.9 м/с;
  brake_vmax 2.0: brake 3 → 3.76 / 0.55;  brake 5 → 3.43 / 0.13;  brake 8 → 3.21 / 0.04
      (при τ 0.11: 3.62 / 3.20 / 2.99) — vmax сноса ~1.0, устойчиво;
  brake_vmax 3.0: brake 5/8 → vmax 1.4–1.8, остаток 0.9–1.2 — раскачка, кап 3.0 лишний.
  Арифметика: демпфер держит снос на 0.21 м/с, потому что kp 90 × (1+3) = 360 PWM/(м/с);
  у DpVins kp 40 → нужен brake 5–8 (240–360) и кап ≥ 2.0, чтобы тормоз не обрезался.
  Для масштаба закон демпфера как есть (kp 90 ki 30 brake 3, τ 0.35): 2.24 / 0.06.
  ⚠️ Стенд не видит предельный цикл σθ с контуром ориентации FCU — гейн ×6–9 на
  торможении может его возбудить; это только полётом (cmd/3).
ЗАПИРАНИЕ BRAKE (cmd_3/wind_right/1, --trim0 −56 = ошибочный трим ПО ветру после гвоздя;
  боковая ось --kp 32,32, brake 5 кап 2, ki 15, τ 0.35): тормоз 32·(1.7+2) = 118 PWM не
  перекрывает ветер 75 + трим 56 → снос не разворачивается, фаза не выходит, трим заморожен:
  brake_t 0 → 21 м без возврата (полёт: 16.7 м/цикл, 46 м); brake_t 4 → 6.0/3.5; 6 → 4.6/1.2;
  8 → 4.0/0.11. Штатно на kp 32: brake_t 0 → 4.28/0.97, 8 → 4.34/0.55, 4–6 хуже (трим учит
  порыв на середине → перелёт). Дефолт dpvins_pos_brake_t 8. На kp 40 запирания нет
  (240 > 131) — авторитет решает, поэтому боковая ось 32 уязвимее продольной 40.
ХВОСТ БРЕЙКА КАК У ДЕМПФЕРА (2026-09-06, --brake-t -1: трим в брейке НЕ заморожен, стоит
  только анти-виндап в упоре — у демпфера таймера нет, его заморозка = насыщение при |v| >
  0.42 м/с). kp 32/32 brake 5 кап 2, τ 0.35 (пик/остаток/vmax):
  ki 15 → 5.57/1.90/2.48 РАСКАЧКА (в брейке ошибка ×6 → эффективный ki 90); ki 13 → 4.02/0.70;
  ki 12 → 3.83/0.54; ki 10 → 2.28/0.03/1.06; ki 8 → 2.45/0.04/1.30 — уровень демпфера (2.24).
  Механика: трим берёт порыв за секунды = feed-forward ветра; с заморозкой порыв 11 с держит
  один П-канал 192 PWM/(м/с) — равновесный снос 0.4 м/с. Запас: τ 0.5 → ki 10 3.77/0.59,
  ki 8 2.40/0.09 (ki 8 переживает лаг); τ 0.11 (twist) → ki 12 2.08/0.06, ki 15 3.84/0.27 —
  twist расширяет окно ki ~на 20 %, пик сам не улучшает. Запирание (trim0 −56) ki 10 →
  3.28/0.09 (замок невозможен: он живёт в ненасыщенном упоре, где трим учится). kp 40 ki 10
  → 1.82/0.01. Порыв 100 PWM-экв. (10 м/с) ki 10 → 4.23/0.01 против 7.93/4.05 у brake_t 8
  ki 15; порыв 44 (5 м/с) 2.47/0.67 против 2.90/0.74. Кандидат dpvins/brake5_tail.txt
  (ki 8, brake_t −1), полёт cmd/4.
Стенд не знает шума VINS и предельного цикла kp против контура ориентации FCU (Tθ 1.6 с) —
σθ он не предскажет, это только полёт. Запуск с хоста (ROS не нужен):
  python3 src/lab/dpvins_gust_stand.py [--gust 75] [--ki 6,15,30] [--kp 40,32] \
      [--brake 0,1.5,3] [--brake-vmax 2] [--brake-t 8|-1] [--trim0 -56] [--tau-meas 0.35]
  # brake — фаза BRAKE станции; trim0 — ошибочный трим после гвоздя (тест запирания)
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
TAU_MEAS = 0.35       # лаг скорости VINS в ноде к истине, ИЗМЕРЕН по bag 184557 (кросс-
                      # корреляция spd= статуса vs истина Gazebo); было 0.15 — стенд льстил
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


def run(ki, gust_pwm, kp_fwd=40.0, kp_lat=32.0, vsmooth=0.3, imax=120.0, cycles=6,
        brake=0.0, brake_v=0.25, brake_vmax=1.0, brake_t=0.0, trim0=None):
    vh = DpVins(kp_fwd=kp_fwd, kp_lat=kp_lat, ki=ki, ki_trim=60.0, imax=imax, max_pwm=150.0,
                cmd_gain=4.0, pos_kp=0.3, pos_vmax=0.3, pos_acc=0.15, vsmooth=vsmooth,
                i_latch=True, brake=brake, brake_v=brake_v, brake_vmax=brake_vmax,
                brake_t=brake_t)
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
        if trim0 is not None and 24.9 < t <= 24.95 and vh._pinx is not None:
            vh._itx = trim0               # ОШИБОЧНЫЙ трим после гвоздя (запирание BRAKE)
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
    ap.add_argument('--brake', default='0', help='фаза BRAKE станции: список, напр. 0,1.5,3')
    ap.add_argument('--brake-v', type=float, default=0.25)
    ap.add_argument('--brake-vmax', type=float, default=1.0)
    ap.add_argument('--tau-meas', type=float, default=None, help='лаг измерения, с (дефолт TAU_MEAS)')
    ap.add_argument('--brake-t', type=float, default=0.0, help='заморозка трима первые N с брейка (0 = всю фазу, <0 = не морозить: правило демпфера)')
    ap.add_argument('--trim0', type=float, default=None, help='ошибочный трим PWM, вписанный после гвоздя (тест запирания)')
    a = ap.parse_args()
    global TAU_MEAS
    if a.tau_meas is not None:
        TAU_MEAS = a.tau_meas
    kp_fwd, kp_lat = (float(x) for x in a.kp.split(','))
    print(f"плант: α {ALPHA} м/с²/PWM, τ_act {TAU_ACT} с, τ_meas {TAU_MEAS} с; порыв {a.gust:g} PWM-экв. "
          f"({GUST['rise']:g}/{GUST['hold']:g}/{GUST['fall']:g} с каждые {GUST['every']:g}); "
          f"kp {kp_fwd:g}/{kp_lat:g} vsmooth {a.vsmooth:g} imax {a.imax:g} τ_meas {TAU_MEAS:g}; "
          f"brake_v {a.brake_v:g} brake_vmax {a.brake_vmax:g} brake_t {a.brake_t:g} trim0 {a.trim0}")
    print(f"{'brake':>5} {'ki':>5} {'пик,м':>6} {'остаток,м':>9} {'vmax':>5} | {'1-й цикл пик':>12}")
    for br in (float(x) for x in a.brake.split(',')):
        for ki in (float(x) for x in a.ki.split(',')):
            p = run(ki, a.gust, kp_fwd, kp_lat, a.vsmooth, a.imax,
                    brake=br, brake_v=a.brake_v, brake_vmax=a.brake_vmax,
                    brake_t=a.brake_t, trim0=a.trim0)
            ss = p[1:]
            print(f"{br:5g} {ki:5g} {sum(q[0] for q in ss) / len(ss):6.2f} "
                  f"{sum(q[1] for q in ss) / len(ss):9.2f} {max(q[2] for q in ss):5.2f} | "
                  f"{p[0][0]:12.2f}")


if __name__ == '__main__':
    main()
