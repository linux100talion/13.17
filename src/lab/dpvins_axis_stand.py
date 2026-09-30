#!/usr/bin/env python3
"""dpvins_axis_stand.py — СТЕНД БОКОВОЙ ОСИ DpVins: ШТАТНЫЙ объект на модели борта, без полётов.

ЗАЧЕМ (2026-09-30). roll_big.json на VinsHold: стик крена 0.6 × 3 с давал +1.4/+0.3/0/0 м под
порывами бокового ветра и +3.9/−2.9/+2.1/−2.1 м без них, демпфер — 4–8 м; после отпускания
боковая ось качается ±0.6 м/с. Старый стенд (dpvins_gust_stand.py) моделирует только ПРОДОЛЬНУЮ
ось, собирает DpVins руками (не штатные ручки) и держит модель борта старого сима. Здесь:
  * DpVins — ШТАТНЫЙ: mission_pkg.recipes.build_vins_stab(BootstrapConfig.baseline(**ручки),
    WindTrim) — ровно тот объект и те ручки профилей, что летают (dpvins/brake5_stop…), с общим
    ветровым тримом; перебор — переопределением полей конфига (--set dpvins_kp_lat_deg=1.6 …);
  * боковая ось: курс 0 → «влево» = ENU y, команда — rc.roll (вправо +);
  * модель борта — СНЯТАЯ по полёту (plant_id по rollbig_rc_nogust_20260930_173309, R² 1.00):
    ускорение на PWM α, апериодика наклона τ_act, чистое запаздывание команды delay, вязкое
    сопротивление β; сила ветра в PWM-экв. (= трим, который её держит): база 2 м/с → 29 PWM
    (wnr= полёта без порывов), порыв до 5 м/с → 72 (сила ветра в Gazebo линейна по скорости);
  * измерение — twist VINS: 10 Гц + апериодика τ_meas;
  * сценарий roll_big: висение, потом крен вправо/влево 0.6 стика × 3 с, паузы 8 с, две пары.
Метрики — как у полётного разбора (edges4/latosc): смещение за стик + 1 с, медиана |v| при стике
в центре, размах качания после отпускания.

  python3 src/lab/dpvins_axis_stand.py                       # борт сегодня, без порывов
  python3 src/lab/dpvins_axis_stand.py --gust 43             # + порывы до 5 м/с
  python3 src/lab/dpvins_axis_stand.py --set dpvins_kp_lat_deg=1.6 dpvins_ki_deg=0.24
  python3 src/lab/dpvins_axis_stand.py --plant old           # модель старого сима (30°, TC 0.1)
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "control"))
sys.path.insert(0, os.path.join(HERE, "..", "mission"))
from control_pkg.domain.rc import RC_CENTER                          # noqa: E402
from control_pkg.domain.setpoint import Setpoint                     # noqa: E402
from control_pkg.domain.state import DroneState                      # noqa: E402
from control_pkg.domain.units import us_from_tilt                    # noqa: E402
from mission_pkg.config import BootstrapConfig                       # noqa: E402
from mission_pkg.recipes import build_vins_stab, build_wind_trim      # noqa: E402

DT = 0.02
# модели борта: α м/с² на PWM, τ_act с, delay с, β 1/с (plant_id 2026-09-30)
PLANTS = {
    'board': dict(alpha=0.00865, tau_act=0.45, delay=0.24, beta=-0.135),   # 20°, ATC_INPUT_TC 0.3
    'old':   dict(alpha=0.01, tau_act=0.26, delay=0.0, beta=0.0),          # стенд до 2026-09-30
}
TAU_MEAS = 0.11           # twist VINS (vins/scale25: BS_VINS_VEL_SRC=twist)
MEAS_HZ = 10.0
BASE_PWM = 29.0           # ветер 2 м/с бокового направления, PWM-экв. (wnr полёта без порывов)
GUST = dict(at=20.0, rise=2.0, hold=5.0, fall=4.0, every=20.0)
# roll_big: (старт, уровень c_right) — 0.6 стика 3 с, паузы 8 с
PULSES = [(30.0, +0.6), (41.0, -0.6), (52.0, +0.6), (63.0, -0.6)]
PULSE_DUR = 3.0
T_END = 76.0


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


def stick_at(t):
    for t0, c in PULSES:
        if t0 <= t < t0 + PULSE_DUR:
            return c
    return 0.0


def run(overrides, plant, gust_pwm=0.0, base_pwm=BASE_PWM, tau_meas=TAU_MEAS):
    cfg = BootstrapConfig.baseline(**overrides)
    wind = build_wind_trim(cfg)
    st = build_vins_stab(cfg, wind)
    p = PLANTS[plant] if isinstance(plant, str) else plant
    nd = max(0, int(round(p['delay'] / DT)))
    queue = [0.0] * (nd + 1)
    t = 0.0
    y = v = f = 0.0                       # «влево» (ENU y), м и м/с; f — исполненный наклон, PWM
    vm, ym, next_meas = 0.0, 0.0, 0.0
    st.enter(DroneState(now_sim=t, vins_valid=True))
    log = []                               # (t, y, v, cmd, stick)
    while t < T_END:
        t += DT
        if t >= next_meas:                 # VINS 10 Гц: двигаем измерение
            next_meas += 1.0 / MEAS_HZ
            vm += (v - vm) * (1 - math.exp(-(1.0 / MEAS_HZ) / tau_meas))
            ym = y
        c = stick_at(t)
        s = DroneState(now_sim=t, vins_valid=True, vins_x=0.0, vins_y=ym, vins_yaw=0.0,
                       att_yaw=0.0, vins_vx=0.0, vins_vy=vm,
                       pilot_roll=RC_CENTER, pilot_pitch=RC_CENTER)
        rc = st.update(s, Setpoint(c_right=c), DT)
        queue.append(us_from_tilt(rc.roll))   # команда СИ → µs-экв. (модель борта в PWM)
        u = queue.pop(0)                   # чистое запаздывание команды
        f += (u - f) * (1 - math.exp(-DT / p['tau_act']))
        w = base_pwm + gust_pwm * gust_env(t)
        v += (p['alpha'] * (w - f) + p['beta'] * v) * DT
        y += v * DT
        if t > 12.0 and t < 12.0 + DT * 1.5:
            v += 0.4                       # толчок → гвоздь по стопу (в полёте — вход в ярус)
        log.append((t, y, v, u, c))
    return log


def metrics(log):
    out = []
    for t0, c in PULSES:
        a = [r for r in log if t0 <= r[0] < t0 + PULSE_DUR + 1.0]
        out.append(-(a[-1][1] - a[0][1]))          # вправо + (как в полётном разборе)
    calm = sorted(abs(r[2]) for r in log if r[0] > 20.0 and r[4] == 0.0)
    swing = []
    for t0, _ in PULSES:                            # размах после отпускания (пауза 8 с)
        seg = [r[2] for r in log if t0 + PULSE_DUR + 1.0 <= r[0] < t0 + PULSE_DUR + 8.0]
        swing.append(max(seg) - min(seg))
    return out, calm[len(calm) // 2], sum(swing) / len(swing)


def parse_set(items):
    ov = {}
    for it in items or []:
        k, _, v = it.partition('=')
        ov[k.strip()] = float(v)
    return ov


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plant', default='board', choices=sorted(PLANTS))
    ap.add_argument('--gust', type=float, default=0.0, help='прибавка порыва, PWM-экв. (43 = до 5 м/с)')
    ap.add_argument('--base', type=float, default=BASE_PWM)
    ap.add_argument('--tau-meas', type=float, default=TAU_MEAS)
    ap.add_argument('--delay-scale', type=float, default=1.0, help='множитель запаздываний (запас)')
    ap.add_argument('--set', nargs='*', help='переопределить поля BootstrapConfig: dpvins_kp_lat_deg=1.6 …')
    a = ap.parse_args()
    plant = dict(PLANTS[a.plant])
    plant['tau_act'] *= a.delay_scale
    plant['delay'] *= a.delay_scale
    ov = parse_set(a.set)
    log = run(ov, plant, a.gust, a.base, a.tau_meas)
    dist, calm, swing = metrics(log)
    print(f"модель {a.plant} (α {plant['alpha']}, τ {plant['tau_act']:.2f}, delay {plant['delay']:.2f}), "
          f"ветер {a.base:g}+{a.gust:g} PWM, ручки {ov or 'штатные'}")
    print("  стик 0.6×3 с, вправо +: " + " / ".join(f"{d:+.1f}" for d in dist)
          + f" м | медиана |v| в центре {calm:.2f} м/с | размах после отпускания {swing:.2f} м/с")


if __name__ == '__main__':
    main()
