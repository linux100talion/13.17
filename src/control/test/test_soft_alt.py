#!/usr/bin/env python3
"""Юнит-тест МЯГКОСТИ демпфера — по ВЫСОТЕ (_IpmGated.soft_alt) и по ИЗМЕРЕННОМУ ШУМУ
канала (soft_noise): демпфер и станция тангажа не дерутся с шумом канала.

Зачем. Полёт lv2_joy_20260829_182126: на 8.3 м канал вида сверху показывает скорость
вперёд с разбросом ±1 м/с при плавной истине (шум пути за кадр 14 → 95 мм на 0.3 → 8.3
м, ab_soft: 231 мм на 17.5 м — тайминг углов × рычаг полосы ∝ h). Брейк станции будился
шумом (порог перевзвода 0.5) 90 раз за 35 с, упор ±150 PWM, борт качало ±1.7 м/с с
периодом 4.7 с, тангаж ±11°; на 5 м — то же в зачатке.

Плант — 1D стенд станции (test_station_brake) + ШУМ ПРИРАЩЕНИЯ ПУТИ ∝ высоте, как в
полёте: путь += v·dt + N(0, σ_inc), σ_inc = 0.014 + 0.012·h м/кадр (замер: 14–19 мм у
земли, 33–43 на 5 м, 95–161 на 8 м, 231 на 17.5); скорость канала — МНК-наклон пути
по окну 9 кадров + сглаживание τ=0.4 с (как ipm_vfwd с ipm_vel_tau; шум скорости
0.2/0.5/0.7 м/с на 0.4/5/8.3 м — как в полёте 0.19/0.46/0.72); оценщик шума — как в
FlowEstimator: EMA(τ=2 с) |приращение − v̂_prev·dt| → ipm_noise_fwd. Ветер 0.65 м/с²
(52 PWM), лётный набор ручек.
Что проверяем:
1. ниже soft_alt — бит-в-бит как без ручки (soft = 1);
2. 5 и 8 м без ручки: брейк будится шумом, PWM и скорость качаются (модель
   воспроизводит полёт); с soft_alt=2: брейк шумом не будится, σ PWM ≤ 0.6 базы,
   σ скорости ниже базы (показатели kp ×√soft, ki ×soft — свип в классе);
3. трим ветра на высоте цел (И-член ≈ 52 PWM);
4. soft по высоте монотонен и ограничен снизу soft_min;
5. оценщик шума повторяет σ_inc планта (≈0.8σ, как замер реплеем 0.81–0.93);
6. мягкость ПО ШУМУ (soft_noise=0.02, высотное правило выкл): у земли soft = 1 и
   поведение как без ручки; на 5/8 м soft ≈ ref/σ̂, брейк молчит, σ PWM ≤ 0.6 базы;
   вместе с высотным правилом — минимум из двух. Порог 0.02 м/кадр ≈ 1.5× шума у
   земли (σ̂ 11–16 мм): мягчить начинает там, где шум удвоился.
Запуск:  python3 src/control/test/test_soft_alt.py
"""
import math
import os
import random
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control_pkg.domain.control.stabilization import DpPitchRate            # noqa: E402
from control_pkg.domain.rc import RC_CENTER                                 # noqa: E402
from control_pkg.domain.setpoint import Setpoint                            # noqa: E402
from control_pkg.domain.state import DroneState                             # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


ALPHA, DT, WIND, WIN = 0.0125, 1.0 / 30.0, 0.65, 9
FLIGHT = dict(kp=90.0, ki=30.0, ki_trim=60.0, kd=0.0, imax=150.0, cmd_gain=5.0,
              pos_kp=0.3, pos_vmax=0.3, pos_brake=3.0, pos_brake_vmax=1.0, pos_acc=0.15,
              anti_windup=True, pos_brake_v=0.25, max_speed=0.0, alt_band=0.0,
              arm_frames=0, pos_alt_band=0.0)


def sigma_inc(h):
    return 0.014 + 0.012 * h


def fly(ax, h, sec=40.0, tau_a=0.2, seed=1, tau_v=0.4):
    """tau_v — сглаживание скорости канала как ipm_vel_tau (0.4 с): без него МНК по 9
    кадрам даёт шум скорости 0.84/1.28 м/с на 5/8.3 м против 0.46/0.72 в полёте."""
    rng = random.Random(seed)
    v = act = x = path = vm = vraw = est = 0.0
    hist = []
    ax.enter(DroneState(flow_seq=-1))
    rows, t = [], 0.0
    for k in range(int(round(sec / DT))):
        t += DT
        inc = v * DT + rng.gauss(0.0, sigma_inc(h))
        path += inc
        hist.append((t, path))
        del hist[:-WIN]
        vm_prev = vm
        if len(hist) >= 4:
            tc = [q[0] for q in hist]; pc = [q[1] for q in hist]
            tm = sum(tc) / len(tc); pm = sum(pc) / len(pc)
            den = sum((a - tm) ** 2 for a in tc)
            vraw = sum((a - tm) * (b - pm) for a, b in zip(tc, pc)) / den
        vm += (vraw - vm) * (1.0 - math.exp(-DT / tau_v))
        est += (1.0 - math.exp(-DT / 2.0)) * (abs(inc - vm_prev * DT) - est)
        rc = ax.update(DroneState(flow_seq=k + 1, now_sim=t, flow_dt=DT, rel_alt=h,
                                  ipm_ok=True, flow_conf=0.5, ipm_vfwd=vm, ipm_fwd=path,
                                  ipm_noise_fwd=est),
                       Setpoint(), DT)
        pwm = rc.pitch - RC_CENTER
        act += (pwm - act) * (1.0 - math.exp(-DT / tau_a))
        v += (-ALPHA * act + WIND) * DT
        x += v * DT
        rows.append((t, v, vm, path, pwm, ax._i, ax._pos_brake, x, ax._soft, est))
    return rows


def metrics(rows, t0=10.0):
    """Качание = ПЕРЕМЕННАЯ часть (σ) PWM и истинной скорости после 10 с (трим ветра —
    постоянная составляющая); брейк-кадры после 10 с (первый брейк — трим)."""
    r = [q for q in rows if q[0] >= t0]
    mp = sum(q[4] for q in r) / len(r); mv = sum(q[1] for q in r) / len(r)
    pwm = math.sqrt(sum((q[4] - mp) ** 2 for q in r) / len(r))
    vr = math.sqrt(sum((q[1] - mv) ** 2 for q in r) / len(r))
    brk = sum(1 for q in r if q[6])
    xs = [q[7] for q in rows]
    vn = math.sqrt(sum((q[2] - q[1]) ** 2 for q in r) / len(r))       # шум скорости канала
    soft = sum(q[8] for q in r) / len(r); est = sum(q[9] for q in r) / len(r)
    return pwm, vr, brk, max(xs) - min(xs), rows[-1][5], soft, vn, est


print("  шум приращения пути ∝ высоте, ветер 52 PWM, 40 с висения:")
print("   h    soft_alt soft_noise | soft  PWM σ   v σ   брейк>10с  размах x  И   | шум v кан.  σ̂ м/кадр (σ_inc)")
res = {}
for h in (0.4, 5.0, 8.3):
    for sa, sn in ((0.0, 0.0), (2.0, 0.0), (0.0, 0.02), (2.0, 0.02)):
        rows = fly(DpPitchRate(**dict(FLIGHT, soft_alt=sa, soft_noise=sn)), h)
        m = metrics(rows)
        res[(h, sa, sn)] = (m, rows)
        print("   %4.1f   %3.1f     %4.2f    | %.2f  %5.0f   %.2f    %4d      %5.2f   %3.0f  |   %.2f       %.3f (%.3f)"
              % (h, sa, sn, m[5], m[0], m[1], m[2], m[3], m[4], m[6], m[7], sigma_inc(h)))

# 1. ниже soft_alt — бит-в-бит
r0, r2 = res[(0.4, 0.0, 0.0)][1], res[(0.4, 2.0, 0.0)][1]
check("0.4 м: soft_alt — soft = 1, поведение бит-в-бит с выключенной ручкой",
      all(abs(a[4] - b[4]) < 1e-9 and abs(a[5] - b[5]) < 1e-9 for a, b in zip(r0, r2)))
# 2. по высоте
for h in (5.0, 8.3):
    m0, m2 = res[(h, 0.0, 0.0)][0], res[(h, 2.0, 0.0)][0]
    check(f"{h} м без ручек: брейк будится шумом ({m0[2]} кадров после 10 с), PWM σ {m0[0]:.0f} > 50",
          m0[2] > 30 and m0[0] > 50.0)
    check(f"{h} м soft_alt=2: брейк шумом почти не будится ({m2[2]} ≤ 30 кадров ≈ 3%), PWM σ "
          f"{m2[0]:.0f} ≤ {0.65 * m0[0]:.0f}, v σ {m2[1]:.2f} < {m0[1]:.2f}",
          m2[2] <= 30 and m2[0] <= 0.65 * m0[0] and m2[1] < m0[1])
    check(f"{h} м soft_alt=2: трим ветра цел (И-член {m2[4]:.0f} ≈ {WIND / ALPHA:.0f})",
          abs(m2[4] - WIND / ALPHA) < 15.0)
# 4. soft по высоте
ax = DpPitchRate(**dict(FLIGHT, soft_alt=2.0))
sf = [ax._soft_factor(DroneState(rel_alt=h)) for h in (0.5, 2.0, 4.0, 8.0, 15.0, 40.0)]
check("soft(h): 1, 1, 0.5, 0.25, 0.13, 0.1 (пол soft_min) на 0.5/2/4/8/15/40 м: "
      + ' '.join(f'{x:.2f}' for x in sf),
      sf[0] == 1.0 and sf[1] == 1.0 and abs(sf[2] - 0.5) < 1e-9 and abs(sf[3] - 0.25) < 1e-9
      and abs(sf[4] - 2.0 / 15.0) < 1e-9 and sf[5] == 0.1)
# 5. оценщик шума
for h in (0.4, 5.0, 8.3):
    m = res[(h, 0.0, 0.0)][0]
    check(f"{h} м: оценщик σ̂ {m[7] * 1000:.1f} мм ≈ 0.8·σ_inc {0.8 * sigma_inc(h) * 1000:.1f} (±20%)",
          abs(m[7] - 0.8 * sigma_inc(h)) < 0.2 * 0.8 * sigma_inc(h))
# 6. мягкость по шуму
r3 = res[(0.4, 0.0, 0.02)][1]
check("0.4 м: soft_noise=0.02 — шум ниже порога, soft = 1, бит-в-бит с выключенной",
      all(abs(a[4] - b[4]) < 1e-9 for a, b in zip(r0, r3)))
for h in (5.0, 8.3):
    m0, m3 = res[(h, 0.0, 0.0)][0], res[(h, 0.0, 0.02)][0]
    want = 0.02 / m3[7]
    check(f"{h} м soft_noise=0.02: soft {m3[5]:.2f} ≈ ref/σ̂ {want:.2f}, брейк ≤ 45 ({m3[2]}), "
          f"PWM σ {m3[0]:.0f} ≤ {0.65 * m0[0]:.0f}, трим {m3[4]:.0f}",
          abs(m3[5] - want) < 0.15 and m3[2] <= 45 and m3[0] <= 0.65 * m0[0]
          and abs(m3[4] - WIND / ALPHA) < 15.0)
m4 = res[(8.3, 2.0, 0.02)][0]
check(f"8.3 м оба правила: soft {m4[5]:.2f} = min(высота {2.0 / 8.3:.2f}, шум {0.02 / m4[7]:.2f})",
      abs(m4[5] - min(2.0 / 8.3, 0.02 / m4[7])) < 0.1)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ МЯГКОСТЬ ПО ВЫСОТЕ/ШУМУ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
