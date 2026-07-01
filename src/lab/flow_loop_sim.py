#!/usr/bin/env python3
"""СИНТЕТИКА B — детерминированный симулятор замкнутого БОКОВОГО контура демпфера.

Чистая numpy-ОДУ боковой оси + ТОТ ЖЕ закон PID, что в alt_hold_bootstrap.py
(_on_flow_image, строки 260-271). Позволяет тюнить kp/ki/kd/τ за миллисекунды,
ДЕТЕРМИНИРОВАННО, без дорогого (RTF 0.07, ~5 мин) и НЕдетерминированного полного
прогона SITL+Gazebo. Инструмент разрывает тупик тюнинга: находим устойчивый режим
в surrogate → подтверждаем 1-2 реальными прогонами.

  Плант:   v̇ = k·roll_off + d(t)     ẏ = v        (roll PWM → боковое ускорение)
  Сенсор:  flow = s·v(t−τ) + шум,  сэмплится на f_cam (кадры)
  PID:     err=flow; i=clip(i+ki·err·dt,±imax); dterm=kd·(err−prev)/dt
           u=clip(kp·err+i+dterm,±max); roll_off=osign·blend·u   ← 1:1 с нодой

Параметры плана (k,s,τ,d) — ГРУБЫЕ оценки из O2-бэга (kp4: roll_off≈20 при v≈5 м/с,
унос до 6 м/с за ~10с). Точная калибровка (system-ID из бэга) — слой C, следующий
шаг; сейчас B ценен качественно: воспроизводит ki-раскачку (H1), показывает ПОТОЛОК
kp при задержке τ и компромисс остаточной скорости.

Запуск (host или контейнер, нужен только numpy):
  python3 src/lab/flow_loop_sim.py                 # один прогон дефолт (kp8 ki0)
  python3 src/lab/flow_loop_sim.py --sweep         # сетка kp×ki → RMS_v/устойчивость
  python3 src/lab/flow_loop_sim.py --kp 8 --ki 2   # воспроизвести H1 (ki-раскачка)
"""
import argparse
import numpy as np


class FlowPID:
    """Закон управления 1:1 с alt_hold_bootstrap.py:_on_flow_image (260-271)."""
    def __init__(self, a):
        self.a = a
        self.i = 0.0
        self.prev_err = 0.0

    def step(self, flow, dt, blend=1.0):
        a = self.a
        err = flow - 0.0
        self.i = float(np.clip(self.i + a.ki * err * dt, -a.imax, a.imax))
        d = a.kd * (err - self.prev_err) / max(1e-3, dt)
        self.prev_err = err
        u = float(np.clip(a.kp * err + self.i + d, -a.max, a.max))
        return a.osign * blend * u


def simulate(a):
    """Интегрирует замкнутый контур. → dict(t, y, v, roll_off, flow) массивы + метрики."""
    dt_fine = 1.0 / a.rate_sim              # шаг интегрирования плана
    dt_cam = 1.0 / a.f_cam                  # интервал кадра (обновление регулятора)
    n = int(a.dur * a.rate_sim)
    delay_steps = max(0, int(round(a.tau * a.rate_sim)))

    v = 0.0
    y = 0.0
    roll_off = 0.0
    pid = FlowPID(a)
    vbuf = [0.0] * (delay_steps + 1)        # линия задержки скорости (для flow)
    rng = np.random.default_rng(a.seed)

    ts = np.empty(n); ys = np.empty(n); vs = np.empty(n)
    ros = np.empty(n); fls = np.empty(n)
    next_cam = 0.0
    for k in range(n):
        t = k * dt_fine
        # возмущение: постоянный lean-снос + опц. медленная синусоида
        d = a.dist + a.dist_sine * np.sin(2 * np.pi * a.dist_freq * t)
        # плант
        v += (a.k * roll_off + d) * dt_fine
        y += v * dt_fine
        vbuf.append(v); vbuf.pop(0)
        # сенсор+регулятор на частоте кадров
        if t >= next_cam:
            v_delayed = vbuf[0]
            # flow = s·v + БИАС(LK) + шум. Биас b — DC-оффсет: ∫flow=s·y+b·t →
            # член b·t разгоняет интеграл (windup) → механизм ki-раскачки (FAQ §ki=0).
            flow = a.s * v_delayed + a.flow_bias + rng.normal(0.0, a.noise)
            roll_off = pid.step(flow, dt_cam, blend=a.blend)
            next_cam += dt_cam
            fls[k] = flow
        else:
            fls[k] = fls[k - 1] if k else 0.0
        ts[k] = t; ys[k] = y; vs[k] = v; ros[k] = roll_off

    # метрики по «безопасному окну» (как drift_check): пропускаем warm-up
    m = ts >= a.warmup
    idx = np.where(m)[0]
    rms_v = float(np.sqrt(np.mean(vs[m] ** 2)))
    max_y = float(np.max(np.abs(ys[m])))
    v_term = float(np.mean(np.abs(vs[idx[-max(1, len(idx) // 10):]])))  # терминальная |v|
    # УСТОЙЧИВОСТЬ — по СКОРОСТИ, не по позиции (velocity-демпфер: y дрейфует линейно
    # by-design при любой ненулевой терминальной v — это НЕ неустойчивость). Смотрим
    # ОГИБАЮЩУЮ max|v| по третям окна (робастно к осцилляции через ноль): растёт → расход.
    t3 = np.array_split(idx, 3)
    a1 = float(np.max(np.abs(vs[t3[0]]))); a3 = float(np.max(np.abs(vs[t3[2]])))
    growth = a3 / max(1e-6, a1)
    if growth > 1.3:
        verdict = "РАСХОДИТСЯ"   # |v| растёт → положит. ОС / windup
    elif growth < 0.8:
        verdict = "затухает"
    else:
        verdict = "устойч."      # |v| вышла на плато (терминальная скорость)
    return dict(t=ts, y=ys, v=vs, roll_off=ros, flow=fls,
                rms_v=rms_v, max_y=max_y, v_term=v_term, growth=growth, verdict=verdict)


def print_run(a, r):
    print(f"kp={a.kp:g} ki={a.ki:g} kd={a.kd:g} osign={a.osign:+g} τ={a.tau:g}s  "
          f"→ RMS_v={r['rms_v']:.2f} м/с  v_терм={r['v_term']:.2f} м/с  max|y|={r['max_y']:.1f} м  "
          f"рост|v|×{r['growth']:.2f}  [{r['verdict']}]")
    # компактная трасса (10 точек)
    n = len(r['t']); step = max(1, n // 10)
    print("     t/y/v/roll_off:", " ".join(
        f"{r['t'][i]:.0f}s:{r['y'][i]:+.0f}/{r['v'][i]:+.1f}/{r['roll_off'][i]:+.0f}"
        for i in range(0, n, step)))


def sweep(a):
    kps = [float(x) for x in a.sweep_kp.split(',')]
    kis = [float(x) for x in a.sweep_ki.split(',')]
    print(f"СВИП kp×ki (osign={a.osign:+g}, τ={a.tau:g}s, dist={a.dist:g}):")
    hdr = 'kp\\ki'
    print(f"{hdr:>7}", *[f"{ki:>10g}" for ki in kis])
    for kp in kps:
        row = []
        for ki in kis:
            a.kp, a.ki = kp, ki
            r = simulate(a)
            row.append(f"{r['rms_v']:.2f}/{r['verdict'][:4]}")
        print(f"{kp:>7g}", *[f"{c:>10}" for c in row])
    print("  ячейка = RMS_v(м/с)/устойчивость. Ниже RMS_v лучше; 'РАСХ' = раскачка.")


def build_args():
    p = argparse.ArgumentParser(description="СИНТЕТИКА B — симулятор бокового контура демпфера")
    # PID (дефолты — как в alt_hold_bootstrap.py, но ki=0: устойчивая база)
    p.add_argument('--kp', type=float, default=8.0)
    p.add_argument('--ki', type=float, default=0.0)
    p.add_argument('--kd', type=float, default=0.0)
    p.add_argument('--imax', type=float, default=120.0)
    p.add_argument('--max', type=float, default=150.0)
    p.add_argument('--osign', type=float, default=1.0)
    p.add_argument('--blend', type=float, default=1.0, help='confidence-fade (1=полный)')
    # плант (ГРУБЫЕ оценки из O2 — калибровать слоем C)
    # ЗНАК k: отрицательный → положительный roll_off ТОРМОЗИТ положительную скорость,
    # т.е. osign=+1 = демпфер (подтверждено реальностью). Тогда osign=-1 (O1) → разгон.
    p.add_argument('--k', type=float, default=-0.010, help='м/с² на 1 PWM roll_off (знак: см. код)')
    p.add_argument('--s', type=float, default=1.0, help='flow(px/кадр) на 1 м/с боковой')
    p.add_argument('--tau', type=float, default=0.15, help='задержка контура, с')
    p.add_argument('--dist', type=float, default=0.6, help='возмущение (lean-снос), м/с²')
    p.add_argument('--dist-sine', dest='dist_sine', type=float, default=0.0)
    p.add_argument('--dist-freq', dest='dist_freq', type=float, default=0.05)
    p.add_argument('--noise', type=float, default=0.3, help='шум потока, px (СКО)')
    p.add_argument('--flow-bias', dest='flow_bias', type=float, default=0.0,
                   help='DC-биас потока LK, px → windup интеграла (механизм ki-раскачки)')
    # интегрирование
    p.add_argument('--f-cam', dest='f_cam', type=float, default=30.0, help='частота кадров, Гц')
    p.add_argument('--rate-sim', dest='rate_sim', type=float, default=200.0)
    p.add_argument('--dur', type=float, default=80.0, help='длительность, с (устойч. случаи выходят на плато)')
    p.add_argument('--warmup', type=float, default=5.0, help='пропуск warm-up в метриках, с')
    p.add_argument('--seed', type=int, default=1317)
    # свип
    p.add_argument('--sweep', action='store_true')
    p.add_argument('--sweep-kp', dest='sweep_kp', default='2,4,8,16,32')
    p.add_argument('--sweep-ki', dest='sweep_ki', default='0,0.5,2')
    return p.parse_args()


def main():
    a = build_args()
    print("СИНТЕТИКА B — замкнутый боковой контур (детерминированный surrogate)")
    print(f"плант: k={a.k:g} s={a.s:g} τ={a.tau:g}s dist={a.dist:g} noise={a.noise:g} "
          f"f_cam={a.f_cam:g}Гц\n")
    if a.sweep:
        sweep(a)
    else:
        print_run(a, simulate(a))
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
