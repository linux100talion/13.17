#!/usr/bin/env python3
"""release_stop.py — СТОП ПОСЛЕ ОТПУСКАНИЯ СТИКА: выбег до гвоздя по фазе станции (brk= статуса).

Повод — полёт lv2_joy_20260906_160730 (cmd/7): «DpHold при отпускании на ходу ведёт себя отлично,
DpVins плывёт». По каждому отпусканию (переход rel → не-rel в brk=): ярус, стик перед отпусканием,
|v| истины при отпускании, время и путь до гвоздя (первый кадр без set), |v| при гвозде,
максимальный уход от точки гвоздя за 5 с, изменение курса за 3 с до отпускания (вираж).
«ГВОЗДЬ НЕ ВЗЯТ» — пилот взял стик раньше, чем станция встала (или 20 с). 160730: DpHold с
4–4.7 м/с — 1.5–2.4 с, 3–5 м; DpVins с 5 м/с — 5–12 с, 7–16 м, на виражах гвоздь не брался
8–11 с → кандидат settle_brake + pin_t (cmd/8). Запуск внутри nav:
  docker exec p1317_nav bash -lc "source /opt/ros/humble/setup.bash; \\
    source /root/sim_ws/install/setup.bash; \\
    python3 /lab/release_stop.py /root/sim_ws/output/joystick/<RUN>"
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gust_hold_compare import latest, load, resolve      # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('--after', type=float, default=5.0, help='окно ухода после гвоздя, с')
    a = ap.parse_args()
    label, bag, _ = resolve(a.run)
    S, T, V = load(bag)
    if not any('brk' in d for d in S):
        sys.exit(f"{label}: поля brk= в статусе нет (прогон до 2026-09-06?)")
    TT = [r[0] for r in T]
    print(f"# {label}: отпускание стика → гвоздь (истина Gazebo)")
    print("  t_rel   ярус  стик rcp/rcr  |v|rel  до гвоздя  путь,м  |v|гв  уход за окно  dyaw3s")
    prev = prev_d = None
    for i, d in enumerate(S):
        if 'brk' not in d:
            continue
        ph = d['brk'].split('/')
        if prev is not None and 'rel' in prev and 'rel' not in ph:
            t0 = d['t']; tr0 = latest(TT, T, t0); trm = latest(TT, T, t0 - 3.0)
            dyaw = ((tr0[6] - trm[6] + 540) % 360 - 180) if (tr0 and trm) else float('nan')
            tp = nxt = None
            for e in S[i:]:
                if 'brk' not in e or e['t'] - t0 > 20:
                    break
                q = e['brk'].split('/')
                if 'rel' in q:
                    nxt = e['t']; break
                if 'set' not in q:
                    tp = e['t']; break
            stick = f"{prev_d.get('rcp', '-')}/{prev_d.get('rcr', '-')}"
            if tp is None:
                why = f"новый стик через {nxt - t0:.1f} с" if nxt else "20 с"
                print(f"  {t0:7.2f}  {d.get('tier', '-'):>3}  {stick:>10}  {tr0[4]:5.2f}  ГВОЗДЬ НЕ ВЗЯТ ({why})  dyaw {dyaw:+5.0f}°")
            else:
                trp = latest(TT, T, tp)
                dist = math.hypot(trp[1] - tr0[1], trp[2] - tr0[2])
                seg = [r for r in T if tp <= r[0] <= tp + a.after]
                dmax = max(math.hypot(r[1] - trp[1], r[2] - trp[2]) for r in seg) if seg else float('nan')
                print(f"  {t0:7.2f}  {d.get('tier', '-'):>3}  {stick:>10}  {tr0[4]:5.2f}  {tp - t0:7.1f} с  {dist:6.2f}  {trp[4]:5.2f}  {dmax:6.2f}        {dyaw:+5.0f}°")
        prev, prev_d = ph, d


if __name__ == '__main__':
    main()
