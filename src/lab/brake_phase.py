#!/usr/bin/env python3
"""brake_phase.py — РАЗБОР ПОРЫВОВ ПО ФАЗЕ СТАНЦИИ (поля brk=/ifz= /mission/status, с 2026-09-06).

До поля фаза BRAKE в bag не была видна, разбор cmd/3–5 шёл по косвенным признакам. Здесь:
1) лента ПЕРЕХОДОВ brk=/ifz= (ярус, стик, скорость статуса и истины, трим стрелки ветра);
2) по циклам порыва (WIND_GUST из env прогона) — через сколько после фронта вошёл BRAKE,
   секунды BRAKE по осям, доля кадров с замороженным тримом в BRAKE, пик от точки фронта;
3) положение ОТНОСИТЕЛЬНО ГВОЗДЯ (а не точки фронта): DpHold — из полей рамы sx/sy vs
   spx/spy; DpVins — гвоздь в bag не пишется, берём VINS-позу первого кадра hold в сегменте
   яруса 1 (гвоздь живёт до стика; при стике сегмент режется). Проекция на ось ветра
   (--wind-deg, дефолт 98 из wind_gust.log) в м: e(фронт), e при входе в BRAKE и скорость
   истины в этот момент, e(max), e при спаде порыва (+rise+hold+fall), минимум после
   (перелёт возврата) и e(конец).
Первый прогон с полем — lv2_joy_20260906_130326 (cmd/5 brake5_axis): DpVins в
установившемся цикле стоит на фронте 0.8 м ПРОТИВ ветра (остаток перелёта), порыв гонит
К гвоздю (фаза не away — тормоза нет), разгон до 1.2 м/с, BRAKE лишь за гвоздём (+3.5 с,
e +0.6), стоп на +2.2; после спада трим-излишек (65 → 14) гонит назад на 1.2 м/с → перелёт
до −2.1 → BRAKE; вход в ярус без гвоздя (set/set, 89–100 с) под порыв = 8 м без тормоза.
Запуск внутри nav (rosbag2_py):
  docker exec p1317_nav bash -lc "source /opt/ros/humble/setup.bash; \\
    source /root/sim_ws/install/setup.bash; \\
    python3 /lab/brake_phase.py /root/sim_ws/output/joystick/<RUN> [--wind-deg 98] [--no-trans]"
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gust_hold_compare import latest, load, parse_gust, resolve      # noqa: E402


def transitions(S, T, TT):
    print("=== ПЕРЕХОДЫ brk=/ifz= (t, ярус, brk, ifz, стик rcp/rcr, spd статуса, |v| истины, трим wnp/wnr) ===")
    prev, n = None, 0
    for d in S:
        key = (d.get('tier'), d.get('brk'), d.get('ifz'))
        if key == prev:
            continue
        tr = latest(TT, T, d['t'])
        vt = tr[4] if tr else float('nan')
        print(f"{d['t']:7.2f} tier {d.get('tier', '-'):>2} brk {d.get('brk', '--'):9} ifz {d.get('ifz', '--'):4} "
              f"rc {d.get('rcp', '-'):>4}/{d.get('rcr', '-'):>4} spd {d.get('spd', '-'):>5} vt {vt:4.2f} "
              f"trim {d.get('wnp', '-'):>4}/{d.get('wnr', '-'):>4}")
        prev, n = key, n + 1
    print(f"# переходов: {n}")


def cycles(S, T, TT, gust):
    print("\n=== ПО ЦИКЛАМ ПОРЫВА: фронт → первый BRAKE, секунды BRAKE по осям, ifz=1 в BRAKE, пик от точки фронта ===")
    k = 0
    while True:
        gs = gust['at'] + gust['every'] * k
        k += 1
        if gs > S[-1]['t']:
            break
        rows = [d for d in S if gs <= d['t'] < gs + gust['every'] and 'brk' in d]
        if len(rows) < 20:
            continue
        first = next((d['t'] for d in rows if 'brk' in d['brk']), None)
        bf = br = 0.0; nb = ifz_b = 0; pv = None
        for d in rows:
            if pv is not None:
                dt = min(d['t'] - pv['t'], 0.5)
                a, b = (pv['brk'].split('/') + ['-'])[:2]
                bf += dt if a == 'brk' else 0.0
                br += dt if b == 'brk' else 0.0
            if 'brk' in d['brk']:
                nb += 1
                ifz_b += '1' in d.get('ifz', '')
            pv = d
        tr0 = latest(TT, T, gs)
        seg = [r for r in T if gs <= r[0] < gs + gust['every']]
        pk = max(math.hypot(r[1] - tr0[1], r[2] - tr0[2]) for r in seg) if seg and tr0 else float('nan')
        vmax = max(r[4] for r in seg) if seg else float('nan')
        tiers = '/'.join(sorted({str(d.get('tier')) for d in rows}))
        print(f"фронт {gs:6.1f} ярус {tiers:>4}  BRAKE через {('%.1f с' % (first - gs)) if first else '   —   '}  "
              f"P {bf:4.1f} с  R {br:4.1f} с  ifz в BRAKE {ifz_b}/{nb}  пик {pk:4.2f} м  vmax {vmax:4.2f}")


def pin_relative(S, T, TT, V, VT, gust, wdeg):
    ux, uy = math.cos(math.radians(wdeg)), math.sin(math.radians(wdeg))
    print(f"\n=== ОТНОСИТЕЛЬНО ГВОЗДЯ: проекция (поза − гвоздь) на ось ветра {wdeg:g}° (+ = по ветру), м ===")
    # DpHold: рама станции в статусе
    print("DpHold (рама sx/sy vs spx/spy): фронт  d0  dmax @t  dend")
    k = 0
    while True:
        gs = gust['at'] + gust['every'] * k
        k += 1
        if gs > S[-1]['t']:
            break
        rows = [d for d in S if gs <= d['t'] < gs + gust['every'] and d.get('tier') == '0'
                and d.get('sf') == '1' and d.get('spx', '--') != '--']
        if len(rows) < 20:
            continue
        dd = [(d['t'], math.hypot(float(d['spx']) - float(d['sx']), float(d['spy']) - float(d['sy'])))
              for d in rows]
        tm, dm = max(dd, key=lambda x: x[1])
        print(f"  {gs:6.1f}  {dd[0][1]:4.2f}  {dm:4.2f} @+{tm - gs:4.1f}  {dd[-1][1]:4.2f}   (кадров {len(dd)})")
    # DpVins: гвоздь = VINS-поза первого hold в каждом сегменте яруса 1 без стика
    segs, cur = [], None
    for d in S:
        if d.get('tier') == '1' and d.get('brk', '').split('/')[0] in ('hold', 'brk', 'set'):
            if cur is None:
                cur = [d['t'], d['t'], None]
            cur[1] = d['t']
            if cur[2] is None and d['brk'].startswith('hold'):
                cur[2] = d['t']
        else:
            if cur is not None:
                segs.append(cur)
            cur = None
    if cur is not None:
        segs.append(cur)
    def vpos(t):
        r = latest(VT, V, t)
        return (r[1], r[2]) if r else None
    for a, b, tp in segs:
        if tp is None or b - a < 10:
            continue
        px, py = vpos(tp)
        def e(t):
            p = vpos(t)
            return (p[0] - px) * ux + (p[1] - py) * uy
        print(f"DpVins сегмент {a:.1f}–{b:.1f}: гвоздь взят t={tp:.2f} (set/set до него — {tp - a:.1f} с без гвоздя,"
              f" e(вход) {e(a):+.2f} м)")
        print("  фронт  e(фронт) e(brk-in) t(brk-in) v_ист  e(max)  t(max)  e(спад) e(min после) t(min)  e(конец)")
        k = 0
        while True:
            gs = gust['at'] + gust['every'] * k
            k += 1
            if gs + gust['every'] > b + 0.01:
                break
            if gs < tp:
                continue
            rows = [d for d in S if gs <= d['t'] < gs + gust['every'] and 'brk' in d]
            if len(rows) < 20:
                continue
            bi = next((d['t'] for d in rows if 'brk' in d['brk']), None)
            ts = [d['t'] for d in rows]; es = [e(t) for t in ts]
            im = max(range(len(es)), key=lambda i: es[i])
            imn = min(range(len(es)), key=lambda i: es[i])
            tr = latest(TT, T, bi) if bi else None
            fall = gs + gust['rise'] + gust['hold'] + gust['fall']
            print(f"  {gs:5.0f}  {es[0]:+7.2f}  {(e(bi) if bi else float('nan')):+8.2f}  {((bi - gs) if bi else float('nan')):7.1f} "
                  f"{(tr[4] if tr else float('nan')):5.2f}  {es[im]:+6.2f}  {ts[im] - gs:5.1f}  {e(fall):+7.2f}  "
                  f"{es[imn]:+8.2f}  {ts[imn] - gs:5.1f}  {es[-1]:+7.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('--wind-deg', type=float, default=98.0, help='направление оси ветра в мире, ° (wind_gust.log)')
    ap.add_argument('--no-trans', action='store_true', help='без ленты переходов')
    a = ap.parse_args()
    label, bag, gust_txt = resolve(a.run)
    S, T, V = load(bag)
    if not S or not any('brk' in d for d in S):
        sys.exit(f"{label}: поля brk= в статусе нет (прогон до 2026-09-06?)")
    TT = [r[0] for r in T]; VT = [r[0] for r in V]
    print(f"# {label}: status {len(S)}, t {S[0]['t']:.1f}..{S[-1]['t']:.1f}; WIND_GUST={gust_txt}")
    if not a.no_trans:
        transitions(S, T, TT)
    gust = parse_gust(gust_txt) if gust_txt else None
    if gust and gust['every'] > 0:
        cycles(S, T, TT, gust)
        pin_relative(S, T, TT, V, VT, gust, a.wind_deg)
    else:
        print("WIND_GUST не найден — только лента переходов")


if __name__ == '__main__':
    main()
