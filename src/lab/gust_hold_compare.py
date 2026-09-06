#!/usr/bin/env python3
"""gust_hold_compare.py — КАК ДЕРЖАТ ЯРУСЫ (DpHold / DpVins / LOITER) на висении под
порывами: по bag'ам freefly-полётов, окна по ярусу лесенки, метрики по истине Gazebo.

Зачем отдельно от hold_quality.py / wind_drift.py. Те меряют ОДНО висение целиком
(круг 10 м, установившийся снос). Здесь один полёт содержит несколько ярусов
подряд (взлёт и висение на DpHold → CH6 → висение на DpVins → посадка), а ветер
— ДЕТЕРМИНИРОВАННЫЕ порывы wind_gust.py (расписание в абсолютном sim-времени).
Поэтому: (а) окно — непрерывный сегмент ЯРУСА (tier= из /mission/status) внутри
спокойного воздуха [конец набора, начало посадки]; (б) внутри окна — каждый
полный ЦИКЛ порыва (every секунд от фронта) меряется отдельно: пик смещения от
точки на фронте (ЖЁСТКОСТЬ к порыву) и остаток в конце цикла (ЯКОРЬ — вернулся
ли). Одной «итоговой точки» мало: серия dphold_vs_dpvins 2026-09-05 показала,
что DpHold жёстче к порыву в 2.5–4 раза (пик 2.4–2.8 м против 6.3–9.5), но без
глобальной рамы копит остаток 0.8–1.5 м за цикл по ветру (в лоб на 10 м — 10 м за
2 мин), а DpVins мягче, но возвращается в точку (остаток 0.1–0.4 м за цикл).

Границы окна (всё sim-время):
  конец набора    — |vz| < 0.2 и газ в центре (rct) 3 с подряд после отрыва (z > 0.3);
  начало посадки  — sa=1 или газ < −80 PWM 1 с подряд (иначе касание z < 0.3 − 2 с);
  сегменты яруса  — по tier= между ними, 0.5 с перед каждым переключением
                    отрезается, сегменты короче --min-seg (10 с) пропускаются.
Метрики окна: exc_max/exc_rms/final — смещение от точки НАЧАЛА окна (м); vmax/vrms —
горизонтальная скорость истины; dz — уход высоты от медианы. По циклам порыва:
pk — пик смещения от точки на фронте, rs — остаток в конце цикла, t_pk — когда пик
(с от фронта; ≈ конец порыва = толкает весь порыв), ang — направление пика
относительно носа (° влево +; ветер слева → ~+90). Здоровье VINS в окне: flips —
переходы яруса внутри (гейт), reb/scl — приращения перерождений и срабатываний
чека занижения, V/ист — отношение средних |v| VINS/истина (масштаб; здоровый 1.00).

Расписание порывов — из WIND_GUST в <RUN>.env прогона (freefly_lv.sh пишет),
--gust перекрывает; без него — только метрики окон.

Запуск ВНУТРИ p1317_nav (src/lab смонтирован как /lab; стек не трогает):
  docker exec p1317_nav bash -lc "source /opt/ros/humble/setup.bash; \\
    source /root/sim_ws/install/setup.bash; \\
    python3 /lab/gust_hold_compare.py \\
      /root/sim_ws/output/joystick/dphold_vs_dpvins/wind_left/{1,5,10,20} \\
      /root/sim_ws/output/joystick/dphold_vs_dpvins/wind_front/{1,5,10,20}"
Аргумент — каталог прогона (с bag/ и .env) или сам каталог bag. --csv DIR — окна и
циклы порыва построчно (для графиков). --no-gusts — без таблицы по циклам.
"""
import argparse
import bisect
import glob
import math
import os
import statistics as st
import sys

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from std_msgs.msg import String

TIER_NAMES = {'0': 'DpHold', '1': 'DpVins', '2': 'LOITER'}
stamp = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def parse_gust(text):
    """'spd=8 at=30 rise=2 hold=5 fall=4 every=20' → dict (как wind_gust.py)."""
    kv = {'at': 60.0, 'rise': 2.0, 'hold': 5.0, 'fall': 4.0, 'every': 0.0}
    for tok in text.split():
        k, _, v = tok.partition('=')
        kv[k] = float(v)
    return kv


def resolve(arg):
    """каталог прогона или bag → (label, bag_path, gust_spec|None)"""
    arg = arg.rstrip('/')
    if os.path.isfile(os.path.join(arg, 'metadata.yaml')):
        bag, run = arg, os.path.dirname(arg)
    else:
        bag, run = os.path.join(arg, 'bag'), arg
    label = '/'.join(run.split('/')[-2:])
    gust = None
    for env in glob.glob(os.path.join(run, '*.env')):
        for line in open(env, encoding='utf-8', errors='replace'):
            if line.startswith('WIND_GUST='):
                gust = line.split('=', 1)[1].strip().strip('"')
    return label, bag, gust


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    r.set_filter(StorageFilter(topics=['/mission/status', '/model/iris_cam/odometry',
                                       '/odometry']))
    S, T, V = [], [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/mission/status':
            d = dict(kv.partition('=')[::2] for kv in
                     deserialize_message(raw, String).data.split())
            if 't' in d:
                d['t'] = float(d['t']); S.append(d)
        else:
            m = deserialize_message(raw, Odometry)
            p, v = m.pose.pose.position, m.twist.twist.linear
            row = (stamp(m), p.x, p.y, p.z, math.hypot(v.x, v.y), v.z,
                   yaw_deg(m.pose.pose.orientation))
            (T if topic == '/model/iris_cam/odometry' else V).append(row)
    S.sort(key=lambda d: d['t']); T.sort(); V.sort()
    return S, T, V


def latest(times, arr, t):
    i = bisect.bisect_right(times, t) - 1
    return arr[max(i, 0)]


def seg(times, arr, a, b):
    return arr[bisect.bisect_left(times, a):bisect.bisect_left(times, b)]


def sustained(S, pred, t_from, dur):
    start = None
    for d in S:
        if d['t'] < t_from:
            continue
        if pred(d):
            start = start if start is not None else d['t']
            if d['t'] - start >= dur:
                return start
        else:
            start = None
    return None


def brake_secs(S, a, b):
    """Секунды фазы BRAKE (поле brk= статуса, любая ось) в [a, b]; nan — поля
    в bag нет (прогоны до 2026-09-06)."""
    tot, prev, seen = 0.0, None, False
    for d in S:
        if d['t'] < a:
            continue
        if d['t'] > b:
            break
        if 'brk' in d:
            seen = True
            if prev is not None and 'brk' in prev.get('brk', ''):
                tot += min(d['t'] - prev['t'], 0.5)
        prev = d
    return tot if seen else float('nan')


def window_metrics(T, TT, a, b, gust):
    rows = seg(TT, T, a, b)
    if len(rows) < 10:
        return None
    x0, y0 = rows[0][1], rows[0][2]
    dist = [math.hypot(r[1] - x0, r[2] - y0) for r in rows]
    spd = [r[4] for r in rows]
    z_med = st.median(r[3] for r in rows)
    yaw = st.median(r[6] for r in rows)
    gusts = []
    if gust and gust['every'] > 0:
        k = 0
        while True:
            gs = gust['at'] + gust['every'] * k
            k += 1
            if gs + gust['every'] > b + 0.01:
                break
            if gs < a:
                continue
            g = seg(TT, T, gs, gs + gust['every'])
            if len(g) < 10:
                continue
            gx, gy = g[0][1], g[0][2]
            dd = [math.hypot(r[1] - gx, r[2] - gy) for r in g]
            ipk = max(range(len(dd)), key=lambda i: dd[i])
            ang = math.degrees(math.atan2(g[ipk][2] - gy, g[ipk][1] - gx)) - yaw
            gusts.append(dict(t=gs, peak=max(dd), resid=dd[-1], vmax=max(r[4] for r in g),
                              t_peak=g[ipk][0] - gs, ang=(ang + 180) % 360 - 180))
    return dict(a=a, b=b, dur=b - a, yaw=yaw, z_med=z_med,
                z_dev=max(abs(r[3] - z_med) for r in rows),
                exc_max=max(dist), exc_rms=math.sqrt(sum(d * d for d in dist) / len(dist)),
                final=dist[-1], vmax=max(spd),
                vrms=math.sqrt(sum(v * v for v in spd) / len(spd)), gusts=gusts)


def analyze(label, bag, gust, min_seg):
    S, T, V = load(bag)
    if not S or not T:
        print(f"{label}: нет /mission/status или истины Gazebo в bag"); return []
    ST = [d['t'] for d in S]; TT = [r[0] for r in T]; VT = [r[0] for r in V]
    t_off = next((r[0] for r in T if r[3] > 0.3), None)
    if t_off is None:
        print(f"{label}: борт не отрывался"); return []

    def calm(d):
        r = latest(TT, T, d['t'])
        return abs(r[5]) < 0.2 and abs(float(d.get('rct', 0))) < 40
    t_calm = sustained(S, calm, t_off + 1.0, 3.0) or (t_off + 5.0)

    def landing(d):
        return d.get('sa') == '1' or float(d.get('rct', 0)) < -80
    t_land = sustained(S, landing, t_calm + 5.0, 1.0)
    if t_land is None:
        touch = next((r[0] for r in T if r[0] > t_calm + 5.0 and r[3] < 0.3), None)
        t_land = (touch - 2.0) if touch else ST[-1]
    # сегменты яруса внутри [t_calm, t_land]
    segs, cur, t0 = [], None, t_calm
    for d in S:
        if d['t'] < t_calm:
            continue
        if d['t'] >= t_land:
            break
        tr = d.get('tier')
        if cur is None:
            cur = tr
        elif tr != cur:
            segs.append((cur, t0, d['t'] - 0.5)); cur, t0 = tr, d['t']
    if cur is not None:
        segs.append((cur, t0, t_land))
    out = []
    for tr, a, b in segs:
        if b - a < min_seg or tr is None:
            continue
        w = window_metrics(T, TT, a, b, gust)
        if w is None:
            continue
        sw = [d for d in S if a <= d['t'] < b]
        d0, d1 = sw[0], sw[-1]
        vv = seg(VT, V, a, b); tv = seg(TT, T, a, b)
        ratio = ((sum(r[4] for r in vv) / len(vv)) / (sum(r[4] for r in tv) / len(tv))
                 if vv and tv and sum(r[4] for r in tv) > 0 else float('nan'))
        for x in w['gusts']:
            x['brk'] = brake_secs(sw, x['t'], x['t'] + gust['every'])
        w.update(label=label, tier=TIER_NAMES.get(tr, tr), t_off=t_off, t_calm=t_calm,
                 t_land=t_land, brk_s=brake_secs(sw, a, b),
                 flips=sum(1 for i in range(1, len(sw)) if sw[i].get('tier') != sw[i - 1].get('tier')),
                 reb=int(d1.get('reb', 0)) - int(d0.get('reb', 0)),
                 scl=int(d1.get('scl', 0)) - int(d0.get('scl', 0)), ratio=ratio)
        out.append(w)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+', help='каталоги прогонов (с bag/ и .env) или bag')
    ap.add_argument('--gust', help='расписание порывов, перекрывает WIND_GUST из .env')
    ap.add_argument('--min-seg', type=float, default=10.0, help='мин. длина окна яруса, с')
    ap.add_argument('--csv', help='каталог для windows.csv / gusts.csv')
    ap.add_argument('--no-gusts', action='store_true')
    a = ap.parse_args()
    wins = []
    for arg in a.runs:
        label, bag, gust_txt = resolve(arg)
        gust_txt = a.gust or gust_txt
        gust = parse_gust(gust_txt) if gust_txt else None
        if gust is None:
            print(f"{label}: WIND_GUST не найден — только метрики окон", file=sys.stderr)
        wins.extend(analyze(label, bag, gust, a.min_seg))
    if not wins:
        sys.exit("окон нет")

    def m(xs, f=st.mean):
        return f(xs) if xs else float('nan')
    print("=== ОКНА ЯРУСОВ (sim-с; истина Gazebo; excursion от точки начала окна) ===")
    print(f"{'прогон':14} {'ярус':7} {'от':>6} {'до':>6} {'dur':>5} {'z':>5} {'yaw°':>5} "
          f"{'exc_max':>7} {'exc_rms':>7} {'final':>6} {'vmax':>5} {'vrms':>5} {'dz':>5} | "
          f"{'циклов':>6} {'pk_mean':>7} {'pk_max':>6} {'rs_mean':>7} {'t_pk':>5} {'ang°':>5} | "
          f"{'flips':>5} {'reb':>3} {'scl':>3} {'V/ист':>5} {'brk_s':>5}")
    for w in wins:
        g = w['gusts']
        print(f"{w['label']:14} {w['tier']:7} {w['a']:6.1f} {w['b']:6.1f} {w['dur']:5.1f} "
              f"{w['z_med']:5.1f} {w['yaw']:5.0f} {w['exc_max']:7.2f} {w['exc_rms']:7.2f} "
              f"{w['final']:6.2f} {w['vmax']:5.2f} {w['vrms']:5.2f} {w['z_dev']:5.2f} | "
              f"{len(g):6d} {m([x['peak'] for x in g]):7.2f} {m([x['peak'] for x in g], max):6.2f} "
              f"{m([x['resid'] for x in g]):7.2f} {m([x['t_peak'] for x in g]):5.1f} "
              f"{m([x['ang'] for x in g]):5.0f} | {w['flips']:5d} {w['reb']:3d} {w['scl']:3d} "
              f"{w['ratio']:5.2f} {w['brk_s']:5.1f}")
    if not a.no_gusts:
        print("\n=== ПО ЦИКЛАМ ПОРЫВА: [фронт pk пик rs остаток v vmax b секунд BRAKE] ===")
        for w in wins:
            if w['gusts']:
                print(f"{w['label']:14} {w['tier']:7} " + "  ".join(
                    f"[{x['t']:.0f}s pk {x['peak']:.2f} rs {x['resid']:.2f} v {x['vmax']:.1f}"
                    f" b {x['brk']:.1f}]"
                    for x in w['gusts']))
    if a.csv:
        import csv
        os.makedirs(a.csv, exist_ok=True)
        with open(os.path.join(a.csv, 'windows.csv'), 'w') as f:
            wr = csv.writer(f)
            wr.writerow(['run', 'tier', 'a', 'b', 'dur', 'z', 'yaw', 'exc_max', 'exc_rms', 'final',
                         'vmax', 'vrms', 'dz', 'n_gusts', 'pk_mean', 'rs_mean', 'flips', 'reb',
                         'scl', 'ratio', 'brk_s'])
            for w in wins:
                g = w['gusts']
                wr.writerow([w['label'], w['tier'], f"{w['a']:.2f}", f"{w['b']:.2f}",
                             f"{w['dur']:.1f}", f"{w['z_med']:.2f}", f"{w['yaw']:.0f}",
                             f"{w['exc_max']:.2f}", f"{w['exc_rms']:.2f}", f"{w['final']:.2f}",
                             f"{w['vmax']:.2f}", f"{w['vrms']:.2f}", f"{w['z_dev']:.2f}", len(g),
                             f"{m([x['peak'] for x in g]):.2f}", f"{m([x['resid'] for x in g]):.2f}",
                             w['flips'], w['reb'], w['scl'], f"{w['ratio']:.2f}",
                             f"{w['brk_s']:.1f}"])
        with open(os.path.join(a.csv, 'gusts.csv'), 'w') as f:
            wr = csv.writer(f)
            wr.writerow(['run', 'tier', 't_front', 'peak', 'resid', 'vmax', 't_peak', 'ang', 'brk'])
            for w in wins:
                for x in w['gusts']:
                    wr.writerow([w['label'], w['tier'], f"{x['t']:.1f}", f"{x['peak']:.2f}",
                                 f"{x['resid']:.2f}", f"{x['vmax']:.2f}", f"{x['t_peak']:.1f}",
                                 f"{x['ang']:.0f}", f"{x['brk']:.1f}"])
        print(f"\nCSV → {a.csv}/windows.csv, gusts.csv")


if __name__ == '__main__':
    main()
