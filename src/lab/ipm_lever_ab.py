#!/usr/bin/env python3
"""ipm_lever_ab.py — судья A/B ВЫНОСА КАМЕРЫ в канале вида сверху (cmd/ipm_lever, 2026-10-02).

Камера на выносе t (борт: 0.14 вперёд, 0.06 ниже IMU). На развороте вынесенная вперёд камера
едет вбок на t_x·ω_z — канал без учёта выноса видит это как снос, демпфер его честно гасит и
УВОДИТ борт. На малой высоте камера на 6 см ниже — масштаб канала без учёта завышен.

По фазам сценария ipm_lever.json (из /mission/status: стики rcy/rcp, высота; ярус 0):
  разворот  — |rcy| > 20: боковая ошибка канала (ipm_vlat − истина), среднее и СКО;
              ожидаемый ложный снос t_x·ω_z (истина ω_z); средняя |v| борта и уход за фазу;
  висение   — стики в центре ≥ 3 с: то же (контроль: обе стороны должны быть одинаковы);
  вперёд    — |rcp| > 20: гейн продольной оси (МНК через ноль, ipm_vfwd = g·истина).
Истина — /model/iris_cam/odometry (twist В ТЕЛЕ: x вперёд, y влево; ω_z влево +); канал —
/flow_dbg9 (x = vlat влево +, y = vfwd, z > 0.5 = годен).

Запуск внутри nav:
  docker exec p1317_nav bash -lc "source /opt/ros/humble/setup.bash; \\
    source /root/sim_ws/install/setup.bash; \\
    python3 /lab/ipm_lever_ab.py /root/sim_ws/output/joystick/<RUN1> [<RUN2> …]"
"""
import bisect
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gust_hold_compare import resolve, stamp                     # noqa: E402
from geometry_msgs.msg import Vector3Stamped                     # noqa: E402
from nav_msgs.msg import Odometry                                # noqa: E402
from rclpy.serialization import deserialize_message              # noqa: E402
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions  # noqa: E402
from std_msgs.msg import String                                  # noqa: E402

TX = 0.14            # вынос вперёд, м (бортовой config.yaml)
STICK = 20           # |rc*| > STICK — стик отклонён


def meta(run):
    d = {}
    for f in os.listdir(run):
        if f.endswith('.env'):
            for line in open(os.path.join(run, f), encoding='utf-8', errors='replace'):
                if '=' in line and not line.startswith('#'):
                    k, v = line.strip().split('=', 1)
                    d[k] = v
    return d


def read(bag):
    rd = SequentialReader()
    rd.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    rd.set_filter(StorageFilter(topics=['/mission/status', '/model/iris_cam/odometry', '/flow_dbg9']))
    S, T, F = [], [], []
    while rd.has_next():
        topic, raw, _ = rd.read_next()
        if topic == '/mission/status':
            d = dict(kv.partition('=')[::2] for kv in deserialize_message(raw, String).data.split())
            if 't' in d:
                d['t'] = float(d['t'])
                S.append(d)
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p, v, w = m.pose.pose.position, m.twist.twist.linear, m.twist.twist.angular
            T.append((stamp(m), p.x, p.y, p.z, v.x, v.y, w.z))
        else:
            m = deserialize_message(raw, Vector3Stamped)
            F.append((stamp(m), m.vector.x, m.vector.y, m.vector.z))
    S.sort(key=lambda d: d['t'])
    T.sort()
    F.sort()
    return S, T, F


def phases(S):
    """[(вид, t0, t1, alt)] по стикам статуса; ярус 0, в воздухе (alt > 0.3)."""
    out, cur = [], None
    for d in S:
        try:
            alt = float(d.get('alt', 'nan'))
            rcy, rcp = abs(int(d.get('rcy', 0))), abs(int(d.get('rcp', 0)))
            rct = abs(int(d.get('rct', 0)))
        except ValueError:
            continue
        kind = None
        if d.get('tier') == '0' and alt > 0.3 and rct < STICK:
            kind = 'разворот' if rcy > STICK else 'вперёд' if rcp > STICK else 'висение'
        if cur and kind == cur[0]:
            cur[2] = d['t']
            cur[3].append(alt)
        else:
            if cur:
                out.append((cur[0], cur[1], cur[2], st.median(cur[3])))
            cur = [kind, d['t'], d['t'], [alt]] if kind else None
    if cur:
        out.append((cur[0], cur[1], cur[2], st.median(cur[3])))
    return [p for p in out if p[2] - p[1] >= 2.5]


def main(runs):
    rows = []
    for arg in runs:
        label, bag, _ = resolve(arg)
        mt = meta(os.path.dirname(bag.rstrip('/')))
        side = f"lever={mt.get('BS_IPM_LEVER', '?')} accw={mt.get('BS_IPM_ACC_WORLD', '0')}"
        S, T, F = read(bag)
        tt = [r[0] for r in T]
        g0 = st.median(r[3] for r in T[:60])
        def tru(t):
            return T[min(bisect.bisect_left(tt, t), len(T) - 1)]
        print(f'\n# {label}  {side}')
        print('  фаза        t0-t1          h     ω_z°/с | бок: ошибка ср  СКО   ждём t_x·ω | '
              '|v| борта ср  уход м | вперёд: гейн')
        for kind, t0, t1, alt in phases(S):
            # первые 1.5 с фазы — переходный (разгон разворота / торможение)
            a = t0 + (1.5 if kind != 'вперёд' else 0.5)
            fs = [f for f in F if a <= f[0] <= t1 and f[3] > 0.5]
            if len(fs) < 20:
                continue
            pr = [(f, tru(f[0])) for f in fs]
            e_lat = [f[1] - r[5] for f, r in pr]
            wz = st.mean(r[6] for f, r in pr)
            vabs = st.mean(math.hypot(r[4], r[5]) for f, r in pr)
            pa, pb = tru(a), tru(t1)
            drift = math.hypot(pb[1] - pa[1], pb[2] - pa[2])
            h = st.median(r[3] - g0 for f, r in pr)
            gain = ''
            if kind == 'вперёд':
                sx = sum(r[4] * f[2] for f, r in pr)
                sxx = sum(r[4] ** 2 for f, r in pr)
                gain = f'{sx / sxx:5.3f}' if sxx > 1e-6 else ''
            em, es = st.mean(e_lat), st.pstdev(e_lat)
            print(f'  {kind:9s} {a:6.1f}-{t1:6.1f}  {h:4.2f}  {math.degrees(wz):+7.1f} | '
                  f'{em:+7.3f} {es:5.3f}   {TX * wz:+6.3f}   | {vabs:6.3f}      {drift:5.2f}  | {gain}')
            rows.append((side, kind, round(h * 2) / 2, wz, em, es, vabs, drift, gain))
    # сводка по сторонам: разворот — |ошибка ср| и уход
    print('\n# СВОДКА (по сторонам BS_IPM_LEVER / BS_IPM_ACC_WORLD)')
    for side in sorted({r[0] for r in rows}):
        for kind in ('разворот', 'висение', 'вперёд'):
            rr = [r for r in rows if r[0] == side and r[1] == kind]
            if not rr:
                continue
            line = (f'  {side} {kind:9s} n={len(rr)}  |бок ошибка| ср {st.mean(abs(r[4]) for r in rr):.3f}'
                    f'  СКО {st.mean(r[5] for r in rr):.3f}  |v| {st.mean(r[6] for r in rr):.3f}'
                    f'  уход {st.mean(r[7] for r in rr):.2f} м')
            g = [float(r[8]) for r in rr if r[8]]
            if g:
                line += f'  гейн вперёд {st.mean(g):.3f}'
            print(line)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1:])
