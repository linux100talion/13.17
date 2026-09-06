#!/usr/bin/env python3
"""ipm_alt_quality.py — КАЧЕСТВО КАНАЛА ВИДА СВЕРХУ ПО ВЫСОТАМ: годится ли IPM опорником для
детекции коллапса масштаба VINS выше 4 м (третий канал handover.vins_sane, BS_VINS_SCALE_*).

По окнам висения на демпфере (ярус 0, стики в центре, плато высоты ≥ 8 с): доля годных кадров,
гейн канала к истине Gazebo (МНК через ноль, боковая и продольная оси; twist истины — в теле),
корреляция, шум (СКО ipm − gain·истина), истинная скорость и масштаб VINS/истина.
Bag lv2_joy_20260906_171454 (висение 10/14/17/23 м, ветер 1 + порывы 8): годных 100 % на всех
высотах, гейн боковой 0.49–0.77 / продольный 0.53–1.14, шум 0.16 (10 м) → 0.19 (14) → 0.22 (17)
→ 0.41 м/с (23 м), VINS/истина 1.00. Вывод: чек занижения (ratio 0.5, ≥ 3 с) можно поднять с
alt_max 4 до 25 м при ipm_min 2 → 1.2 (3σ на 23 м): реплей vins_sane_replay.py — 0 ложных на
6 здоровых bag, эмуляция коллапса ×0.2 (--vins-scale) ловится на плечах при истине 1.3–2 м/с;
на висении без хода не ловится по построению (нужно движение ≥ ~2 м/с — при реальном коллапсе
его даёт сам унос, как в 114248). Запуск внутри nav:
  docker exec p1317_nav bash -lc "source /opt/ros/humble/setup.bash; \\
    source /root/sim_ws/install/setup.bash; \\
    python3 /lab/ipm_alt_quality.py /root/sim_ws/output/joystick/<RUN>"
"""
import argparse
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('--min-win', type=float, default=8.0, help='минимальная длина окна висения, с')
    args = ap.parse_args()
    label, bag, _ = resolve(args.run)
    run = label
    rd = SequentialReader()
    rd.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    rd.set_filter(StorageFilter(topics=['/mission/status', '/model/iris_cam/odometry', '/flow_dbg9', '/odometry']))
    S, T, F, V = [], [], [], []
    while rd.has_next():
        topic, raw, _ = rd.read_next()
        if topic == '/mission/status':
            d = dict(kv.partition('=')[::2] for kv in deserialize_message(raw, String).data.split())
            if 't' in d: d['t'] = float(d['t']); S.append(d)
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry); v = m.twist.twist.linear
            T.append((stamp(m), m.pose.pose.position.z, v.x, v.y))          # twist В ТЕЛЕ
        elif topic == '/flow_dbg9':
            m = deserialize_message(raw, Vector3Stamped); F.append((stamp(m), m.vector.x, m.vector.y, m.vector.z))  # vlat, vfwd, ok
        else:
            m = deserialize_message(raw, Odometry); v = m.twist.twist.linear; V.append((stamp(m), math.hypot(v.x, v.y)))
    S.sort(key=lambda d: d["t"]); T.sort(); F.sort(); V.sort()
    tt = [r[0] for r in T]; vt = [r[0] for r in V]
    def tru(t):
        i = min(bisect.bisect_left(tt, t), len(T)-1); return T[i]
    def vins(t):
        i = min(bisect.bisect_left(vt, t), len(V)-1); return V[i][1] if V else float('nan')
    # окна висения: ярус 0, стики в центре, высота-плато ≥ 8 с (по alt статуса, шаг 0.5 м)
    wins = []; cur = None
    for d in S:
        ok = d.get('tier') == '0' and abs(int(d.get('rcp', 0))) < 20 and abs(int(d.get('rcr', 0))) < 20 and 'alt' in d
        a = float(d['alt']) if ok else None
        if ok and cur is not None and abs(a - cur[2]) < 0.7:
            cur[1] = d['t']
        else:
            if cur is not None and cur[1] - cur[0] >= args.min_win: wins.append(cur)
            cur = [d['t'], d['t'], a] if ok else None
    if cur is not None and cur[1] - cur[0] >= args.min_win: wins.append(cur)
    print(f"# {run}: окна висения DpHold ≥ 8 с (ярус 0, стики центр, плато высоты)")
    print("  t0     t1    dur  alt  | ipm_ok% |  gain_lat gain_fwd  corr_lat | шум_lat шум_fwd (СКО ipm−gain·ист, м/с) | ист |v| mean/max | VINS/ист")
    for t0, t1, alt in wins:
        rows = [f for f in F if t0 <= f[0] <= t1]
        if len(rows) < 40: continue
        ok = [f for f in rows if f[3] > 0.5]
        pairs = [(f, tru(f[0])) for f in ok]
        # gain по МНК через ноль: ipm = g·ист (боковая: y тела = лево+, ipm vlat лево+; продольная x)
        def fit(ix, tx):
            sx = sum(p[1][tx]*p[0][ix] for p in pairs); sxx = sum(p[1][tx]**2 for p in pairs)
            g = sx/sxx if sxx > 1e-6 else float('nan')
            res = [p[0][ix] - g*p[1][tx] for p in pairs]
            n = len(res); sd = math.sqrt(sum(r*r for r in res)/n) if n else float('nan')
            mx = st.mean(p[1][tx] for p in pairs); my = st.mean(p[0][ix] for p in pairs)
            cov = sum((p[1][tx]-mx)*(p[0][ix]-my) for p in pairs); vx = sum((p[1][tx]-mx)**2 for p in pairs); vy = sum((p[0][ix]-my)**2 for p in pairs)
            corr = cov/math.sqrt(vx*vy) if vx > 0 and vy > 0 else float('nan')
            return g, sd, corr
        gl, sdl, cl = fit(1, 3); gf, sdf, cf = fit(2, 2)
        vm = [math.hypot(p[1][2], p[1][3]) for p in pairs]
        vr = [vins(p[0][0])/math.hypot(p[1][2], p[1][3]) for p in pairs if math.hypot(p[1][2], p[1][3]) > 0.3]
        print(f"{t0:6.1f} {t1:6.1f} {t1-t0:5.1f} {alt:4.1f} | {100*len(ok)/len(rows):5.0f}%  |  {gl:7.2f} {gf:8.2f}  {cl:8.2f} | {sdl:7.2f} {sdf:7.2f}                        | {st.mean(vm):4.2f}/{max(vm):4.2f}    | {st.mean(vr) if vr else float('nan'):4.2f}")


if __name__ == '__main__':
    main()
