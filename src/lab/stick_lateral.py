#!/usr/bin/env python3
"""stick_lateral.py — БОКОВОЙ СНОС НА СТИКЕ: держит ли стабилизатор линию, пока пилот
рулит другой осью. Повод — cmd/4 в движении (lv2_joy_20260906_113224): на живом стике
трим DpVins не учился вовсе (любой стик морозил обе оси), крейсер стиком тангажа с ветром
сбоку сносило 0.56–0.76 м/с против 0.16 у DpHold — кандидат latch_axis (cmd/5).

Сегменты стика — |rcp| или |rcr| > 20 PWM по /mission/status не короче --min-seg с;
на каждом: ярус, стик в начале, средняя скорость вперёд и БОКОВАЯ скорость тела
(right+, истина Gazebo /model/iris_cam/odometry в осях курса истины), её RMS, боковой
снос ∫v_lat dt, трим стрелки ветра (wnp, wnr) в начале и в конце и его модуль — по нему
видно, живёт ли трим на ходу (стоит → модуль и компоненты константы; вращается с курсом
→ модуль константа, компоненты едут). Запуск внутри nav (rosbag2_py):
  docker exec p1317_nav bash -lc "source /opt/ros/humble/setup.bash; \\
    source /root/sim_ws/install/setup.bash; \\
    python3 /lab/stick_lateral.py /root/sim_ws/output/joystick/<RUN> [--min-seg 3]"
Критерий cmd/5: vlat_mean ≤ 0.3 м/с на плечах ≥ 10 с, wnr на ходу меняется.
"""
import argparse
import bisect
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gust_hold_compare import resolve, stamp, yaw_deg          # noqa: E402
from nav_msgs.msg import Odometry                              # noqa: E402
from rclpy.serialization import deserialize_message            # noqa: E402
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions  # noqa: E402
from std_msgs.msg import String                                # noqa: E402


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    r.set_filter(StorageFilter(topics=['/mission/status', '/model/iris_cam/odometry']))
    S, T = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/mission/status':
            d = dict(kv.partition('=')[::2] for kv in deserialize_message(raw, String).data.split())
            if 't' in d:
                d['t'] = float(d['t']); S.append(d)
        else:
            m = deserialize_message(raw, Odometry)
            p, v = m.pose.pose.position, m.twist.twist.linear
            T.append((stamp(m), p.x, p.y, v.x, v.y, math.radians(yaw_deg(m.pose.pose.orientation))))
    S.sort(key=lambda d: d['t']); T.sort()
    return S, T


def segments(S, min_seg, thr=20):
    segs, cur = [], None
    for d in S:
        live = abs(int(d.get('rcp', 0))) > thr or abs(int(d.get('rcr', 0))) > thr
        if live and cur is None:
            cur = [d['t'], d['t'], d]
        elif live:
            cur[1] = d['t']
        elif cur is not None:
            if cur[1] - cur[0] >= min_seg:
                segs.append(cur)
            cur = None
    if cur is not None and cur[1] - cur[0] >= min_seg:
        segs.append(cur)
    return segs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', help='каталог прогона (с bag/) или сам bag')
    ap.add_argument('--min-seg', type=float, default=3.0, help='минимальная длина сегмента стика, с')
    a = ap.parse_args()
    label, bag, _ = resolve(a.run)
    S, T = load(bag)
    tt = [x[0] for x in T]
    print(f"# {label}: сегменты стика ≥ {a.min_seg:g} с; боковая скорость тела (right+) по истине "
          f"Gazebo, курс истины; трим — стрелка ветра статуса (wnp, wnr)")
    print("  t0     t1   dur tier   rcp   rcr  vfwd  vlat_mean vlat_rms  lat_disp  трим начало→конец   |трим|")
    for t0, t1, d0 in segments(S, a.min_seg):
        d1 = max((d for d in S if d['t'] <= t1), key=lambda d: d['t'])
        n = 0; sf = sl = sl2 = disp = 0.0; tp = None
        i0 = bisect.bisect_left(tt, t0)
        for x in T[i0:]:
            t, _, _, vx, vy, ps = x
            if t > t1:
                break
            vf = vx * math.cos(ps) + vy * math.sin(ps)
            vl = -vx * math.sin(ps) + vy * math.cos(ps)
            if tp is not None:
                disp += vl * (t - tp)
            tp = t; n += 1; sf += vf; sl += vl; sl2 += vl * vl
        if n == 0:
            continue
        wp0, wr0 = float(d0.get('wnp', 0)), float(d0.get('wnr', 0))
        wp1, wr1 = float(d1.get('wnp', 0)), float(d1.get('wnr', 0))
        print(f"{t0:6.1f} {t1:6.1f} {t1 - t0:5.1f}  {d0.get('tier', '-'):>3} {int(d0.get('rcp', 0)):5d} "
              f"{int(d0.get('rcr', 0)):5d} {sf / n:5.2f} {sl / n:9.2f} {math.sqrt(sl2 / n):8.2f} "
              f"{disp:9.2f}   ({wp0:4.0f},{wr0:4.0f})→({wp1:4.0f},{wr1:4.0f})  "
              f"{math.hypot(wp0, wr0):4.0f}→{math.hypot(wp1, wr1):4.0f}")


if __name__ == '__main__':
    main()
