#!/usr/bin/env python3
"""vins_twist_check.py — TWIST ОДОМЕТРИИ VINS ПРОТИВ РАЗНОСТИ ПОЗЫ И ИСТИНЫ: годится ли
twist как источник скорости для стека (BS_VINS_VEL_SRC=twist, cmd/6).

По bag прогона: (1) рама twist — МНК-поворот к истине Gazebo в полёте (|v| > 0.5) против
того же поворота для конечной разности позы (совпали → twist в раме позы); (2) лаг к истине
и масштаб |v| кросс-корреляцией по штампам (сетка 0.02 с) для twist и для разности + EMA 0.4
(как VinsTrack); (3) шум на висении (истина |v| < 0.1): средняя ошибка и СКО.
Bag lv2_joy_20260906_130326: поворот +3.2° / +2.7°, лаг 0.00 / 0.14 с, масштаб 1.008 / 0.997,
шум 0.008±0.005 / 0.025±0.049 м/с. Запуск внутри nav:
  docker exec p1317_nav bash -lc "source /opt/ros/humble/setup.bash; \\
    source /root/sim_ws/install/setup.bash; \\
    python3 /lab/vins_twist_check.py /root/sim_ws/output/joystick/<RUN>"
"""
import argparse
import bisect
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gust_hold_compare import resolve, stamp                        # noqa: E402
from nav_msgs.msg import Odometry                                   # noqa: E402
from rclpy.serialization import deserialize_message                 # noqa: E402
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions  # noqa: E402


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    r.set_filter(StorageFilter(topics=['/odometry', '/model/iris_cam/odometry']))
    V, T = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        m = deserialize_message(raw, Odometry)
        p, v = m.pose.pose.position, m.twist.twist.linear
        (V if topic == '/odometry' else T).append((stamp(m), p.x, p.y, v.x, v.y))
    V.sort(); T.sort()
    return V, T


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('--ema', type=float, default=0.4, help='EMA разности позы (VinsTrack 0.4)')
    ap.add_argument('--t-min', type=float, default=25.0, help='с какого sim-времени считать шум висения')
    a = ap.parse_args()
    label, bag, _ = resolve(a.run)
    V, T = load(bag)
    print(f"# {label}: vins {len(V)} truth {len(T)}")
    D, vx, vy = [], 0.0, 0.0
    for i in range(1, len(V)):
        dt = V[i][0] - V[i - 1][0]
        if dt <= 0:
            continue
        fx, fy = (V[i][1] - V[i - 1][1]) / dt, (V[i][2] - V[i - 1][2]) / dt
        vx += a.ema * (fx - vx); vy += a.ema * (fy - vy)
        D.append((V[i][0], vx, vy))
    W = [(t, x, y) for (t, _, _, x, y) in V]
    tt = [r[0] for r in T]

    def tru(t):
        i = min(max(bisect.bisect_left(tt, t), 1), len(T) - 1)
        p, q = T[i - 1], T[i]
        w = (t - p[0]) / (q[0] - p[0]) if q[0] > p[0] else 0.0
        return (p[3] + w * (q[3] - p[3]), p[4] + w * (q[4] - p[4]))

    def fit_yaw(rows):
        sxx = sxy = 0.0
        for t, x, y in rows:
            tx, ty = tru(t)
            if math.hypot(tx, ty) < 0.5:
                continue
            sxx += x * tx + y * ty; sxy += x * ty - y * tx
        return math.degrees(math.atan2(sxy, sxx))
    print(f"поворот рамы к истине (МНК, |v|>0.5): twist {fit_yaw(W):+.1f}°, разность позы {fit_yaw(D):+.1f}° "
          f"(совпали → twist в раме позы)")

    def lag_scale(rows, name):
        best = None
        mags = [(t, math.hypot(x, y)) for t, x, y in rows]
        for lag in [i * 0.02 for i in range(41)]:
            s_vt = s_vv = s_tt = 0.0; n = 0
            for t, m in mags:
                if t - lag < tt[0] or t > tt[-1]:
                    continue
                tm = math.hypot(*tru(t - lag))
                if tm < 0.3:
                    continue
                s_vt += m * tm; s_vv += m * m; s_tt += tm * tm; n += 1
            if n < 50:
                continue
            corr = s_vt / math.sqrt(s_vv * s_tt)
            if best is None or corr > best[1]:
                best = (lag, corr, s_vt / s_tt)
        if best:
            print(f"{name}: лаг к истине {best[0]:.2f} с (corr {best[1]:.4f}), масштаб |v|/|ист| {best[2]:.3f}")
    lag_scale(W, "twist            ")
    lag_scale(D, f"разность+EMA {a.ema:<4}")

    def hover_noise(rows, name):
        xs = [math.hypot(x - tru(t)[0], y - tru(t)[1]) for t, x, y in rows
              if t > a.t_min and math.hypot(*tru(t)) < 0.1]
        if xs:
            m = sum(xs) / len(xs); sd = math.sqrt(sum((v - m) ** 2 for v in xs) / len(xs))
            print(f"{name}: ошибка на висении (|ист|<0.1): средняя {m:.3f} м/с, СКО {sd:.3f}, n {len(xs)}")
    hover_noise(W, "twist            ")
    hover_noise(D, f"разность+EMA {a.ema:<4}")


if __name__ == '__main__':
    main()
