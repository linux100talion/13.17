#!/usr/bin/env python3
"""ВЫБРОСЫ измерения: причина разгона или его следствие?

В серии E2 прогоны разошлись на две группы: тормозящие уход (E2s1: 1 выброс) и убегающие
(E2s2: 22 выброса, |kf_vel| втрое выше). Счётчика за прогон мало — он не отличает «сломанное
измерение разогнало борт» от «разгон сломал измерение». Отличает ПОРЯДОК во времени.

Скрипт берёт `/flow_dbg4` (kf_rejects — счётчик НАРАСТАЮЩИЙ, дифференцируем) и одометрию,
и печатает для каждого прогона:
  t_rej  — когда пошла первая ОЧЕРЕДЬ выбросов (>= REJ_RUN штук за REJ_WIN секунд;
           одиночные выбросы бывают и на здоровом висении, их не считаем);
  t_run  — когда продольная скорость впервые перевалила RUN_V и БОЛЬШЕ не возвращалась
           (начало разгона, а не случайный всплеск);
  Δ      — t_run − t_rej. Положительная — выбросы БЫЛИ РАНЬШЕ (кандидат в причину),
           отрицательная — выбросы пошли ПОСЛЕ разгона (симптом).

⚠️ Знак Δ — не доказательство причинности, а её необходимое условие: следствие не может
предшествовать причине, но совпадение по времени может иметь и общий корень.

Запуск:
  docker run --rm -v /root/13.17/src/lab:/lab:ro \
    -v /root/13.17/docker/sim/output:/out:ro ros:humble-ros-base bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/reject_timing.py /out/E2s*_bag'
"""
import math
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
HOVER_Z = 2.0
REJ_RUN, REJ_WIN = 3, 1.0     # очередь: 3 выброса за секунду
RUN_V = 1.0                   # м/с — порог «поехал», а не дрожит


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, d4 = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((st(m), p.x, p.y, p.z, yaw_of(m.pose.pose.orientation)))
        elif topic == '/flow_dbg4':
            m = deserialize_message(raw, Vector3Stamped)
            d4.append((st(m), m.vector.z))          # z = kf_rejects (нарастающий)
    return np.array(od) if od else None, np.array(d4) if d4 else None


def main(bags):
    print(f"{'прогон':10s} | {'t_rej':>6s} | {'t_run':>6s} | {'Δ, с':>6s} | "
          f"{'выбросов':>8s} | порядок")
    for bag in bags:
        name = bag.rstrip('/').split('/')[-1].replace('_bag', '')
        od, d4 = load(bag)
        if od is None or d4 is None or len(od) < 10:
            print(f"{name:10s} | данных нет")
            continue
        h = od[od[:, 3] > HOVER_Z]
        if len(h) < 10:
            print(f"{name:10s} | висения нет")
            continue
        t0 = h[0, 0]
        yaw0 = h[0, 4]
        # продольная скорость в системе борта на входе в висение
        t = h[:, 0] - t0
        fwd = (h[:, 1] - h[0, 1]) * math.cos(yaw0) + (h[:, 2] - h[0, 2]) * math.sin(yaw0)
        v = np.gradient(fwd, t)
        # начало разгона: первый момент, после которого |v| уже не падает ниже порога
        над = np.abs(v) > RUN_V
        t_run = float('nan')
        for i in range(len(над)):
            if над[i] and над[i:].mean() > 0.9:
                t_run = t[i]
                break
        # первая очередь выбросов внутри висения
        d = d4[(d4[:, 0] >= t0) & (d4[:, 0] <= h[-1, 0])]
        t_rej = float('nan')
        if len(d) > 1:
            tr, cr = d[:, 0] - t0, d[:, 1]
            for i in range(len(tr)):
                j = np.searchsorted(tr, tr[i] + REJ_WIN)
                if j < len(cr) and cr[j] - cr[i] >= REJ_RUN:
                    t_rej = tr[i]
                    break
        delta = t_run - t_rej
        if math.isnan(delta):
            order = 'разгона нет' if math.isnan(t_run) else 'очередей выбросов нет'
        else:
            order = 'выбросы РАНЬШЕ (кандидат в причину)' if delta > 0 else 'выбросы ПОСЛЕ (симптом)'
        f = lambda x: '  —  ' if math.isnan(x) else f'{x:6.1f}'
        print(f"{name:10s} | {f(t_rej)} | {f(t_run)} | {f(delta)} | "
              f"{int(d[-1, 1] - d[0, 1]) if len(d) else 0:8d} | {order}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
