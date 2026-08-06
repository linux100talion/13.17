#!/usr/bin/env python3
"""ГДЕ накапливается уход — на НАБОРЕ высоты или на ВИСЕНИИ?

Разбор E2 упёрся в наблюдение: счётчик сегментов (`/flow_dbg4.x`) обнуляется в середине
прогона. Обнуляет его только `reset_keyframe()`, а его зовёт вход в шаг миссии
(`plan/step.py`, «начало сегмента = точка удержания»). Значит по этому обнулению видна
граница climb3 → hover_1, и уход можно разложить по шагам.

Это важно, потому что диагноз «контур не держит висение» и «контур выбросил борт ещё на
наборе, а висение началось уже за 20 м от дома» дают РАЗНЫЕ следующие шаги, а по суммарному
уходу за прогон они неразличимы. Забор считается от точки взлёта, поэтому прогон, потерявший
25 м на наборе, падает за забор сразу после входа в висение — и выглядит как провал висения.

Печатает по каждому прогону: смещение к моменту границы, смещение за висение после неё,
и обе скорости.

Запуск:
  docker run --rm -v /root/13.17/src/lab:/lab:ro \
    -v /root/13.17/docker/sim/output:/out:ro ros:humble-ros-base bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/step_split.py /out/E2s1_bag ...'
"""
import math
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


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
            d4.append((st(m), m.vector.x))
    return (np.array(od) if od else None), (np.array(d4) if d4 else None)


def main(bags):
    print("по каждому прогону: [t_старта высота_нач→кон] пройдено / длительность = скорость")
    rows = []
    for bag in bags:
        name = bag.rstrip('/').split('/')[-1].replace('_bag', '')
        od, d4 = load(bag)
        if od is None or d4 is None or len(d4) < 5:
            print(f"{name:8s} | данных нет")
            continue
        # границы шагов = ВСЕ обнуления счётчика сегментов (climb / hover / land)
        segs = d4[:, 1]
        drops = np.where((segs[1:] == 0) & (segs[:-1] > 0))[0]
        if len(drops) == 0:
            print(f"{name:8s} | границ шагов не видно (счётчик не обнулялся)")
            continue
        fly = od[od[:, 3] > 0.5]
        if len(fly) < 10:
            print(f"{name:8s} | полёта нет")
            continue
        bounds = [0] + [int(np.searchsorted(fly[:, 0], d4[i + 1, 0])) for i in drops]
        bounds = sorted({min(max(b, 0), len(fly) - 1) for b in bounds} | {len(fly) - 1})
        d = lambda a, b: math.hypot(fly[b, 1] - fly[a, 1], fly[b, 2] - fly[a, 2])
        segs_out = []
        for a, b in zip(bounds[:-1], bounds[1:]):
            dt = fly[b, 0] - fly[a, 0]
            if dt < 1.0:
                continue
            segs_out.append((fly[a, 0] - fly[0, 0], d(a, b), dt, fly[a, 3], fly[b, 3]))
        if len(segs_out) < 2:
            print(f"{name:8s} | шагов меньше двух")
            continue
        print(f"{name:8s} |" + " ".join(
            f" [{t0:4.0f}с {z0:.1f}→{z1:.1f}м] {m:5.1f}м / {dt:4.1f}с = {m / dt:4.2f} м/с |"
            for t0, m, dt, z0, z1 in segs_out))
        rows.append((segs_out[0][1], sum(s[1] for s in segs_out[1:])))
    if len(rows) > 1:
        c = np.array([r[0] for r in rows])
        v = np.array([r[1] for r in rows])
        print(f"\nпо серии (n={len(rows)}): на наборе {c.mean():.1f} ± {c.std(ddof=1):.1f} м, "
              f"за висение {v.mean():.1f} ± {v.std(ddof=1):.1f} м, "
              f"доля набора {c.sum() / (c.sum() + v.sum()) * 100:.0f}%")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
