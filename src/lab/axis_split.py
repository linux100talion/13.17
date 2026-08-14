#!/usr/bin/env python3
"""Уход за висение, разложенный на ПРОДОЛЬНЫЙ и БОКОВОЙ — и форма траектории.

Зачем отдельно от `hover_stats.py`. Тот меряет уход одним числом (радиусом), а на трёх
осях по `Dp*` вопрос другой: КАКАЯ ось уезжает. В `D2s` (10 прогонов) 77-93% ухода
оказались продольными (+20.9 м вперёд против +3.8 м вбок при выключенном ветре) — это и
навело на опорный канал тангажа, а не на крен.

Второй столбец, ради которого скрипт есть: РАЗМАХ против КОНЦА. Если размах много больше
конечного смещения — борт уходил и возвращался (контур помнит точку удержания и работает,
пусть и с перерегулированием). Если они равны — уехал монотонно, то есть контур гасит, но
дома не помнит. Свип E1 различил случаи именно так: на kf_alt_max 0.06/0.15 размах равен
концу, на 0.25 — 30 м размаха при +13 конца (разворот через ноль).

Оси считаются в системе борта на ПЕРВОМ кадре висения (курс там же), окно — z > 2.0 м,
чтобы не считать взлёт и накат при посадке.

Запуск (нода не нужна, стенд между сериями лежит):
  REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
  docker run --rm -v $REPO/src/lab:/lab:ro \
    -v $REPO/docker/sim/output:/out:ro ros:humble-ros-base bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/axis_split.py /out/E2s1_bag ...'

⚠️ Принимает ПОЛНЫЙ путь к бэгу (`/out/<имя>_bag`), не голое имя.
"""
import math
import sys

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
HOVER_Z = 2.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    rows = []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            rows.append((st(m), p.x, p.y, p.z, yaw_of(m.pose.pose.orientation)))
    return np.array(rows) if rows else None


def main(bags):
    print(f"{'прогон':10s} | {'уход м':>7s} | {'вперёд':>7s} | {'вбок':>7s} | "
          f"{'размах':>7s} | {'висение с':>9s}")
    rows = []
    for bag in bags:
        name = bag.rstrip('/').split('/')[-1].replace('_bag', '')
        a = load(bag)
        h = a[a[:, 3] > HOVER_Z] if a is not None else np.empty((0, 5))
        if len(h) < 10:
            print(f"{name:10s} | висения нет (z > {HOVER_Z} м не набралось)")
            continue
        x0, y0, yaw0 = h[0, 1], h[0, 2], h[0, 4]
        dx, dy = h[:, 1] - x0, h[:, 2] - y0
        fwd = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        t = h[:, 0] - h[0, 0]
        print(f"{name:10s} | {np.hypot(dx, dy)[-1]:7.2f} | {fwd[-1]:+7.2f} | {lat[-1]:+7.2f} | "
              f"{fwd.max() - fwd.min():7.2f} | {t[-1]:9.1f}")
        k = max(1, len(t) // 12)
        print(f"           продольно: min {fwd.min():+.1f} (t={t[fwd.argmin()]:.0f}с)  "
              f"max {fwd.max():+.1f} (t={t[fwd.argmax()]:.0f}с)  |  "
              + ' '.join(f'{v:+.0f}' for v in fwd[::k]))
        rows.append((fwd[-1], lat[-1]))
    if len(rows) > 1:
        f = np.array([r[0] for r in rows])
        l = np.array([r[1] for r in rows])
        print(f"\nпо серии (n={len(rows)}): вперёд {f.mean():+.2f} ± {f.std(ddof=1):.2f} м, "
              f"вбок {l.mean():+.2f} ± {l.std(ddof=1):.2f} м, "
              f"продольная доля {abs(f).sum() / (abs(f).sum() + abs(l).sum()) * 100:.0f}%")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
