#!/usr/bin/env python3
"""КАЧЕСТВО УДЕРЖАНИЯ — то, что видно на видео, а не итоговая точка.

Зачем отдельно от `axis_split.py`. Тот меряет, ГДЕ борт оказался к концу висения, и по
этой метрике серии `D2s` и `E2` неразличимы (уход +20.9 против +20.2 м). А на видео
разница видна. Значит метрика не та: одинаковый итоговый уход набирается и монотонным
побегом на 3 м/с, и медленным сползанием, которое контур раз за разом придерживает.

Здесь считается ПОВЕДЕНИЕ за висение:
  до 10 м   — сколько секунд борт держался в круге 10 м (бюджет спокойной жизни);
  внутри    — доля времени висения внутри этого круга;
  v_макс    — пиковая горизонтальная скорость (насколько сильно разгонялся);
  v_нач/кон — средняя скорость в первой и последней трети висения: контур со временем
              придерживает борт (кон < нач) или разгоняет (кон > нач);
  возврат   — на сколько метров борт вернулся от самой дальней точки к концу висения
              (>0 = уходил и его притянуло назад, 0 = уехал и не вернулся).

Запуск:
  REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
  docker run --rm -v $REPO/src/lab:/lab:ro \
    -v $REPO/docker/sim/output:/out:ro ros:humble-ros-base bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/hold_quality.py /out/E2s1_bag ...'
"""
import math
import sys

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
HOVER_Z = 2.0
R_CALM = 10.0        # м — круг «спокойной жизни»


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    rows = []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            rows.append((st(m), p.x, p.y, p.z))
    return np.array(rows) if rows else None


def stats(bag):
    a = load(bag)
    if a is None:
        return None
    h = a[a[:, 3] > HOVER_Z]
    if len(h) < 20:
        return None
    t = h[:, 0] - h[0, 0]
    d = np.hypot(h[:, 1] - h[0, 1], h[:, 2] - h[0, 2])
    v = np.abs(np.gradient(d, t))
    calm = t[d > R_CALM][0] if (d > R_CALM).any() else t[-1]
    third = len(t) // 3
    i_max = int(d.argmax())
    return dict(calm=calm, inside=100.0 * (d <= R_CALM).mean(), vmax=v.max(),
                v0=v[:third].mean(), v1=v[-third:].mean(),
                back=max(0.0, d[i_max] - d[-1]), total=t[-1], end=d[-1])


def main(bags):
    print(f"{'прогон':8s} | {'до 10м':>6s} | {'внутри':>6s} | {'v_макс':>6s} | "
          f"{'v_нач':>5s} | {'v_кон':>5s} | {'возврат':>7s} | {'конец':>6s}")
    rows = []
    for bag in bags:
        name = bag.rstrip('/').split('/')[-1].replace('_bag', '')
        s = stats(bag)
        if s is None:
            print(f"{name:8s} | висения нет")
            continue
        print(f"{name:8s} | {s['calm']:5.1f}с | {s['inside']:5.0f}% | {s['vmax']:5.2f} | "
              f"{s['v0']:5.2f} | {s['v1']:5.2f} | {s['back']:6.1f}м | {s['end']:5.1f}м")
        rows.append(s)
    if len(rows) > 1:
        g = lambda k: np.array([r[k] for r in rows])
        n = len(rows)
        print(f"\nсерия (n={n}): до 10м {g('calm').mean():.1f} ± {g('calm').std(ddof=1):.1f} с, "
              f"внутри {g('inside').mean():.0f}%, v_макс {g('vmax').mean():.2f} м/с, "
              f"v_нач {g('v0').mean():.2f} → v_кон {g('v1').mean():.2f} м/с, "
              f"возврат {g('back').mean():.1f} м, вернулись {(g('back') > 1.0).sum()}/{n}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
