#!/usr/bin/env python3
"""flight_timeline.py — ПОКАДРОВАЯ лента одного прогона: истинная скорость, канал вида сверху,
команда тангажа/крена, высота. Нужна, чтобы отличить «гейт закрыт» от «ось командует,
но не справляется»: по средним это неразличимо."""
import math
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
name = sys.argv[1]
bag = name if '/' in name else f'/out/{name}_bag'
step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id='sqlite3'),
       ConverterOptions('cdr', 'cdr'))
D = {k: [] for k in ('d1', 'd2', 'd8', 'd9', 'od')}
M = {'/flow_dbg': 'd1', '/flow_dbg2': 'd2', '/flow_dbg8': 'd8', '/flow_dbg9': 'd9'}
while r.has_next():
    t, raw, _ = r.read_next()
    if t in M:
        m = deserialize_message(raw, Vector3Stamped)
        D[M[t]].append((st(m), m.vector.x, m.vector.y, m.vector.z))
    elif t == '/model/iris_cam/odometry':
        m = deserialize_message(raw, Odometry)
        p, v = m.pose.pose.position, m.twist.twist.linear
        D['od'].append((st(m), p.x, p.y, p.z, math.hypot(v.x, v.y), v.z))
D = {k: np.array(v) for k, v in D.items()}
od = D['od']
i0 = int(np.argmax(od[:, 3] > 0.5))
t0 = od[i0, 0]
print(f'{name}: t=с от отрыва')
print(f'{"t":>5} | {"H":>5} | {"vz":>5} | {"V ист":>6} | {"ipm впер":>8} | {"ipm вбок":>8} | '
      f'{"ok":>3} | {"тангаж":>6} | {"крен":>5} | {"путь":>5}')
for t in np.arange(0.0, min(30.0, od[-1, 0] - t0), step):
    s = lambda a, c: (float(np.interp(t0 + t, a[:, 0], a[:, c])) if len(a) else float('nan'))
    d = math.hypot(s(od, 1) - od[i0, 1], s(od, 2) - od[i0, 2])
    print(f'{t:5.1f} | {s(od, 3):5.1f} | {s(od, 5):+5.2f} | {s(od, 4):6.2f} | '
          f'{s(D["d8"], 2):8.2f} | {s(D["d9"], 1):8.2f} | {s(D["d8"], 3):3.0f} | '
          f'{s(D["d2"], 1):6.0f} | {s(D["d1"], 1):5.0f} | {d:5.1f}')
