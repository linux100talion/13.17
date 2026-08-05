"""Разложение выхода ПРОДОЛЬНОГО контура на П и Д (как в таблице ToDo4.md).

Д-член считается прямо: kd * kf_vel (/flow_dbg3.y). П — остаток от выхода
(/flow_dbg2.x), потому что уставка контура в бэг не пишется.
"""
import math, os, sys
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from geometry_msgs.msg import Vector3Stamped

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
KD = float(os.environ.get('KD', '5000'))


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, d2, d3 = [], [], []
    while r.has_next():
        t, raw, ts = r.read_next()
        if t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            od.append((st(m), m.pose.pose.position.z))
        elif t == '/flow_dbg2':
            m = deserialize_message(raw, Vector3Stamped)
            d2.append((st(m), m.vector.x))
        elif t == '/flow_dbg3':
            m = deserialize_message(raw, Vector3Stamped)
            d3.append((st(m), m.vector.x, m.vector.y, m.vector.z))
    return np.array(od), np.array(d2), np.array(d3)


print(f'{"прогон":8} | {"kd":>5} | {"Д СКО":>6} | {"Д пик":>6} | {"П СКО":>6} | '
      f'{"|kf_vel|":>8} | {"вых СКО":>7} | {"перевес Д":>9}')
for bag in sys.argv[1:]:
    kd = 1500.0 if 'N1s' in bag else KD
    od, d2, d3 = load(bag)
    if not len(od) or not len(d3):
        print(f'{os.path.basename(bag)}: пусто'); continue
    z = od[:, 1]
    hi = z > 0.9 * np.percentile(z, 90)
    t0 = od[int(np.argmax(hi)), 0]
    sel3 = (d3[:, 0] >= t0) & (d3[:, 0] <= t0 + 40)
    t3, vel = d3[sel3, 0], d3[sel3, 2]
    out = np.interp(t3, d2[:, 0], d2[:, 1])
    dterm = np.clip(kd * vel, -150, 150)
    pterm = out - dterm
    dom = 100.0 * np.mean(np.abs(dterm) > np.abs(pterm))
    print(f'{os.path.basename(bag).replace("_bag",""):8} | {kd:5.0f} | {np.std(dterm):6.0f} | '
          f'{np.percentile(np.abs(dterm),99):6.0f} | {np.std(pterm):6.0f} | '
          f'{np.mean(np.abs(vel)):8.4f} | {np.std(out):7.0f} | {dom:8.0f}%')
