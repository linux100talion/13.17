#!/usr/bin/env python3
"""A/B метрики нырка LOITER (ab_loiteryaw): по bag печатает эпизоды yaw-стика
в LOITER с наклонами (истина Gazebo), темпом рыскания на стик и скоростью.
Запуск: python3 ab_metrics.py /root/sim_ws/output/joystick/<RUN>/bag/scene_bag_0.db3
"""
import math
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
db = sqlite3.connect(BAG)
topics = {name: (tid, get_message(typ)) for tid, name, typ in
          db.execute('SELECT id, name, type FROM topics')}


def read(topic):
    tid, mt = topics[topic]
    for ts, data in db.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (tid,)):
        yield ts * 1e-9, deserialize_message(data, mt)


def rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


joy_t, joy_yaw, joy_pitch = [], [], []
for t, m in read('/joy'):
    joy_t.append(t); joy_yaw.append(-m.axes[3]); joy_pitch.append(m.axes[1])
joy_t = np.array(joy_t); joy_yaw = np.array(joy_yaw); joy_pitch = np.array(joy_pitch)
t0 = joy_t[0]

mode_t, mode_v = [], []
for t, m in read('/mavros/state'):
    if not mode_v or m.mode != mode_v[-1]:
        mode_t.append(t); mode_v.append(m.mode)


def mode_at(t):
    i = np.searchsorted(mode_t, t) - 1
    return mode_v[max(i, 0)]


gt_t, gt_p, gt_r, gt_yawu, gt_v, gt_x, gt_y = [], [], [], [], [], [], []
for t, m in read('/model/iris_cam/odometry'):
    r, p, y = rpy(m.pose.pose.orientation)
    gt_t.append(t); gt_p.append(math.degrees(p)); gt_r.append(math.degrees(r))
    gt_yawu.append(y)
    gt_v.append(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
    gt_x.append(m.pose.pose.position.x); gt_y.append(m.pose.pose.position.y)
gt_t = np.array(gt_t); gt_p = np.array(gt_p); gt_r = np.array(gt_r)
gt_yaw = np.degrees(np.unwrap(np.array(gt_yawu))); gt_v = np.array(gt_v)
gt_x = np.array(gt_x); gt_y = np.array(gt_y)

# --- эпизоды yaw-стика ---
active = np.abs(joy_yaw) > 0.2
episodes = []
i = 0
while i < len(joy_t):
    if active[i]:
        j = i
        last = i
        while j < len(joy_t) - 1 and joy_t[j + 1] - joy_t[last] < 0.8:
            j += 1
            if active[j]:
                last = j
        episodes.append((joy_t[i], joy_t[last]))
        i = j + 1
    else:
        i += 1

print(f'{"t0":>7} {"dur":>4} {"mode":>9} {"ystick":>6} {"pstick":>6} '
      f'{"rate°/с":>8} {"Δyaw°":>6} {"|pit|max":>8} {"|rol|max":>8} '
      f'{"tilt":>5} {"vmax":>5}')
rows = []
for a, b in episodes:
    b2 = b + 2.5
    m_gt = (gt_t >= a) & (gt_t <= b2)
    m_joy = (joy_t >= a) & (joy_t <= b)
    if not m_gt.any() or not m_joy.any():
        continue
    ys = joy_yaw[m_joy][np.argmax(np.abs(joy_yaw[m_joy]))]
    ps = joy_pitch[m_joy][np.argmin(joy_pitch[m_joy])]
    # темп рыскания: Δкурс за само нажатие / длительность
    ya = np.interp(a, gt_t, gt_yaw); yb = np.interp(b + 0.15, gt_t, gt_yaw)
    dur = max(b - a, 0.1)
    rate = (yb - ya) / dur
    pit = np.abs(gt_p[m_gt]).max(); rol = np.abs(gt_r[m_gt]).max()
    tilt = np.degrees(np.arccos(
        np.cos(np.radians(gt_p[m_gt])) * np.cos(np.radians(gt_r[m_gt])))).max()
    vmax = gt_v[m_gt].max()
    mode = mode_at(a)
    rows.append((mode, ys, pit, rol, tilt, rate))
    print(f'{a - t0:7.1f} {b - a:4.1f} {mode:>9} {ys:6.2f} {ps:6.2f} '
          f'{rate:8.1f} {yb - ya:6.0f} {pit:8.1f} {rol:8.1f} {tilt:5.1f} {vmax:5.2f}')

lo = [r for r in rows if r[0] == 'LOITER']
if lo:
    print(f'\nLOITER: эпизодов {len(lo)}, макс tilt {max(r[4] for r in lo):.1f}°, '
          f'макс |pitch| {max(r[2] for r in lo):.1f}°, '
          f'макс |roll| {max(r[3] for r in lo):.1f}°')
    print('авторитет yaw (°/с на ед. стика): ' +
          ', '.join(f'{abs(r[5] / r[1]):.0f}' for r in lo if abs(r[1]) > 0.3))
print(f'вся выборка LOITER-фаз: макс скорость '
      f'{gt_v[[mode_at(t) == "LOITER" for t in gt_t]].max():.2f} м/с' if len(gt_t) else '')
print(f'смещение от старта макс: '
      f'{np.hypot(gt_x - gt_x[0], gt_y - gt_y[0]).max():.1f} м')
