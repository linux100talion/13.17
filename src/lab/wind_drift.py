#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""СНОС ПОД ВЕТРОМ: с какой скоростью борт уезжает и какой крен при этом держит.

Вопрос, ради которого написано: П-демпфер скорости против ПОСТОЯННОЙ силы приходит в
равновесие на НЕНУЛЕВОЙ скорости (v_уст = a_ветра/(g·k)) — наклон при этом выходит
ровно правильный, но получить его контур может только из остаточной скорости. Значит
сравнивать надо не «ушёл/не ушёл», а УСТАНОВИВШУЮСЯ СКОРОСТЬ сноса: без управления она
одна, с демпфером должна быть во столько раз меньше, во сколько велик гейн.

Окно: висение выше 2.5 м, с отступом от набора и от начала снижения.
"""
import math
import os
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

print('%-6s| %5s | %7s | %7s | %7s | %7s | %6s | %6s'
      % ('прогон', 'висел', 'уход м', 'v_уст', 'v_кон', 'курс°', 'PWMкр', 'PWMтг'))
for b in sys.argv[1:]:
    u = '/out/%s_bag' % b
    if not os.path.isdir(u):
        print('%-6s| нет бэга' % b)
        continue
    r = SequentialReader()
    r.open(StorageOptions(uri=u, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, d1, d2 = [], [], []
    while r.has_next():
        t, raw, _ = r.read_next()
        if t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((st(m), p.x, p.y, p.z, yaw(m.pose.pose.orientation)))
        elif t == '/flow_dbg':       # x = roll_off (PWM от центра)
            m = deserialize_message(raw, Vector3Stamped)
            d1.append((st(m), m.vector.x))
        elif t == '/flow_dbg2':      # x = pitch_off
            m = deserialize_message(raw, Vector3Stamped)
            d2.append((st(m), m.vector.x))
    if not od:
        print('%-6s| нет одометрии' % b)
        continue
    od = np.array(od)
    g = np.arange(od[0, 0], od[-1, 0], 0.05)
    z = np.interp(g, od[:, 0], od[:, 3])
    x = np.interp(g, od[:, 0], od[:, 1])
    y = np.interp(g, od[:, 0], od[:, 2])
    hd = np.degrees(np.interp(g, od[:, 0], np.unwrap(od[:, 4])))
    idx = np.nonzero(z > 2.5)[0]
    if len(idx) < 100:
        print('%-6s| висения нет (выше 2.5 м < 5 с)' % b)
        continue
    S = slice(idx[0] + 40, idx[-1] - 40)
    xs, ys, ts = x[S], y[S], g[S]
    x0, y0 = xs[0], ys[0]
    dist = np.hypot(xs - x0, ys - y0)
    sp = np.hypot(np.gradient(xs, 0.05), np.gradient(ys, 0.05))
    # УСТАНОВИВШАЯСЯ скорость — по второй половине окна: первая занята разгоном от нуля
    # до равновесия, и среднее по всему окну её занижает.
    h = len(sp) // 2
    pwm = lambda d: (np.mean(np.abs(np.interp(ts, np.array(d)[:, 0], np.array(d)[:, 1])))
                     if len(d) > 10 else float('nan'))
    print('%-6s| %5.0f | %7.1f | %7.2f | %7.2f | %+7.1f | %6.1f | %6.1f'
          % (b, ts[-1] - ts[0], dist[-1], sp[h:].mean(), sp[-20:].mean(),
             hd[S][-1] - hd[S][0], pwm(d1), pwm(d2)))
