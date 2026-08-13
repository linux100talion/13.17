#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ЧТО ЗВЕНИТ ПО КУРСУ: собственный контур оси или снос, перетекающий в её сигнал.

Вопрос, ради которого написано. Под ветром курс раскачивает, и причин ровно две, а
лечатся они противоположно:
  1. СОБСТВЕННЫЙ ЗВОН контура — гейна слишком много для фазы, которая осталась. Тогда
     звон растёт вместе с НАГРУЗКОЙ и виден как насыщение курсового PWM.
  2. ФАНТОМ ОТ СНОСА — flow_yaw это медиана горизонтального сдвига картинки, и боковая
     скорость даёт её так же, как разворот. Тогда контур честно отрабатывает поворот,
     которого нет, звон идёт СИНХРОННО с боковой скоростью, и растёт он не с силой
     ветра, а с тем, сколько ветра достаётся БОКОВОЙ оси.
Разделяет их пара прогонов «сильный ветер в нос» / «слабый по диагонали»: у механизма 1
хуже там, где ветер сильнее, у механизма 2 — там, где он косой.

Колонки:
  СКОкурс  СКО истинного курса в окне висения, град (сам факт раскачки)
  разм     размах истинного курса, град
  СКОωz    СКО истинной угловой скорости, град/с
  T        период колебания курса по переходам через среднее, с (nan = нет колебания)
  сат      доля кадров, где курсовой PWM упёрся в потолок (насыщение контура)
  |PWM|    средний модуль курсового PWM
  corr_ωz  связь flow_yaw с ИСТИННОЙ ωz: 1 = сигнал честный, 0 = сигнал не про поворот
  фант     доля дисперсии flow_yaw, НЕ объяснённая истинной ωz (= 1 − corr²)
  corr_бок связь ОСТАТКА flow_yaw (за вычетом истинной ωz) с БОКОВОЙ скоростью —
           прямая улика механизма 2
  vбок/vвп средние скорости в осях борта, м/с (какая ось нагружена)
"""
import math
import os
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def period(sig, dt):
    """Период по переходам через среднее. Меряется на ДЕТРЕНДЕННОМ сигнале: под ветром
    курс ещё и медленно уползает, и без снятия тренда уползание съело бы переходы."""
    d = sig - np.mean(sig)
    if np.std(d) < 1e-9:
        return float('nan')
    cr = np.nonzero(np.diff(np.sign(d)))[0]
    return 2.0 * (cr[-1] - cr[0]) * dt / (len(cr) - 1) if len(cr) >= 3 else float('nan')


def corr(a, b):
    if len(a) < 10 or np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float('nan')
    return float(np.corrcoef(a, b)[0, 1])


print('%-6s| %7s | %6s | %6s | %5s | %5s | %6s | %7s | %5s | %8s | %6s | %6s'
      % ('прогон', 'СКОкурс', 'разм', 'СКОωz', 'T', 'сат', '|PWM|',
         'corr_ωz', 'фант', 'corr_бок', 'vбок', 'vвп'))
for b in sys.argv[1:]:
    u = '/out/%s_bag' % b
    if not os.path.isdir(u):
        print('%-6s| нет бэга' % b)
        continue
    r = SequentialReader()
    r.open(StorageOptions(uri=u, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, fy, y6, wz = [], [], [], []
    while r.has_next():
        t, raw, _ = r.read_next()
        if t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((st(m), p.x, p.y, p.z, yaw(m.pose.pose.orientation)))
        elif t == '/flow_dbg2':          # z = flow_yaw (сигнал канала курса)
            m = deserialize_message(raw, Vector3Stamped)
            fy.append((st(m), m.vector.z))
        elif t == '/flow_dbg6':          # z = курсовой PWM
            m = deserialize_message(raw, Vector3Stamped)
            y6.append((st(m), m.vector.z))
        elif t == '/gz_imu/data_flu':    # истинная ωz (FLU, без шума FCU)
            m = deserialize_message(raw, Imu)
            wz.append((st(m), m.angular_velocity.z))
    if not od:
        print('%-6s| нет одометрии' % b)
        continue
    od = np.array(od)
    dt = 0.05
    g = np.arange(od[0, 0], od[-1, 0], dt)
    z = np.interp(g, od[:, 0], od[:, 3])
    x = np.interp(g, od[:, 0], od[:, 1])
    y = np.interp(g, od[:, 0], od[:, 2])
    hd = np.degrees(np.interp(g, od[:, 0], np.unwrap(od[:, 4])))
    idx = np.nonzero(z > 2.5)[0]
    if len(idx) < 100:
        print('%-6s| висения нет' % b)
        continue
    S = slice(idx[0] + 40, idx[-1] - 40)
    ts, h = g[S], hd[S]
    vx, vy = np.gradient(x[S], dt), np.gradient(y[S], dt)
    hr = np.radians(h)
    vfwd = vx * np.cos(hr) + vy * np.sin(hr)
    vlat = -vx * np.sin(hr) + vy * np.cos(hr)

    grid = lambda d: (np.interp(ts, np.array(d)[:, 0], np.array(d)[:, 1])
                      if len(d) > 10 else np.full_like(ts, np.nan))
    s_fy, s_pwm, s_wz = grid(fy), grid(y6), np.degrees(grid(wz))

    sat = (np.mean(np.abs(s_pwm) >= 149.0) if np.isfinite(s_pwm).all()
           else float('nan'))
    c_wz = corr(s_fy, s_wz)
    # ОСТАТОК: что осталось в сигнале курса после вычитания честной ωz (в МНК-масштабе).
    # Именно он должен коррелировать с боковой скоростью, если сигнал ловит снос.
    if np.isfinite(s_fy).all() and np.isfinite(s_wz).all() and np.std(s_wz) > 1e-9:
        k = np.polyfit(s_wz, s_fy, 1)
        res = s_fy - np.polyval(k, s_wz)
        c_lat = corr(res, vlat)
    else:
        c_lat = float('nan')

    print('%-6s| %7.2f | %6.1f | %6.2f | %5.1f | %5.2f | %6.1f | %7.2f | %5.2f | %+8.2f '
          '| %+6.2f | %+6.2f'
          % (b, np.std(h), h.max() - h.min(), np.std(s_wz), period(h, dt), sat,
             np.mean(np.abs(s_pwm)), c_wz, 1 - c_wz ** 2 if np.isfinite(c_wz) else np.nan,
             c_lat, vlat.mean(), vfwd.mean()))
