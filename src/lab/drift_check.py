#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drift_check — оценка дрейфа по ИСТИННОЙ позе Gazebo (оракул) для тюнинга FLOW-DAMP.

Раскладывает горизонтальный дрейф дрона на ПРОДОЛЬНУЮ (pitch) и БОКОВУЮ (roll) оси
по курсу и считает RMS скорости + смещение в БЕЗОПАСНОМ окне полёта. В v0 демпфер
гасит только боковую ось (ROLL), продольная (pitch=центр) не управляется → служит
встроенным baseline для условий ИМЕННО этого прогона. Нормированная метрика
`боковая/продольная RMS_v` (≈1.0 = нет демпфирования, ниже = лучше) почти не зависит
от выбора окна и не требует отдельного baseline-прогона.

ВАЖНО (окно): дрон взлетает ~10-я sim-сек, дальше лишь ~10 с безопасного полёта —
потом ALT_HOLD-дрейф уводит его за край сцены / в краш. Эти сэмплы НЕ должны попасть
в статистику. Окно = [взлёт (Z>Z_TO), +SAFE_SEC] с ДОСРОЧНЫМ обрезанием при выходе за
радиус R_SAFE (вылет/деградация фич). Проекция скорости — per-sample yaw (без смаза
от ухода курса).

Запускать В КОНТЕЙНЕРЕ nav (нужен rosbag2_py):
  docker exec -i p1317_nav bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/drift_check.py'

Env: SCENE_BAG (default /root/sim_ws/output/scene_bag), ODOM_TOPIC
(/model/iris_cam/odometry), SAFE_SEC (10), R_SAFE (25, м), Z_TO (2.0, м).
"""

import math
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

BAG = os.environ.get('SCENE_BAG', '/root/sim_ws/output/scene_bag')
ODOM = os.environ.get('ODOM_TOPIC', '/model/iris_cam/odometry')
SAFE_SEC = float(os.environ.get('SAFE_SEC', '10'))   # сек безопасного полёта после взлёта
R_SAFE = float(os.environ.get('R_SAFE', '25'))       # м: дальше — обрезаем окно (вылет)
Z_TO = float(os.environ.get('Z_TO', '2.0'))          # м: порог «взлетел»


def main():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    T, X, Y, Z, YAW = [], [], [], [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == ODOM:
            m = deserialize_message(data, Odometry)
            T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            X.append(p.x); Y.append(p.y); Z.append(p.z)
            YAW.append(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z)))
    if not T:
        raise RuntimeError(f'нет сообщений {ODOM} в {BAG}')
    T, X, Y, Z, YAW = map(np.array, (T, X, Y, Z, YAW))
    t = T - T[0]

    to_idx = np.where(Z > Z_TO)[0]
    if len(to_idx) == 0:
        print(f'дрон не взлетел (Z<{Z_TO})'); return
    i_to = to_idx[0]; t_to = t[i_to]
    x0, y0 = X[i_to], Y[i_to]
    r_h = np.hypot(X - x0, Y - y0)

    # конец окна: t_to+SAFE_SEC ИЛИ первый выход за R_SAFE (что раньше)
    t_end = t_to + SAFE_SEC
    cut = 'время'
    out = np.where((t > t_to) & (r_h > R_SAFE))[0]
    if len(out) > 0 and t[out[0]] < t_end:
        t_end = t[out[0]]; cut = f'радиус R_SAFE={R_SAFE:.0f}м'
    win = (t >= t_to) & (t <= t_end)
    iw = np.where(win)[0]
    print(f'взлёт t={t_to:.1f}s (Z>{Z_TO}) | безопасное окно [{t_to:.1f},{t_end:.1f}]s '
          f'({t_end - t_to:.1f}s, обрезано по: {cut})')
    print(f'сэмплов в окне: {win.sum()} | высота {Z[iw].min():.1f}..{Z[iw].max():.1f}м | '
          f'гориз.дрейф от взлёта макс {r_h[iw].max():.1f}м')
    print(f'курс yaw в окне: {math.degrees(YAW[iw].min()):+.0f}..{math.degrees(YAW[iw].max()):+.0f}° '
          f'(размах {math.degrees(YAW[iw].max() - YAW[iw].min()):.0f}°)')

    # per-sample yaw: проекция МГНОВЕННОЙ скорости в тело (без смаза от ухода курса)
    xa, ya, ta, yawa = X[iw], Y[iw], t[iw], YAW[iw]
    dt = np.diff(ta); vx = np.diff(xa) / dt; vy = np.diff(ya) / dt
    ym = 0.5 * (yawa[:-1] + yawa[1:])
    c, s = np.cos(ym), np.sin(ym)
    vf = vx * c + vy * s        # продольная (pitch)
    vr = -vx * s + vy * c       # боковая (roll, демпфер)

    def rms(a):
        return float(np.sqrt(np.mean(a ** 2)))

    # смещение в теле к концу окна (вектор старт→конец, проекция на средний курс)
    ymean = np.median(yawa)
    dxx, dyy = xa[-1] - xa[0], ya[-1] - ya[0]
    disp_f = dxx * math.cos(ymean) + dyy * math.sin(ymean)
    disp_r = -dxx * math.sin(ymean) + dyy * math.cos(ymean)
    print()
    print(f'{"ось":<30}{"RMS скор м/с":>14}{"смещ за окно":>16}')
    print('-' * 60)
    print(f'{"продольная (pitch, baseline)":<30}{rms(vf):>14.2f}{disp_f:>14.1f}м')
    print(f'{"боковая (roll, ДЕМПФЕР)":<30}{rms(vr):>14.2f}{disp_r:>14.1f}м')
    ratio = rms(vr) / max(1e-6, rms(vf))
    print(f'\nНОРМ. МЕТРИКА боковая/продольная RMS_v = {ratio:.2f}  '
          f'(≈1.0 нет демпф., ниже лучше; дефолты kp8/ki2 ≈ 0.21)')


if __name__ == '__main__':
    main()
