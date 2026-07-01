#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yaw_check — оценка удержания КУРСА по ИСТИННОЙ позе Gazebo (оракул) для тюнинга YAW-hold.

Аналог drift_check.py, но по yaw: считает СКО / размах / дрейф истинного курса в
БЕЗОПАСНОМ окне полёта. Метрика тюнинга визуального yaw-hold (фаза 2): свипаем гейны
`BS_YAWH_KP/KI/SMOOTH`, минимизируем СКО yaw в окне. Ниже СКО/размах — тем ровнее держим.

Окно как в drift_check: [взлёт (Z>Z_TO), +SAFE_SEC], чтобы не ловить транзиенты
набора/посадки (они дают ложный большой размах). Дрейф = yaw(конец)−yaw(старт окна).

Запускать В КОНТЕЙНЕРЕ nav:
  docker exec -i -e SAFE_SEC=30 p1317_nav bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/yaw_check.py'

Env: SCENE_BAG (default /root/sim_ws/output/scene_bag), ODOM_TOPIC
(/model/iris_cam/odometry), SAFE_SEC (30), Z_TO (2.0). Опц. WIN_T0/WIN_T1 —
абсолютное окно (sim-сек от старта bag), переопределяет взлёт+SAFE_SEC.
"""

import math
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

BAG = os.environ.get('SCENE_BAG', '/root/sim_ws/output/scene_bag')
ODOM = os.environ.get('ODOM_TOPIC', '/model/iris_cam/odometry')
SAFE_SEC = float(os.environ.get('SAFE_SEC', '30'))
Z_TO = float(os.environ.get('Z_TO', '2.0'))


def unwrap_deg(yaw_deg):
    """Разворачиваем ±180° скачки, чтобы СКО/дрейф считались непрерывно."""
    return np.degrees(np.unwrap(np.radians(yaw_deg)))


def main():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    T, Z, YAW = [], [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == ODOM:
            m = deserialize_message(data, Odometry)
            q = m.pose.pose.orientation
            T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            Z.append(m.pose.pose.position.z)
            YAW.append(math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                               1 - 2 * (q.y * q.y + q.z * q.z))))
    if not T:
        raise RuntimeError(f'нет сообщений {ODOM} в {BAG}')
    T = np.array(T) - T[0]
    Z = np.array(Z)
    YAW = unwrap_deg(np.array(YAW))

    to = np.where(Z > Z_TO)[0]
    if len(to) == 0:
        print(f'дрон не взлетел (Z<{Z_TO})'); return
    t_to = T[to[0]]
    t_end = t_to + SAFE_SEC
    w0 = os.environ.get('WIN_T0'); w1 = os.environ.get('WIN_T1')
    cut = f'взлёт+{SAFE_SEC:.0f}с'
    if w0 and w1:
        t_to, t_end = float(w0), float(w1); cut = 'WIN_T0/T1'
    win = (T >= t_to) & (T <= t_end)
    y = YAW[win]
    if len(y) < 2:
        print('пусто в окне'); return

    y0 = y - np.median(y)   # относительно медианы окна (СКО не зависит от абс. курса)
    print(f'окно [{t_to:.1f},{t_end:.1f}]s ({t_end - t_to:.1f}s, {cut}) | сэмплов {len(y)}')
    print()
    print(f'{"метрика курса (yaw)":<34}{"значение":>12}')
    print('-' * 46)
    print(f'{"СКО yaw (ровность удержания)":<34}{y.std():>10.2f}°')
    print(f'{"размах yaw (max−min)":<34}{y.max() - y.min():>10.2f}°')
    print(f'{"дрейф yaw (конец−старт окна)":<34}{y[-1] - y[0]:>+10.2f}°')
    print(f'{"средний курс окна":<34}{np.median(y):>+10.2f}°')
    print('\n(ниже СКО/размах — ровнее держим; дрейф → знак/ki; спин/рост → osign неверный)')


if __name__ == '__main__':
    main()
