#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ПЕРЕИГРЫВАНИЕ бэга: старый и новый закон канала курса по ОДНИМ И ТЕМ ЖЕ кадрам.

Зачем именно переигрывание. В полёте канал курса замкнут контуром, и по полётным данным
масштаб датчика не измерить: контур нулит то, что видит. Единственный чистый способ —
прогнать один и тот же записанный поток кадров двумя законами и сравнить оба с истиной
(разворот по gz-одометрии).

Что сравнивается (`yaw_trans_fix` в FlowEstimator):
  СТАРЫЙ — `yaw_flow` = медиана горизонтального потока после снятия roll/pitch. Стоит на
           допущении «в дальней сцене трансляция ≈0», которое ломается движением:
           замер yaw_fidelity дал долю увиденного разворота +0.96 на неподвижном борту и
           −0.09 на трёх осях, где борт идёт 1-4 м/с.
  НОВЫЙ  — свободный член подгонки u(y) = a + b·(y − y_гор): поток от трансляции ∝ 1/Z ∝
           (y − y_гор), от вращения от строки почти не зависит.

Печатает по каждому закону: долю увиденного разворота (∫flow·dt / S против истины),
корреляцию мгновенного сигнала с истинной ω_z и шум сигнала. Доля 1.0 = честно.

⚠️ Нужен бэг С КАДРАМИ (`/image_color`). Серии гоняются с потрошением (`STRIP=1`), в них
кадров нет — прогон под переигрывание пишется отдельно, со `STRIP=0`.

Запуск (в контейнере nav — нужен cv_bridge против CUDA-OpenCV):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash;
    YR_BAG=/root/sim_ws/output/E3f1_bag python3 /lab/yaw_replay.py'
Env: YR_BAG, YR_MAXF (ограничить кадры), YR_IMU (mavros|gz), YR_S (паспорт S, 0.324).
"""
import math
import os
import sys

import numpy as np

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float64

sys.path.insert(0, '/root/sim_ws/src/control')
from control_pkg.perception.flow_estimator import FlowEstimator   # noqa: E402

BAG = os.environ.get('YR_BAG', '/root/sim_ws/output/E3f1_bag')
MAXF = int(os.environ.get('YR_MAXF', 0))
IMU_TOPIC = '/gz_imu/data_flu' if os.environ.get('YR_IMU') == 'gz' else '/mavros/imu/data'
S_PX = float(os.environ.get('YR_S', 0.324))     # px/кадр на °/с — паспорт Y4
CAM_W, CAM_H = 960, 540
# ⚠️ Та же матрица, что у ЛЁТНОЙ ноды (bootstrap_node.FLOW_R) — с наклоном камеры 15°.
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
HOVER_Z = 2.0


def euler(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read(bag):
    br = CvBridge()
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    frames, od, imu, alt = [], [], [], []
    while r.has_next():
        topic, raw, ts = r.read_next()
        if topic == '/image_color':
            m = deserialize_message(raw, Image)
            img = br.imgmsg_to_cv2(m, desired_encoding='bgr8')
            frames.append((stamp(m), cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)))
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((stamp(m), p.z) + euler(m.pose.pose.orientation))
        elif topic == IMU_TOPIC:
            m = deserialize_message(raw, Imu)
            w = m.angular_velocity
            imu.append((stamp(m), w.x, w.y, w.z) + euler(m.orientation))
        elif topic == '/mavros/global_position/rel_alt':
            m = deserialize_message(raw, Float64)
            alt.append((ts * 1e-9, m.data))
    frames.sort(key=lambda f: f[0])
    return frames, np.array(od), np.array(imu), np.array(alt)


def replay(frames, imu, alt, fix):
    est = FlowEstimator(CAM_W / 2.0, CAM_W / 2.0, CAM_W / 2.0, CAM_H / 2.0, FLOW_R,
                        rotflow_sign=1.0, pitch_smooth_n=9, roll_smooth_n=25,
                        yaw_smooth_n=5, yaw_trans_fix=fix)
    ts, fy = [], []
    for t, gray in frames:
        i = int(np.argmin(np.abs(imu[:, 0] - t)))
        a = float(np.interp(t, alt[:, 0], alt[:, 1])) if len(alt) else None
        out = est.process(gray, t, imu[i, 1:4], pitch=imu[i, 5], alt=a)
        if out is None:
            continue
        ts.append(t)
        fy.append(out['yaw_flow'])
    return np.array(ts), np.array(fy)


def main():
    frames, od, imu, alt = read(BAG)
    print(f'бэг {BAG}: кадров {len(frames)}, одометрии {len(od)}, imu {len(imu)} ({IMU_TOPIC})')
    if not len(frames):
        sys.exit('⚠️ в бэге нет /image_color — переигрывать нечего (прогон шёл со STRIP=1)')
    if not len(imu) or not len(od):
        sys.exit('⚠️ нет imu или одометрии')
    # окно висения — как во всех разборах серии
    h = od[od[:, 1] > HOVER_Z]
    t0, t1 = (h[0, 0], h[-1, 0]) if len(h) > 20 else (od[0, 0], od[-1, 0])
    frames = [f for f in frames if t0 <= f[0] <= t1]
    if MAXF:
        frames = frames[:MAXF]
    yaw_t = h[:, 0]
    yaw_d = np.degrees(np.unwrap(h[:, 4]))
    true_turn = yaw_d[-1] - yaw_d[0]
    wz = np.gradient(yaw_d, yaw_t)            # истинная ω_z, °/с
    print(f'окно висения {t1 - t0:.1f} с, кадров в нём {len(frames)}, '
          f'истинный разворот {true_turn:+.0f}°')
    print(f"\n{'закон':10s} | {'∫flow px':>9s} | {'→ градусов':>10s} | {'доля':>5s} | "
          f"{'corr с ω_z':>10s} | {'σ сигнала':>9s}")
    for tag, fix in (('старый', False), ('новый', True)):
        ts, fy = replay(frames, imu, alt, fix)
        if len(ts) < 10:
            print(f'{tag:10s} | кадров мало')
            continue
        integ = float(np.trapz(fy, ts))
        deg = integ / S_PX
        frac = deg / true_turn if abs(true_turn) > 3 else float('nan')
        c = np.corrcoef(np.interp(ts, yaw_t, wz), fy)[0, 1]
        print(f'{tag:10s} | {integ:+9.2f} | {deg:+10.0f} | {frac:5.2f} | '
              f'{c:+10.2f} | {fy.std():9.3f}')
    print('\nдоля 1.0 = сигнал видит разворот один в один; 0 = слеп; минус = ещё и знак не тот')


if __name__ == '__main__':
    main()
