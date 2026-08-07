#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ГДЕ продольный канал МЕНЯЕТ ЗНАК: разбор накопителя по СЕГМЕНТАМ.

Зачем. После правки канала курса борт стал разгонять себя сам (пиковая скорость 3.8 →
5.9 м/с, в одном прогоне 11.4). Знаки проводки при этом верны: PWM>центр = нос вверх =
разгон назад, и corr(PWM, смещение) положителен, то есть выход тормозит. Ломается САМ
СИГНАЛ: в E3s5 corr(kf_logs, смещение) = −0.77 — канал показывает «уехал назад», когда
борт едет вперёд, и контур честно толкает ПО движению. Это положительная обратная связь.

Механизм, который проверяем. `kf_logs` наружу = `kf_acc + текущий сегмент`, где `kf_acc` —
СУММА закрытых сегментов. Ошибка знака в одном закрытом сегменте уезжает в накопитель
НАВСЕГДА: следующие сегменты считаются от испорченной опоры. Значит инверсию канала надо
искать не в кадре, а в конкретном сегменте.

Скрипт переигрывает бэг боевым FlowEstimator и на каждом закрытии сегмента сравнивает
ЗАСЧИТАННОЕ значение с ИСТИННЫМ продольным перемещением за тот же интервал (интеграл
проекции скорости на нос борта — сигнал живёт в связанной системе). Печатает по сегментам
крутизну и признак инверсии, и итог: сколько сегментов ушло в накопитель с ВЕРНЫМ знаком,
сколько с обратным, и сколько метров «наврал» накопитель.

⚠️ Нужен бэг С КАДРАМИ (`/image_color`) — серии гоняются с потрошением.

Запуск (в контейнере nav — нужен cv_bridge):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash;
    KS_BAG=/root/sim_ws/output/E3f1_bag python3 /lab/kf_sign_check.py'
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

BAG = os.environ.get('KS_BAG', '/root/sim_ws/output/E3f1_bag')
IMU_TOPIC = '/gz_imu/data_flu' if os.environ.get('KS_IMU') == 'gz' else '/mavros/imu/data'
CAM_W, CAM_H = 960, 540
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
HOVER_Z = 2.0
MIN_M = 0.3          # сегменты короче этого по истине не судим: делить на шум нельзя
SEGMIN = float(os.environ.get('KS_SEGMIN', -1))   # kf_seg_min_sec; <0 = дефолт оценщика
QUIET = os.environ.get('KS_QUIET') == '1'         # печатать только итог (свип по сроку)


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
            od.append((stamp(m), p.x, p.y, p.z) + euler(m.pose.pose.orientation))
        elif topic == IMU_TOPIC:
            m = deserialize_message(raw, Imu)
            w = m.angular_velocity
            imu.append((stamp(m), w.x, w.y, w.z) + euler(m.orientation))
        elif topic == '/mavros/global_position/rel_alt':
            m = deserialize_message(raw, Float64)
            alt.append((ts * 1e-9, m.data))
    frames.sort(key=lambda f: f[0])
    return frames, np.array(od), np.array(imu), np.array(alt)


def main():
    frames, od, imu, alt = read(BAG)
    print(f'бэг {BAG}: кадров {len(frames)}, одометрии {len(od)}, imu {len(imu)}')
    if not len(frames) or not len(od) or not len(imu):
        sys.exit('⚠️ нет кадров/одометрии/imu — переигрывать нечего')
    h = od[od[:, 3] > HOVER_Z]
    t0, t1 = (h[0, 0], h[-1, 0]) if len(h) > 20 else (od[0, 0], od[-1, 0])
    frames = [f for f in frames if t0 <= f[0] <= t1]
    # истинный продольный путь в СВЯЗАННОЙ системе (ось камеры), нарастающим итогом
    dx, dy = np.diff(h[:, 1]), np.diff(h[:, 2])
    ym = h[:-1, 6]
    fwd = np.concatenate([[0.0], np.cumsum(dx * np.cos(ym) + dy * np.sin(ym))])
    ft = h[:, 0]

    kw = {} if SEGMIN < 0 else {'kf_seg_min_sec': SEGMIN}
    est = FlowEstimator(CAM_W / 2.0, CAM_W / 2.0, CAM_W / 2.0, CAM_H / 2.0, FLOW_R,
                        rotflow_sign=1.0, pitch_smooth_n=9, roll_smooth_n=25,
                        yaw_smooth_n=5, **kw)
    segs = []
    prev_acc, prev_segs, seg_t0 = 0.0, 0, None
    for t, gray in frames:
        i = int(np.argmin(np.abs(imu[:, 0] - t)))
        a = float(np.interp(t, alt[:, 0], alt[:, 1])) if len(alt) else None
        out = est.process(gray, t, imu[i, 1:4], pitch=imu[i, 5], alt=a)
        if out is None:
            continue
        if seg_t0 is None:
            seg_t0 = t
        if est.kf_segs != prev_segs:                      # сегмент ЗАЧТЁН в накопитель
            banked = est.kf_acc - prev_acc
            d_true = float(np.interp(t, ft, fwd) - np.interp(seg_t0, ft, fwd))
            segs.append((seg_t0 - t0, t - seg_t0, d_true, banked))
            prev_acc, prev_segs, seg_t0 = est.kf_acc, est.kf_segs, t

    if not segs:
        sys.exit('сегментов не закрылось — судить нечего')
    print(f'\nзакрытых сегментов {len(segs)}, накопитель к концу {est.kf_acc:+.4f}, '
          f'истинный продольный путь {fwd[-1]:+.1f} м')
    print(f"\n{'t,с':>5s} | {'длит':>5s} | {'истина,м':>8s} | {'зачтено':>8s} | "
          f"{'log/м':>8s} | знак")
    ok_n = bad_n = 0
    ok_m = bad_m = 0.0
    for tt, dur, d_true, banked in segs:
        if abs(d_true) < MIN_M:
            mark, slope = 'мало хода', float('nan')
        else:
            slope = banked / d_true
            good = slope > 0
            mark = 'ВЕРНО' if good else '⚠️ ОБРАТНЫЙ'
            if good:
                ok_n += 1
                ok_m += abs(d_true)
            else:
                bad_n += 1
                bad_m += abs(d_true)
        if not QUIET:
            print(f'{tt:5.1f} | {dur:5.2f} | {d_true:+8.2f} | {banked:+8.4f} | '
                  f'{slope:+8.4f} | {mark}')
    tot = ok_n + bad_n
    if tot:
        print(f'\nсегментов с ВЕРНЫМ знаком {ok_n}/{tot} ({100 * ok_n / tot:.0f}%), '
              f'с ОБРАТНЫМ {bad_n}/{tot} ({100 * bad_n / tot:.0f}%)')
        print(f'пути под верным знаком {ok_m:.1f} м, под обратным {bad_m:.1f} м '
              f'({100 * bad_m / max(ok_m + bad_m, 1e-6):.0f}% хода ушло в накопитель наоборот)')


if __name__ == '__main__':
    main()
