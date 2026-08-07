#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""СТЕНД ФИЛЬТРОВ сегмента: какой признак отличает верный сегмент от инвертированного.

Задача. `kf_sign_check` показал: 31% закрытых сегментов уходят в накопитель с ОБРАТНЫМ
знаком и уносят 33% пройденного пути. Накопитель при этом складывает всегда ±0.03 (порог
`kf_seg_max`), то есть величина хода в нём не кодируется — только частота закрытий.
Нужен ФИЛЬТР: не банковать сегмент, которому нельзя верить.

Как устроен стенд. ОДИН проход по кадрам (переигрывание дорогое), на каждом закрытии
сегмента пишутся признаки + истина. Дальше кандидаты-фильтры сравниваются на этой
таблице БЕЗ повторного переигрывания.

Признаки сегмента:
  dur     — длительность, с (быстрые закрытия подозрительны: порог набран шумом);
  n_pts   — сколько точек осталось в опоре к закрытию;
  val     — засчитанное значение (обычно ±kf_seg_max);
  dv_imu  — изменение продольной скорости за сегмент по IMU, м/с.

Про IMU. Он НЕ знает скорость (замер: без известной начальной скорости знак смещения
угадывается в 40-45% случаев — хуже монетки), но отлично знает ЕЁ ИЗМЕНЕНИЕ (с известной
начальной — corr +0.96…+0.99, знак 100% на окнах до 1 с). Поэтому признак берётся
масштабонезависимый: знак изменения средней скорости между соседними сегментами у зрения
должен совпадать со знаком `dv_imu`. Цена метра при этом может быть какой угодно.

Метрика фильтра: доля пути, ушедшего в накопитель с ОБРАТНЫМ знаком (было 33%), и сколько
верного пути фильтр при этом выбросил.

Запуск (в контейнере nav):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash;
    KG_BAG=/root/sim_ws/output/E3f1_bag python3 /lab/kf_gate_lab.py'
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

BAG = os.environ.get('KG_BAG', '/root/sim_ws/output/E3f1_bag')
IMU_TOPIC = os.environ.get('KG_IMU', '/mavros/imu/data')
CAM_W, CAM_H = 960, 540
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
HOVER_Z, MIN_M, G = 2.0, 0.3, 9.80665


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
            frames.append((stamp(m), cv2.cvtColor(br.imgmsg_to_cv2(m, 'bgr8'),
                                                 cv2.COLOR_BGR2GRAY)))
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((stamp(m), p.x, p.y, p.z) + euler(m.pose.pose.orientation))
        elif topic == IMU_TOPIC:
            m = deserialize_message(raw, Imu)
            w, a = m.angular_velocity, m.linear_acceleration
            od_e = euler(m.orientation)
            imu.append((stamp(m), w.x, w.y, w.z) + od_e + (a.x,))
        elif topic == '/mavros/global_position/rel_alt':
            m = deserialize_message(raw, Float64)
            alt.append((ts * 1e-9, m.data))
    frames.sort(key=lambda f: f[0])
    return frames, np.array(od), np.array(imu), np.array(alt)


def collect(frames, od, imu, alt):
    """Один проход: таблица закрытых сегментов с признаками и истиной."""
    h = od[od[:, 3] > HOVER_Z]
    t0, t1 = (h[0, 0], h[-1, 0]) if len(h) > 20 else (od[0, 0], od[-1, 0])
    frames = [f for f in frames if t0 <= f[0] <= t1]
    dx, dy = np.diff(h[:, 1]), np.diff(h[:, 2])
    fwd = np.concatenate([[0.0], np.cumsum(dx * np.cos(h[:-1, 6]) + dy * np.sin(h[:-1, 6]))])
    ft = h[:, 0]
    # продольное ускорение борта: удельная сила по X минус проекция g по углам того же IMU
    a_fwd = imu[:, 7] + G * np.sin(imu[:, 5])
    cum_dv = np.concatenate([[0.0], np.cumsum(np.diff(imu[:, 0]) *
                                              (a_fwd[:-1] + a_fwd[1:]) / 2)])

    est = FlowEstimator(CAM_W / 2.0, CAM_W / 2.0, CAM_W / 2.0, CAM_H / 2.0, FLOW_R,
                        rotflow_sign=1.0, pitch_smooth_n=9, roll_smooth_n=25, yaw_smooth_n=5)
    rows, prev_acc, prev_segs, seg_t0 = [], 0.0, 0, None
    for t, gray in frames:
        i = int(np.argmin(np.abs(imu[:, 0] - t)))
        a = float(np.interp(t, alt[:, 0], alt[:, 1])) if len(alt) else None
        out = est.process(gray, t, imu[i, 1:4], pitch=imu[i, 5], alt=a)
        if out is None:
            continue
        if seg_t0 is None:
            seg_t0 = t
        if est.kf_segs != prev_segs:
            rows.append(dict(
                t=seg_t0 - t0, dur=t - seg_t0, val=est.kf_acc - prev_acc,
                n_pts=out['kf_n'],
                dv_imu=float(np.interp(t, imu[:, 0], cum_dv) -
                             np.interp(seg_t0, imu[:, 0], cum_dv)),
                d_true=float(np.interp(t, ft, fwd) - np.interp(seg_t0, ft, fwd))))
            prev_acc, prev_segs, seg_t0 = est.kf_acc, est.kf_segs, t
    return rows


def score(rows, keep, name):
    """Доля пути, ушедшего в накопитель наоборот, и сколько верного пути выброшено."""
    bad = sum(r['d_true'] for r in rows if keep(r) and r['val'] * r['d_true'] < 0
              and abs(r['d_true']) >= MIN_M for _ in [0])
    bad = sum(abs(r['d_true']) for r in rows
              if keep(r) and r['val'] * r['d_true'] < 0 and abs(r['d_true']) >= MIN_M)
    good = sum(abs(r['d_true']) for r in rows
               if keep(r) and r['val'] * r['d_true'] > 0 and abs(r['d_true']) >= MIN_M)
    lost = sum(abs(r['d_true']) for r in rows
               if not keep(r) and r['val'] * r['d_true'] > 0 and abs(r['d_true']) >= MIN_M)
    n = sum(1 for r in rows if keep(r) and abs(r['d_true']) >= MIN_M)
    tot = good + bad
    print(f"{name:34s} | {n:4d} | {100 * bad / max(tot, 1e-9):7.0f}% | "
          f"{good:8.1f} | {bad:6.1f} | {lost:8.1f}")


def main():
    frames, od, imu, alt = read(BAG)
    print(f'бэг {BAG}: кадров {len(frames)}, imu {len(imu)} ({IMU_TOPIC})')
    if not len(frames) or not len(imu):
        sys.exit('⚠️ нет кадров или IMU')
    rows = collect(frames, od, imu, alt)
    print(f'закрытых сегментов: {len(rows)}\n')
    # признак «зрение и IMU согласны по ИЗМЕНЕНИЮ скорости» — масштабонезависимый
    for i, r in enumerate(rows):
        v = r['val'] / max(r['dur'], 1e-3)
        r['v_vis'] = v
        r['dv_vis'] = v - (rows[i - 1]['v_vis'] if i else 0.0)
    print(f"{'фильтр':34s} | {'сегм':>4s} | {'наоборот':>8s} | "
          f"{'верно,м':>8s} | {'мимо,м':>6s} | {'потеряно,м':>10s}")
    score(rows, lambda r: True, 'без фильтра (как сейчас)')
    for d in (0.3, 0.5, 0.7, 1.0):
        score(rows, lambda r, d=d: r['dur'] >= d, f'длительность >= {d} с')
    for n in (60, 80, 100):
        score(rows, lambda r, n=n: r['n_pts'] >= n, f'точек в опоре >= {n}')
    for th in (0.05, 0.1, 0.2):
        score(rows, lambda r, th=th: abs(r['dv_imu']) < th or
              r['dv_vis'] * r['dv_imu'] > 0, f'IMU согласен по Δv (порог {th})')
    score(rows, lambda r: r['dur'] >= 0.5 and (abs(r['dv_imu']) < 0.1 or
                                               r['dv_vis'] * r['dv_imu'] > 0),
          'длит>=0.5 И IMU согласен')


if __name__ == '__main__':
    main()
