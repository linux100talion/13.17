#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kf_vel_check — ОТКУДА брать СКОРОСТЬ опоры: переигрывание бэга боевым FlowEstimator.

Задача. Канал ПОЛОЖЕНИЯ (kf_logs) после починки высоты стал точным: по J2 corr с
истинным удалением +0.86 при крутизне 1.50 %/м (физика ~1.35). А ПРОИЗВОДНАЯ этого
же канала, которой демпфирует DpPitchHold, коррелирует с истинной скоростью лишь на
+0.30, и её СКО 0.091 log/с = 8 м/с в пересчёте — при истинных 1-3 м/с. Шаг kf_logs
между кадрами имеет СКО 0.0116 при физически ожидаемом 0.0007.

Гипотеза, которую проверяем: скорость считается по СГЛАЖЕННОМУ (медиана 9 кадров)
отчёту, а медианный фильтр держит выход постоянным и потом перескакивает — то есть
превращает шум в СТУПЕНЬКИ, а производная от ступенек и есть тот мусор. Тогда МНК
надо строить по СЫРОМУ значению сегмента, а сглаживание оставить только положению.

Что делает скрипт. Гоняет тот же FlowEstimator (те же параметры, что в полёте) по
кадрам /image_color из бэга, подавая ему ω из IMU, тангаж из IMU и высоту из баро —
как в полёте. На каждом кадре пишет и СЫРОЕ значение сегмента, и сглаженный отчёт.
Затем считает скорость обоими способами при разных окнах и сравнивает с истиной
(продольная проекция одометрии). Печатает corr, крутизну и остаток.

Запуск (в контейнере nav):
  KV_BAG=/root/sim_ws/output/J2_althold_bag python3 /lab/kf_vel_check.py
Переменные: KV_BAG, KV_MAXF (ограничить кадры), KV_WINS ("0.5,1,2,3").
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

BAG = os.environ.get('KV_BAG', '/root/sim_ws/output/J2_althold_bag')
MAXF = int(os.environ.get('KV_MAXF', 0))
WINS = [float(w) for w in os.environ.get('KV_WINS', '0.5,1,1.5,2,3').split(',')]
# ПРОРЕЖИВАНИЕ: брать каждый N-й кадр. Нода в полёте успевала 19.2 Гц из 30 доступных
# (замер J2: /flow_dbg3 1151 против /image_color 1748) — восприятие CPU-связано, и
# треть кадров теряется. Прореживание воспроизводит это условие офлайн: если шум
# опоры вырастает до полётного, причина в темпе, а не в самом канале.
DECIM = int(os.environ.get('KV_DECIM', 1))
# ИСТОЧНИК ω и ТАНГАЖА. Нода в полёте берёт /gz_imu/data_flu — gz-IMU, пропущенный
# через low-pass 5 Гц (фильтр настроен под VINS: срезает лимит-цикл rate-loop, см.
# src/sim/imu_frd_to_flu.py). Оценщик вычитает вращательный поток по этой ω, и если
# фильтр её запаздывает/ослабляет, остаток вращения уезжает в оценку перемещения.
# Офлайн по умолчанию берётся MAVROS (EKF, без этого фильтра) — отсюда и подозрение,
# что полётный шум опоры втрое больше офлайнового при одном и том же коде.
# KV_IMU=gz — считать ровно тем, что видит нода; mavros — как раньше.
IMU_SRC = os.environ.get('KV_IMU', 'mavros')
IMU_TOPIC = '/gz_imu/data_flu' if IMU_SRC == 'gz' else '/mavros/imu/data'

# Интринсики и экстринсики — как у ноды (960×540, pinhole 90° hfov; см. ros_perception)
CAM_W, CAM_H = 960, 540
R_CAM_IMU = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)


def euler(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


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


def lsq_slope(ts, vs, win):
    """Наклон МНК в скользящем окне win секунд (как в оценщике). NaN, пока не набралось."""
    out = np.full(len(ts), np.nan)
    for i in range(len(ts)):
        lo = ts[i] - win
        j = np.searchsorted(ts, lo)
        if i - j + 1 < 4 or ts[i] - ts[j] < 0.5 * win:
            continue
        tc = ts[j:i + 1] - ts[j:i + 1].mean()
        vc = vs[j:i + 1] - vs[j:i + 1].mean()
        d = np.dot(tc, tc)
        if d > 0:
            out[i] = np.dot(tc, vc) / d
    return out


def main():
    frames, od, imu, alt = read(BAG)
    if DECIM > 1:
        frames = frames[::DECIM]
    if MAXF:
        frames = frames[:MAXF]
    print(f'бэг {BAG}: кадров {len(frames)}, одометрии {len(od)}, '
          f'imu {len(imu)} ({IMU_TOPIC}), баро {len(alt)}')
    if not len(imu):
        print(f'⚠️ в бэге нет {IMU_TOPIC} — нечем считать ω/тангаж'); return
    est = FlowEstimator(CAM_W / 2.0, CAM_W / 2.0, CAM_W / 2.0, CAM_H / 2.0, R_CAM_IMU,
                        rotflow_sign=1.0, pitch_smooth_n=9, roll_smooth_n=25, yaw_smooth_n=5)
    ts, raw, rep, acc = [], [], [], []
    for t, gray in frames:
        i = int(np.argmin(np.abs(imu[:, 0] - t)))
        omega = imu[i, 1:4]
        pitch = imu[i, 5]
        a = float(np.interp(t, alt[:, 0], alt[:, 1])) if len(alt) else None
        out = est.process(gray, t, omega, pitch=pitch, alt=a)
        if out is None:
            continue
        ts.append(t)
        rep.append(out['kf_logs'])                       # сглаженный отчёт (как в полёте)
        # сырое значение сегмента + накопитель = НЕсглаженное положение
        raw.append((est._kf_logs_prev or 0.0) + est.kf_acc)
        acc.append(est.kf_acc)
    ts = np.array(ts); raw = np.array(raw); rep = np.array(rep)
    # истина: продольная проекция в курсе на момент начала окна
    yaw0 = float(np.interp(ts[0], od[:, 0], np.unwrap(od[:, 6])))
    vx = np.gradient(od[:, 1], od[:, 0]); vy = np.gradient(od[:, 2], od[:, 0])
    vtrue = np.interp(ts, od[:, 0], vx * math.cos(yaw0) + vy * math.sin(yaw0))
    x0 = np.interp(ts[0], od[:, 0], od[:, 1]); y0 = np.interp(ts[0], od[:, 0], od[:, 2])
    fwd = ((np.interp(ts, od[:, 0], od[:, 1]) - x0) * math.cos(yaw0)
           + (np.interp(ts, od[:, 0], od[:, 2]) - y0) * math.sin(yaw0))

    # ОКНО ОЦЕНКИ. По умолчанию — весь прогон, но тогда цифры НЕ сравнимы с полётными:
    # в выборку попадают земля, набор и посадка, где борт почти неподвижен, и шум на
    # кадр выходит вдвое-втрое меньше висенного. KV_HOVER=1 — оставить только плато
    # высоты (то же окно, по которому считаются полётные замеры).
    if os.environ.get('KV_HOVER') == '1':
        zi = np.interp(ts, od[:, 0], od[:, 3])
        plateau = zi > 0.9 * np.percentile(zi, 90)
        if plateau.sum() > 50:
            i0 = int(np.argmax(plateau))
            sel = (ts >= ts[i0]) & (ts <= ts[i0] + 40.0)
            ts, raw, rep = ts[sel], raw[sel], rep[sel]
            vtrue, fwd = vtrue[sel], fwd[sel]
            print(f'\nокно оценки: ВИСЕНИЕ, {sel.sum()} кадров, высота '
                  f'{zi[sel].min():.1f}..{zi[sel].max():.1f} м')
    print(f'\nПОЛОЖЕНИЕ (переигранное): corr сглаж {np.corrcoef(rep, fwd)[0, 1]:+.2f}, '
          f'сырое {np.corrcoef(raw, fwd)[0, 1]:+.2f}')
    print(f'шаг между кадрами: сглаж СКО {np.std(np.diff(rep)):.4f}, '
          f'сырое СКО {np.std(np.diff(raw)):.4f}')
    print(f'\nСКОРОСТЬ: corr с истиной / крутизна m-log/(м/с) / СКО остатка в м/с')
    print(f'{"окно":>6} | {"по СГЛАЖЕННОМУ":>28} | {"по СЫРОМУ":>28}')
    for w in WINS:
        row = [f'{w:5.2f}c |']
        for src in (rep, raw):
            v = lsq_slope(ts, src, w)
            g = ~np.isnan(v)
            if g.sum() < 20:
                row.append(f'{"мало данных":>28} |')
                continue
            a1 = np.polyfit(vtrue[g], v[g], 1)[0]
            cc = np.corrcoef(vtrue[g], v[g])[0, 1]
            res = np.std(v[g] - np.polyval(np.polyfit(vtrue[g], v[g], 1), vtrue[g])) / abs(a1)
            row.append(f'  corr{cc:+.2f} крут{a1 * 1000:+6.1f} ост{res:5.2f} |')
        print(' '.join(row))


if __name__ == '__main__':
    main()
