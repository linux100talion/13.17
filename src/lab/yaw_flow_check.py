#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yaw_flow_check — ДИАГНОСТИКА bias визуального yaw_flow (источник дрейфа YAW-hold).

Прогоняет записанные кадры /image_mono + гироскоп /gz_imu/data_flu через ТОТ ЖЕ
FlowEstimator, что и боевая нода (alt_hold_bootstrap.py), получая yaw_flow(t), и
сравнивает с ИСТИНОЙ Gazebo (/model/iris_cam/odometry): истинной yaw-скоростью и
боковой/продольной скоростью. Цель — понять, ОТ ЧЕГО постоянное смещение yaw_flow:

  yf ≈ a*yaw_rate + b*lateral_vel + d*forward_vel + c   (МНК в окне висения)

Гипотеза (docker/sim/yaw_pid.md): bias — УТЕЧКА ТРАНСЛЯЦИИ (b≠0) + остаточный DC (c).
Если так → фикс washout/depth-aware. Контр-гипотезы, которые тест отсеет: gyro-bias
по z (c без корреляции с движением), ошибка масштаба.

Дрейф-замыкание: демпфер (ki=0) гонит yf→0, поэтому постоянный bias yf соответствует
ненулевой ИСТИННОЙ yaw-скорости → линейный увод. Скрипт переводит bias yf в
предсказанный дрейф (°) через масштаб a и сравнивает с фактическим дрейфом истины.

Запуск ВНУТРИ nav (нужен cv2 из overlay для LK):
  docker exec -i p1317_nav bash -lc \
    'source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash; \
     source /root/sim_ws/install/setup.bash; python3 /lab/yaw_flow_check.py'

Env: SCENE_BAG(/root/sim_ws/output/scene_bag), IMG_TOPIC(/image_mono),
IMU_TOPIC(/gz_imu/data_flu), ODOM_TOPIC(/model/iris_cam/odometry),
SAFE_SEC(30), Z_TO(2.0). Опц. WIN_T0/WIN_T1 — абсолютное окно (sim-сек от старта bag).
"""

import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flow_estimator import FlowEstimator

# Константы FlowEstimator — те же, что в alt_hold_bootstrap.py (:60-62).
FLOW_FX = FLOW_FY = 640.0
FLOW_CX, FLOW_CY = 640.0, 360.0
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
FLOW_RSIGN = 1.0

BAG = os.environ.get('SCENE_BAG', '/root/sim_ws/output/scene_bag')
IMG_TOPIC = os.environ.get('IMG_TOPIC', '/image_mono')
IMU_TOPIC = os.environ.get('IMU_TOPIC', '/gz_imu/data_flu')
ODOM = os.environ.get('ODOM_TOPIC', '/model/iris_cam/odometry')
SAFE_SEC = float(os.environ.get('SAFE_SEC', '30'))
Z_TO = float(os.environ.get('Z_TO', '2.0'))


def _stamp(hdr):
    return hdr.stamp.sec + hdr.stamp.nanosec * 1e-9


def read_bag():
    """Один проход: собираем imu, odom (массивы) и кадры (stamp+сырые байты)."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    imu_t, imu_w = [], []
    od_t, od_z, od_yaw, od_wz, od_vx, od_vy = [], [], [], [], [], []
    imgs = []   # (stamp, h, w, bytes)
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == IMU_TOPIC:
            m = deserialize_message(data, Imu)
            imu_t.append(_stamp(m.header))
            imu_w.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        elif topic == ODOM:
            m = deserialize_message(data, Odometry)
            q = m.pose.pose.orientation
            od_t.append(_stamp(m.header))
            od_z.append(m.pose.pose.position.z)
            od_yaw.append(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z)))
            od_wz.append(m.twist.twist.angular.z)      # истинная yaw-скорость (body), rad/s
            od_vx.append(m.twist.twist.linear.x)       # продольная скорость (body), м/с
            od_vy.append(m.twist.twist.linear.y)       # боковая скорость (body), м/с
        elif topic == IMG_TOPIC:
            m = deserialize_message(data, Image)
            if m.encoding in ('mono8', '8UC1'):
                imgs.append((_stamp(m.header), m.height, m.width, bytes(m.data)))
    return (dict(t=np.array(imu_t), w=np.array(imu_w).reshape(-1, 3)),
            dict(t=np.array(od_t), z=np.array(od_z), yaw=np.array(od_yaw),
                 wz=np.array(od_wz), vx=np.array(od_vx), vy=np.array(od_vy)),
            imgs)


def main():
    imu, od, imgs = read_bag()
    if len(imgs) < 3:
        raise RuntimeError(f'мало кадров {IMG_TOPIC} в {BAG}: {len(imgs)}')
    if len(imu['t']) < 3:
        raise RuntimeError(f'нет гиро {IMU_TOPIC} в {BAG}')
    if len(od['t']) < 3:
        raise RuntimeError(f'нет одометрии {ODOM} в {BAG}')

    # yaw_flow «как в ноде»: для каждого кадра — последняя ω с stamp ≤ stamp кадра.
    est = FlowEstimator(FLOW_FX, FLOW_FY, FLOW_CX, FLOW_CY, FLOW_R, FLOW_RSIGN,
                        smooth_n=1, yaw_smooth_n=1)   # RAW (без сглаживания) — видим сам bias
    iw_t = imu['t']
    ts, yf, dts, ns = [], [], [], []
    for (st, h, w, buf) in imgs:
        gray = np.frombuffer(buf, dtype=np.uint8).reshape(h, w)
        k = int(np.searchsorted(iw_t, st, side='right')) - 1
        omega = imu['w'][k] if k >= 0 else np.zeros(3)
        res = est.process(gray, st, omega)
        if res is not None:
            ts.append(st); yf.append(res['yaw_flow']); dts.append(res['dt']); ns.append(res['n'])
    ts = np.array(ts); yf = np.array(yf); dts = np.array(dts); ns = np.array(ns)

    # окно висения: [взлёт Z>Z_TO, +SAFE_SEC] (как yaw_check), либо WIN_T0/T1.
    t0bag = min(ts[0], od['t'][0], imu['t'][0])
    to_idx = np.where(od['z'] > Z_TO)[0]
    if len(to_idx) == 0:
        print(f'дрон не взлетел (Z<{Z_TO})'); return
    t_to = od['t'][to_idx[0]]
    t_end = t_to + SAFE_SEC
    cut = f'взлёт+{SAFE_SEC:.0f}с'
    w0 = os.environ.get('WIN_T0'); w1 = os.environ.get('WIN_T1')
    if w0 and w1:
        t_to, t_end = t0bag + float(w0), t0bag + float(w1); cut = 'WIN_T0/T1'

    win = (ts >= t_to) & (ts <= t_end)
    if win.sum() < 10:
        print(f'мало кадров в окне ({win.sum()})'); return
    tw, yw, dtw = ts[win], yf[win], dts[win]

    # истину интерполируем на моменты кадров
    yaw_rate = np.interp(tw, od['t'], od['wz'])          # rad/s
    lat_vel = np.interp(tw, od['t'], od['vy'])           # м/с (боковая)
    fwd_vel = np.interp(tw, od['t'], od['vx'])           # м/с (продольная)
    yaw_true = np.interp(tw, od['t'], np.unwrap(od['yaw']))  # rad (для дрейфа)

    def corr(a, b):
        if a.std() < 1e-9 or b.std() < 1e-9:
            return float('nan')
        return float(np.corrcoef(a, b)[0, 1])

    # МНК: yf ~ a*yaw_rate + b*lat_vel + d*fwd_vel + c
    A = np.column_stack([yaw_rate, lat_vel, fwd_vel, np.ones_like(tw)])
    coef, *_ = np.linalg.lstsq(A, yw, rcond=None)
    a, b, d, c = coef
    resid = yw - A @ coef
    ss_tot = np.sum((yw - yw.mean()) ** 2)
    r2 = 1.0 - np.sum(resid ** 2) / ss_tot if ss_tot > 1e-12 else float('nan')

    # дрейф-замыкание: демпфер гонит yf→0. Постоянный bias yf (=intercept c при
    # нулевых скоростях) через масштаб a (px/кадр на rad/s) → остаточная истинная
    # yaw-скорость ≈ −c/a, за окно → предсказанный дрейф. Сравниваем с фактическим.
    dur = tw[-1] - tw[0]
    fps = 1.0 / np.median(dtw)
    drift_true_deg = math.degrees(yaw_true[-1] - yaw_true[0])
    pred_rate = (-c / a) if abs(a) > 1e-9 else float('nan')   # rad/s
    pred_drift_deg = math.degrees(pred_rate * dur) if math.isfinite(pred_rate) else float('nan')

    print(f'bag {BAG}')
    print(f'окно [{t_to - t0bag:.1f},{t_end - t0bag:.1f}]s ({dur:.1f}s, {cut}) | '
          f'кадров {len(tw)} | ~{fps:.1f} fps | треков медиана {int(np.median(ns))}')
    print()
    print(f'{"величина":<40}{"среднее":>12}{"СКО":>12}')
    print('-' * 64)
    print(f'{"yaw_flow (px/кадр) — что видит регулятор":<40}{yw.mean():>12.3f}{yw.std():>12.3f}')
    print(f'{"истин. yaw-скорость (°/с)":<40}{math.degrees(yaw_rate.mean()):>12.3f}{math.degrees(yaw_rate.std()):>12.3f}')
    print(f'{"истин. боковая скорость vy (м/с)":<40}{lat_vel.mean():>12.3f}{lat_vel.std():>12.3f}')
    print(f'{"истин. продольная скорость vx (м/с)":<40}{fwd_vel.mean():>12.3f}{fwd_vel.std():>12.3f}')
    print()
    print('КОРРЕЛЯЦИИ yaw_flow с истиной:')
    print(f'  corr(yf, yaw_rate)  = {corr(yw, yaw_rate):+.3f}   (ожидаем сильную +: yf измеряет yaw)')
    print(f'  corr(yf, lat_vel)   = {corr(yw, lat_vel):+.3f}   (≠0 → УТЕЧКА ТРАНСЛЯЦИИ)')
    print(f'  corr(yf, fwd_vel)   = {corr(yw, fwd_vel):+.3f}')
    print()
    print('МНК  yf = a·yaw_rate + b·lat_vel + d·fwd_vel + c :')
    print(f'  a (px/кадр на rad/s) = {a:+.3f}   масштаб yaw')
    print(f'  b (px/кадр на м/с)   = {b:+.3f}   вклад боковой трансляции')
    print(f'  d (px/кадр на м/с)   = {d:+.3f}   вклад продольной')
    print(f'  c (px/кадр)          = {c:+.3f}   ОСТАТОЧНЫЙ DC-bias (не от движения)')
    print(f'  R²                   = {r2:.3f}')
    print()
    print('ДРЕЙФ-ЗАМЫКАНИЕ (демпфер гонит yf→0):')
    print(f'  предсказанная остаточная yaw-скорость −c/a = {math.degrees(pred_rate) if math.isfinite(pred_rate) else float("nan"):+.3f} °/с')
    print(f'  → предсказанный дрейф за окно            = {pred_drift_deg:+.2f}°')
    print(f'  ФАКТИЧЕСКИЙ дрейф истины за окно         = {drift_true_deg:+.2f}°')
    print()
    print('Трактовка: |b| велик и corr(yf,lat_vel)≠0 → утечка трансляции (фикс B/C).')
    print('           |c| велик без корреляций → DC-bias (gyro-z/масштаб) → washout (A).')
    print('           предсказанный≈фактический дрейф → bias yf количественно объясняет увод.')


if __name__ == '__main__':
    main()
