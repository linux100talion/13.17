#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pitch_flow_check — ID продольного зрительного сигнала (looming) для PITCH-демпфера.

Прогоняет записанные /image_mono + /gz_imu/data_flu через FlowEstimator и сравнивает
ДВА кандидата продольного сигнала с ИСТИННОЙ продольной скоростью vx (из
/model/iris_cam/odometry):
  - longitudinal = median(tr[:,1])  — вертикальный трансл. поток (аналог lateral по Y);
  - divergence   = ∂u/∂xn+∂v/∂yn    — расширение поля (looming, ∝ Tz/Z).

Нужен bag с РЕАЛЬНЫМ продольным движением — снимать под --pitch-excite (открытый
контур: pitch = заданный профиль, ROLL держит gz). Прошлый gz-hold bag не годится
(дрон стоит). Для каждого сигнала: corr с vx, МНК S ~ a·vx + b·vy + c, R², масштаб.
Смотрим, у кого годный SNR/обусловленность — по нему строить PITCH-демпфер (или нет).

Запуск ВНУТРИ nav (cv2 из overlay):
  docker exec -i -e SAFE_SEC=28 p1317_nav bash -lc \
    'source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash; \
     source /root/sim_ws/install/setup.bash; python3 /lab/pitch_flow_check.py'

Env: SCENE_BAG, IMG_TOPIC(/image_mono), IMU_TOPIC(/gz_imu/data_flu),
ODOM_TOPIC(/model/iris_cam/odometry), SAFE_SEC(28), Z_TO(2.0), WIN_T0/WIN_T1.
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

FLOW_FX = FLOW_FY = 640.0
FLOW_CX, FLOW_CY = 640.0, 360.0
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
FLOW_RSIGN = 1.0

BAG = os.environ.get('SCENE_BAG', '/root/sim_ws/output/scene_bag')
IMG_TOPIC = os.environ.get('IMG_TOPIC', '/image_mono')
IMU_TOPIC = os.environ.get('IMU_TOPIC', '/gz_imu/data_flu')
ODOM = os.environ.get('ODOM_TOPIC', '/model/iris_cam/odometry')
SAFE_SEC = float(os.environ.get('SAFE_SEC', '28'))
Z_TO = float(os.environ.get('Z_TO', '2.0'))
SMOOTH_N = int(os.environ.get('SMOOTH_N', '1'))   # причинная медиана по N кадрам (1=выкл)


def causal_median(x, n):
    """Медиана по последним n сэмплам (как буфер в ноде): x[i]=median(x[i-n+1..i])."""
    if n <= 1:
        return x
    out = np.empty_like(x)
    for i in range(len(x)):
        out[i] = np.median(x[max(0, i - n + 1):i + 1])
    return out


def _stamp(hdr):
    return hdr.stamp.sec + hdr.stamp.nanosec * 1e-9


def read_bag():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    imu_t, imu_w = [], []
    od_t, od_z, od_vx, od_vy, od_wz = [], [], [], [], []
    imgs = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == IMU_TOPIC:
            m = deserialize_message(data, Imu)
            imu_t.append(_stamp(m.header))
            imu_w.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        elif topic == ODOM:
            m = deserialize_message(data, Odometry)
            od_t.append(_stamp(m.header))
            od_z.append(m.pose.pose.position.z)
            od_vx.append(m.twist.twist.linear.x)   # продольная скорость (body), м/с
            od_vy.append(m.twist.twist.linear.y)   # боковая (должна ~0: gz держит roll)
            od_wz.append(m.twist.twist.angular.z)
        elif topic == IMG_TOPIC:
            m = deserialize_message(data, Image)
            if m.encoding in ('mono8', '8UC1'):
                imgs.append((_stamp(m.header), m.height, m.width, bytes(m.data)))
    return (dict(t=np.array(imu_t), w=np.array(imu_w).reshape(-1, 3)),
            dict(t=np.array(od_t), z=np.array(od_z), vx=np.array(od_vx),
                 vy=np.array(od_vy), wz=np.array(od_wz)),
            imgs)


def corr(a, b):
    if a.std() < 1e-9 or b.std() < 1e-9:
        return float('nan')
    return float(np.corrcoef(a, b)[0, 1])


def report_signal(name, S, vx, vy):
    """corr + МНК S ~ a·vx + b·vy + c."""
    A = np.column_stack([vx, vy, np.ones_like(vx)])
    coef, *_ = np.linalg.lstsq(A, S, rcond=None)
    a, b, c = coef
    resid = S - A @ coef
    ss = np.sum((S - S.mean()) ** 2)
    r2 = 1.0 - np.sum(resid ** 2) / ss if ss > 1e-12 else float('nan')
    print(f'\n[{name}]  среднее={S.mean():+.4f}  СКО={S.std():.4f}')
    print(f'  corr(S, vx) = {corr(S, vx):+.3f}   (ждём сильную: сигнал измеряет продольную)')
    print(f'  corr(S, vy) = {corr(S, vy):+.3f}   (утечка боковой)')
    print(f'  МНК S = a·vx + b·vy + c :  a={a:+.4f}  b={b:+.4f}  c={c:+.4f}  R²={r2:.3f}')
    return corr(S, vx), r2


def main():
    imu, od, imgs = read_bag()
    if len(imgs) < 3 or len(imu['t']) < 3 or len(od['t']) < 3:
        raise RuntimeError(f'мало данных: img={len(imgs)} imu={len(imu["t"])} odom={len(od["t"])}')

    est = FlowEstimator(FLOW_FX, FLOW_FY, FLOW_CX, FLOW_CY, FLOW_R, FLOW_RSIGN)
    iw_t = imu['t']
    ts, lon, div, ns = [], [], [], []
    for (st, h, w, buf) in imgs:
        gray = np.frombuffer(buf, dtype=np.uint8).reshape(h, w)
        k = int(np.searchsorted(iw_t, st, side='right')) - 1
        omega = imu['w'][k] if k >= 0 else np.zeros(3)
        res = est.process(gray, st, omega)
        if res is not None:
            ts.append(st); lon.append(res['longitudinal']); div.append(res['divergence']); ns.append(res['n'])
    ts = np.array(ts); lon = np.array(lon); div = np.array(div); ns = np.array(ns)
    if SMOOTH_N > 1:                       # причинная медиана (как в ноде)
        lon = causal_median(lon, SMOOTH_N); div = causal_median(div, SMOOTH_N)

    t0bag = min(ts[0], od['t'][0], imu['t'][0])
    to_idx = np.where(od['z'] > Z_TO)[0]
    if len(to_idx) == 0:
        print(f'дрон не взлетел (Z<{Z_TO})'); return
    t_to = od['t'][to_idx[0]]; t_end = t_to + SAFE_SEC; cut = f'взлёт+{SAFE_SEC:.0f}с'
    w0 = os.environ.get('WIN_T0'); w1 = os.environ.get('WIN_T1')
    if w0 and w1:
        t_to, t_end = t0bag + float(w0), t0bag + float(w1); cut = 'WIN_T0/T1'

    win = (ts >= t_to) & (ts <= t_end)
    if win.sum() < 10:
        print(f'мало кадров в окне ({win.sum()})'); return
    tw = ts[win]
    vx = np.interp(tw, od['t'], od['vx'])
    vy = np.interp(tw, od['t'], od['vy'])

    print(f'bag {BAG}')
    print(f'окно [{t_to - t0bag:.1f},{t_end - t0bag:.1f}]s ({tw[-1]-tw[0]:.1f}s, {cut}) | '
          f'кадров {len(tw)} | треков медиана {int(np.median(ns))}')
    print(f'истинная продольная vx: среднее={vx.mean():+.3f} СКО={vx.std():.3f} м/с '
          f'(размах {vx.min():+.2f}..{vx.max():+.2f}) | боковая vy СКО={vy.std():.3f}')
    if vx.std() < 0.05:
        print('⚠️ vx почти не гуляет (<0.05 м/с) — возбуждения мало, ID недостоверен '
              '(pitch-excite не отработал? поднять BS_RE_AMP)')

    c_lon, r2_lon = report_signal('longitudinal (вертикальный поток)', lon[win], vx, vy)
    c_div, r2_div = report_signal('divergence (looming)', div[win], vx, vy)

    print('\nВЕРДИКТ:')
    best = 'longitudinal' if abs(c_lon) >= abs(c_div) else 'divergence'
    print(f'  Лучше коррелирует с продольной скоростью: {best} '
          f'(|corr| {max(abs(c_lon), abs(c_div)):.3f})')
    print('  |corr|>~0.5 и R²>~0.3 → сигнал годен для PITCH-демпфера (строить контур).')
    print('  Оба слабые → продольная ось на монокуляре без depth не тянет '
          '(как и предупреждала спека) → пилот/иной сенсор.')


if __name__ == '__main__':
    main()
