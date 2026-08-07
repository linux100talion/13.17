#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ЗАПАЗДЫВАНИЕ ОРИЕНТАЦИИ в канале вида сверху — стенд A/B по ОДНИМ кадрам.

⚠️ СТЕНД ГОНЯЕТ БОЕВОЙ КОД, А НЕ СВОЮ КОПИЮ. Выпрямление и скорость считает настоящий
`FlowEstimator._ipm_update` из `control_pkg`, углы на кадр — настоящая `attitude_at` из
`control_pkg.infrastructure.ros_perception`. Стенд отвечает только за чтение бэга и
за арифметику сверки. Своя копия геометрии (как в `ipm_flow_test.py`) для A/B не годится:
она может «подтвердить» правку, которой в полёте нет.

ЧТО ЛЕЧИМ. `_ipm_rectify` строит полосу земли по текущим крену/тангажу, то есть вращение
оно снимает САМО. Течёт не вращение, а РАССИНХРОН: ATTITUDE идёт 12.5 Гц, камера 20-30 Гц,
а `RosPerception` до правки брал углы как «последнее пришедшее», без привязки к штампу
кадра. Внутри одного интервала ATTITUDE угол держится СТУПЕНЬКОЙ, истинный уходит со
скоростью ω — ошибка растёт линейно, полоса земли уезжает, и в скорость подмешивается
член, ПРОПОРЦИОНАЛЬНЫЙ УГЛОВОЙ СКОРОСТИ.

ЧИСЛО, КОТОРОЕ ЖДЁМ. Точка земли 4.5 м впереди при h=3 и наклоне 0.26 рад лежит на 163 px
ниже центра (fy≈480 на 960×540); ошибка крена δφ двигает её вбок на δφ·163 px, метр на
пиксель там 4.5/480 → 1.53 м на радиан. Регрессия лётного сигнала на истину (серия G3)
дала 0.85 / 1.04 / 1.32 / 1.01 на спокойных прогонах и 2.13 на дёрганом — сходится.

ЧТО СРАВНИВАЕТСЯ (три способа взять угол на кадр, кадры и код одни и те же):
    hold   — последнее сообщение с t ≤ штампа кадра. ЭТО ПРЕЖНЕЕ ПОВЕДЕНИЕ (extrap=False).
    extrap — hold + гироскоп·Δt (правка: `att_extrap` в config.py).
    near   — ближайшее по времени сообщение. НЕ реализуемо в полёте (заглядывает вперёд),
             нужно как ПОТОЛОК: показывает, сколько вообще можно выиграть синхронизацией.

МЕРКА — не «стало красивее», а РЕГРЕССИЯ измеренной скорости на истину:
    v_изм = a·v_ист + b·ωx + c·ωy + d
`a` — масштаб канала (цель 1.0), `b` — та самая утечка крена в М НА РАД/С (цель 0),
R² — сколько дисперсии объяснено. Правка засчитана, если |b| упал, а `a` не уехал.

⚠️ Нужен бэг С КАДРАМИ (`STRIP=0` в hover_series.sh).
⚠️ Высота — ИЗ ОДОМЕТРИИ, а не из баро: у `rel_alt` в бэге wall-штампы против sim-штампов
кадров, при RTF≈0.07 интерполяция упирается в край (правило 11 в tune.md).

Запуск (в контейнере nav — нужен cv2 и control_pkg):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /root/sim_ws/install/setup.bash;
    PYTHONPATH=/root/sim_ws/src/control:$PYTHONPATH \
    AE_BAG=/root/sim_ws/output/I1s1_bag python3 /lab/att_extrap_test.py'
⚠️ `PYTHONPATH` на ИСХОДНИКИ, а не на install: colcon КОПИРУЕТ ament_python-пакет при
сборке, и без этого стенд молча возьмёт версию кода на момент последнего colcon build —
то есть проверит не то, что лежит в дереве. Ровно та ошибка, ради которой стенд вообще
переписан на боевой код.
Env: AE_BAG, AE_MAXF, AE_EXTRAP_MAX (потолок дотяжки, 0.2).
"""
import math
import os
import sys

import numpy as np

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image, Imu

from control_pkg.infrastructure.ros_perception import attitude_at
from control_pkg.perception.flow_estimator import FlowEstimator

BAG = os.environ.get('AE_BAG', '/root/sim_ws/output/I1s1_bag')
MAXF = int(os.environ.get('AE_MAXF', 0))
EXTRAP_MAX = float(os.environ.get('AE_EXTRAP_MAX', 0.2))
HOVER_Z = 2.0
# ровно то, что кладёт в оценщик bootstrap_node (FLOW_R) и RosPerception (интринсики)
CAM_W, CAM_H = 960, 540
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]


def euler(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    frames, od, imu = [], [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/image_mono':
            m = deserialize_message(raw, Image)
            if m.encoding in ('mono8', '8UC1'):
                frames.append((stamp(m),
                               np.frombuffer(m.data,
                                             dtype=np.uint8).reshape(m.height, m.width).copy()))
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((stamp(m), p.x, p.y, p.z) + euler(m.pose.pose.orientation))
        elif topic == '/mavros/imu/data':
            # ровно тот источник, что читает нода: и углы, и ω из ОДНОГО сообщения
            m = deserialize_message(raw, Imu)
            w = m.angular_velocity
            imu.append((stamp(m),) + euler(m.orientation) + (w.x, w.y, w.z))
    frames.sort(key=lambda f: f[0])
    return frames, np.array(od), np.array(imu)


def att_for(imu, t, mode):
    """Углы (тангаж, крен) на момент кадра. `hold`/`extrap` — БОЕВАЯ `attitude_at`."""
    if mode == 'near':
        i = int(np.argmin(np.abs(imu[:, 0] - t)))
        return imu[i, 2], imu[i, 1]
    j = np.searchsorted(imu[:, 0], t, side='right') - 1
    if j < 0:
        return imu[0, 2], imu[0, 1]
    # буфер гироскопа ноды — тот же поток; отдаём хвост, как он лежит в `_gyro_buf`
    lo = max(0, j - 119)
    buf = [(row[0], row[4], row[5], row[6]) for row in imu[lo:j + 1]]
    return attitude_at(imu[j, 2], imu[j, 1], imu[j, 0], buf, t,
                       extrap=(mode == 'extrap'), extrap_max=EXTRAP_MAX)


def replay(frames, od, imu, mode):
    """Канал вида сверху БОЕВЫМ кодом. Возвращает (t, v_вперёд, v_вбок) по кадрам."""
    est = FlowEstimator(CAM_W / 2.0, CAM_W / 2.0, CAM_W / 2.0, CAM_H / 2.0, FLOW_R)
    ts, vf, vl = [], [], []
    for t, gray in frames:
        pitch, roll = att_for(imu, t, mode)
        h = float(np.interp(t, od[:, 0], od[:, 3]))
        est._ipm_update(gray, t, max(h, 0.5), pitch, roll)
        if est.ipm_ok:
            ts.append(t)
            vf.append(est.ipm_vfwd)
            vl.append(est.ipm_vlat)
    return np.array(ts), np.array(vf), np.array(vl)


def main():
    frames, od, imu = read(BAG)
    print(f'бэг {BAG}: кадров {len(frames)}, одометрии {len(od)}, imu {len(imu)}')
    if not len(frames) or not len(od) or not len(imu):
        sys.exit('⚠️ нет кадров/одометрии/imu — нужен бэг со STRIP=0')
    hov = od[od[:, 3] > HOVER_Z]
    if len(hov) < 20:
        sys.exit('⚠️ висения в бэге нет')
    t0, t1 = hov[0, 0], hov[-1, 0]
    frames = [f for f in frames if t0 <= f[0] <= t1]
    if MAXF:
        frames = frames[:MAXF]
    if len(frames) < 40:
        sys.exit(f'⚠️ кадров в висении мало ({len(frames)})')
    dtm = float(np.median(np.diff([f[0] for f in frames])))
    dta = float(np.median(np.diff(imu[:, 0])))
    print(f'висение {t1 - t0:.1f} с, кадров {len(frames)} ({1 / dtm:.1f} Гц), '
          f'ATTITUDE {1 / dta:.1f} Гц → ступенька до {dta * 1000:.0f} мс')

    # истина: скорость в СВЯЗАННОЙ системе
    dx, dy = np.diff(hov[:, 1]), np.diff(hov[:, 2])
    ym = hov[:-1, 6]
    fwd = np.concatenate([[0.0], np.cumsum(dx * np.cos(ym) + dy * np.sin(ym))])
    lat = np.concatenate([[0.0], np.cumsum(-dx * np.sin(ym) + dy * np.cos(ym))])
    vt_fwd_all = np.gradient(fwd, hov[:, 0])
    vt_lat_all = np.gradient(lat, hov[:, 0])

    print(f"\n{'способ':7s} | {'ось':10s} | {'масштаб a':>9s} | {'утечка b, м':>11s} | "
          f"{'c, м':>6s} | {'R²':>5s} | {'R² без ω':>8s} | {'СКО ош':>6s}")
    for mode in ('hold', 'extrap', 'near'):
        ts, vf, vl = replay(frames, od, imu, mode)
        if len(ts) < 20:
            print(f'{mode:7s} | измерений мало ({len(ts)})')
            continue
        wx = np.interp(ts, imu[:, 0], imu[:, 4])
        wy = np.interp(ts, imu[:, 0], imu[:, 5])
        for name, v, tr in (('продольная', vf, vt_fwd_all), ('боковая', vl, vt_lat_all)):
            vt = np.interp(ts, hov[:, 0], tr)
            ok = np.isfinite(v) & (np.abs(v) > 0)
            if ok.sum() < 20:
                continue
            A = np.column_stack([vt[ok], wx[ok], wy[ok], np.ones(int(ok.sum()))])
            c, *_ = np.linalg.lstsq(A, v[ok], rcond=None)
            r2 = 1 - (v[ok] - A @ c).var() / v[ok].var()
            A0 = np.column_stack([vt[ok], np.ones(int(ok.sum()))])
            c0, *_ = np.linalg.lstsq(A0, v[ok], rcond=None)
            r20 = 1 - (v[ok] - A0 @ c0).var() / v[ok].var()
            print(f'{mode:7s} | {name:10s} | {c[0]:+9.2f} | {c[1]:+11.2f} | '
                  f'{c[2]:+6.2f} | {r2:5.2f} | {r20:8.2f} | {np.std(v[ok] - vt[ok]):6.2f}')
    print('\nПравка засчитана, если |b| у extrap заметно меньше, чем у hold, а масштаб a '
          'остался около 1.0.\nnear — недостижимый потолок (заглядывает вперёд), мерка '
          'того, сколько вообще даёт синхронизация.')


if __name__ == '__main__':
    main()
