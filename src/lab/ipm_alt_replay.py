#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ВЫСОТА ПЕРЦЕПЦИИ В КАНАЛЕ ВИДА СВЕРХУ — A/B по ОДНИМ кадрам из бэга.

Вопрос, ради которого стенд написан (прогон lv2_joy_20260826_183305, fine-мир):
демпфер у земли выдал РОВНО 0 PWM, потому что `/flow_dbg8.z` = код 1 (гейт
высоты) во ВСЕХ кадрах воздуха. Гейт судит по ВЫСОТЕ ПЕРЦЕПЦИИ (`perc_alt_src`),
а она в GPS-denied профиле = EKF local z, смещённый вниз на ~0.27 м — больше,
чем вся высота полёта (истинные 0.27-0.37 м). Стенд отвечает на вопрос
«а если бы высота была ПРАВИЛЬНОЙ — канал бы ожил и мерил бы верно?» ДО того,
как тратить на проверку лётный прогон.

⚠️ СТЕНД ГОНЯЕТ БОЕВОЙ КОД: выпрямление, гейты и скорость считает настоящий
`FlowEstimator._ipm_update` из `control_pkg` (как `att_extrap_test.py`). Стенд
отвечает только за чтение бэга и арифметику сверки.

A/B/C — ТРИ ОЦЕНЩИКА НА ОДНОМ ПОТОКЕ КАДРОВ, отличие ровно одно — высота:
    A (`ekf`)  — alt = max(0, z `/mavros/local_position/pose`) — КАК ЛЕТАЛИ
                 (`perc_alt_src=local` + кламп в `ros_perception._on_lpos_alt`);
    B (`true`) — alt = истинная AGL (`/model/iris_cam/odometry` z − база земли);
    C (`latch`)— alt = max(0, z − z₀), где z₀ = медиана EKF z за 2 с ДО арма
                 (`/mavros/state`): КАНДИДАТ-ФИКС «нулевать высоту перцепции по
                 арму». Смещение EKF z постоянное, а ДЕЛЬТА точна (замер 183305:
                 земля −0.29 → воздух −0.00 при истинных +0.27) — значит латч
                 нуля обязан дать почти B, и это проверяется ЗДЕСЬ, до полёта.
A обязан воспроизвести запись (те же коды брака в `/flow_dbg8.z`) — это проверка
фиделити стенда; B даёт потолок; C — то, что реально можно выкатить.

⚠️ УГЛЫ И ω — ИСТИНА GAZEBO, а не MAVROS: в freefly-бэгах `/mavros/imu/data` не
пишется (TOPICS_EXTRA), а `/model/iris_cam/odometry` есть всегда. Значит стенд
чуть ОПТИМИСТИЧНЕЕ полёта по ориентации (нет ступеньки ATTITUDE 12.5 Гц и лага);
для вопроса «жив ли канал» это безопасно — если он мёртв даже с идеальными
углами, в полёте тем более.

Конфиг оценщика — ЛЁТНЫЙ (config.py + .env прогона), не класс-дефолты:
ipm_model/derot/wz_tau/win/adapt/vel_tau/alt_floor/scale_ref задаются env.

Запуск (в контейнере nav — нужен cv2 и control_pkg):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash;
    PYTHONPATH=/root/sim_ws/src/control:$PYTHONPATH \
    IA_BAG=/root/sim_ws/output/joystick/.../bag python3 /lab/ipm_alt_replay.py'
⚠️ PYTHONPATH на ИСХОДНИКИ, а не на install: colcon КОПИРУЕТ ament_python-пакет,
иначе стенд молча проверит версию кода на момент последнего colcon build.
ВИЗУАЛИЗАЦИЯ (IA_WARP_MP4): рядом с метриками стенд умеет писать видео «что видит
канал» — слева исходный кадр с НАРИСОВАННОЙ полосой земли (четыре угла, спроецированные
БОЕВЫМ `_ipm_px`), справа сам выпрямленный варп, тот самый, по которому считается LK.
Геометрия берётся из `_ipm_prev_geo` оценщика, а не пересчитывается стендом — рисуем
ровно то, что канал и обработал. Строка кода причины сверху: видно, на каких кадрах
он отваливается и почему. Сама рисовалка — в общем `ipm_panel.py`: ею же пишет
`ipm_video.py` артефакт прогона `scene_ipm.mp4` (рядом с `scene_hud.mp4`), так что
стенд и прогон показывают ОДНО И ТО ЖЕ.

Env: IA_BAG, IA_CAM_W/IA_CAM_H (960×540), IA_T0/IA_T1 (окно воздуха, с от начала
bag; по умолчанию считается по истинной AGL > IA_AIR), IA_CSV (дамп по кадрам),
IA_WARP_MP4 (путь к видео варпа), IA_WARP_VAR (A/B/C, чей варп писать; default C),
IA_WARP_ZOOM (увеличение варпа, default 3).
"""
import math
import os
import sys
from collections import Counter

import numpy as np

import cv2
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image

from control_pkg.perception.flow_estimator import FlowEstimator

from ipm_panel import FAIL_NAME, warp_panel      # общая рисовалка (см. ipm_panel.py)

BAG = os.environ.get('IA_BAG', '/root/sim_ws/output/scene_bag')
CAM_W = float(os.environ.get('IA_CAM_W', 960))
CAM_H = float(os.environ.get('IA_CAM_H', 540))
AIR = float(os.environ.get('IA_AIR', 0.15))       # м AGL: «в воздухе»
CSV = os.environ.get('IA_CSV', '')
WARP_MP4 = os.environ.get('IA_WARP_MP4', '')
WARP_VAR = os.environ.get('IA_WARP_VAR', 'C')[0].upper()
WARP_ZOOM = int(os.environ.get('IA_WARP_ZOOM', 3))
# ровно то, что кладёт в оценщик bootstrap_node (FLOW_R)
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
# ЛЁТНЫЙ конфиг прогона (config.py + .env): derot/wz_tau/win из lv2_..._183305.env,
# остальное — лётные дефолты config.py
CFG = dict(ipm_model=os.environ.get('IA_MODEL', 'exact'),
           ipm_derot=float(os.environ.get('IA_DEROT', 1.0)),
           ipm_wz_tau=float(os.environ.get('IA_WZTAU', 2.0)),
           ipm_win=float(os.environ.get('IA_WIN', 0.5)),
           ipm_adapt=float(os.environ.get('IA_ADAPT', 1.05)),
           ipm_vel_tau=float(os.environ.get('IA_VELTAU', 0.4)),
           ipm_alt_floor=float(os.environ.get('IA_ALTFLOOR', 0.5)),
           ipm_scale_ref=float(os.environ.get('IA_SCALEREF', 3.0)),
           ipm_acc_tau=float(os.environ.get('IA_ACCTAU', 0.0)))


def euler(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read_refs(bag):
    """Первый проход: истина Gazebo + EKF local z (кадры НЕ грузим — их много)."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    r.set_filter(__import__('rosbag2_py').StorageFilter(
        topics=['/model/iris_cam/odometry', '/mavros/local_position/pose',
                '/mavros/state']))
    od, lp, t_arm = [], [], None
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/mavros/state':
            m = deserialize_message(raw, State)
            if m.armed and t_arm is None:
                t_arm = stamp(m)
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            v, w = m.twist.twist.linear, m.twist.twist.angular
            od.append((stamp(m), p.x, p.y, p.z) + euler(m.pose.pose.orientation)
                      + (v.x, v.y, v.z, w.x, w.y, w.z))
        else:
            m = deserialize_message(raw, PoseStamped)
            lp.append((stamp(m), m.pose.position.z))
    return np.array(od), np.array(lp), t_arm


def main():
    od, lp, t_arm = read_refs(BAG)
    if not len(od) or not len(lp):
        sys.exit('⚠️ в bag нет /model/iris_cam/odometry или /mavros/local_position/pose')
    t0 = od[0, 0]
    ground = float(np.median(od[:60, 3]))          # борт стоит на земле
    agl_all = od[:, 3] - ground
    print(f'bag {BAG}')
    print(f'  истина: {len(od)} сэмпл., {od[-1,0]-t0:.1f} sim-с, база земли z={ground:.3f} м')
    # окно воздуха
    if os.environ.get('IA_T0'):
        w0, w1 = float(os.environ['IA_T0']), float(os.environ['IA_T1'])
    else:
        air = agl_all > AIR
        if not air.any():
            sys.exit('⚠️ борт не отрывался — сравнивать нечего')
        w0 = od[np.argmax(air), 0] - t0
        w1 = od[len(air) - 1 - np.argmax(air[::-1]), 0] - t0
    print(f'  окно воздуха: {w0:.1f}…{w1:.1f} с (AGL > {AIR} м), '
          f'AGL сред {agl_all[(od[:,0]-t0>=w0)&(od[:,0]-t0<=w1)].mean():.2f} '
          f'макс {agl_all.max():.2f} м')
    # СВЕРКА КОНВЕНЦИИ twist: body FLU против производной мировой позиции
    yaw = od[:, 6]
    vx_w = np.gradient(od[:, 1], od[:, 0]); vy_w = np.gradient(od[:, 2], od[:, 0])
    fwd_chk = vx_w * np.cos(yaw) + vy_w * np.sin(yaw)
    lat_chk = -vx_w * np.sin(yaw) + vy_w * np.cos(yaw)
    m = np.abs(fwd_chk) + np.abs(lat_chk) > 0.2
    if m.sum() > 20:
        print(f'  сверка twist ↔ d(поза)/dt: corr вперёд '
              f'{np.corrcoef(od[m,7], fwd_chk[m])[0,1]:+.2f}, вбок '
              f'{np.corrcoef(od[m,8], lat_chk[m])[0,1]:+.2f} (ждём +1 при body-FLU)')

    # z₀ латча: медиана EKF z за 2 с до арма (нода латчит на переходе armed)
    if t_arm is None:
        sys.exit('⚠️ в bag нет /mavros/state с armed — латч не воспроизвести')
    pre = (lp[:, 0] >= t_arm - 2.0) & (lp[:, 0] <= t_arm)
    z0 = float(np.median(lp[pre, 1])) if pre.sum() else 0.0
    print(f'  арм в t={t_arm-t0:.1f} с, z₀ латча = {z0:+.2f} м '
          f'(по {int(pre.sum())} сэмпл. за 2 с до арма)')
    ests = {'A ekf  ': FlowEstimator(CAM_W/2, CAM_W/2, CAM_W/2, CAM_H/2, FLOW_R, **CFG),
            'B true ': FlowEstimator(CAM_W/2, CAM_W/2, CAM_W/2, CAM_H/2, FLOW_R, **CFG),
            'C latch': FlowEstimator(CAM_W/2, CAM_W/2, CAM_W/2, CAM_H/2, FLOW_R, **CFG)}
    rows = {k: [] for k in ests}
    print(f'  конфиг оценщика: ' + ' '.join(f'{k}={v}' for k, v in CFG.items()))

    # второй проход: кадры потоком (в RAM не копим), оба оценщика на одном кадре
    r = SequentialReader()
    r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    r.set_filter(__import__('rosbag2_py').StorageFilter(topics=['/image_color']))
    n = 0
    warp_w, warp_frames, warp_t, warp_sz = None, [], [], None
    while r.has_next():
        _topic, raw, _ = r.read_next()
        msg = deserialize_message(raw, Image)
        t = stamp(msg)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == 'bgr8':
            gray = cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_BGR2GRAY)
        elif msg.encoding in ('mono8', '8UC1'):
            gray = buf.reshape(msg.height, msg.width).copy()
        else:
            continue
        roll = float(np.interp(t, od[:, 0], od[:, 4]))
        pitch = float(np.interp(t, od[:, 0], od[:, 5]))
        wz = float(np.interp(t, od[:, 0], od[:, 12]))
        alt_true = float(np.interp(t, od[:, 0], od[:, 3])) - ground
        alt_ekf = max(0.0, float(np.interp(t, lp[:, 0], lp[:, 1])))
        alt_latch = max(0.0, float(np.interp(t, lp[:, 0], lp[:, 1])) - z0)
        alt_of = {'A': alt_ekf, 'B': alt_true, 'C': alt_latch}
        for key, est in ests.items():
            alt = alt_of[key[0]]
            est._ipm_update(gray, t, alt, pitch, roll, wz)
            rows[key].append((t - t0, est.ipm_fail, float(est.ipm_ok),
                              est.ipm_vfwd, est.ipm_vlat, alt))
            if WARP_MP4 and key[0] == WARP_VAR and w0 <= t - t0 <= w1:
                img = warp_panel(gray, est, alt, pitch, roll, t - t0,
                                 zoom=WARP_ZOOM, agl=alt_true)
                if warp_w is None:
                    warp_frames.append(img); warp_t.append(t)
                    if len(warp_frames) >= 60:      # оценка fps по первым кадрам
                        dt = float(np.median(np.diff(warp_t)))
                        fps = max(1.0, min(1.0 / dt if dt > 0 else 10.0, 60.0))
                        warp_sz = (img.shape[1], img.shape[0])
                        warp_w = cv2.VideoWriter(
                            WARP_MP4, cv2.VideoWriter_fourcc(*'mp4v'), fps, warp_sz)
                        for fr in warp_frames:
                            warp_w.write(fr)
                        warp_frames.clear()
                else:
                    # кадр чужого размера cv2 МОЛЧА выбросил бы (панель теперь
                    # фиксирована, но терять кадры без единого слова — дороже)
                    if (img.shape[1], img.shape[0]) != warp_sz:
                        img = cv2.resize(img, warp_sz, interpolation=cv2.INTER_AREA)
                    warp_w.write(img)
        n += 1
    if WARP_MP4 and warp_w is not None:
        warp_w.release()
        print(f'  видео варпа (вариант {WARP_VAR}) → {WARP_MP4}')
    elif WARP_MP4:
        print('  ⚠️ кадров меньше пробы fps — видео варпа не записано')
    print(f'  кадров прокручено: {n}')

    for key in ests:
        a = np.array(rows[key])
        sel = (a[:, 0] >= w0) & (a[:, 0] <= w1)
        what = {'A': 'EKF local z, КАК ЛЕТАЛИ', 'B': 'ИСТИННАЯ AGL (потолок)',
                'C': 'EKF local z − z₀ латча на арме (КАНДИДАТ-ФИКС)'}[key[0]]
        print(f'\n=== {key} (alt = {what}) ===')
        print(f'  alt на вход: сред {a[sel,5].mean():.2f} макс {a[sel,5].max():.2f} м')
        c = Counter(int(v) for v in a[sel, 1])
        for f, cnt in c.most_common():
            print(f'    код {f} {FAIL_NAME.get(f,"?"):18s} {cnt:5d}  '
                  f'{100.0*cnt/max(1,sel.sum()):5.1f}%')
        print(f'  ipm_ok (скорости можно верить): {100.0*a[sel,2].mean():.0f}% кадров')
        ok = sel & (a[:, 2] > 0.5)
        if ok.sum() < 20:
            print('  измерений мало — метрику не считаем')
            continue
        ts = a[ok, 0] + t0
        vt_f = np.interp(ts, od[:, 0], od[:, 7])       # body FLU: x вперёд
        vt_l = np.interp(ts, od[:, 0], od[:, 8])       # body FLU: y ВЛЕВО
        for name, meas, true in (('продольная (вперёд+)', a[ok, 3], vt_f),
                                 ('боковая (влево+)', a[ok, 4], vt_l)):
            A = np.column_stack([true, np.ones(len(true))])
            (g, b), *_ = np.linalg.lstsq(A, meas, rcond=None)
            cor = np.corrcoef(meas, true)[0, 1] if true.std() > 1e-6 else float('nan')
            print(f'  {name:22s} gain {g:+.2f}  corr {cor:+.2f}  '
                  f'сдвиг {np.mean(meas-true):+.2f} м/с  СКО ош {np.std(meas-true):.2f}  '
                  f'|v_ист| сред {np.abs(true).mean():.2f} м/с')
    if CSV:
        with open(CSV, 'w') as f:
            f.write('t,var,fail,ok,vfwd,vlat,alt\n')
            for key in ests:
                for row in rows[key]:
                    f.write(f'{row[0]:.3f},{key.strip()},{int(row[1])},{row[2]:.0f},'
                            f'{row[3]:.4f},{row[4]:.4f},{row[5]:.3f}\n')
        print(f'\nдамп по кадрам → {CSV}')


if __name__ == '__main__':
    main()
