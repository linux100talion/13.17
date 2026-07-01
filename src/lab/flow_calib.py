#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""СЛОЙ C — калибровка параметров планта симулятора B из РЕАЛЬНОГО бэга (system-ID).

Делает surrogate B (flow_loop_sim.py) ПРЕДСКАЗАТЕЛЬНЫМ: вытаскивает k, s, bias, d, τ
из уже снятого прогона демпфера, чтобы B называл kp под целевую терминальную скорость,
а не гадал. Мост «реальность → синтетика»: реальные кадры+гиро гоняются оффлайн через
ТОТ ЖЕ FlowEstimator (детерминированно), а истинная поза Gazebo даёт скорость/ускорение.

Что фитим (из O2-бэга: /image_color, /gz_imu/data_flu, /model/iris_cam/odometry):
  s, bias : flow ≈ s·v_right + bias      (реплей flow vs истинная боковая скорость) —
            прямое сенсорное отношение, хорошо обусловлено; bias = реальный DC LK-снос.
  τ       : лаг кросс-корреляции flow ↔ v_right (задержка сенсорного тракта).
  k       : из физики руля (roll_off PWM → угол крена → a=g·tanθ); знак — из osign
            (osign=+1 демпфирует → положит. roll_off тормозит положит. v_right ⇒ k<0).
  d       : из стационара a_right = k·roll_off + d (среднее по окну).
Валидация: предсказанная v_term = d/(|k|·kp·s) vs наблюдаемая RMS(v_right) прогона.

Запуск В КОНТЕЙНЕРЕ nav:
  docker exec -i -e SAFE_SEC=30 -e R_SAFE=40 -e CAL_KP=4 p1317_nav \
    bash -lc 'source /opt/ros/humble/setup.bash; python3 /lab/flow_calib.py'

Env: SCENE_BAG, ODOM_TOPIC(/model/iris_cam/odometry), IMU_TOPIC(/gz_imu/data_flu),
IMG_TOPIC(/image_color), SAFE_SEC(30), R_SAFE(40), Z_TO(2.0), CAL_KP(4), CAL_OSIGN(1),
ANGLE_MAX_DEG(45) — предельный крен ALT_HOLD (ArduCopter ANGLE_MAX, дефолт 4500 сдг).
"""
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image

sys.path.insert(0, '/lab')
from flow_estimator import FlowEstimator  # noqa: E402

# интринсики/экстринсики — зеркало alt_hold_bootstrap.py:59-61
FX = FY = 640.0
CX, CY = 640.0, 360.0
R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]

BAG = os.environ.get('SCENE_BAG', '/root/sim_ws/output/scene_bag')
ODOM = os.environ.get('ODOM_TOPIC', '/model/iris_cam/odometry')
IMU = os.environ.get('IMU_TOPIC', '/gz_imu/data_flu')
IMG = os.environ.get('IMG_TOPIC', '/image_color')
SAFE_SEC = float(os.environ.get('SAFE_SEC', '30'))
R_SAFE = float(os.environ.get('R_SAFE', '40'))
Z_TO = float(os.environ.get('Z_TO', '2.0'))
CAL_KP = float(os.environ.get('CAL_KP', '4'))       # kp прогона (O2 = 4)
CAL_OSIGN = float(os.environ.get('CAL_OSIGN', '1'))
ANGLE_MAX = math.radians(float(os.environ.get('ANGLE_MAX_DEG', '45')))
G = 9.81


def read_bag():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    od, im, ig = [], [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == ODOM:
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position; q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                       p.x, p.y, p.z, yaw))
        elif topic == IMU:
            m = deserialize_message(data, Imu)
            w = m.angular_velocity
            ig.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, w.x, w.y, w.z))
        elif topic == IMG:
            m = deserialize_message(data, Image)
            im.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                       m.height, m.width, m.encoding, bytes(m.data)))
    return (np.array(od, dtype=float), im,
            np.array(ig, dtype=float))


def to_gray(h, w, enc, buf):
    a = np.frombuffer(buf, dtype=np.uint8)
    if enc in ('mono8', '8UC1'):
        return a[:h * w].reshape(h, w)
    if enc in ('bgr8', 'rgb8'):
        img = a[:h * w * 3].reshape(h, w, 3).astype(np.float32)
        # luminance (порядок каналов не критичен для LK-текстуры)
        return (0.299 * img[:, :, 2] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 0]).astype(np.uint8)
    raise RuntimeError(f'неизвестная кодировка {enc}')


def main():
    od, im, ig = read_bag()
    if len(od) == 0 or len(im) == 0 or len(ig) == 0:
        raise RuntimeError('в бэге нет нужных топиков (odom/img/imu)')
    t0 = od[0, 0]
    ot = od[:, 0] - t0; X, Y, Z, YAW = od[:, 1], od[:, 2], od[:, 3], od[:, 4]

    # --- окно (как drift_check): взлёт Z>Z_TO, +SAFE_SEC, обрезка по R_SAFE ---
    to = np.where(Z > Z_TO)[0]
    if len(to) == 0:
        print('дрон не взлетел'); return 1
    i_to = to[0]; t_to = ot[i_to]; x0, y0 = X[i_to], Y[i_to]
    rh = np.hypot(X - x0, Y - y0)
    t_end = t_to + SAFE_SEC
    out = np.where((ot > t_to) & (rh > R_SAFE))[0]
    if len(out) and ot[out[0]] < t_end:
        t_end = ot[out[0]]
    print(f'окно [{t_to:.1f},{t_end:.1f}]s ({t_end-t_to:.1f}s), kp={CAL_KP:g} osign={CAL_OSIGN:+g}')

    # --- истинная боковая скорость/ускорение (per-sample yaw) ---
    win = (ot >= t_to) & (ot <= t_end); iw = np.where(win)[0]
    xa, ya, ta, yawa = X[iw], Y[iw], ot[iw], YAW[iw]
    dt = np.diff(ta); vx = np.diff(xa) / dt; vy = np.diff(ya) / dt
    ym = 0.5 * (yawa[:-1] + yawa[1:])
    vr = -vx * np.sin(ym) + vy * np.cos(ym)          # боковая скорость (roll)
    tv = 0.5 * (ta[:-1] + ta[1:])
    # сглаживание (EMA a=0.4, как gt_vy в ноде), затем ускорение
    vr_s = vr.copy()
    for i in range(1, len(vr_s)):
        vr_s[i] = 0.6 * vr_s[i - 1] + 0.4 * vr[i]
    ar = np.gradient(vr_s, tv)                        # боковое ускорение

    # --- реплей flow через FlowEstimator (гиро — ближайший до кадра) ---
    est = FlowEstimator(FX, FY, CX, CY, R, rotflow_sign=1.0)
    igt = ig[:, 0] - t0
    ft, fl, fc = [], [], []
    for (ts_abs, h, w, enc, buf) in im:
        ts = ts_abs - t0
        if ts < t_to - 1.0 or ts > t_end + 0.5:
            continue
        j = np.searchsorted(igt, ts) - 1
        omega = ig[max(0, j), 1:4]
        res = est.process(to_gray(h, w, enc, buf), ts, omega)
        if res is not None:
            ft.append(ts); fl.append(res['lateral']); fc.append(res['conf'])
    ft, fl, fc = np.array(ft), np.array(fl), np.array(fc)
    inwin = (ft >= t_to) & (ft <= t_end)
    ft, fl, fc = ft[inwin], fl[inwin], fc[inwin]
    print(f'сэмплов: odom {len(iw)}, flow {len(ft)} (conf {fc.mean():.2f})')
    if len(ft) < 10:
        print('мало flow-сэмплов'); return 1

    # --- ФИТ s,bias: flow ≈ s·v_right + bias (v_right интерп. на времена кадров) ---
    vr_at_f = np.interp(ft, tv, vr_s)
    A = np.column_stack([vr_at_f, np.ones_like(vr_at_f)])
    (s_fit, bias_fit), *_ = np.linalg.lstsq(A, fl, rcond=None)
    resid = fl - A @ [s_fit, bias_fit]
    r2 = 1.0 - np.var(resid) / max(1e-9, np.var(fl))
    noise_fit = float(np.std(resid))            # шум потока (СКО остатка), px

    # --- τ: лаг кросс-корреляции flow ↔ v_right (сенсорная задержка) ---
    fs = np.interp(tv, ft, fl)          # flow на равномерной odom-сетке
    a_ = fs - fs.mean(); b_ = vr_s - vr_s.mean()
    cc = np.correlate(a_, b_, mode='full')
    lags = np.arange(-len(b_) + 1, len(b_))
    dt_med = float(np.median(np.diff(tv)))
    tau_fit = max(0.0, lags[np.argmax(cc)] * dt_med)

    # --- k из физики руля; d из стационара ---
    roll_off = CAL_OSIGN * CAL_KP * fl          # реконструкция (ki=kd=0, blend≈1)
    roll_off = np.clip(roll_off, -150, 150)
    k_mag = G * math.tan(ANGLE_MAX) / 500.0     # м/с² на 1 PWM (полный стик 500 → ANGLE_MAX)
    k_fit = -abs(k_mag) if CAL_OSIGN > 0 else abs(k_mag)   # знак: osign=+1 → демпфер → k<0
    ro_at = np.interp(tv, ft, roll_off)
    d_fit = float(np.mean(ar - k_fit * ro_at))  # a = k·roll_off + d → d = <a - k·roll_off>

    rms_vr = float(np.sqrt(np.mean(vr_s ** 2)))
    print()
    print('=== КАЛИБРОВКА планта (для flow_loop_sim.py) ===')
    print(f'  s    = {s_fit:+.3f}  px/кадр на 1 м/с   (R²={r2:.2f} — доля дисперсии flow от v)')
    print(f'  noise= {noise_fit:.3f} px СКО   (шум потока; при R²={r2:.2f} — {100*(1-r2):.0f}% flow = шум/прочее)')
    print(f'  bias = {bias_fit:+.3f} px   (реальный DC LK-снос → windup при ki>0)')
    print(f'  τ    = {tau_fit:.2f} с   (лаг flow↔v_right)')
    print(f'  k    = {k_fit:+.4f} м/с² на 1 PWM   (руль: ANGLE_MAX={math.degrees(ANGLE_MAX):.0f}°)')
    print(f'  d    = {d_fit:+.3f} м/с²   (возмущение lean-сноса, стационар)')
    print()
    # --- валидация: предсказанная терминальная скорость vs наблюдаемая ---
    c = abs(k_fit) * CAL_KP * abs(s_fit)
    v_term_pred = abs(d_fit) / max(1e-9, c)
    print(f'валидация: c=|k|·kp·|s|={c:.3f} → v_term_pred={v_term_pred:.2f} м/с | '
          f'наблюд. RMS(v_right)={rms_vr:.2f} м/с')
    print()
    print('Команда B с калибровкой (проверь kp-свип на этих параметрах):')
    print(f'  python3 /lab/flow_loop_sim.py --sweep --k {k_fit:.4f} --s {abs(s_fit):.3f} '
          f'--tau {tau_fit:.2f} --dist {abs(d_fit):.3f} --flow-bias {abs(bias_fit):.3f} '
          f'--noise {noise_fit:.3f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
