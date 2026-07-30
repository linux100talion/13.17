#!/usr/bin/env python3
"""Разбор прогона с командой: поехала ли уставка, поехал ли за ней борт, удержал ли.

Окно манёвра берём НЕ из лога, а из самого бэга: /flow_dbg5.z (скорость уставки) не
ноль ровно на mv_*-сегменте. Истина — продольная проекция одометрии в курсе на начало
манёвра. Зрение — /flow_dbg3.x (kf_logs), в метрах через крутизну SLOPE.
"""
import math
import os

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from geometry_msgs.msg import Vector3Stamped

BAG = os.environ.get('MV_BAG', '/root/sim_ws/output/L1_mvfwd_bag')
SLOPE = 0.0145          # log на метр (замер J2/K1s: 1.45-1.58 %/м)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def euler_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
d5, d3, d2, od = [], [], [], []
while r.has_next():
    topic, raw, ts = r.read_next()
    if topic == '/flow_dbg5':
        m = deserialize_message(raw, Vector3Stamped)
        d5.append((stamp(m), m.vector.x, m.vector.y, m.vector.z))
    elif topic == '/flow_dbg3':
        m = deserialize_message(raw, Vector3Stamped)
        d3.append((stamp(m), m.vector.x, m.vector.y, m.vector.z))
    elif topic == '/flow_dbg2':
        m = deserialize_message(raw, Vector3Stamped)
        d2.append((stamp(m), m.vector.x))
    elif topic == '/model/iris_cam/odometry':
        m = deserialize_message(raw, Odometry)
        p = m.pose.pose.position
        od.append((stamp(m), p.x, p.y, p.z, euler_yaw(m.pose.pose.orientation)))
d5 = np.array(d5); d3 = np.array(d3); d2 = np.array(d2); od = np.array(od)
print(f'бэг {BAG}: dbg5 {len(d5)}, dbg3 {len(d3)}, одометрия {len(od)}')
if not len(d5):
    raise SystemExit('⚠️ /flow_dbg5 в бэге нет — уставку не проверить')

# --- окно манёвра: скорость уставки не ноль ---
mv = np.abs(d5[:, 3]) > 1e-6
if not mv.any():
    raise SystemExit('⚠️ скорость уставки везде ноль — команда не дошла')
i0, i1 = int(np.argmax(mv)), len(mv) - 1 - int(np.argmax(mv[::-1]))
t0, t1 = d5[i0, 0], d5[i1, 0]
rate = np.median(d5[mv, 3])
print(f'\nМАНЁВР: {t1 - t0:.1f} с sim, скорость уставки {rate:+.4f} log/с '
      f'= {rate / SLOPE:+.2f} м/с')

# --- истина: продольная проекция в курсе на начало манёвра ---
yaw0 = float(np.interp(t0, od[:, 0], np.unwrap(od[:, 4])))
x0 = float(np.interp(t0, od[:, 0], od[:, 1]))
y0 = float(np.interp(t0, od[:, 0], od[:, 2]))
fwd = ((od[:, 1] - x0) * math.cos(yaw0) + (od[:, 2] - y0) * math.sin(yaw0))
lat = (-(od[:, 1] - x0) * math.sin(yaw0) + (od[:, 2] - y0) * math.cos(yaw0))

sp_m = (d5[:, 1] - d5[i0, 1]) / SLOPE            # уставка в метрах от начала манёвра
vis_m = np.interp(d5[:, 0], d3[:, 0], d3[:, 1]) / SLOPE if len(d3) else np.zeros(len(d5))
true_m = np.interp(d5[:, 0], od[:, 0], fwd)
lat_m = np.interp(d5[:, 0], od[:, 0], lat)

seg_mv = (d5[:, 0] >= t0) & (d5[:, 0] <= t1)
seg_hold = (d5[:, 0] > t1) & (d5[:, 0] <= t1 + 20.0)
print(f'\n{"":22} | {"уставка":>9} | {"зрение":>9} | {"ИСТИНА":>9}')
print(f'{"конец манёвра":22} | {sp_m[seg_mv][-1]:8.2f}м | {vis_m[seg_mv][-1]:8.2f}м | '
      f'{true_m[seg_mv][-1]:8.2f}м')
if seg_hold.any():
    print(f'{"среднее за hover20":22} | {sp_m[seg_hold].mean():8.2f}м | '
          f'{vis_m[seg_hold].mean():8.2f}м | {true_m[seg_hold].mean():8.2f}м')
    print(f'{"СКО за hover20":22} | {"":9} | {"":9} | {true_m[seg_hold].std():8.2f}м')
    print(f'{"дрейф за hover20":22} | {"":9} | {"":9} | '
          f'{true_m[seg_hold][-1] - true_m[seg_hold][0]:+8.2f}м')

# --- едет ли борт ВСЛЕД за уставкой (а не сам по себе) ---
if seg_mv.sum() > 5:
    v_true = np.polyfit(d5[seg_mv, 0], true_m[seg_mv], 1)[0]
    print(f'\nсредняя ИСТИННАЯ скорость на манёвре: {v_true:+.2f} м/с '
          f'(команда {rate / SLOPE:+.2f} м/с)')
    print(f'боковой снос за манёвр: {lat_m[seg_mv][-1] - lat_m[seg_mv][0]:+.2f} м')
    print(f'ошибка до уставки: старт {d5[i0, 2] / SLOPE:+.2f} м, '
          f'конец {d5[i1, 2] / SLOPE:+.2f} м')

# --- качество канала зрения на этом прогоне ---
ok = np.isfinite(vis_m) & np.isfinite(true_m)
win = ok & (d5[:, 0] >= t0 - 10.0)
if win.sum() > 30:
    cc = np.corrcoef(vis_m[win], true_m[win])[0, 1]
    a1 = np.polyfit(true_m[win], vis_m[win] * SLOPE, 1)[0]
    print(f'\nканал положения (манёвр+висение): corr {cc:+.2f}, '
          f'крутизна {a1 * 100:.2f} %/м')

# --- команда на провод ---
if len(d2):
    p = np.interp(d5[:, 0], d2[:, 0], d2[:, 1])
    print(f'тангаж-команда: манёвр {p[seg_mv].mean():+.0f} PWM, '
          f'висение после {p[seg_hold].mean() if seg_hold.any() else float("nan"):+.0f} PWM, '
          f'в потолке |150| {100.0 * (np.abs(p[seg_mv]) >= 149).mean():.0f}% кадров')
