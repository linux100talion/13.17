#!/usr/bin/env python3
# attitude.py — угол крена/тангажа/рыскания во времени из bag (/mavros/imu/data
# ориентация). Робастно при низкой частоте IMU: угол — медленный сигнал, прямо
# показывает осцилляции (частота+амплитуда) и расхождение (растущий размах) →
# для тюнинга PID (ATC_RAT_RLL_*, см. docker/sim/FAQ_rate_loop.md).
# По окнам полёта печатает диапазон, RMS относительно среднего и доминирующую
# частоту колебаний. Запуск внутри nav:
#   python3 /lab/attitude.py [bag] [imu_topic]
import sys, math, numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
BAG = sys.argv[1] if len(sys.argv) > 1 else '/root/sim_ws/output/scene_bag'
TOPIC = sys.argv[2] if len(sys.argv) > 2 else '/mavros/imu/data'

def quat_to_euler(x, y, z, w):
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    s = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(s)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

r = SequentialReader(); r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('', ''))
ts = []; R = []; P = []; Y = []
while r.has_next():
    t, d, ns = r.read_next()
    if t != TOPIC: continue
    m = deserialize_message(d, Imu); o = m.orientation
    if o.w == 0 and o.x == 0 and o.y == 0 and o.z == 0:
        continue  # пустая ориентация
    ro, pi, ya = quat_to_euler(o.x, o.y, o.z, o.w)
    ts.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9); R.append(ro); P.append(pi); Y.append(ya)
ts = np.array(ts); R = np.array(R); P = np.array(P); Y = np.array(Y)
if len(ts) < 20:
    print(f'мало ориентации в {TOPIC}: {len(ts)} (ATTITUDE стримился?)'); raise SystemExit(1)
fs = (len(ts)-1)/(ts[-1]-ts[0])
print(f'{TOPIC}: {len(ts)} сэмплов; sim-окно {ts[-1]-ts[0]:.1f}s; sim-rate ≈ {fs:.1f} Гц')
n = len(ts)
def stats(sig, lab):
    rng = sig.max() - sig.min()
    ac = sig - sig.mean(); rms = np.sqrt((ac**2).mean())
    F = np.abs(np.fft.rfft(ac)); f = np.fft.rfftfreq(len(ac), 1/fs)
    pk = f[1+int(np.argmax(F[1:]))] if len(F) > 2 else 0.0
    print(f'  {lab:5} размах={rng:6.1f}°  RMS={rms:6.1f}°  пик={pk:5.2f} Гц  '
          f'[{sig.min():.0f}..{sig.max():.0f}]')
for lab, (a, b) in {'GROUND (0-8%)': (0.0, 0.08), 'AIR (15-55%)': (0.15, 0.55),
                    'LATE→удар (55-95%)': (0.55, 0.95)}.items():
    i0, i1 = int(n*a), int(n*b)
    print(f'\n[{lab}]  сэмплов {i1-i0}')
    stats(R[i0:i1], 'roll'); stats(P[i0:i1], 'pitch'); stats(Y[i0:i1], 'yaw')
