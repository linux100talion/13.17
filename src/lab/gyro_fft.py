#!/usr/bin/env python3
# gyro_fft.py — FFT гироскопа из rosbag (диагностика осцилляций / лимит-цикла
# rate-loop, см. docker/sim/FAQ_rate_loop.md). Делит запись на окна
# ground/air/late и по каждой оси (roll/pitch/yaw) печатает RMS (°/с) и пик (Гц).
# Sim-частота гиро считается из header.stamp. Нужен IMU в bag (TOPICS_EXTRA=
# "/mavros/imu/data_raw"). Запуск внутри nav:
#   python3 /lab/gyro_fft.py [bag] [imu_topic]
import sys, numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
BAG = sys.argv[1] if len(sys.argv) > 1 else '/root/sim_ws/output/scene_bag'
TOPIC = sys.argv[2] if len(sys.argv) > 2 else '/mavros/imu/data_raw'
r = SequentialReader(); r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('', ''))
ts = []; gx = []; gy = []; gz = []
while r.has_next():
    t, d, ns = r.read_next()
    if t != TOPIC: continue
    m = deserialize_message(d, Imu)
    ts.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9)
    gx.append(m.angular_velocity.x); gy.append(m.angular_velocity.y); gz.append(m.angular_velocity.z)
ts = np.array(ts); gx = np.array(gx); gy = np.array(gy); gz = np.array(gz)
if len(ts) < 50:
    print(f'мало IMU в {TOPIC}: {len(ts)}'); raise SystemExit(1)
fs = (len(ts)-1)/(ts[-1]-ts[0])
print(f'IMU сэмплов: {len(ts)}; sim-окно {ts[-1]-ts[0]:.1f}s; sim-rate ≈ {fs:.1f} Гц')
n = len(ts)
def fft_rms(sig, lab):
    sig = sig - sig.mean(); rms = np.sqrt((sig**2).mean())*180/np.pi
    F = np.abs(np.fft.rfft(sig)); f = np.fft.rfftfreq(len(sig), 1/fs)
    pk = f[1+int(np.argmax(F[1:]))] if len(F) > 2 else 0.0
    print(f'  {lab:5} RMS={rms:6.1f} °/с   пик={pk:5.2f} Гц')
for lab, (a, b) in {'GROUND (0-8%)': (0.0, 0.08), 'AIR (15-55%)': (0.15, 0.55),
                    'LATE (55-90%)': (0.55, 0.90)}.items():
    i0, i1 = int(n*a), int(n*b)
    print(f'\n[{lab}]  сэмплов {i1-i0}')
    fft_rms(gx[i0:i1], 'roll'); fft_rms(gy[i0:i1], 'pitch'); fft_rms(gz[i0:i1], 'yaw')
