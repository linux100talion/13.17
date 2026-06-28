#!/usr/bin/env python3
# imu_rate.py — печатает SIM-частоту /mavros/imu/data_raw (по header.stamp, не wall).
# Нужен nav_up.sh для подтверждения IMU-стрима ПО ЧАСТОТЕ (а не только по факту):
# на низком RTF wall-rate мизерный, поэтому меряем sim-time из штампов сообщений.
# Печатает одно число (Гц) в stdout; exit 0 если кадров хватило, иначе exit 1.
import sys, time
import rclpy
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

N = int(sys.argv[1]) if len(sys.argv) > 1 else 40   # сколько сообщений собрать
WALL_TIMEOUT = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

rclpy.init()
node = rclpy.create_node('imu_rate')
stamps = []
node.create_subscription(
    Imu, '/mavros/imu/data_raw',
    lambda m: stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
    qos_profile_sensor_data)

t0 = time.time()
while len(stamps) < N and time.time() - t0 < WALL_TIMEOUT:
    rclpy.spin_once(node, timeout_sec=0.2)

if len(stamps) < 3:
    print('0.0')
    sys.exit(1)
dt = stamps[-1] - stamps[0]
hz = (len(stamps) - 1) / dt if dt > 0 else 0.0
print(f'{hz:.1f}')
sys.exit(0)
