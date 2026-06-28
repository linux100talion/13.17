#!/usr/bin/env python3
# two_clocks.py — wall-fps vs sim-Гц топика (по header.stamp). Показывает разрыв
# реального и sim-времени при низком RTF: видео/VINS живут в sim-времени (header.
# stamp), а лог camera_node — в wall. Запуск внутри nav-контейнера:
#   python3 /lab/two_clocks.py [topic]      (default /image_color; *imu* → Imu)
import sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/image_color'
if 'imu' in TOPIC:
    from sensor_msgs.msg import Imu as Msg
else:
    from sensor_msgs.msg import Image as Msg
rclpy.init(); n = rclpy.create_node('two_clocks')
stamps = []   # (wall_recv, sim_header)
n.create_subscription(Msg, TOPIC,
    lambda m: stamps.append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec*1e-9)),
    qos_profile_sensor_data)
t0 = time.time()
while len(stamps) < 25 and time.time()-t0 < 40:
    rclpy.spin_once(n, timeout_sec=0.5)
if len(stamps) < 3:
    print(f'мало кадров {TOPIC}: {len(stamps)}'); raise SystemExit(1)
wall = stamps[-1][0]-stamps[0][0]; sim = stamps[-1][1]-stamps[0][1]; nf = len(stamps)-1
print(f'{TOPIC}: {len(stamps)} сообщений')
print(f'WALL: {wall:.2f}s → {nf/wall:.2f} fps (реальное время)')
print(f'SIM:  {sim:.2f}s → {nf/sim:.2f} Гц (header.stamp — так считают make_video/VINS)')
print(f'RTF ≈ {sim/wall:.3f}')
