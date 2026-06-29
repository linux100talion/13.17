#!/usr/bin/env python3
"""
imu_frd_to_flu.py — конвертер gz-IMU FRD→FLU + low-pass для VINS.

gz IMU-сенсор (мостится в /gz_imu/data @250Гц, обход MAVLink — см. todo3) публикует
в FRD (z-вниз, az=-9.8) из-за <pose>180 0 0</pose> сенсора. Экстринсики камера-IMU
в sim.yaml — под MAVROS-FLU. Делаем drop-in замену:
  1) FRD→FLU (поворот 180° вокруг X): ax→ax, ay→−ay, az→−az; gx→gx, gy→−gy, gz→−gz;
  2) LOW-PASS: у gz IMU-сенсора НЕТ модели шума → он честно отдаёт РЕАЛЬНУЮ тряску
     дрона = лимит-цикл rate-loop (~4-9Гц, см. FAQ_rate_loop.md). Камера на 10Гц его
     не видит (Найквист 5Гц) → IMU/камера рассогласованы → VINS failureDetection ~10с
     после init (тюнинг acc_n/gyr_n не помог — там БЕЛЫЙ шум, а это структурная
     осцилляция). 1-й порядок LP (cutoff GZ_IMU_LP_HZ, дефолт 10Гц; 0=выкл) срезает
     лимит-цикл, оставляя медленный полёт. На борту аналог — INS_GYRO_FILTER на FCU.

На боевом Orin этого нет: IMU берётся с FCU через MAVROS (FLU, фильтрован, ~200Гц).
Запускается в nav_up.sh, /gz_imu/data → /gz_imu/data_flu.
"""
import math
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

OUT_TOPIC = '/gz_imu/data_flu'
IN_TOPIC = '/gz_imu/data'
LP_HZ = float(os.environ.get('GZ_IMU_LP_HZ', '5.0'))    # cutoff, Гц (0 = без фильтра)
# 5Гц выбран по FFT gz-IMU из bag: roll/pitch лимит-цикл ~7.5Гц, yaw ВЧ-осцилляция
# ~70Гц; камера Найквист 5Гц. 5Гц убивает yaw-70 в ноль (−23дБ), гасит 7.5 (−5дБ),
# медленный полёт (<1Гц) сохраняет. Тюнить ниже (3-4) если VINS ещё капризит.


class ImuFRDtoFLU(Node):
    def __init__(self):
        super().__init__('imu_frd_to_flu')
        self.lp_hz = LP_HZ
        self.rc = 1.0 / (2.0 * math.pi * self.lp_hz) if self.lp_hz > 0 else 0.0
        self.fa = [0.0, 0.0, 0.0]   # состояние LP по accel (FLU)
        self.fg = [0.0, 0.0, 0.0]   # состояние LP по gyro (FLU)
        self.prev_t = None
        self.have_state = False
        self.pub = self.create_publisher(Imu, OUT_TOPIC, qos_profile_sensor_data)
        self.create_subscription(Imu, IN_TOPIC, self._cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'imu_frd_to_flu: {IN_TOPIC}(FRD) → {OUT_TOPIC}(FLU), '
            f'low-pass {"OFF" if self.lp_hz <= 0 else f"{self.lp_hz:.0f}Гц"}')

    def _cb(self, m):
        # 1) FRD→FLU (флип y,z)
        a = [m.linear_acceleration.x, -m.linear_acceleration.y, -m.linear_acceleration.z]
        g = [m.angular_velocity.x, -m.angular_velocity.y, -m.angular_velocity.z]
        # 2) low-pass 1-го порядка (adaptive alpha по dt из sim-штампа)
        if self.lp_hz > 0:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            dt = (t - self.prev_t) if (self.prev_t is not None) else 0.0
            self.prev_t = t
            if not self.have_state:
                self.fa, self.fg = list(a), list(g); self.have_state = True
            elif dt > 0.0:
                alpha = dt / (self.rc + dt)
                for i in range(3):
                    self.fa[i] += alpha * (a[i] - self.fa[i])
                    self.fg[i] += alpha * (g[i] - self.fg[i])
            a, g = self.fa, self.fg
        o = Imu()
        o.header = m.header
        o.linear_acceleration.x, o.linear_acceleration.y, o.linear_acceleration.z = a
        o.angular_velocity.x, o.angular_velocity.y, o.angular_velocity.z = g
        o.orientation = m.orientation
        o.orientation_covariance = m.orientation_covariance
        o.linear_acceleration_covariance = m.linear_acceleration_covariance
        o.angular_velocity_covariance = m.angular_velocity_covariance
        self.pub.publish(o)


def main():
    rclpy.init()
    node = ImuFRDtoFLU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
