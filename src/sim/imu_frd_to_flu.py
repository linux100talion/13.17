#!/usr/bin/env python3
"""
imu_frd_to_flu.py — конвертер gz-IMU из FRD в FLU для VINS.

gz IMU-сенсор (мостится в /gz_imu/data @250Гц, обход MAVLink-тракта — см. todo3)
публикует в аэрокосмическом FRD (z-вниз: на земле az=-9.8) из-за
modelXYZToAirplaneXForwardZDown 180° в model.sdf. А экстринсики камера-IMU в
sim.yaml считались под MAVROS-IMU, который во FLU (ROS-конвенция, z-вверх, az=+9.8).
Чтобы /gz_imu/data стал drop-in заменой /mavros/imu/data_raw В ТОМ ЖЕ ФРЕЙМЕ (но на
250Гц вместо ~21), флипаем оси по карте FRD→FLU (поворот 180° вокруг X):
    accel: ax→ax, ay→−ay, az→−az
    gyro:  gx→gx, gy→−gy, gz→−gz
Знак z подтверждён эмпирически (гравитация); x/y по конвенции (validate: VINS-init).
VINS ориентацию IMU не использует — копируем как есть.

На боевом Orin этого нет: там IMU берётся с FCU через MAVROS (уже FLU, ~200Гц).
Запускается в nav_up.sh (nohup), вход /gz_imu/data → выход /gz_imu/data_flu.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

OUT_TOPIC = '/gz_imu/data_flu'
IN_TOPIC = '/gz_imu/data'


class ImuFRDtoFLU(Node):
    def __init__(self):
        super().__init__('imu_frd_to_flu')
        # SensorData QoS (best-effort) — как /mavros/imu/data_raw, чтобы VINS-подписка
        # (совместима с best-effort) принимала так же, как сейчас MAVROS-IMU.
        self.pub = self.create_publisher(Imu, OUT_TOPIC, qos_profile_sensor_data)
        self.create_subscription(Imu, IN_TOPIC, self._cb, qos_profile_sensor_data)
        self.get_logger().info(f'imu_frd_to_flu: {IN_TOPIC} (FRD) → {OUT_TOPIC} (FLU)')

    def _cb(self, m):
        o = Imu()
        o.header = m.header
        o.linear_acceleration.x = m.linear_acceleration.x
        o.linear_acceleration.y = -m.linear_acceleration.y
        o.linear_acceleration.z = -m.linear_acceleration.z
        o.angular_velocity.x = m.angular_velocity.x
        o.angular_velocity.y = -m.angular_velocity.y
        o.angular_velocity.z = -m.angular_velocity.z
        # ориентацию VINS не использует — копируем; ковариации тоже
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
