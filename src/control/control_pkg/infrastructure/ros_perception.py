#!/usr/bin/env python3
"""RosPerception — адаптер: камера (mono8) + гироскоп (FLU) → FlowEstimator → flow_* в DroneState.

Зрительная часть боевого пре-VINS демпфера. Подписывается на кадр и ω, гоняет
FlowEstimator (control_pkg.perception), результат отдаёт домену через merge(state):
кладёт flow_lateral/flow_yaw/flow_conf/flow_dt и ИНКРЕМЕНТИРУЕТ flow_seq на КАЖДЫЙ
обработанный кадр (домен интегрирует PID покадрово по seq, а не по 20-Гц тику).

Интринсики берутся из разрешения камеры (pinhole 90° hfov: fx=fy=W/2, cx=W/2, cy=H/2).
R_cam_imu и rotflow_sign — из sim.yaml/монолита (подтверждены flow_derotation_check).
"""
import numpy as np

from ..perception.flow_estimator import FlowEstimator


class RosPerception:
    def __init__(self, node, cam_w, cam_h, R_cam_imu, rotflow_sign=1.0,
                 smooth_n=1, yaw_smooth_n=5,
                 image_topic='/image_mono', imu_topic='/gz_imu/data_flu'):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, Imu
        fx = fy = cam_w / 2.0          # pinhole 90° hfov
        cx, cy = cam_w / 2.0, cam_h / 2.0
        self._est = FlowEstimator(fx, fy, cx, cy, R_cam_imu, rotflow_sign,
                                  smooth_n=smooth_n, yaw_smooth_n=yaw_smooth_n)
        self._omega = np.zeros(3)
        self._lateral = self._yaw = self._conf = 0.0
        self._dt = 0.0
        self._seq = 0
        node.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        node.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)

    def _on_imu(self, m):
        self._omega = np.array([m.angular_velocity.x, m.angular_velocity.y,
                                m.angular_velocity.z])

    def _on_image(self, m):
        if m.encoding not in ('mono8', '8UC1'):
            return
        gray = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        res = self._est.process(gray, stamp, self._omega)
        if res is None:
            return
        self._lateral = res['lateral']
        self._yaw = res['yaw_flow']
        self._conf = res['conf']
        self._dt = res['dt']
        self._seq += 1               # НОВЫЙ кадр → домен продвинет PID

    def merge(self, s):
        """Влить свежие агрегаты потока в снапшот телеметрии (как pilot_* в ноде)."""
        s.flow_lateral = self._lateral
        s.flow_yaw = self._yaw
        s.flow_conf = self._conf
        s.flow_dt = self._dt
        s.flow_seq = self._seq
        return s
