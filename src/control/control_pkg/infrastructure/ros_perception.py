#!/usr/bin/env python3
"""RosPerception — адаптер: камера (mono8) + гироскоп (FLU) → FlowEstimator → flow_* в DroneState.

Зрительная часть боевого пре-VINS демпфера. Подписывается на кадр и ω, гоняет
FlowEstimator (control_pkg.perception), результат отдаёт домену через merge(state):
кладёт flow_lateral/flow_yaw/flow_conf/flow_dt и ИНКРЕМЕНТИРУЕТ flow_seq на КАЖДЫЙ
обработанный кадр (домен интегрирует PID покадрово по seq, а не по 20-Гц тику).

Интринсики берутся из разрешения камеры (pinhole 90° hfov: fx=fy=W/2, cx=W/2, cy=H/2).
R_cam_imu и rotflow_sign — из sim.yaml/монолита (подтверждены flow_derotation_check).
"""
import math

import numpy as np

from ..perception.flow_estimator import FlowEstimator


class RosPerception:
    def __init__(self, node, cam_w, cam_h, R_cam_imu, rotflow_sign=1.0,
                 roll_smooth_n=1, pitch_smooth_n=1, yaw_smooth_n=5,
                 image_topic='/image_mono', imu_topic='/gz_imu/data_flu'):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, Imu
        from std_msgs.msg import Float64
        fx = fy = cam_w / 2.0          # pinhole 90° hfov
        cx, cy = cam_w / 2.0, cam_h / 2.0
        self._est = FlowEstimator(fx, fy, cx, cy, R_cam_imu, rotflow_sign,
                                  roll_smooth_n=roll_smooth_n, pitch_smooth_n=pitch_smooth_n,
                                  yaw_smooth_n=yaw_smooth_n)
        self._omega = np.zeros(3)
        self._pitch = 0.0
        self._alt = None
        self._lateral = self._longitudinal = self._yaw = self._conf = 0.0
        self._kf_dx = self._kf_dy = self._kf_logs = self._kf_rot = 0.0
        self._kf_vel = 0.0
        self._kf_n = self._kf_age = self._kf_reseeds = 0
        self._kf_segs = self._kf_rejects = 0
        self._kf_valid = False
        self._dt = 0.0
        self._seq = 0
        node.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        # баро-высота: опора действительна только пока высота не ушла (см. flow_estimator)
        node.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self._on_alt, qos_profile_sensor_data)
        node.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)

    def _on_imu(self, m):
        self._omega = np.array([m.angular_velocity.x, m.angular_velocity.y,
                                m.angular_velocity.z])
        # тангаж из ориентации того же сообщения — для компенсации наклона камеры
        # (на борту это оценка FCU; её дрейф 0.19°/с за секундное окно даёт 0.2°,
        # что мало против рабочих ±10°)
        q = m.orientation
        self._pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))

    def _on_alt(self, m):
        self._alt = float(m.data)

    def _on_image(self, m):
        if m.encoding not in ('mono8', '8UC1'):
            return
        gray = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        res = self._est.process(gray, stamp, self._omega, self._pitch, self._alt)
        if res is None:
            return
        self._lateral = res['lateral']
        self._longitudinal = res['longitudinal']   # looming → DpPitchHold
        self._yaw = res['yaw_flow']
        self._conf = res['conf']
        # ОПОРА: смещение картинки от опорного кадра (не скорость)
        self._kf_dx, self._kf_dy = res['kf_dx'], res['kf_dy']
        self._kf_logs, self._kf_rot = res['kf_logs'], res['kf_rot']
        self._kf_vel = res['kf_vel']
        self._kf_n, self._kf_age = res['kf_n'], res['kf_age']
        self._kf_reseeds, self._kf_valid = res['kf_reseeds'], res['kf_valid']
        self._kf_segs, self._kf_rejects = res['kf_segs'], res['kf_rejects']
        self._dt = res['dt']
        self._seq += 1               # НОВЫЙ кадр → домен продвинет PID

    def merge(self, s):
        """Влить свежие агрегаты потока в снапшот телеметрии (как pilot_* в ноде)."""
        s.flow_lateral = self._lateral
        s.flow_longitudinal = self._longitudinal
        s.flow_yaw = self._yaw
        s.flow_conf = self._conf
        s.flow_dt = self._dt
        s.flow_seq = self._seq
        s.kf_dx, s.kf_dy = self._kf_dx, self._kf_dy
        s.kf_logs, s.kf_rot = self._kf_logs, self._kf_rot
        s.kf_vel = self._kf_vel
        s.kf_n, s.kf_age = self._kf_n, self._kf_age
        s.kf_reseeds, s.kf_valid = self._kf_reseeds, self._kf_valid
        s.kf_segs, s.kf_rejects = self._kf_segs, self._kf_rejects
        return s

    def reset_keyframe(self):
        """Назначить точку удержания: следующий кадр станет опорным."""
        self._est.reset_keyframe()
