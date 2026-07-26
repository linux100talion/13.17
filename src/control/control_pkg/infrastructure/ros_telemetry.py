#!/usr/bin/env python3
"""RosTelemetry — адаптер порта Telemetry: подписки MAVROS/VINS/Gazebo → DroneState.

Наполняет единый снапшот из колбэков; snapshot() отдаёт его домену со свежим now_sim.
QoS для rel_alt и rc/in — SensorData (BEST_EFFORT): MAVROS публикует их так, дефолтная
RELIABLE-подписка их НЕ получает. Ground-truth скорость — конечная разность по sim-времени
с EMA (twist-фрейм одометрии неоднозначен, считаем сами). Всё как в монолите.
"""
import math

from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from ..domain.state import DroneState


class RosTelemetry:
    def __init__(self, node, clock):
        self._clock = clock
        self._s = DroneState()
        self._gt_px = self._gt_py = None
        self._gt_pt = None
        node.create_subscription(State, '/mavros/state', self._on_state, 10)
        node.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self._on_relalt, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/vins_estimator/odometry', self._on_odom, 10)
        node.create_subscription(RCIn, '/mavros/rc/in', self._on_rcin, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/model/iris_cam/odometry', self._on_gt, 10)

    def _on_state(self, m):
        self._s.mode = m.mode
        self._s.armed = m.armed

    def _on_relalt(self, m):
        self._s.rel_alt = float(m.data)

    def _on_odom(self, m):
        self._s.vins_odom_count += 1
        self._s.vins_last_sim = self._clock.now_sim()

    def _on_rcin(self, m):
        if len(m.channels) >= 3:
            self._s.rcin_throttle = m.channels[2]

    def _on_gt(self, m):
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = self._clock.now_sim()
        if self._gt_pt is not None and t > self._gt_pt:
            dt = t - self._gt_pt
            a = 0.4   # EMA-сглаживание скорости
            self._s.gt_vx = (1.0 - a) * self._s.gt_vx + a * (x - self._gt_px) / dt
            self._s.gt_vy = (1.0 - a) * self._s.gt_vy + a * (y - self._gt_py) / dt
        self._gt_px, self._gt_py, self._gt_pt = x, y, t
        self._s.gt_x, self._s.gt_y, self._s.gt_yaw = x, y, yaw
        self._s.gt_valid = True

    def snapshot(self) -> DroneState:
        self._s.now_sim = self._clock.now_sim()
        return self._s
