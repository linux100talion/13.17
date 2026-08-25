#!/usr/bin/env python3
"""RosTelemetry — адаптер порта Telemetry: подписки MAVROS/VINS/Gazebo → DroneState.

Наполняет единый снапшот из колбэков; snapshot() отдаёт его домену со свежим now_sim.
QoS для rel_alt и rc/in — SensorData (BEST_EFFORT): MAVROS публикует их так, дефолтная
RELIABLE-подписка их НЕ получает. Ground-truth скорость — конечная разность по sim-времени
с EMA (twist-фрейм одометрии неоднозначен, считаем сами). Всё как в монолите.
"""
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from ..application.ripeness import VinsRipeness
from ..domain.state import DroneState


class RosTelemetry:
    def __init__(self, node, clock, alt_src='global'):
        self._clock = clock
        self._s = DroneState()
        self._ripe = VinsRipeness()   # детектор зрелости VINS (2-я ступень гейта)
        self._gt_px = self._gt_py = None
        self._gt_pt = None
        self._vins_px = self._vins_py = None
        self._vins_pt = None
        node.create_subscription(State, '/mavros/state', self._on_state, 10)
        # Источник rel_alt: 'global' — GLOBAL_POSITION_INT (замерзает без GPS);
        # 'baro' — сырой барометр (GPS-denied / боевой борт). См. baro_alt.py.
        if alt_src == 'baro':
            from .baro_alt import BaroAlt
            self._baro = BaroAlt(node, self._set_relalt)
        else:
            node.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                     self._on_relalt, qos_profile_sensor_data)
        # /odometry — фактический топик форка VINS-MONO-ROS2 (в ROS2 нет приватного
        # пространства ноды; старый /vins_estimator/odometry не имел издателя).
        node.create_subscription(Odometry, '/odometry', self._on_odom, 10)
        node.create_subscription(RCIn, '/mavros/rc/in', self._on_rcin, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/model/iris_cam/odometry', self._on_gt, 10)
        # Пульс позиции EKF: local_position публикуется, ПОКА у EKF есть позиция
        # (после GPS-kill замолкает). Содержимое не нужно — только свежесть
        # (гейт WaitEkfPos перед армом, урок LV4). QoS sensor: совместим с любым.
        node.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._on_lpos, qos_profile_sensor_data)

    def _on_state(self, m):
        self._s.mode = m.mode
        self._s.armed = m.armed

    def _on_relalt(self, m):
        self._s.rel_alt = float(m.data)

    def _set_relalt(self, alt):
        self._s.rel_alt = float(alt)

    def _on_odom(self, m):
        self._s.vins_odom_count += 1
        t = self._clock.now_sim()
        if self._s.vins_odom_count == 1:
            self._s.vins_first_sim = t    # старт потока — для гейта зрелости
        self._s.vins_last_sim = t
        # детектор зрелости (2-я ступень гейта): residual поза/скорость +
        # вертикальный ratio к rel_alt (баро при alt_src=baro, global на GPS).
        # Время — HEADER-ШТАМП одометрии, не now_sim прихода: джиттер доставки
        # раздувает конечную разность Δp/Δt и residual врёт вверх (прогон
        # 052917: res=0.24 при офлайн-поле 0.05-0.10 — детектор молчал,
        # зрелость открыл таймер).
        th = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, v = m.pose.pose.position, m.twist.twist.linear
        self._ripe.on_odom(th, (p.x, p.y, p.z), (v.x, v.y, v.z),
                           self._s.rel_alt)
        self._s.vins_res = self._ripe.res if self._ripe.res is not None else -1.0
        self._s.vins_ratio = (self._ripe.ratio
                              if self._ripe.ratio is not None else -1.0)
        self._s.vins_ripe_det = self._ripe.ready
        # Поза VINS + скорость конечной разностью (twist-фрейм неоднозначен — как gt).
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._vins_pt is not None and t > self._vins_pt:
            dt = t - self._vins_pt
            a = 0.4
            self._s.vins_vx = (1.0 - a) * self._s.vins_vx + a * (x - self._vins_px) / dt
            self._s.vins_vy = (1.0 - a) * self._s.vins_vy + a * (y - self._vins_py) / dt
        self._vins_px, self._vins_py, self._vins_pt = x, y, t
        self._s.vins_x, self._s.vins_y, self._s.vins_yaw = x, y, yaw
        self._s.vins_valid = True

    def _on_rcin(self, m):
        if len(m.channels) >= 3:
            self._s.rcin_throttle = m.channels[2]

    def _on_lpos(self, m):
        self._s.ekf_pos_last_sim = self._clock.now_sim()
        self._s.ekf_z = float(m.pose.position.z)   # высота глазами EKF3 → HUD

    def _on_gt(self, m):
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        z = m.pose.pose.position.z
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
        self._s.gt_z = z
        self._s.gt_valid = True

    def snapshot(self) -> DroneState:
        self._s.now_sim = self._clock.now_sim()
        return self._s
