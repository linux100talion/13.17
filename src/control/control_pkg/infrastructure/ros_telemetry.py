#!/usr/bin/env python3
"""RosTelemetry — адаптер порта Telemetry: подписки MAVROS/VINS/Gazebo → DroneState.

Наполняет единый снапшот из колбэков; snapshot() отдаёт его домену со свежим now_sim.
QoS для rel_alt и rc/in — SensorData (BEST_EFFORT): MAVROS публикует их так, дефолтная
RELIABLE-подписка их НЕ получает. Ground-truth скорость — конечная разность по sim-времени
с EMA (twist-фрейм одометрии неоднозначен, считаем сами). Скорость VINS — тоже
конечная разность, но по HEADER-ШТАМПАМ (VinsTrack): джиттер доставки в неё не
попадает. Всё остальное как в монолите.
"""
import math

from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String

from ..application.ripeness import VinsRipeness
from ..application.vins_track import VinsTrack
from ..domain.state import DroneState


class RosTelemetry:
    def __init__(self, node, clock, alt_src='global', vel_src='diff'):
        self._clock = clock
        self._s = DroneState()
        # ИСТОЧНИК СКОРОСТИ VINS для стека/гейта (config.vins_vel_src): 'diff' —
        # конечная разность позы по штампам + EMA (VinsTrack, как было); 'twist' —
        # скорость из самого сообщения одометрии (состояние эстиматора, преинтеграция
        # IMU). Проверено по bag 130326 (twist_check): рама twist = рама позы (МНК-
        # поворот к истине +3.2° против +2.7° у разности), масштаб 1.008, лаг к истине
        # по штампам 0.00 с против 0.14 у разности, шум на висении 0.008±0.005 м/с
        # против 0.025±0.049 — в 10 раз тише. Разность остаётся для детекта
        # перерождения (VinsTrack) и зрелости (residual). Лаг петли DpVins 0.35 →
        # ~0.11 с (приход) — запас по фазе под kp/ki (стенд dpvins_gust_stand).
        self._vel_src = vel_src
        self._ripe = VinsRipeness()   # детектор зрелости VINS (2-я ступень гейта)
        self._gt_px = self._gt_py = None
        self._gt_pt = None
        # скорость VINS по ШТАМПАМ + детект перерождения потока (vins_track.py):
        # dt по времени прихода раздувал скорость на догоняющей пачке после
        # стопора эстиматора → ложный «разнос» (lv2_joy_20260905_114248)
        self._track = VinsTrack()
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
        # состояние моста VINS→EKF (ray_tracer bridge_gate) — в статус brg=/brw=/brl=/brc=
        node.create_subscription(String, '/nn1/bridge', self._on_bridge, 10)
        node.create_subscription(RCIn, '/mavros/rc/in', self._on_rcin, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/model/iris_cam/odometry', self._on_gt, 10)
        # Пульс позиции EKF: local_position публикуется, ПОКА у EKF есть позиция
        # (после GPS-kill замолкает). Содержимое не нужно — только свежесть
        # (гейт WaitEkfPos перед армом, урок LV4). QoS sensor: совместим с любым.
        node.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._on_lpos, qos_profile_sensor_data)
        # Оценка ветра EKF3 (drag-фьюжн, WIND msg): скорость воздушной массы в
        # мире ENU → стрелка ветра HUD (первичный источник). Топик есть всегда
        # (плагин wind_estimation), данные — только при EK3_DRAG_BCOEF>0 и WIND
        # в стриме (nav_up). QoS sensor: mavros шлёт BEST_EFFORT.
        node.create_subscription(TwistWithCovarianceStamped,
                                 '/mavros/wind_estimation', self._on_wind,
                                 qos_profile_sensor_data)
        # Детектор посадки FCU (EXTENDED_SYS_STATE.landed_state) — для детекта
        # касания SoftLand в дополнение к баро/gt. Нужен стрим EXTENDED_STATUS
        # (nav_up.sh, stream_id 2); без него топик молчит → fcu_landed=-1, детект
        # живёт на баро/gt как раньше. Ленивый импорт: как State/RCIn, но
        # отдельным try — старым образам без сообщения не мешаем.
        try:
            from mavros_msgs.msg import ExtendedState
            node.create_subscription(ExtendedState, '/mavros/extended_state',
                                     self._on_ext, 10)
        except ImportError:
            pass

    def _on_ext(self, m):
        self._s.fcu_landed = int(m.landed_state)
        self._s.fcu_landed_sim = self._clock.now_sim()

    def _on_state(self, m):
        self._s.mode = m.mode
        self._s.armed = m.armed

    def _on_relalt(self, m):
        self._s.rel_alt = float(m.data)

    def _set_relalt(self, alt):
        self._s.rel_alt = float(alt)

    def _on_odom(self, m):
        t = self._clock.now_sim()
        # Время измерения — HEADER-ШТАМП (sim-время кадра), не now_sim прихода:
        # джиттер доставки раздувает конечную разность Δp/Δt (детектор зрелости —
        # прогон 052917: res=0.24 при офлайн-поле 0.05-0.10; скорость для гейта
        # здоровья — прогон 20260905_114248: стопор эстиматора 1.5 с + пачка →
        # 3.44 м/с при twist ≤1.47 → ложный демоут + /restart в полёте).
        th = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        if self._track.on_odom(th, x, y):
            # ПЕРЕРОЖДЕНИЕ потока (рестарт/переинициализация VINS сама по себе):
            # новая рама и новый масштаб — зрелость считается заново, счётчик и
            # время первой одометрии обнуляются (ярус 1 ждёт vins_min/ripe_sec)
            self._s.vins_rebirths += 1
            self._s.vins_odom_count = 0
        self._s.vins_odom_count += 1
        if self._s.vins_odom_count == 1:
            self._s.vins_first_sim = t    # старт потока — для гейта зрелости
        self._s.vins_last_sim = t         # свежесть — по ПРИХОДУ (молчащий VINS)
        # детектор зрелости (2-я ступень гейта): residual поза/скорость +
        # вертикальный ratio к rel_alt (баро при alt_src=baro, global на GPS).
        p, v = m.pose.pose.position, m.twist.twist.linear
        self._ripe.on_odom(th, (p.x, p.y, p.z), (v.x, v.y, v.z),
                           self._s.rel_alt)
        self._s.vins_res = self._ripe.res if self._ripe.res is not None else -1.0
        self._s.vins_ratio = (self._ripe.ratio
                              if self._ripe.ratio is not None else -1.0)
        self._s.vins_ripe_det = self._ripe.ready
        # Поза VINS + скорость конечной разностью по штампам (twist-фрейм
        # неоднозначен — как gt; EMA a=0.4 внутри VinsTrack).
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._vel_src == 'twist':
            self._s.vins_vx, self._s.vins_vy = float(v.x), float(v.y)
        else:
            self._s.vins_vx, self._s.vins_vy = self._track.vx, self._track.vy
        self._s.vins_x, self._s.vins_y, self._s.vins_yaw = x, y, yaw
        self._s.vins_valid = True

    def _on_bridge(self, m):
        # 'open|closed <причина> <подтяжек в окне> <закрытий> <перерождений>'
        w = m.data.split()
        if len(w) >= 5:
            self._s.bridge_open = (w[0] == 'open')
            self._s.bridge_why = w[1]
            self._s.bridge_relatch = int(w[2])
            self._s.bridge_closes = int(w[3])
            self._s.bridge_seen = True

    def reset_vins_stream(self) -> None:
        """Нода послала /restart VINS: поток объявлен оборванным ЗДЕСЬ И СЕЙЧАС,
        не дожидаясь протухания (2 с) — счётчик и первое время обнуляются, ярус 1
        падает на демпфер сразу и ждёт зрелость новой рамы. Скорость с нуля.
        Перерождение засчитывается, если поток был (на арме одометрии ещё нет)."""
        if self._s.vins_odom_count > 0:
            self._s.vins_rebirths += 1
        self._s.vins_odom_count = 0
        self._s.vins_first_sim = -1e9
        self._s.vins_vx = self._s.vins_vy = 0.0
        self._track.reset()

    def _on_rcin(self, m):
        if len(m.channels) >= 3:
            self._s.rcin_throttle = m.channels[2]

    def _on_lpos(self, m):
        self._s.ekf_pos_last_sim = self._clock.now_sim()
        self._s.ekf_z = float(m.pose.position.z)   # высота глазами EKF3 → HUD

    def _on_wind(self, m):
        # ветер EKF3 в мире ENU (скорость воздушной массы, куда дует); свежесть
        # по sim-часам (на земле FCU шлёт последний в-воздухе замер, гейтим по age)
        self._s.wind_ekf_wx = float(m.twist.twist.linear.x)
        self._s.wind_ekf_wy = float(m.twist.twist.linear.y)
        self._s.wind_ekf_sim = self._clock.now_sim()

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
