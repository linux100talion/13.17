#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FLOW-DAMP v0 — scale-free velocity-демпфер БОКОВОГО сноса по ОДНОЙ форвард-камере.

Спека: docker/sim/FLOW_DAMP_spec.md. Обоснование: docker/sim/FAQ_vins.md (6-15).
Ветка: nn2_c3_vins_althold_4.

Идея (путь B — прямой регулятор, НЕ через EKF3):
  /image_mono + /gz_imu/data_flu
    → sparse LK (поток между соседними кадрами)
    → derotate: вычесть ВРАЩАТЕЛЬНЫЙ поток (ω_cam = R · ω_imu) → остаётся ТРАНСЛЯЦИОННЫЙ
    → аффинный фит поля → БОКОВОЙ поток (средняя горизонт. компонента)
    → PID(боковой − стик_setpoint) → ROLL-offset PWM → /mavros/rc/override
  pitch / yaw / throttle — СКВОЗНЫЕ с /mavros/rc/in (пилот рулит вперёд/назад, курс, газ).
  confidence (число удержанных треков) → fade-out в чистый ALT_HOLD при плохом потоке.

v0 = ТОЛЬКО боковой (ROLL). Продольный (looming→PITCH) и визуальный yaw — ФАЗА 2
(хуки помечены TODO[phase2]). Масштаб НЕ нужен: ноль потока = ноль скорости.

ВНИМАНИЕ — НЕ оттюнено и НЕ провалидировано (скелет):
  * гейны PID, пороги confidence, k_stick — дефолты-заглушки, тюнятся прогоном;
  * ЗНАКИ: направление extrinsicRotation (cam↔imu) и знаки rot-flow требуют валидации
    (см. todo4: оракул estimate_extrinsic). Помечено TODO[sign].
Запуск (в контейнере nav, дрон уже в воздухе в ALT_HOLD, напр. через liftland --hold-only):
  ros2 run ... либо  python3 src/lab/flow_damp_node.py --ros-args -p kp:=...
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, Imu
from mavros_msgs.msg import OverrideRCIn, State, RCIn

try:
    import cv2
except ImportError:  # на всякий — нода без cv2 бессмысленна
    cv2 = None

RC_CENTER = 1500       # центр стика ArduCopter (как в alt_hold_bootstrap.py)
RC_MIN, RC_MAX = 1000, 2000
# Каналы 1..4 = roll/pitch/throttle/yaw. Индексы 0..3 в channels[].
CH_ROLL, CH_PITCH, CH_THR, CH_YAW = 0, 1, 2, 3
NOCHANGE = 65535       # OverrideRCIn: «не трогать канал»


class FlowEstimator:
    """Чистая зрительная часть: кадр+гироскоп → (боковой_поток, дивергенция, confidence).

    Изолирована от управления НАРОЧНО (см. спеку §8 «изоляция переменной»): её можно
    юнит-тестить/визуализировать отдельно, не трогая RC override.
    """

    def __init__(self, fx, fy, cx, cy, R_cam_imu, max_feats=200):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.R = np.asarray(R_cam_imu, dtype=np.float64).reshape(3, 3)
        self.max_feats = max_feats
        self.prev_gray = None
        self.prev_pts = None
        self.prev_stamp = None
        # параметры LK / детектора углов
        self._lk = dict(winSize=(21, 21), maxLevel=3,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        self._feat = dict(maxCorners=max_feats, qualityLevel=0.01, minDistance=8, blockSize=7)

    def _detect(self, gray):
        pts = cv2.goodFeaturesToTrack(gray, mask=None, **self._feat)
        return pts

    def process(self, gray, stamp, omega_imu):
        """omega_imu: угловая скорость в FLU (rad/s). Возвращает dict или None (нет данных)."""
        out = None
        if self.prev_gray is not None and self.prev_pts is not None and len(self.prev_pts) > 0:
            dt = max(1e-3, stamp - self.prev_stamp)
            nxt, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray,
                                                    self.prev_pts, None, **self._lk)
            st = st.reshape(-1).astype(bool)
            p0 = self.prev_pts.reshape(-1, 2)[st]
            p1 = nxt.reshape(-1, 2)[st]
            n = len(p0)
            if n >= 8:
                # измеренный поток в пикселях/кадр
                flow = (p1 - p0)
                # --- derotation -------------------------------------------------
                # ω в фрейме КАМЕРЫ: ω_cam = R · ω_imu   (R = extrinsicRotation)
                # TODO[sign]: направление R (cam→imu vs imu→cam) на валидацию (оракул).
                w = self.R @ np.asarray(omega_imu, dtype=np.float64)
                wx, wy, wz = w
                # нормированные координаты точек (p0)
                xn = (p0[:, 0] - self.cx) / self.fx
                yn = (p0[:, 1] - self.cy) / self.fy
                # вращательный поток (Longuet-Higgins/Prazdny), нормир. плоскость, ×dt:
                # TODO[sign]: знаки/перестановку проверить вместе с extrinsic.
                u_rot_n = (xn * yn * wx - (1.0 + xn ** 2) * wy + yn * wz) * dt
                v_rot_n = ((1.0 + yn ** 2) * wx - xn * yn * wy - xn * wz) * dt
                u_rot = self.fx * u_rot_n
                v_rot = self.fy * v_rot_n
                # остаточный ТРАНСЛЯЦИОННЫЙ поток (пиксели/кадр)
                tr = flow - np.column_stack([u_rot, v_rot])
                # --- агрегаты (v0: только боковой) ------------------------------
                # робастная средняя горизонт. компонента = прокси боковой скорости.
                lateral = float(np.median(tr[:, 0]))
                # TODO[phase2]: дивергенция (looming) из аффинного фита tr по (xn,yn):
                #   divergence ∝ ∂u/∂x + ∂v/∂y → прокси продольной скорости → PITCH.
                divergence = 0.0
                out = dict(lateral=lateral, divergence=divergence,
                           n=n, dt=dt, conf=float(n) / float(self.max_feats))
        # обновляем «prev» (переинициализируем фичи каждый кадр — дёшево для скелета)
        self.prev_gray = gray
        self.prev_pts = self._detect(gray)
        self.prev_stamp = stamp
        return out


class FlowDampNode(Node):
    def __init__(self):
        super().__init__('flow_damp_node')
        if cv2 is None:
            raise RuntimeError('cv2 не найден — нода потока не запустится')

        # --- параметры (дефолты из sim.yaml; тюнинг/валидация = TODO) -----------
        self.declare_parameter('fx', 640.0)
        self.declare_parameter('fy', 640.0)
        self.declare_parameter('cx', 640.0)
        self.declare_parameter('cy', 360.0)
        # extrinsicRotation из sim.yaml (R камера↔IMU). TODO[sign]: направление на оракул.
        self.declare_parameter('extrinsic_rotation',
                               [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708])
        # PID по боковому потоку (пиксели/кадр → PWM). Заглушки — тюнятся прогоном.
        self.declare_parameter('kp', 8.0)
        self.declare_parameter('ki', 2.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('imax', 120.0)        # анти-виндап (PWM)
        self.declare_parameter('max_offset', 150.0)  # максимум ROLL-offset от центра (PWM)
        self.declare_parameter('k_stick', 0.02)      # стик(PWM от центра) → желаемый поток (px/кадр)
        self.declare_parameter('conf_min', 0.05)     # ниже — fade-out начинается
        self.declare_parameter('conf_full', 0.20)    # выше — полный авторитет демпфера
        self.declare_parameter('image_topic', '/image_mono')
        self.declare_parameter('imu_topic', '/gz_imu/data_flu')

        g = lambda n: self.get_parameter(n).value
        self.kp, self.ki, self.kd = g('kp'), g('ki'), g('kd')
        self.imax, self.max_offset, self.k_stick = g('imax'), g('max_offset'), g('k_stick')
        self.conf_min, self.conf_full = g('conf_min'), g('conf_full')

        self.est = FlowEstimator(g('fx'), g('fy'), g('cx'), g('cy'), g('extrinsic_rotation'))

        # состояние
        self.omega = np.zeros(3)       # последняя угловая скорость IMU (FLU)
        self.rcin = None               # последний RCIn (для сквозного pitch/yaw/throttle/стик)
        self.mode = ''
        self.armed = False
        self._i = 0.0                  # интегратор PID
        self._prev_err = 0.0

        # QoS: картинка/IMU — best-effort sensor; override — reliable.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, g('image_topic'), self._on_image, sensor_qos)
        self.create_subscription(Imu, g('imu_topic'), self._on_imu, sensor_qos)
        self.create_subscription(RCIn, '/mavros/rc/in', self._on_rcin, 10)
        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.get_logger().info('FLOW-DAMP v0: боковой демпфер (ROLL). pitch/yaw/throttle сквозные.')

    # --- колбэки ------------------------------------------------------------
    def _on_imu(self, m: Imu):
        self.omega = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])

    def _on_rcin(self, m: RCIn):
        self.rcin = list(m.channels)

    def _on_state(self, m: State):
        self.mode = m.mode
        self.armed = m.armed

    def _on_image(self, m: Image):
        # Image(mono8) → numpy grayscale без cv_bridge (избегаем ABI-сюрпризов).
        if m.encoding not in ('mono8', '8UC1'):
            return  # v0 ждёт /image_mono (mono8)
        gray = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        res = self.est.process(gray, stamp, self.omega)
        if res is None:
            return
        self._control(res)

    # --- контур управления (v0: только ROLL) --------------------------------
    def _control(self, res):
        # стик roll (сквозной из RCIn) задаёт ЖЕЛАЕМЫЙ боковой поток (setpoint).
        roll_stick = (self.rcin[CH_ROLL] if self.rcin else RC_CENTER)
        desired = self.k_stick * (roll_stick - RC_CENTER)   # px/кадр
        err = res['lateral'] - desired                      # зануляем ОШИБКУ, не сам поток

        # PID
        self._i = float(np.clip(self._i + self.ki * err * res['dt'], -self.imax, self.imax))
        d = self.kd * (err - self._prev_err) / max(1e-3, res['dt'])
        self._prev_err = err
        u = self.kp * err + self._i + d                     # PWM-смещение по ROLL
        u = float(np.clip(u, -self.max_offset, self.max_offset))
        # TODO[sign]: итоговый знак ROLL-offset (вправо/влево) сверить прогоном.

        # confidence → fade-out: blend между демпфером и сквозным стиком пилота.
        conf = res['conf']
        blend = float(np.clip((conf - self.conf_min) / max(1e-6, self.conf_full - self.conf_min),
                              0.0, 1.0))
        roll_out = blend * (RC_CENTER + u) + (1.0 - blend) * roll_stick

        self._publish(int(round(roll_out)))

    def _publish(self, roll_pwm):
        msg = OverrideRCIn()
        ch = [NOCHANGE] * 18
        ch[CH_ROLL] = int(np.clip(roll_pwm, RC_MIN, RC_MAX))
        # СКВОЗНЫЕ каналы (v0): pitch/throttle/yaw — как с пульта (пилот рулит).
        if self.rcin:
            ch[CH_PITCH] = int(self.rcin[CH_PITCH])
            ch[CH_THR] = int(self.rcin[CH_THR])
            ch[CH_YAW] = int(self.rcin[CH_YAW])
        # TODO[phase2]: ch[CH_PITCH] = looming-зануление; ch[CH_YAW] = визуальный yaw-hold.
        msg.channels = ch
        self.rc_pub.publish(msg)


def main():
    rclpy.init()
    node = FlowDampNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
