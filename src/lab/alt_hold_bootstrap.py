#!/usr/bin/env python3
"""
ALT_HOLD bootstrap: взлёт без GPS и инициализация VINS в полёте.

Зачем: GUIDED — позиционный режим, без GPS / сошедшегося VINS он даже не латчится.
ALT_HOLD держит высоту по барометру и НЕ требует горизонтальной позиции, поэтому им
можно оторваться от земли и СОЗДАТЬ движение (climb + раскачка), нужное монокуляр-
ному VINS для инициализации (масштаб/гравитация ненаблюдаемы в покое). Контекст и
обоснование: src/nav/FAQ_gps.md, план — src/nav/todo.txt.

Особенность ALT_HOLD: высота управляется throttle-стиком (пружинный, ЦЕНТР = hold):
газ вверх → подъём, отпустил в центр → держит высоту. Авто-взлёта, как в GUIDED
(cmd/takeoff), тут нет → нужен НЕПРЕРЫВНЫЙ RC override (/mavros/rc/override, ~20 Гц).
Поэтому это НОДА, а не bash: она обязана публиковать override весь полёт в ALT_HOLD,
иначе FCU по таймауту вернётся к своему RC и дрон просядет. Завершить ALT_HOLD-фазу
можно только переходом в самоподдерживающийся режим, которому override не нужен:
GUIDED (--handover) либо LAND. «Просто зависнуть в ALT_HOLD и выйти» — небезопасно.

Конечный автомат (бюджеты — в SIM-времени по /clock, RTF-независимо):
  PREARM (ALT_HOLD, throttle=min) → ARM → CLIMB (throttle>центр до --alt)
    → EXCITE (throttle=центр + station-keeping forward/back +τ/−2τ/+τ + медленный
      yaw) — ждём сходимости VINS, дрон держится около точки старта (не уезжает)
    → [--handover] GUIDED (наблюдаем рывок)  |  иначе: OBSERVE → LAND (самодостаточно)

--handover ВЫКЛ по умолчанию: выравнивание кадра VINS→NED (yaw-коррекция в
ray_tracer) ещё не реализовано, поэтому переход в GUIDED даст рывок (расхождение
нуля VINS и датума). Сначала смотрим саму инициализацию (climb→init→land без рывка),
handover — отдельным прогоном.

Запуск внутри nav:  python3 /lab/alt_hold_bootstrap.py [--alt 3] [--handover] ...
Через секвенсор:    src/lab/bootstrap.sh  (команда `bootstrap` в capture_scene.sh).
"""
import argparse
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import OverrideRCIn, State, RCIn
from mavros_msgs.srv import SetMode, CommandBool

# --- RC (PWM) ---------------------------------------------------------------
RC_CENTER = 1500       # центр стика; для throttle в ALT_HOLD центр = «держать высоту»
RC_MIN_THR = 1000      # газ в минимум — нужно для арминга в ALT_HOLD
# Каналы 1..4 = roll/pitch/throttle/yaw (ArduCopter). 5..18 — не трогаем:
# 65535 (UINT16_MAX) = «ИГНОРИРОВАТЬ канал», не отдаём его радио и не оверрайдим.
# За удержание РЕЖИМА отвечает не этот канал, а SITL-параметр FLTMODE_CH 0
# (sitl-extra.parm): он убирает RC-переключатель режима, чтобы радио не перебивало
# set_mode("ALT_HOLD"). Подробности — там и в hold_alt_hold() ниже.
RC_NOCHANGE = 65535

# FLOW-DAMP (--flow-hold): интринсики + extrinsicRotation камеры из sim.yaml.
# Матрица R и rotflow_sign=+1 ПОДТВЕРЖДЕНЫ flow_derotation_check (Шаг 1: остаток
# 0.55× baseline на 1189 чистых вращательных кадрах gz-hold+yaw, неверный знак ×2.24).
FLOW_FX = FLOW_FY = 640.0
FLOW_CX, FLOW_CY = 640.0, 360.0
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]

# Состояния автомата
S_PREARM, S_ARM, S_CLIMB, S_EXCITE, S_HANDOVER, S_OBSERVE, S_LAND, S_DONE = range(8)
S_NAME = {S_PREARM: "PREARM", S_ARM: "ARM", S_CLIMB: "CLIMB", S_EXCITE: "EXCITE",
          S_HANDOVER: "HANDOVER", S_OBSERVE: "OBSERVE", S_LAND: "LAND", S_DONE: "DONE"}


class AltHoldBootstrap(Node):
    def __init__(self, a):
        super().__init__('alt_hold_bootstrap')
        # use_sim_time: все бюджеты/таймеры считаем по /clock (sim-время). На низком
        # RTF (CPU-бокс ~0.07) wall-секунды = мизер sim-времени — climb/excite не
        # успели бы. Как в fly_square.py / arm.sh (бюджет в sim-сек).
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.a = a

        # выходы
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        # debug для system-ID (flow_calib.py): sim-штампованный roll_off/flow/conf.
        # Vector3Stamped, т.к. OverrideRCIn без header → под низким RTF не привязать к sim-времени.
        self.dbg_pub = self.create_publisher(Vector3Stamped, '/flow_dbg', 10)
        # входы
        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        # rel_alt и rc/in MAVROS публикует с SensorData QoS (BEST_EFFORT) — дефолтная
        # RELIABLE-подписка их НЕ получает («incompatible QoS, no messages», 1-й
        # прогон: rel_alt=None → нода летела вслепую, не выходила из CLIMB).
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self._on_relalt, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/vins_estimator/odometry', self._on_odom, 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self._on_rcin, qos_profile_sensor_data)
        # Ground-truth одометрия из Gazebo (СИМ-костыль для gz-position-hold):
        # истинная поза+скорость тела в world. Мостится ros_gz_bridge из
        # /model/iris_cam/odometry. На боевом Orin её НЕТ (там референс — VINS).
        self.create_subscription(Odometry, '/model/iris_cam/odometry', self._on_gt_odom, 10)
        # сервисы
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        # состояние
        self.mode = None
        self.armed = False
        self.rel_alt = None
        self.odom_count = 0
        self.last_odom_sim = -1e9
        self.rcin_thr = None
        self.rcin_logged = False

        # gz-position-hold: истинная поза/скорость (world) + сетпойнт удержания
        self.gt_have = False
        self.gt_x = self.gt_y = 0.0
        self.gt_yaw = 0.0
        self.gt_vx = self.gt_vy = 0.0      # world-скорость (конечная разность)
        self.gt_px = self.gt_py = None     # пред. позиция для разности
        self.gt_pt = None                  # пред. sim-время
        self.hold_sp = None                # (x0, y0) сетпойнт, фиксируется на входе в hold
        self.gz_ix = self.gz_iy = 0.0      # интеграл ошибки позиции (world) — I-член
        self.gz_it = None                  # пред. sim-время для dt интеграла
        self._gz_log_t = -1e9              # троттл отладочного лога gz-hold

        # --- FLOW-DAMP (--flow-hold): боковой демпфер по камере (вариант a спеки) --
        # Зрение+PID в _on_flow_image (раз на кадр), контроль в tick (S_EXCITE/flow),
        # публикация — общий _wall_publish 20 Гц. Реюз hold-only-каркаса.
        self.flow_omega = np.zeros(3)      # последняя ω IMU (FLU) для derotation
        self.flow_roll_off = 0.0           # последний выход PID (PWM-смещение ROLL)
        self.flow_conf = 0.0               # последняя confidence (для /flow_dbg)
        self.flow_last_sim = -1e9          # sim-время свежего кадра (fade на сталле)
        self._flow_i = 0.0                 # интегратор PID
        self._flow_prev_err = 0.0
        self.est = None
        if a.flow_hold:
            from sensor_msgs.msg import Image, Imu
            from flow_estimator import FlowEstimator
            self.est = FlowEstimator(FLOW_FX, FLOW_FY, FLOW_CX, FLOW_CY,
                                     FLOW_R, a.flow_rsign, smooth_n=a.flow_smooth)
            self.create_subscription(Image, a.flow_image_topic, self._on_flow_image,
                                     qos_profile_sensor_data)
            self.create_subscription(Imu, a.flow_imu_topic, self._on_flow_imu,
                                     qos_profile_sensor_data)
            self.get_logger().info(
                f"FLOW-DAMP: демпфер ROLL по {a.flow_image_topic}+{a.flow_imu_topic} "
                f"(kp={a.flow_kp} ki={a.flow_ki} rsign={a.flow_rsign:+.0f} "
                f"osign={a.flow_osign:+.0f})")

        self.state = S_PREARM
        self.state_t0 = None          # базируется лениво при первом тике с живым /clock
        self.last_cmd = -1e9          # троттлинг вызовов сервисов (раз в ~1 sim-сек)
        self.last_mode_assert = -1e9  # отдельный троттл для ре-ассерта ALT_HOLD
        self.result = "?"
        self.finished = False

        # текущая команда RC (обновляется в каждом состоянии)
        self.roll = RC_CENTER
        self.pitch = RC_CENTER
        self.throttle = RC_MIN_THR
        self.yaw = RC_CENTER

        # Автомат — на sim-времени (ROSClock/use_sim_time): бюджеты фаз RTF-
        # независимы (как arm.sh/fly_square). 20 Гц sim ≈ 1.4 Гц wall при RTF 0.07.
        self.timer = self.create_timer(0.05, self.tick)   # автомат + publish (sim-время)
        # override публикуется в tick() (sim-таймер) — точки СМЕНЫ значения на sim-
        # времени → детерминированный контур (раньше publish был только из wall-цикла
        # main → смена в случайных sim-точках → недетерминированный демпфер). Wall-цикл
        # main ДОПОЛНИТЕЛЬНО ре-публикует то же значение для свежести override на FCU
        # (на низком RTF sim-publish ~1.4 Гц wall сам по себе мог бы протухнуть).
        self.get_logger().info(
            f"alt_hold_bootstrap: alt={a.alt}м handover={a.handover} "
            f"excite=±{a.excite}PWM/{a.excite_period}s vins_timeout={a.vins_timeout}s (sim)")

    # --- утилиты -------------------------------------------------------------
    def now_sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self):
        # Ленивое базирование: первый тик с живым /clock задаёт точку отсчёта
        # состояния. Иначе now_sim()=0 до прихода /clock, а затем скачок к реальному
        # sim-времени дал бы ложное превышение бюджета (мгновенный аборт).
        if self.state_t0 is None:
            self.state_t0 = self.now_sim()
            return 0.0
        return self.now_sim() - self.state_t0

    def goto(self, st):
        self.get_logger().info(f">>> {S_NAME[self.state]} → {S_NAME[st]}")
        self.state = st
        self.state_t0 = None          # ре-базируется на следующем тике
        self.last_cmd = -1e9

    def vins_converged(self):
        # VINS публикует /odometry только ПОСЛЕ инициализации (solver NON_LINEAR),
        # поэтому устойчивый поток = сходимость. Требуем N сообщений + свежесть.
        return (self.odom_count >= self.a.vins_min and
                (self.now_sim() - self.last_odom_sim) < 2.0)

    def _try(self, fn):  # троттлим вызов сервиса до ~1 раза в sim-секунду
        if self.now_sim() - self.last_cmd >= 1.0:
            self.last_cmd = self.now_sim()
            fn()

    def set_mode(self, mode):
        if self.mode_cli.service_is_ready():
            req = SetMode.Request(); req.custom_mode = mode
            self.mode_cli.call_async(req)

    def hold_alt_hold(self):
        # Защитный ре-ассерт: на фазах ARM/CLIMB/EXCITE режим обязан быть ALT_HOLD.
        # Основной механизм удержания — FLTMODE_CH 0 в SITL (радио не трогает режим);
        # это страховка на случай смены режима полётником (EKF/failsafe). Отдельный
        # троттл (~раз в 2 sim-сек), чтобы не конфликтовать с throttle вызовов arm().
        # '' — транзиент /mavros/state до первого валидного heartbeat (не режим).
        if self.mode not in (None, "", "ALT_HOLD") and \
                self.now_sim() - self.last_mode_assert >= 2.0:
            self.last_mode_assert = self.now_sim()
            self.get_logger().warn(f"режим={self.mode} ≠ ALT_HOLD — ре-ассерт")
            self.set_mode("ALT_HOLD")

    def arm(self):
        if self.arm_cli.service_is_ready():
            req = CommandBool.Request(); req.value = True
            self.arm_cli.call_async(req)

    # --- входы ---------------------------------------------------------------
    def _on_state(self, m): self.mode = m.mode; self.armed = m.armed
    def _on_relalt(self, m): self.rel_alt = float(m.data)
    def _on_odom(self, m): self.odom_count += 1; self.last_odom_sim = self.now_sim()

    def _on_rcin(self, m):
        if len(m.channels) >= 3:
            self.rcin_thr = m.channels[2]

    def _on_gt_odom(self, m):
        # истинная поза Gazebo (world) + world-скорость через конечную разность по
        # sim-времени (twist-фрейм одометрии неоднозначен — считаем сами, надёжнее).
        x = m.pose.pose.position.x; y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.gt_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        t = self.now_sim()
        if self.gt_pt is not None and t > self.gt_pt:
            dt = t - self.gt_pt; a = 0.4   # EMA-сглаживание скорости
            self.gt_vx = (1.0-a)*self.gt_vx + a*(x - self.gt_px)/dt
            self.gt_vy = (1.0-a)*self.gt_vy + a*(y - self.gt_py)/dt
        self.gt_px, self.gt_py, self.gt_pt = x, y, t
        self.gt_x, self.gt_y = x, y
        self.gt_have = True

    # --- FLOW-DAMP колбэки (только при --flow-hold) -------------------------
    def _on_flow_imu(self, m):
        self.flow_omega = np.array([m.angular_velocity.x, m.angular_velocity.y,
                                    m.angular_velocity.z])

    def _on_flow_image(self, m):
        # mono8 → grayscale без cv_bridge; LK+derotation в FlowEstimator; PID ЗДЕСЬ
        # (раз на свежий кадр, dt=res['dt'] — настоящий интервал кадра, без двойного
        # интегрирования в 20-Гц tick). tick лишь переносит flow_roll_off в self.roll.
        if self.est is None or m.encoding not in ('mono8', '8UC1'):
            return
        gray = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        res = self.est.process(gray, stamp, self.flow_omega)
        if res is None:
            return
        a = self.a
        # стик в sim не подаём → desired=0 (центр): гасим снос (дрейф/ветер).
        err = res['lateral'] - 0.0
        self._flow_i = float(np.clip(self._flow_i + a.flow_ki * err * res['dt'],
                                     -a.flow_imax, a.flow_imax))
        d = a.flow_kd * (err - self._flow_prev_err) / max(1e-3, res['dt'])
        self._flow_prev_err = err
        u = float(np.clip(a.flow_kp * err + self._flow_i + d, -a.flow_max, a.flow_max))
        # confidence (число треков) → плавный fade-out: мало фич → демпфер к нулю.
        conf = res['conf']
        blend = float(np.clip((conf - a.flow_conf_min) /
                              max(1e-6, a.flow_conf_full - a.flow_conf_min), 0.0, 1.0))
        # flow_osign — направление ROLL-торможения (TODO: тюнить Шагом 3, как gz_rsign).
        self.flow_roll_off = a.flow_osign * blend * u
        self.flow_conf = conf
        self.flow_last_sim = self.now_sim()

    def _roll_excite_cmd(self):
        """system-ID: заданный roll_off (PWM-offset от центра). Линейный чирп f0→f1 за
        chirp сек (богатый спектр → s/τ/частотная характеристика), затем квадрат ±amp с
        полупериодом step (чистый DC → идентификация k). roll_off экзогенный."""
        a = self.a
        t = self.elapsed()
        if t <= a.roll_excite_chirp:
            T = max(1e-3, a.roll_excite_chirp)
            phase = 2.0 * math.pi * (a.roll_excite_f0 * t +
                                     0.5 * (a.roll_excite_f1 - a.roll_excite_f0) * t * t / T)
            return a.roll_excite_amp * math.sin(phase)
        n = int((t - a.roll_excite_chirp) / max(1e-3, a.roll_excite_step))
        return a.roll_excite_amp * (1.0 if n % 2 == 0 else -1.0)

    # --- публикация RC override (WALL-таймер 20 Гц) -------------------------
    def _wall_publish(self):
        # Гейт публикации: в DONE и в GUIDED (после handover) override НЕ нужен —
        # GUIDED держит позицию сам. До этого — держим override свежим на wall-частоте.
        if self.state == S_DONE or (self.state == S_HANDOVER and self.mode == "GUIDED"):
            return
        msg = OverrideRCIn()
        ch = [RC_NOCHANGE] * 18
        ch[0] = int(self.roll)
        ch[1] = int(self.pitch)
        ch[2] = int(self.throttle)
        ch[3] = int(self.yaw)
        msg.channels = ch
        self.rc_pub.publish(msg)

    # --- автомат -------------------------------------------------------------
    def tick(self):
        # ДЕТЕРМИНИЗМ override: сначала логика автомата (sim-таймер 20 Гц обновляет
        # self.roll/pitch/...), затем СРАЗУ публикуем на ЭТОМ sim-тике. self.roll
        # меняется ТОЛЬКО здесь → точки смены привязаны к sim-времени, а не к wall-
        # clock (раньше publish шёл только из wall-цикла → FCU видел смену в случайных
        # sim-точках run-to-run → недетерминированный боковой контур демпфера). Wall-
        # цикл в main оставлен для СВЕЖЕСТИ: между тиками re-шлёт то же значение (новых
        # точек смены не вносит, значение между тиками константно).
        self._tick_logic()
        self._wall_publish()
        # sim-штампованный debug для калибровки (flow_calib.py): фактический roll_off,
        # текущий flow (0 если демпфер выкл), confidence. Пишется в bag → system-ID.
        d = Vector3Stamped()
        d.header.stamp = self.get_clock().now().to_msg()
        d.vector.x = float(self.roll - RC_CENTER)
        d.vector.y = float(getattr(self, 'flow_roll_off', 0.0))
        d.vector.z = float(getattr(self, 'flow_conf', 0.0))
        self.dbg_pub.publish(d)

    def _tick_logic(self):
        st = self.state

        if st == S_PREARM:
            self.roll = self.pitch = self.yaw = RC_CENTER
            self.throttle = RC_MIN_THR            # газ в минимум для арминга
            self._try(lambda: self.set_mode("ALT_HOLD"))
            if self.mode == "ALT_HOLD":
                self.goto(S_ARM)
            elif self.elapsed() > self.a.mode_budget:
                self.get_logger().warn(f"⚠️ ALT_HOLD не залатчился (mode={self.mode}) — пробуем дальше")
                self.goto(S_ARM)

        elif st == S_ARM:
            self.throttle = RC_MIN_THR
            self.hold_alt_hold()
            self._try(self.arm)
            if self.armed:
                self.goto(S_CLIMB)
            elif self.elapsed() > self.a.arm_budget:
                self.get_logger().error(f"⚠️ арм не прошёл (armed={self.armed}) — аборт")
                self.result = "ARM_FAIL"; self.goto(S_DONE)

        elif st == S_CLIMB:
            self.throttle = self.a.throttle_climb     # газ вверх → подъём
            self.hold_alt_hold()
            if not self.rcin_logged and self.rcin_thr is not None and self.elapsed() > 2:
                self.get_logger().info(f"    rc/in throttle={self.rcin_thr} (override проходит, если ≈{self.a.throttle_climb})")
                self.rcin_logged = True
            if self.rel_alt is not None and self.rel_alt >= self.a.alt:
                self.get_logger().info(f"    набрали {self.rel_alt:.1f}м (цель {self.a.alt}м)")
                self.goto(S_EXCITE)
            elif self.elapsed() > self.a.climb_budget:
                if self.rel_alt is not None and self.rel_alt >= 0.5:
                    self.get_logger().warn(f"⚠️ climb-бюджет вышел, высота {self.rel_alt:.1f}м — раскачиваем как есть")
                    self.goto(S_EXCITE)
                else:
                    self.get_logger().error(f"⚠️ не взлетели (rel_alt={self.rel_alt}) — RC override не принят? аборт→LAND")
                    self.result = "CLIMB_FAIL"; self.goto(S_LAND)

        elif st == S_EXCITE:
            # hold-only (liftland): диагностика дрейфа ALT_HOLD — БЕЗ раскачки.
            # Держим уровень (все стики в центре) hold_sec sim-сек и садимся. Если
            # дрон при этом уезжает за край — причина в AHRS-наклоне/остаточной
            # скорости, а не в excite (изоляция). Реюзает arm/climb/land/override.
            if self.a.hold_only:
                self.throttle = self.a.throttle_hold
                self.hold_alt_hold()
                self.yaw = RC_CENTER
                if self.a.gz_hold and self.gt_have:
                    # PD position-hold по ИСТИННОЙ позе Gazebo: держим сетпойнт
                    # (позиция на входе в hold). Ошибку+скорость из world переводим
                    # в тело (по yaw) → offset PWM по pitch(вперёд)/roll(вправо).
                    if self.hold_sp is None:
                        self.hold_sp = (self.gt_x, self.gt_y)
                        self.gz_ix = self.gz_iy = 0.0; self.gz_it = self.now_sim()
                        self.get_logger().info(
                            f"    gz-hold: центр=({self.gt_x:.2f},{self.gt_y:.2f}) "
                            f"kp={self.a.gz_kp} kd={self.a.gz_kd} ki={self.a.gz_ki} "
                            f"traj_r={self.a.gz_traj_r} traj_t={self.a.gz_traj_t}")
                    # Траекторный сетпойнт для параллакса VINS: водим точку удержания
                    # по кругу R вокруг центра (PID следует за ней → контролируемое
                    # движение, не убегает). Спираль из центра (offset (cosθ-1, sinθ)
                    # → старт в центре без рывка) + плавный разгон радиуса за 1 период.
                    # traj_r=0 → чистый холд (фикс. сетпойнт).
                    spx, spy = self.hold_sp
                    if self.a.gz_traj_r > 0.0:
                        tt = self.elapsed()
                        th = 2.0*math.pi*tt / self.a.gz_traj_t
                        reff = self.a.gz_traj_r * min(1.0, tt / self.a.gz_traj_t)
                        spx += reff * (math.cos(th) - 1.0)
                        spy += reff * math.sin(th)
                    ex = self.gt_x - spx
                    ey = self.gt_y - spy
                    # I-член: интегрируем ошибку в WORLD (yaw-инвариантно), потом
                    # поворачиваем в тело. Anti-windup: клампим состояние так, чтобы
                    # вклад Ki*i не превышал gz_imax PWM по каждой оси.
                    now = self.now_sim()
                    if self.a.gz_ki > 0 and self.gz_it is not None and now > self.gz_it:
                        dt = now - self.gz_it
                        self.gz_ix += ex*dt; self.gz_iy += ey*dt
                        cap = self.a.gz_imax / self.a.gz_ki
                        self.gz_ix = max(-cap, min(cap, self.gz_ix))
                        self.gz_iy = max(-cap, min(cap, self.gz_iy))
                    self.gz_it = now
                    c = math.cos(self.gt_yaw); s = math.sin(self.gt_yaw)
                    e_fwd =  ex*c + ey*s;            e_rgt = -ex*s + ey*c
                    v_fwd =  self.gt_vx*c + self.gt_vy*s
                    v_rgt = -self.gt_vx*s + self.gt_vy*c
                    i_fwd =  self.gz_ix*c + self.gz_iy*s
                    i_rgt = -self.gz_ix*s + self.gz_iy*c
                    mx = self.a.gz_max
                    po = self.a.gz_psign * (self.a.gz_kp*e_fwd + self.a.gz_kd*v_fwd + self.a.gz_ki*i_fwd)
                    ro = self.a.gz_rsign * (self.a.gz_kp*e_rgt + self.a.gz_kd*v_rgt + self.a.gz_ki*i_rgt)
                    po = max(-mx, min(mx, po)); ro = max(-mx, min(mx, ro))
                    self.pitch = RC_CENTER + int(po)
                    # ГИБРИД (gz+flow): если flow_hold тоже включён — продольную (pitch)
                    # держит gz-истина (дрон НЕ уезжает за сцену), а боковую (roll)
                    # отдаём флоу-демпферу (его и тюним, изолированно). Только gz → roll
                    # тоже от gz (полный position-hold, как раньше).
                    if self.a.roll_excite:
                        # system-ID: заданный чирп/ступени на roll (демпфер выкл),
                        # pitch держит gz. roll_off экзогенный → чистая калибровка.
                        self.roll = RC_CENTER + int(self._roll_excite_cmd())
                    elif self.a.flow_hold:
                        fresh = (self.now_sim() - self.flow_last_sim) < 0.5
                        self.roll = RC_CENTER + (int(self.flow_roll_off) if fresh else 0)
                    else:
                        self.roll = RC_CENTER + int(ro)
                    # Наложенный yaw (derotation-тест): позиция держится PID'ом, а
                    # курс качаем ± с периодом gz_yaw_period → много чистых
                    # вращательных кадров без трансляции. 0 → штатный холд без yaw.
                    if self.a.gz_yaw > 0.0:
                        y_sign = 1 if (int(self.elapsed() / self.a.gz_yaw_period) % 2 == 0) else -1
                        self.yaw = RC_CENTER + int(y_sign * self.a.gz_yaw)
                    # отладка: что контроллер видит и командует (раз в ~2 sim-сек).
                    # Связь e→коррекция→отклик однозначно ловит знак/фрейм.
                    if self.now_sim() - self._gz_log_t >= 2.0:
                        self._gz_log_t = self.now_sim()
                        self.get_logger().info(
                            f"gz: yaw={math.degrees(self.gt_yaw):+.0f} sp=({spx-self.hold_sp[0]:+.1f},{spy-self.hold_sp[1]:+.1f}) "
                            f"e=({ex:+.1f},{ey:+.1f}) v=({self.gt_vx:+.2f},{self.gt_vy:+.2f}) "
                            f"i=({i_fwd:+.1f},{i_rgt:+.1f}) pitch_off={int(po):+d} "
                            f"roll_off={int(self.roll - RC_CENTER):+d}"
                            f"{'(excite)' if self.a.roll_excite else '(flow)' if self.a.flow_hold else ''}")
                elif self.a.flow_hold:
                    # боковой демпфер: ROLL ← поток (PID в _on_flow_image), pitch/yaw
                    # центр (продольный/курс = фаза 2/пилот). Сталл кадров → центр
                    # (fade в чистый ALT_HOLD); override продолжает идти wall-таймером.
                    self.pitch = RC_CENTER
                    fresh = (self.now_sim() - self.flow_last_sim) < 0.5
                    self.roll = RC_CENTER + (int(self.flow_roll_off) if fresh else 0)
                    if self.now_sim() - self._gz_log_t >= 2.0:
                        self._gz_log_t = self.now_sim()
                        self.get_logger().info(
                            f"flow: roll_off={int(self.flow_roll_off):+d} "
                            f"fresh={fresh} (e_world=({self.gt_x:+.1f},{self.gt_y:+.1f}) "
                            f"если есть gt)")
                else:
                    self.roll = self.pitch = RC_CENTER   # gz нет → просто уровень
                if self.elapsed() > self.a.hold_sec:
                    self.get_logger().info(f"    hold-only {self.a.hold_sec}s истекли — садимся")
                    self.result = "HOLD_DONE"; self.goto(S_LAND)
                return
            # throttle=центр (держим высоту) + station-keeping раскачка для
            # параллакса/IMU excitation. Дрон ДОЛЖЕН остаться в круге R~peak около
            # старта, а не улетать за край сцены в «жёлтый экран».
            #
            # Подвох ALT_HOLD: стик pitch = угол наклона = УСКОРЕНИЕ (двойной
            # интегратор), поэтому симметричный «вперёд τ / назад τ» НЕ возвращает
            # позицию — за цикл уносит ~v·τ. Используем профиль ускорения +τ/−2τ/+τ
            # (translate, длительность 4τ): скорость 0→+→−→0 и позиция ВОЗВРАЩАЕТСЯ
            # в исходную к концу цикла (peak ~ a·τ², масштаб через excite).
            #
            # ⚠️ Компенсация работает ТОЛЬКО при постоянном курсе: импульсы
            # вперёд/назад гасятся лишь когда смотрят в одну сторону в МИРОВОЙ
            # системе. Поэтому yaw НЕЛЬЗЯ лить непрерывно во время translate (так
            # было раньше → курс проворачивался внутри цикла → проекция импульсов в
            # мир не обнулялась → дрон линейно уезжал). Yaw даём ОТДЕЛЬНЫМ импульсом
            # в точке возврата (v≈0, x≈0) МЕЖДУ translate-циклами: курс меняется
            # ступенькой, а сам translate идёт при фиксированном курсе → station-
            # keeping держится. Знак yaw чередуем каждый цикл → «подметаем» сцену ±
            # вокруг старта (translate-ось разворачивается → параллакс на 2D-диске).
            self.throttle = self.a.throttle_hold
            self.hold_alt_hold()
            self.roll = RC_CENTER
            amp = self.a.excite
            T = self.a.excite_period
            yaw_amp = self.a.yaw_rate
            yaw_dur = self.a.yaw_dur if yaw_amp > 0 else 0.0   # 0 yaw → чистый translate
            cycle = 4.0 * T + yaw_dur
            t = self.elapsed()
            local = t % cycle
            n = int(t / cycle)
            if local < yaw_dur:
                # YAW-импульс в точке возврата: поворот курса БЕЗ трансляции
                self.pitch = RC_CENTER
                y_sign = 1 if (n % 2 == 0) else -1
                self.yaw = RC_CENTER + y_sign * yaw_amp
            else:
                # TRANSLATE при ФИКСИРОВАННОМ курсе: +τ/−2τ/+τ → импульсы в мире
                # компенсируются, позиция возвращается к старту в конце цикла
                tt = local - yaw_dur
                p_sign = -1.0 if (T <= tt < 3.0 * T) else 1.0
                self.pitch = RC_CENTER + int(p_sign * amp)
                self.yaw = RC_CENTER
            if self.vins_converged():
                self.get_logger().info(f"    ✅ VINS сошёлся ({self.odom_count} odom-сообщений)")
                self.result = "VINS_OK"
                self.goto(S_HANDOVER if self.a.handover else S_OBSERVE)
            elif self.elapsed() > self.a.vins_timeout:
                self.get_logger().warn(f"⚠️ VINS не сошёлся за {self.a.vins_timeout}s "
                                       f"({self.odom_count} odom) — садимся")
                self.result = "VINS_TIMEOUT"; self.goto(S_LAND)

        elif st == S_HANDOVER:
            # Переход в GUIDED: он самоудерживает позицию, override больше не нужен.
            # ⚠️ Здесь и проявится рывок (кадр VINS не выровнен к NED) — это и смотрим.
            self.roll = self.pitch = self.yaw = RC_CENTER
            self.throttle = self.a.throttle_hold
            self._try(lambda: self.set_mode("GUIDED"))
            if self.mode == "GUIDED":
                self.get_logger().info("    в GUIDED — дрон удерживает позицию по ExternalNav (наблюдаем рывок)")
                self.result += "+GUIDED"; self.goto(S_DONE)
            elif self.elapsed() > self.a.mode_budget:
                self.get_logger().warn("⚠️ GUIDED не залатчился — садимся")
                self.goto(S_LAND)

        elif st == S_OBSERVE:
            # Чистое наблюдение init: держим высоту (центр), без раскачки, --observe
            # sim-секунд, затем садимся. Самодостаточный безопасный манёвр.
            self.roll = self.pitch = self.yaw = RC_CENTER
            self.throttle = self.a.throttle_hold
            if self.elapsed() > self.a.observe:
                self.goto(S_LAND)

        elif st == S_LAND:
            # LAND сам снижает и игнорирует throttle → override можно отпустить.
            self.roll = self.pitch = self.yaw = RC_CENTER
            self.throttle = self.a.throttle_hold
            self._try(lambda: self.set_mode("LAND"))
            touched = (self.rel_alt is not None and self.rel_alt <= self.a.ground_z)
            if touched or (self.mode == "LAND" and not self.armed and self.elapsed() > 3):
                self.get_logger().info(f"    касание (rel_alt={self.rel_alt}, armed={self.armed})")
                self.goto(S_DONE)
            elif self.elapsed() > self.a.land_budget:
                self.get_logger().warn(f"⚠️ касание не подтверждено (rel_alt={self.rel_alt}) — выходим")
                self.goto(S_DONE)

        elif st == S_DONE:
            self.get_logger().info(f">>> ИТОГ: {self.result} (mode={self.mode}, armed={self.armed}, "
                                   f"rel_alt={self.rel_alt}, odom={self.odom_count})")
            self.finished = True
            return
        # override публикуется в обёртке tick() сразу после этой логики (sim-тик,
        # детерминированные точки смены) + доп. wall-re-публикация в main для свежести.


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--alt', type=float, default=3.0, help='целевая высота climb, м (default 3)')
    p.add_argument('--handover', action='store_true',
                   help='после сходимости VINS перейти в GUIDED (увидеть рывок); по умолчанию OFF → OBSERVE+LAND')
    p.add_argument('--excite', type=int, default=80, help='амплитуда импульсов roll/pitch от центра, PWM (default 80)')
    p.add_argument('--excite-period', dest='excite_period', type=float, default=3.0,
                   help='базовая длительность τ профиля раскачки +τ/−2τ/+τ, sim-сек (цикл=4τ, default 3)')
    p.add_argument('--yaw-rate', dest='yaw_rate', type=int, default=30,
                   help='амплитуда yaw-импульса в EXCITE, PWM от центра 1500 (0=без yaw, default 30)')
    p.add_argument('--yaw-dur', dest='yaw_dur', type=float, default=1.5,
                   help='длительность yaw-импульса МЕЖДУ translate-циклами, sim-сек '
                        '(курс меняем только тут, во время translate курс фиксирован; default 1.5)')
    p.add_argument('--vins-timeout', dest='vins_timeout', type=float, default=90.0,
                   help='сколько ждать сходимости VINS в EXCITE, sim-сек (default 90)')
    p.add_argument('--vins-min', dest='vins_min', type=int, default=40,
                   help='сколько odom-сообщений считать сходимостью (default 40)')
    p.add_argument('--observe', type=float, default=15.0,
                   help='держать высоту после init перед посадкой (без handover), sim-сек (default 15)')
    p.add_argument('--hold-only', dest='hold_only', action='store_true',
                   help='liftland-режим: БЕЗ раскачки — climb→держать уровень hold_sec→land (диагностика дрейфа)')
    p.add_argument('--hold-sec', dest='hold_sec', type=float, default=30.0,
                   help='сколько держать уровень в hold-only, sim-сек (default 30)')
    # gz-position-hold (СИМ-костыль): держим точку по истинной позе Gazebo
    p.add_argument('--gz-hold', dest='gz_hold', action='store_true',
                   help='hold-only: держать ПОЗИЦИЮ по истинной позе Gazebo (/model/iris_cam/odometry)')
    p.add_argument('--gz-kp', dest='gz_kp', type=float, default=40.0,
                   help='gz-hold: PWM на метр ошибки позиции (default 40)')
    p.add_argument('--gz-kd', dest='gz_kd', type=float, default=120.0,
                   help='gz-hold: PWM на (м/с) скорости — демпфирование (default 120)')
    p.add_argument('--gz-ki', dest='gz_ki', type=float, default=8.0,
                   help='gz-hold: PWM на (м·с) интеграла — убирает статич. ошибку (0=без I, default 8)')
    p.add_argument('--gz-imax', dest='gz_imax', type=float, default=100.0,
                   help='gz-hold: макс вклад I-члена, PWM (anti-windup, default 100)')
    p.add_argument('--gz-max', dest='gz_max', type=float, default=150.0,
                   help='gz-hold: макс |offset| PWM по roll/pitch (default 150 ≈ 13°)')
    # Знаки эмпирически выверены отладкой (pitch_off<0 → ускорение ВПЕРЁД, не назад;
    # roll аналогично): для торможения сноса оба знака = +1. Конвенция pwm оказалась
    # противоположной комментарию excite. Оставлены тюнящимися на всякий случай.
    p.add_argument('--gz-psign', dest='gz_psign', type=float, default=1.0,
                   help='gz-hold: знак pitch-коррекции (±1; +1 выверен отладкой)')
    p.add_argument('--gz-rsign', dest='gz_rsign', type=float, default=1.0,
                   help='gz-hold: знак roll-коррекции (±1; +1 выверен отладкой)')
    p.add_argument('--gz-traj-r', dest='gz_traj_r', type=float, default=0.0,
                   help='gz-hold: радиус кругового траекторного сетпойнта, м (0=чистый холд; для параллакса VINS ~1-3)')
    p.add_argument('--gz-traj-t', dest='gz_traj_t', type=float, default=20.0,
                   help='gz-hold: период обхода круга, sim-сек (default 20)')
    # Наложенный yaw поверх удержания позиции — для ЧИСТОГО derotation-теста:
    # дрон стоит на месте (PID держит позицию, он yaw-инвариантен), а курс
    # вращается → поток rotation-доминирован → flow_derotation_check видит саму
    # derotation, не маскированную трансляционным дрейфом ALT_HOLD.
    p.add_argument('--gz-yaw', dest='gz_yaw', type=float, default=0.0,
                   help='gz-hold: амплитуда наложенного yaw, PWM от центра '
                        '(0=без вращения; ~80 ≈ умеренное ω для derotation-теста)')
    p.add_argument('--gz-yaw-period', dest='gz_yaw_period', type=float, default=6.0,
                   help='gz-hold: период смены знака наложенного yaw, sim-сек (default 6)')
    # FLOW-DAMP (--flow-hold): боковой velocity-демпфер по камере вместо gz-истины.
    # Спека docker/sim/FLOW_DAMP_spec.md (вариант a). Живёт в каркасе hold-only,
    # реюзит wall-publisher 20 Гц. gz-hold и flow-hold взаимоисключающи (gz приоритетнее).
    p.add_argument('--flow-hold', dest='flow_hold', action='store_true',
                   help='hold-only: гасить боковой снос демпфером по камере '
                        '(image_mono + gz-гироскоп) → ROLL-override; pitch/yaw центр')
    p.add_argument('--flow-kp', dest='flow_kp', type=float, default=8.0,
                   help='flow: P по боковому потоку (px/кадр → PWM), default 8')
    p.add_argument('--flow-ki', dest='flow_ki', type=float, default=2.0, help='flow: I (default 2)')
    p.add_argument('--flow-kd', dest='flow_kd', type=float, default=0.0, help='flow: D (default 0)')
    p.add_argument('--flow-imax', dest='flow_imax', type=float, default=120.0,
                   help='flow: anti-windup, макс |I| PWM (default 120)')
    p.add_argument('--flow-max', dest='flow_max', type=float, default=150.0,
                   help='flow: макс |ROLL-offset| PWM (default 150 ≈ 13°)')
    p.add_argument('--flow-conf-min', dest='flow_conf_min', type=float, default=0.05,
                   help='flow: ниже этой confidence начинается fade-out (default 0.05)')
    p.add_argument('--flow-conf-full', dest='flow_conf_full', type=float, default=0.20,
                   help='flow: выше — полный авторитет демпфера (default 0.20)')
    p.add_argument('--flow-rsign', dest='flow_rsign', type=float, default=1.0,
                   help='flow: знак derotation (rotflow_sign; +1 подтверждён Шагом 1)')
    p.add_argument('--flow-osign', dest='flow_osign', type=float, default=1.0,
                   help='flow: знак ROLL-offset/направление торможения (TODO: тюнить Шагом 3)')
    p.add_argument('--flow-smooth', dest='flow_smooth', type=int, default=1,
                   help='flow: временное сглаживание lateral, медиана по N кадрам (1=выкл; ~5 режет белый шум ~√N)')
    p.add_argument('--flow-image-topic', dest='flow_image_topic', default='/image_mono',
                   help='flow: топик камеры mono8 (default /image_mono)')
    p.add_argument('--flow-imu-topic', dest='flow_imu_topic', default='/gz_imu/data_flu',
                   help='flow: топик FLU-гироскопа (default /gz_imu/data_flu)')
    # ROLL-EXCITE — открытый контур для system-ID (docker/sim/HowToFlow_PID_synth.md).
    # Под gz-hold-pitch подаёт ЗАДАННЫЙ профиль roll_off (чирп→ступени), демпфер ВЫКЛ.
    # roll_off становится ЭКЗОГЕННЫМ → flow_calib.py чисто разделяет k/s/τ/d (не как в
    # замкнутом O2, где roll_off∝v_right). Требует записи /mavros/rc/override в bag.
    p.add_argument('--roll-excite', dest='roll_excite', action='store_true',
                   help='system-ID: заданный roll_off (чирп+ступени) на roll, pitch gz-held')
    p.add_argument('--roll-excite-amp', dest='roll_excite_amp', type=float, default=50.0,
                   help='roll-excite: амплитуда, PWM от центра (default 50)')
    p.add_argument('--roll-excite-f0', dest='roll_excite_f0', type=float, default=0.15,
                   help='roll-excite: начальная частота чирпа, Гц (default 0.15)')
    p.add_argument('--roll-excite-f1', dest='roll_excite_f1', type=float, default=1.5,
                   help='roll-excite: конечная частота чирпа, Гц (default 1.5)')
    p.add_argument('--roll-excite-chirp', dest='roll_excite_chirp', type=float, default=25.0,
                   help='roll-excite: длительность чирпа, sim-сек; дальше ступени (default 25)')
    p.add_argument('--roll-excite-step', dest='roll_excite_step', type=float, default=3.0,
                   help='roll-excite: полупериод ступени после чирпа, sim-сек (default 3)')
    p.add_argument('--throttle-climb', dest='throttle_climb', type=int, default=1650,
                   help='PWM газа на подъём (default 1650)')
    p.add_argument('--throttle-hold', dest='throttle_hold', type=int, default=RC_CENTER,
                   help='PWM газа на удержание = центр (default 1500)')
    p.add_argument('--mode-budget', dest='mode_budget', type=float, default=40.0, help='бюджет латча режима, sim-сек')
    p.add_argument('--arm-budget', dest='arm_budget', type=float, default=40.0, help='бюджет арминга, sim-сек')
    p.add_argument('--climb-budget', dest='climb_budget', type=float, default=60.0, help='бюджет набора высоты, sim-сек')
    p.add_argument('--land-budget', dest='land_budget', type=float, default=120.0, help='бюджет посадки, sim-сек')
    p.add_argument('--ground-z', dest='ground_z', type=float, default=0.3, help='порог касания по rel_alt, м')
    args = p.parse_args()
    if args.flow_hold:
        args.hold_only = True   # flow-демпфер живёт в каркасе hold-only (фаза EXCITE)
    if args.roll_excite:
        args.hold_only = True   # тот же каркас; pitch держит gz, roll = заданный чирп
        args.gz_hold = True

    rclpy.init()
    node = AltHoldBootstrap(args)
    try:
        # ДЕТЕРМИНИЗМ: точки СМЕНЫ override теперь задаёт tick() (sim-таймер) —
        # см. tick(). Этот wall-цикл (time.monotonic ~20 Гц) лишь РЕ-публикует
        # текущее (между sim-тиками неизменное) значение для СВЕЖЕСТИ override на FCU,
        # независимо от RTF. Значение между тиками константно → wall-re-publish новых
        # точек смены не вносит. spin_once крутит автомат (sim-таймер) и колбэки.
        last_pub = 0.0
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - last_pub >= 0.05:
                last_pub = now
                node._wall_publish()
    except KeyboardInterrupt:
        node.get_logger().info("Прервано — садимся вручную (make land).")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
