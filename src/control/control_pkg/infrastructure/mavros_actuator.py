#!/usr/bin/env python3
"""MavrosActuator — адаптер портов RcOutput + FlightMode + SetpointOutput + AttitudeOutput.

RcOutput → /mavros/rc/override (каналы 1..4 = roll/pitch/throttle/yaw, 5..18 =
RC_NOCHANGE «игнор»). FlightMode → сервисы set_mode/cmd/arming (fire-and-forget,
call_async — как в монолите). SetpointOutput → /mavros/setpoint_raw/local
(PositionTarget: позиция + курс в локальной раме EKF) — им шаг RthTrack ведёт борт
домой по своему треку в GUIDED. Один адаптер держит все три порта: у них одна шина
MAVROS.

Реверс каналов ПОЛЁТНИКА (`RCn_REVERSED`) применяется и к override (как к приёмнику),
а домен говорит в стандартной конвенции ArduPilot — адаптер читает `RC1..4_REVERSED`
(+ `TRIM`) у FCU и зеркалит реверсные каналы (`rc_reverse.py`, разбор — laptop_move.md
§5.7). Пока реверс не прочитан — override НЕ шлём, отпускаем каналы (release): на
борту управление остаётся у физического приёмника, а не у канала с неизвестным знаком.

AttitudeOutput → /mavros/setpoint_raw/attitude (SET_ATTITUDE_TARGET; полётник принимает
его только в GUIDED/GUIDED_NOGPS). Домен говорит в осях ArduPilot (domain/attitude.py), MAVROS
ждёт оси ROS (тело FLU, мир ENU) и переводит в NED/FRD сам: тангаж FLU = −тангаж FRD, темп
курса FLU = −темп FRD. Курс в кватернионе — СВОЯ цель: старт от текущего курса AHRS
(/mavros/imu/data), дальше интеграл темпа; полётник между сообщениями двигает цель тем же
темпом (AC_AttitudeControl::input_quaternion). Реверс каналов к углам НЕ применяется —
SET_ATTITUDE_TARGET идёт мимо обработки пульта. Пересчёт PWM → углы и потолок наклона —
att_convert.py по параметрам полётника; пока они не прочитаны, углы не шлём (полётник по
GUID_TIMEOUT сам выравнивается и держит высоту — безопаснее кривого масштаба).
"""
import math
import time

from mavros_msgs.msg import AttitudeTarget, OverrideRCIn, ParamEvent, PositionTarget
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

from ..domain.modes import to_fcu

from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

from ..domain.rc import RC_NOCHANGE, RC_RELEASE, RcCommand
from .att_convert import (FcuStickParams, limit_tilt, rc_to_attitude,
                          thrust_from_climb)
from .rc_reverse import PARAMS as _REV_PARAMS, RcReverse


class MavrosActuator:
    def __init__(self, node):
        self._rc_pub = node.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self._mode_cli = node.create_client(SetMode, '/mavros/set_mode')
        self._arm_cli = node.create_client(CommandBool, '/mavros/cmd/arming')
        self._cmd_cli = node.create_client(CommandLong, '/mavros/cmd/command')
        self._sp_pub = node.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        # реверс каналов FCU: событие MAVROS на каждый PARAM_VALUE (ловит и живую
        # правку параметра) + опрос get_parameters раз в секунду, пока не прочитан
        # (нода могла подняться ПОСЛЕ того, как MAVROS вытянул параметры — событий
        # тогда не будет). BEST_EFFORT: совместим с любым QoS издателя (подписка
        # RELIABLE на BEST_EFFORT молчит без ошибки — память mavros-qos-silent).
        self._log = node.get_logger()
        self._clock = node.get_clock()
        self._rev = RcReverse()
        self._rev_warn_t = None
        node.create_subscription(
            ParamEvent, '/mavros/param/event', self._on_param_event,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._param_get = node.create_client(GetParameters, '/mavros/param/get_parameters')
        self._rev_timer = node.create_timer(1.0, self._poll_reverse)
        # AttitudeOutput: параметры пересчёта (тем же опросом), курс AHRS, цель курса
        self._att = FcuStickParams()
        self._att_pub = node.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self._yaw_now = None             # ENU-курс AHRS, рад
        self._yaw_tgt = None             # ENU-цель курса, рад (None = взять с AHRS)
        self._att_t = None               # monotonic последней отправки углов
        self._att_warn_t = None
        node.create_subscription(Imu, '/mavros/imu/data', self._on_imu, qos_profile_sensor_data)

    # --- реверс каналов FCU ---
    def _learn(self, name, value) -> None:
        was_att = self._att.ready()
        if self._att.on_param(name, value) and self._att.ready():
            self._log.info(f"углы: параметры FCU {'прочитаны' if not was_att else 'изменились'} — "
                           f"{self._att.describe()}")
        before = self._rev.describe() if self._rev.ready() else None
        self._rev.on_param(name, value)
        # лог — когда реверс впервые прочитан или ПОМЕНЯЛСЯ (TRIM и повторы молчат)
        if self._rev.ready() and self._rev.describe() != before:
            (self._log.error if self._rev.throttle_reversed() else self._log.info)(
                f"реверс каналов FCU: {self._rev.describe()}")

    def _on_param_event(self, m: ParamEvent) -> None:
        v = m.value
        if v.type == ParameterType.PARAMETER_INTEGER:
            self._learn(m.param_id, v.integer_value)
        elif v.type == ParameterType.PARAMETER_DOUBLE:
            self._learn(m.param_id, v.double_value)

    def _poll_reverse(self) -> None:
        names = ([n for n in _REV_PARAMS if not self._rev.ready()]
                 + self._att.missing())
        if not names:
            return
        if not self._param_get.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = names

        def done(fut):
            try:
                vals = fut.result().values
            except Exception:
                return
            for name, v in zip(names, vals):
                if v.type == ParameterType.PARAMETER_INTEGER:
                    self._learn(name, v.integer_value)
                elif v.type == ParameterType.PARAMETER_DOUBLE:
                    self._learn(name, v.double_value)
        self._param_get.call_async(req).add_done_callback(done)

    # --- RcOutput ---
    def publish(self, cmd: RcCommand) -> None:
        ch4 = self._rev.apply([int(cmd.roll), int(cmd.pitch),
                               int(cmd.throttle), int(cmd.yaw)])
        if ch4 is None:
            # реверс не прочитан (или перевёрнут газ) — знак канала неизвестен:
            # не оверрайдим, отпускаем к приёмнику; лог раз в 5 с
            now = self._clock.now().nanoseconds * 1e-9
            if self._rev_warn_t is None or now - self._rev_warn_t > 5.0:
                self._rev_warn_t = now
                why = (self._rev.describe() if self._rev.throttle_reversed()
                       else "не прочитаны " + " ".join(self._rev.missing()))
                self._log.warn(f"override не шлю (release): реверс каналов FCU — {why}")
            self.release()
            return
        msg = OverrideRCIn()
        msg.channels = ch4 + [RC_NOCHANGE] * 14
        self._rc_pub.publish(msg)

    def release(self) -> None:
        """ОТПУСТИТЬ ch1..4 обратно радио (RC_RELEASE = 0 — не RC_NOCHANGE!):
        у ArduPilot нулевой override не активен, и FCU в тот же кадр возвращается
        к физическому приёмнику. Зовётся сторожем свежести пульта (PilotLink),
        когда источник стиков замолчал: держать override с замороженными стиками
        опаснее, чем выйти из цепочки. Семантика нуля ПРОВЕРЕНА замером в SITL
        2026-09-22 (src/lab/rc_release_check.py): ноль возвращает канал к RC-входу
        за 0.06 с, а 65535 — только через 2.98 с (истечение RC_OVERRIDE_TIME), то
        есть это именно release. На реальном борту замер повторить тем же скриптом."""
        msg = OverrideRCIn()
        msg.channels = [RC_RELEASE] * 4 + [RC_NOCHANGE] * 14
        self._rc_pub.publish(msg)

    # --- AttitudeOutput ---
    def _on_imu(self, m) -> None:
        q = m.orientation
        self._yaw_now = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def att_ready(self) -> bool:
        """Параметры пересчёта в углы прочитаны (готовность к арму нодой)."""
        return self._att.ready()

    def hold_yaw_at_ahrs(self) -> None:
        """На земле цель курса НЕ интегрируем: следующая отправка возьмёт текущий курс
        AHRS. Иначе жест арма (курс вправо в упор) накапливал в цели +30° (поводок), и
        после взлёта полётник разворачивал нос в наборе (attout_rcrev 161302/163209:
        DesYaw 117° при курсе 87° на входе в полёт — боковой унос на демпфере)."""
        self._yaw_tgt = None

    def publish_rc_as_attitude(self, rc: RcCommand) -> None:
        """Шаг 1 переезда: домен ещё в PWM — пересчёт так же, как ALT_HOLD (att_convert)."""
        if not self._att.ready():
            self._att_not_ready()
            return
        self.publish_attitude(rc_to_attitude(rc, self._att.p))

    def _att_not_ready(self) -> None:
        now = time.monotonic()
        if self._att_warn_t is None or now - self._att_warn_t > 5.0:
            self._att_warn_t = now
            why = ("не прочитаны " + " ".join(self._att.missing()) if not self._att.ready()
                   else "нет курса AHRS (/mavros/imu/data)")
            self._log.warn(f"углы не шлю: {why}")

    def publish_attitude(self, cmd) -> None:
        if not self._att.ready() or self._yaw_now is None:
            self._att_not_ready()
            return
        cmd = limit_tilt(cmd, self._att.p)
        now = time.monotonic()
        # цель курса: после паузы потока (>0.5 с) — с текущего курса AHRS, иначе
        # интеграл темпа (ENU против часовой = минус темп FRD); поводок ±30° к AHRS,
        # чтобы цель не убегала, если борт темп не исполняет
        if self._yaw_tgt is None or self._att_t is None or now - self._att_t > 0.5:
            self._yaw_tgt = self._yaw_now
        else:
            self._yaw_tgt -= cmd.yaw_rate * (now - self._att_t)
            err = math.atan2(math.sin(self._yaw_tgt - self._yaw_now),
                             math.cos(self._yaw_tgt - self._yaw_now))
            lim = math.radians(30.0)
            if abs(err) > lim:
                self._yaw_tgt = self._yaw_now + math.copysign(lim, err)
        self._yaw_tgt = math.atan2(math.sin(self._yaw_tgt), math.cos(self._yaw_tgt))
        self._att_t = now
        # оси ROS: тело FLU (тангаж — минус), мир ENU; кватернион ZYX
        r, p, y = cmd.roll, -cmd.pitch, self._yaw_tgt
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        msg = AttitudeTarget()
        msg.header.stamp = self._clock.now().to_msg()
        msg.type_mask = 0                # всё в ходу: кватернион + три темпа + thrust
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.body_rate.x = 0.0            # полётник принимает темпы только все три сразу
        msg.body_rate.y = 0.0
        msg.body_rate.z = -cmd.yaw_rate  # FLU: против часовой +
        msg.thrust = thrust_from_climb(cmd.climb, self._att.p)
        self._att_pub.publish(msg)

    # --- SetpointOutput ---
    # Маска: командуем ТОЛЬКО позицию (и курс, если дан) — скорости/ускорения и
    # темп курса игнорируются полётником. Значения в ENU: плагин setpoint_raw
    # переводит их в NED сам, поэтому x/y/z берутся прямо из нашей рамы EKF.
    _MASK_POS = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY
                 | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX
                 | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                 | PositionTarget.IGNORE_YAW_RATE)

    def publish_pos(self, x: float, y: float, z: float, yaw=None) -> None:
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = self._MASK_POS | (PositionTarget.IGNORE_YAW if yaw is None else 0)
        msg.position.x, msg.position.y, msg.position.z = float(x), float(y), float(z)
        if yaw is not None:
            msg.yaw = float(yaw)
        self._sp_pub.publish(msg)

    # --- FlightMode ---
    def set_mode(self, mode: str) -> None:
        if self._mode_cli.service_is_ready():
            req = SetMode.Request()
            # имя, которого MAVROS не знает (SMART_RTL), уходит НОМЕРОМ —
            # иначе запрос умирает в его таблице режимов (domain/modes.py)
            req.custom_mode = to_fcu(mode)
            self._mode_cli.call_async(req)

    def arm(self, value: bool = True) -> None:
        if self._arm_cli.service_is_ready():
            req = CommandBool.Request()
            req.value = value
            self._arm_cli.call_async(req)

    def force_disarm(self) -> None:
        """Принудительный дизарм (MAV_CMD_COMPONENT_ARM_DISARM, param2=21196):
        проходит, даже когда детектор посадки FCU не взводится и штатный
        cmd/arming отвергается. Только для страховки freefly НА ЗЕМЛЕ (см.
        Freefly: жест дизарма, удержанный пилотом дольше порога)."""
        if self._cmd_cli.service_is_ready():
            req = CommandLong.Request()
            req.command = 400
            req.param1 = 0.0
            req.param2 = 21196.0
            self._cmd_cli.call_async(req)

    def ready(self) -> bool:
        return self._mode_cli.service_is_ready() and self._arm_cli.service_is_ready()
