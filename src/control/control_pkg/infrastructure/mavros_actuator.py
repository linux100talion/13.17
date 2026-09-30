#!/usr/bin/env python3
"""MavrosActuator — адаптер портов RcOutput + FlightMode + SetpointOutput.

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
"""
from mavros_msgs.msg import OverrideRCIn, ParamEvent, PositionTarget
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

from ..domain.modes import to_fcu

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

from ..domain.rc import RC_NOCHANGE, RC_RELEASE, RcCommand
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

    # --- реверс каналов FCU ---
    def _learn(self, name, value) -> None:
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
        if self._rev.ready():
            return
        if not self._param_get.service_is_ready():
            return
        names = list(_REV_PARAMS)
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
