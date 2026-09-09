#!/usr/bin/env python3
"""MavrosActuator — адаптер портов RcOutput + FlightMode + SetpointOutput.

RcOutput → /mavros/rc/override (каналы 1..4 = roll/pitch/throttle/yaw, 5..18 =
RC_NOCHANGE «игнор»). FlightMode → сервисы set_mode/cmd/arming (fire-and-forget,
call_async — как в монолите). SetpointOutput → /mavros/setpoint_raw/local
(PositionTarget: позиция + курс в локальной раме EKF) — им шаг RthTrack ведёт борт
домой по своему треку в GUIDED. Один адаптер держит все три порта: у них одна шина
MAVROS.
"""
from mavros_msgs.msg import OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

from ..domain.modes import to_fcu

from ..domain.rc import RC_NOCHANGE, RcCommand


class MavrosActuator:
    def __init__(self, node):
        self._rc_pub = node.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self._mode_cli = node.create_client(SetMode, '/mavros/set_mode')
        self._arm_cli = node.create_client(CommandBool, '/mavros/cmd/arming')
        self._cmd_cli = node.create_client(CommandLong, '/mavros/cmd/command')
        self._sp_pub = node.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

    # --- RcOutput ---
    def publish(self, cmd: RcCommand) -> None:
        msg = OverrideRCIn()
        ch = [RC_NOCHANGE] * 18
        ch[0] = int(cmd.roll)
        ch[1] = int(cmd.pitch)
        ch[2] = int(cmd.throttle)
        ch[3] = int(cmd.yaw)
        msg.channels = ch
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
