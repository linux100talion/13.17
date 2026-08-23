#!/usr/bin/env python3
"""MavrosActuator — адаптер портов RcOutput + FlightMode.

RcOutput → /mavros/rc/override (каналы 1..4 = roll/pitch/throttle/yaw, 5..18 =
RC_NOCHANGE «игнор»). FlightMode → сервисы set_mode/cmd/arming (fire-and-forget,
call_async — как в монолите). Один адаптер держит оба порта: у обоих одна шина MAVROS.
"""
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

from ..domain.rc import RC_NOCHANGE, RcCommand


class MavrosActuator:
    def __init__(self, node):
        self._rc_pub = node.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self._mode_cli = node.create_client(SetMode, '/mavros/set_mode')
        self._arm_cli = node.create_client(CommandBool, '/mavros/cmd/arming')
        self._cmd_cli = node.create_client(CommandLong, '/mavros/cmd/command')

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

    # --- FlightMode ---
    def set_mode(self, mode: str) -> None:
        if self._mode_cli.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
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
