#!/usr/bin/env python3
"""RosLogger + RosDebugSink — мелкие адаптеры портов Logger и DebugSink.

DebugSink → /flow_dbg (Vector3Stamped): sim-штампованный roll_off/flow/conf для
system-ID (flow_calib.py). Vector3Stamped, т.к. OverrideRCIn без header → под низким
RTF не привязать к sim-времени.
"""
from geometry_msgs.msg import Vector3Stamped


class RosLogger:
    def __init__(self, node):
        self._log = node.get_logger()

    def info(self, m: str) -> None:
        self._log.info(m)

    def warn(self, m: str) -> None:
        self._log.warn(m)

    def error(self, m: str) -> None:
        self._log.error(m)


class RosDebugSink:
    def __init__(self, node):
        self._node = node
        self._pub = node.create_publisher(Vector3Stamped, '/flow_dbg', 10)

    def publish(self, roll_off: float, flow_off: float, conf: float, stamp: float) -> None:
        d = Vector3Stamped()
        d.header.stamp = self._node.get_clock().now().to_msg()
        d.vector.x = float(roll_off)
        d.vector.y = float(flow_off)
        d.vector.z = float(conf)
        self._pub.publish(d)
