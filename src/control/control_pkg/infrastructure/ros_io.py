#!/usr/bin/env python3
"""RosLogger + RosDebugSink — мелкие адаптеры портов Logger и DebugSink.

DebugSink → /flow_dbg (Vector3Stamped): sim-штампованный roll_off/flow_lateral/conf для
system-ID (flow_calib.py). Vector3Stamped, т.к. OverrideRCIn без header → под низким
RTF не привязать к sim-времени. publish_axes() шлёт заодно /flow_dbg2
(pitch_off/flow_longitudinal/flow_yaw) — полный флоу-дамп для свипа/диагностики.
"""
from geometry_msgs.msg import Vector3Stamped

from ..domain.rc import RC_CENTER


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
        self._pub2 = node.create_publisher(Vector3Stamped, '/flow_dbg2', 10)
        self._pub3 = node.create_publisher(Vector3Stamped, '/flow_dbg3', 10)

    def publish(self, roll_off: float, flow_off: float, conf: float, stamp: float) -> None:
        d = Vector3Stamped()
        d.header.stamp = self._node.get_clock().now().to_msg()
        d.vector.x = float(roll_off)
        d.vector.y = float(flow_off)
        d.vector.z = float(conf)
        self._pub.publish(d)

    def publish_axes(self, s, rc) -> None:
        """Полный флоу-дамп (sim-штамп): /flow_dbg = (roll_off, flow_lateral, conf),
        /flow_dbg2 = (pitch_off, flow_longitudinal, flow_yaw). Для свипа: сверить
        сигнал ↔ команду ↔ истинную скорость (odometry) в одном sim-времени."""
        t = self._node.get_clock().now().to_msg()
        d = Vector3Stamped()
        d.header.stamp = t
        d.vector.x = float(rc.roll - RC_CENTER)
        d.vector.y = float(s.flow_lateral)
        d.vector.z = float(s.flow_conf)
        self._pub.publish(d)
        d2 = Vector3Stamped()
        d2.header.stamp = t
        d2.vector.x = float(rc.pitch - RC_CENTER)
        d2.vector.y = float(s.flow_longitudinal)
        d2.vector.z = float(s.flow_yaw)
        self._pub2.publish(d2)
        # /flow_dbg3 = ОПОРА: (log масштаба, сдвиг X, доля видимой опоры). Топик уже
        # стоял в TOPICS_EXTRA калиб-прогонов, но публиковать в него было нечего.
        d3 = Vector3Stamped()
        d3.header.stamp = t
        d3.vector.x = float(s.kf_logs)
        d3.vector.y = float(s.kf_dx)
        d3.vector.z = float(s.kf_n)
        self._pub3.publish(d3)
