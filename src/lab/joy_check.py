#!/usr/bin/env python3
"""Наземная сверка пульта (TX12 USB-джойстиком → /joy): живая печать того, что
УВИДИТ нода. Интерпретация — тем же ядром joy_sticks(), что у JoyPilot
(control_pkg), т.е. это буквально вход RcTransmitter/Arbiter, а не «примерно так».

Порядок сверки (двигать по ОДНОМУ стику за раз, полёт не нужен):
  roll вправо → roll растёт к 1900; pitch ОТ СЕБЯ → ? (записать факт);
  throttle вверх → thr к 1900; yaw вправо → yaw к 1900;
  тумблер CH6 → MANUAL/AUTO щёлкает.
PWM печатается УЖЕ с боевыми знаками (JOY_SIGNS_DEFAULT: roll/yaw в EdgeTX-HID
зеркальны, выверено полётом) — т.е. «вправо → 1900» должен выполняться буквально.
Новая зеркальная ось лечится параметром signs у JoyPilot — НЕ пересборкой модели.

Запуск — через обёртку (сама поднимет joy_linux_node): bash /lab/joy_check.sh
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

try:
    from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT, joy_sticks
except ImportError:                     # workspace ещё не собран — берём исходники
    sys.path.insert(0, '/root/sim_ws/src/control')
    from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT, joy_sticks


class JoyCheck(Node):
    def __init__(self):
        super().__init__('joy_check')
        self._n = 0
        self.create_subscription(Joy, '/joy', self._on, qos_profile_sensor_data)
        print("Жду /joy … двигай стики (Ctrl+C — выход)")

    def _on(self, m):
        self._n += 1
        r, p, t, y, sw = joy_sticks(m.axes, JOY_SIGNS_DEFAULT)
        ax = " ".join(f"{a:+.2f}" for a in m.axes)
        print(f"\raxes[{len(m.axes)}]: {ax} | roll={r} pitch={p} thr={t} yaw={y} "
              f"| {'MANUAL' if sw else 'AUTO  '} (#{self._n})    ",
              end="", flush=True)


def main():
    rclpy.init()
    node = JoyCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
