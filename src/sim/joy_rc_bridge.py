#!/usr/bin/env python3
"""joy_rc_bridge — пульт в полётник SITL через его родной RC-вход (мост A, laptop_move.md §5.8).

ЗАЧЕМ. Приёмника на ноуте нет, и до сих пор стики пилота попадали в полётник SITL только
через override лётной ноды (`/joy` → нода → `/mavros/rc/override`). Переезд на углы уводит
ноду из override: ALT_HOLD — только пилот (по радио), нода — GUIDED_NOGPS. Чтобы сим
проверял ту же схему, что борт, пульту нужен настоящий RC-вход полётника. У SITL он
встроенный: UDP-порт `rcin_port` (5501 + 10·instance, AP_HAL_SITL/SITL_cmdline.cpp:253),
пакет — 8 или 16 `uint16` LE PWM (AP_RCProtocol_UDP.cpp: read_all_socket_input), SITL
отдаёт его в `radio_in` 50 раз в секунду — тот же путь, которым приходит физический
приёмник. Проверено по исходникам образа sim-simulator (a824813).

ЧТО ШЛЁТ. Раскладка каналов — как у TX12 на борту (AETR, CH5 — режим FCU, CH6 — SC,
CH7 — SF, кнопки — CH9+ по коду EdgeTX classic):
  CH1..4 — крен/тангаж/газ/курс тем же ядром `joy_sticks`, что у ноды (знаки JOY_SIGNS,
           ±400 µs от 1500) — пилот в ALT_HOLD получает ровно те PWM, что давал override;
  CH5..7 — axes[4..6] линейно: −1..+1 → 1000..2000;
  CH8    — 1500 (на TX12 CH8 дерётся с CH7 за axes[6], не используем);
  CH9..16 — buttons[0..7]: нажата → 2000, иначе 1000.
Пока лётная нода пишет override, он сильнее RC-входа — мост ничего не меняет в полёте.

ПОТЕРЯ /joy. SITL держит последний пакет ВЕЧНО (нет пакетов — нет и failsafe), а
настоящий приёмник при потере связи шлёт свои значения. Поэтому мост при /joy старше
`--stale` сек шлёт «failsafe приёмника»: стики в центре, газ 950 (как SIM_RC_FAIL=2).
Что сделает полётник, решают его FS_THR_* (в симе FS_THR_ENABLE 0 — ничего).

Запуск (bootstrap_arch2.sh поднимает сам рядом с источником /joy):
  python3 /root/sim_ws/src/sim/joy_rc_bridge.py [--signs=-1,1,-1,-1] [--port 5501]
"""
import argparse
import socket
import struct
import sys
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

try:
    from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT, joy_sticks
except ImportError:
    sys.path.insert(0, '/root/sim_ws/src/control')
    from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT, joy_sticks

N_CH = 16
FAILSAFE_THR = 950       # «газ 950» — failsafe приёмника (SIM_RC_FAIL=2 в SITL)


def aux_pwm(v: float) -> int:
    """Ось тумблера −1..+1 → 1000..2000 (EdgeTX: −100 %..+100 %)."""
    return int(round(1500 + 500 * max(-1.0, min(1.0, float(v)))))


def joy_to_channels(axes, buttons, signs):
    """Чистое ядро: /joy → 16 PWM в раскладке TX12."""
    r, p, t, y, _ = joy_sticks(axes, signs)
    ch = [r, p, t, y]
    for i in (4, 5, 6):
        ch.append(aux_pwm(axes[i]) if i < len(axes) else 1500)
    ch.append(1500)                                   # CH8 не используем
    for i in range(N_CH - 8):
        ch.append(2000 if i < len(buttons) and int(buttons[i]) else 1000)
    return ch


def failsafe_channels():
    ch = [1500] * N_CH
    ch[2] = FAILSAFE_THR
    for i in range(8, N_CH):
        ch[i] = 1000
    return ch


class JoyRcBridge(Node):
    def __init__(self, args):
        super().__init__('joy_rc_bridge')        # wall-время: RC-вход SITL живёт по нему
        self._signs = args.signs
        self._stale = args.stale
        self._dst = (args.host, args.port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._axes, self._buttons = [], []
        self._last = None                        # monotonic последнего /joy
        self._fs = None                          # текущее состояние failsafe (для лога)
        self.create_subscription(Joy, '/joy', self._on_joy, qos_profile_sensor_data)
        self.create_timer(1.0 / args.rate, self._tick)
        self.get_logger().info(
            f"мост /joy → RC-вход SITL udp://{args.host}:{args.port}, {args.rate:g} Гц, "
            f"знаки {tuple(self._signs)}, failsafe после {self._stale:g} с без /joy")

    def _on_joy(self, m):
        self._axes, self._buttons = list(m.axes), list(m.buttons)
        self._last = time.monotonic()

    def _tick(self):
        fs = self._last is None or time.monotonic() - self._last > self._stale
        ch = failsafe_channels() if fs else joy_to_channels(self._axes, self._buttons, self._signs)
        if fs != self._fs:
            self._fs = fs
            # два отдельных вызова: rclpy запрещает менять уровень лога на одной строке
            if fs:
                self.get_logger().warn("нет /joy — шлю failsafe приёмника (стики центр, газ 950)")
            else:
                self.get_logger().info("есть /joy — шлю стики пульта")
        self._sock.sendto(struct.pack('<16H', *ch), self._dst)


def parse_signs(s):
    if not s:
        return JOY_SIGNS_DEFAULT
    v = tuple(float(x) for x in s.split(','))
    if len(v) != 4:
        raise SystemExit(f"--signs: нужно 4 числа через запятую, дано {s!r}")
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--signs', default='', help='знаки осей roll,pitch,thr,yaw (как BS_JOY_SIGNS)')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5501, help='rcin_port SITL (5501 + 10·instance)')
    ap.add_argument('--rate', type=float, default=50.0, help='Гц отправки (SITL читает 50)')
    ap.add_argument('--stale', type=float, default=0.5, help='с без /joy до failsafe')
    args, _ = ap.parse_known_args()
    args.signs = parse_signs(args.signs)
    rclpy.init()
    node = JoyRcBridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
