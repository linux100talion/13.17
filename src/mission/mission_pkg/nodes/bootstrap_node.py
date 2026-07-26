#!/usr/bin/env python3
"""bootstrap_node — точка входа среза 1 (ARCH2): gz-hold + shuttle.

Composition root: argparse → BootstrapConfig → сборка ROS-адаптеров (control_pkg) +
MissionRunner (mission_pkg) → spin. Сам домен/приложение о ROS не знают; здесь их
проводка. Флаги совместимы с alt_hold_bootstrap.py (подмножество, нужное срезу).

Детерминизм override: точки СМЕНЫ значения задаёт sim-таймер (_tick, 20 Гц sim); wall-
цикл в main лишь РЕ-публикует неизменное между тиками значение для свежести на FCU.

Запуск (внутри nav-контейнера, после colcon build):
    ros2 run mission_pkg bootstrap_arch2 --alt 3 --gz-shuttle-a 5
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from control_pkg.application.control_stack import ControlStack
from control_pkg.domain.control.excitation import NoExcitation
from control_pkg.domain.control.stabilization import GzPositionHold
from control_pkg.domain.control.trajectory import Shuttle
from control_pkg.domain.rc import RC_CENTER, RcCommand
from control_pkg.infrastructure.mavros_actuator import MavrosActuator
from control_pkg.infrastructure.ros_clock import RosClock
from control_pkg.infrastructure.ros_io import RosDebugSink, RosLogger
from control_pkg.infrastructure.ros_telemetry import RosTelemetry

from ..application.mission_runner import S_DONE, MissionRunner
from ..config import BootstrapConfig


class BootstrapArch2Node(Node):
    def __init__(self, cfg: BootstrapConfig):
        super().__init__('alt_hold_bootstrap_arch2')
        # Все бюджеты/таймеры — по sim-времени (/clock), RTF-независимо.
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.cfg = cfg

        # адаптеры (инфраструктура)
        self.clock = RosClock(self)
        self.telemetry = RosTelemetry(self, self.clock)
        self.actuator = MavrosActuator(self)     # RcOutput + FlightMode
        self.logger = RosLogger(self)
        self.debug = RosDebugSink(self)

        # домен/приложение (проводка стратегий среза 1)
        stack = ControlStack(
            GzPositionHold(cfg.gz_kp, cfg.gz_kd, cfg.gz_ki, cfg.gz_imax,
                           cfg.gz_max, cfg.gz_psign, cfg.gz_rsign),
            Shuttle(cfg.gz_shuttle_a, cfg.gz_shuttle_v, cfg.gz_shuttle_pause,
                    cfg.gz_shuttle_fwd),
            NoExcitation(),
        )
        self.runner = MissionRunner(cfg, self.clock, self.actuator, stack, self.logger)

        self._last_rc = RcCommand()
        self.timer = self.create_timer(0.05, self._tick)   # автомат + publish (sim-время)
        self.logger.info(
            f"alt_hold_bootstrap ARCH2: gz-hold+shuttle alt={cfg.alt}м "
            f"a={cfg.gz_shuttle_a} v={cfg.gz_shuttle_v} fwd={cfg.gz_shuttle_fwd} (sim)")

    def _tick(self):
        s = self.telemetry.snapshot()
        rc = self.runner.tick(s)
        self._last_rc = rc
        self._publish(rc)
        # sim-штампованный debug (роль-стека roll_off; flow/conf=0 в срезе 1)
        self.debug.publish(float(rc.roll - RC_CENTER), 0.0, 0.0, s.now_sim)

    def _publish(self, rc: RcCommand):
        # В DONE override больше не нужен (дрон сел/LAND сам).
        if self.runner.state == S_DONE:
            return
        self.actuator.publish(rc)

    @property
    def finished(self) -> bool:
        return self.runner.finished


def _parse():
    p = argparse.ArgumentParser()
    p.add_argument('--alt', type=float, default=3.0)
    p.add_argument('--throttle-climb', dest='throttle_climb', type=int, default=1650)
    p.add_argument('--throttle-hold', dest='throttle_hold', type=int, default=RC_CENTER)
    p.add_argument('--ground-z', dest='ground_z', type=float, default=0.3)
    p.add_argument('--mode-budget', dest='mode_budget', type=float, default=40.0)
    p.add_argument('--arm-budget', dest='arm_budget', type=float, default=40.0)
    p.add_argument('--climb-budget', dest='climb_budget', type=float, default=60.0)
    p.add_argument('--land-budget', dest='land_budget', type=float, default=120.0)
    p.add_argument('--gz-kp', dest='gz_kp', type=float, default=40.0)
    p.add_argument('--gz-kd', dest='gz_kd', type=float, default=120.0)
    p.add_argument('--gz-ki', dest='gz_ki', type=float, default=8.0)
    p.add_argument('--gz-imax', dest='gz_imax', type=float, default=100.0)
    p.add_argument('--gz-max', dest='gz_max', type=float, default=150.0)
    p.add_argument('--gz-psign', dest='gz_psign', type=float, default=1.0)
    p.add_argument('--gz-rsign', dest='gz_rsign', type=float, default=1.0)
    p.add_argument('--gz-shuttle-a', dest='gz_shuttle_a', type=float, default=5.0)
    p.add_argument('--gz-shuttle-v', dest='gz_shuttle_v', type=float, default=1.5)
    p.add_argument('--gz-shuttle-pause', dest='gz_shuttle_pause', type=float, default=2.0)
    p.add_argument('--gz-shuttle-fwd', dest='gz_shuttle_fwd', action='store_true')
    a = p.parse_args()
    return BootstrapConfig(**vars(a))


def main():
    cfg = _parse()
    rclpy.init()
    node = BootstrapArch2Node(cfg)
    try:
        # wall-цикл (time.monotonic ~20 Гц) РЕ-публикует текущее значение для свежести
        # override на FCU (независимо от RTF); точки смены задаёт sim-таймер _tick.
        last_pub = 0.0
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - last_pub >= 0.05:
                last_pub = now
                node._publish(node._last_rc)
    except KeyboardInterrupt:
        node.logger.info("Прервано — садимся вручную (make land).")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
