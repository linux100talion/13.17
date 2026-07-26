#!/usr/bin/env python3
"""Порты — контракты между доменом/приложением и инфраструктурой (hexagonal).

Protocol'ы (structural typing): инфраструктура их реализует ROS-адаптерами, домен
зависит только от этих сигнатур. FlightMode (КОМАНДЫ set_mode/arm) намеренно
отделён от RcOutput (RC-override) — разная семантика и разные топики/сервисы.
"""
from typing import Protocol

from .rc import RcCommand
from .state import DroneState


class Clock(Protocol):
    def now_sim(self) -> float: ...


class Telemetry(Protocol):
    def snapshot(self) -> DroneState: ...


class RcOutput(Protocol):
    def publish(self, cmd: RcCommand) -> None: ...


class FlightMode(Protocol):
    def set_mode(self, mode: str) -> None: ...
    def arm(self) -> None: ...
    def ready(self) -> bool: ...


class PilotInput(Protocol):
    def sticks(self) -> RcCommand: ...   # сырой PWM с /mavros/rc/in (радио ИЛИ SITL)
    def mode_switch(self) -> int: ...    # тумблер авто/ручной — для арбитража


class Logger(Protocol):
    def info(self, m: str) -> None: ...
    def warn(self, m: str) -> None: ...
    def error(self, m: str) -> None: ...


class DebugSink(Protocol):
    def publish(self, roll_off: float, flow_off: float, conf: float, stamp: float) -> None: ...
