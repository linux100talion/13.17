#!/usr/bin/env python3
"""Порты — контракты между доменом/приложением и инфраструктурой (hexagonal).

Protocol'ы (structural typing): инфраструктура их реализует ROS-адаптерами, домен
зависит только от этих сигнатур. FlightMode (КОМАНДЫ set_mode/arm) намеренно
отделён от RcOutput (RC-override) — разная семантика и разные топики/сервисы.

⚠️ Порты СВЕРЯЮТСЯ с адаптерами офлайн-тестом `test/test_ports.py` (AST-сверка
имён и сигнатур, без импорта rclpy — работает и на хосте без ROS). До 2026-09-01
ports.py не импортировал никто, и DebugSink разъехался с реальностью (у
RosDebugSink давно 6 методов, в порте был 1) — порт без проверки гниёт молча.
Меняешь поверхность адаптера — меняй порт, тест напомнит.

Без портов (сознательно): RosPerception и BaroAlt — они не вызываются доменом,
а НАПОЛНЯЮТ DroneState (снапшот и есть их контракт); StationFrame читает нода
через dbg() (пулл, как hold_dbg/rate_dbg — см. station_frame.py).
"""
from typing import Protocol, runtime_checkable

from .rc import RcCommand
from .state import DroneState


@runtime_checkable
class Clock(Protocol):
    def now_sim(self) -> float: ...


@runtime_checkable
class Telemetry(Protocol):
    def snapshot(self) -> DroneState: ...


@runtime_checkable
class RcOutput(Protocol):
    def publish(self, cmd: RcCommand) -> None: ...


@runtime_checkable
class FlightMode(Protocol):
    def set_mode(self, mode: str) -> None: ...
    def arm(self) -> None: ...
    def ready(self) -> bool: ...


@runtime_checkable
class PilotInput(Protocol):
    def sticks(self) -> RcCommand: ...   # сырой PWM с /mavros/rc/in (радио ИЛИ SITL)
    def mode_switch(self) -> int: ...    # тумблер авто/ручной — для арбитража
    def stab_level(self) -> int: ...     # потолок лесенки SC 0..2 (схема SF-мастер);
                                         # вне схемы адаптеры отдают 0
    def land_switch(self) -> bool: ...   # кнопка посадки (SA): уровень «нажата»;
                                         # адаптеры без источника отдают False


@runtime_checkable
class Logger(Protocol):
    def info(self, m: str) -> None: ...
    def warn(self, m: str) -> None: ...
    def error(self, m: str) -> None: ...


@runtime_checkable
class DebugSink(Protocol):
    """Отладочный даунлинк лётной ноды: /flow_dbg* + /mission/status.

    Полная поверхность RosDebugSink (ros_io.py — там расписано, ЧТО лежит в каждом
    топике и зачем). publish_hold/rate_*/hold_yaw молчат на None — так ноде не
    нужно знать, какая ось какого режима в стеке."""
    def publish(self, roll_off: float, flow_off: float, conf: float,
                stamp: float) -> None: ...
    def publish_status(self, line: str) -> None: ...
    def publish_hold(self, hold) -> None: ...
    def publish_rate_roll(self, dbg) -> None: ...
    def publish_rate_pitch(self, dbg) -> None: ...
    def publish_hold_yaw(self, hold, yaw_off: float = 0.0) -> None: ...
    def publish_axes(self, s: DroneState, rc: RcCommand) -> None: ...
