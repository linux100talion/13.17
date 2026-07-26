#!/usr/bin/env python3
"""Контракты трёх ролей управления (Strategy). Чистый домен.

Trajectory  — выдаёт НАМЕРЕНИЕ (смещение уставки), не знает про PWM.
Stabilization — намерение + обратная связь → RC по регулируемым осям.
Excitation  — экзогенный зонд для system-ID с политикой осей (ADDITIVE/REPLACE).

Три роли, а не две: движение и стабилизация комбинируются тремя разными способами
(впрыск сетпойнта / аддитивно / замена оси) — одна плоская сумма их не выражает.
"""
from abc import ABC, abstractmethod

from ..rc import RcCommand
from ..setpoint import AxisPolicy, MotionIntent, Setpoint
from ..state import DroneState


class TrajectoryStrategy(ABC):
    @abstractmethod
    def intent(self, s: DroneState, t: float) -> MotionIntent:
        """t — sim-время с момента входа в фазу (ControlStack.enter)."""

    def done(self, t: float) -> bool:
        return False   # челнок сам сообщит «отлетал» → триггер land


class StabilizationStrategy(ABC):
    axes: frozenset = frozenset()          # какие оси регулирует: {"roll","pitch","yaw"}

    def enter(self, s: DroneState) -> None:
        """Сброс внутреннего состояния (интеграторы) при switch. По умолч. — ничего."""

    @abstractmethod
    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand: ...


class ExcitationStrategy(ABC):
    @abstractmethod
    def offset(self, s: DroneState, t: float) -> dict:
        """ось → (PWM-offset от центра, AxisPolicy). Пусто = ничего не подмешивать."""

    def done(self, t: float) -> bool:
        return False   # excite_total → триггер land
