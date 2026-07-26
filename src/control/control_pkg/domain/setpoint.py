#!/usr/bin/env python3
"""Setpoint / MotionIntent / AxisPolicy — value objects композиции управления.

MotionIntent — что хочет Trajectory (смещение уставки от точки входа, в ТЕЛЕ:
вперёд/вправо, метры). Setpoint — абсолютная цель в МИРЕ (ControlStack собирает
её из origin + intent). AxisPolicy — как Excitation комбинируется со стабилизатором
на конкретной оси (ключ, из-за отсутствия которого наивная сумма motion+stab неверна).
"""
from dataclasses import dataclass
from enum import Enum, auto


class AxisPolicy(Enum):
    REGULATE = auto()   # стабилизатор ведёт ось к уставке (дефолт)
    ADDITIVE = auto()   # зонд ПОВЕРХ выхода стабилизатора (pitch-excite)
    REPLACE = auto()    # зонд ВЫТЕСНЯЕТ стабилизатор на оси (roll-excite, ALT_HOLD translate)


@dataclass
class MotionIntent:
    d_fwd: float = 0.0     # смещение уставки вперёд от точки входа, м (тело)
    d_right: float = 0.0   # смещение вправо, м (тело)


@dataclass
class Setpoint:
    x: float = 0.0         # абсолютная цель в МИРЕ, м
    y: float = 0.0
