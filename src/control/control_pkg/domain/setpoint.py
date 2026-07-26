#!/usr/bin/env python3
"""Setpoint / MotionIntent / AxisPolicy — value objects композиции управления.

PROFILE-ONLY модель: движение — это ВЕЗДЕ стик-профиль (нормированный уровень стика
[-1..1] по осям тела). Метрических расстояний (`d_*`) больше нет — до абсолютной
локализации (NN1/выровненный VINS) они не опираемы. Профиль = MotionIntent во времени.

Один профиль → любой стек, но каждый стабилизатор читает его по-своему:
- Dp* (демпфер): c_* = ЦЕЛЬ скорости (гасит визуальную скорость к ней);
- Gz*/Vins (позиция): c_* = команда скорости → ИНТЕГРИРУЕТ в движущуюся уставку (Loiter).

AxisPolicy — как Excitation комбинируется со стабилизатором на оси (regulate/add/replace).
"""
from dataclasses import dataclass
from enum import Enum, auto


class AxisPolicy(Enum):
    REGULATE = auto()   # стабилизатор ведёт ось к уставке (дефолт)
    ADDITIVE = auto()   # зонд ПОВЕРХ выхода стабилизатора (pitch-excite)
    REPLACE = auto()    # зонд ВЫТЕСНЯЕТ стабилизатор на оси (roll-excite, translate)


@dataclass
class MotionIntent:
    # НОРМИРОВАННЫЙ стик-уровень [-1..1] по осям тела — единственный «язык» движения.
    c_fwd: float = 0.0     # продольный (вперёд+)
    c_right: float = 0.0   # боковой (вправо+)
    c_yaw: float = 0.0     # рыскание


@dataclass
class Setpoint:
    # то, что ControlStack передаёт стабилизатору: та же стик-команда c_*.
    c_fwd: float = 0.0
    c_right: float = 0.0
    c_yaw: float = 0.0
