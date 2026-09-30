#!/usr/bin/env python3
"""Единицы домена управления: СИ внутри, PWM-эквивалент на границе — один масштаб на всех.

ЗАЧЕМ (перевод домена в СИ, 2026-09-30). Стабилизаторы считают в радианах (наклон) и рад/с
(курс), гейны в конфиге и профилях — в градусах (ключи с суффиксом _DEG), в коде — радианы.
Носитель команды между стабилизатором и стеком пока прежний — RcCommand (µs от центра 1500):
по нему идут стек, арбитр, override, отладочные топики и статус. Переход СИ ↔ µs — ТОЛЬКО
через этот модуль, фиксированным масштабом варианта A (решение пилота 2026-09-30):

  наклон  20° на 500 µs  (0.040 °/µs) — ANGLE_SPAN_DEG
  курс    90 °/с на 500 µs (0.18 °/с на µs) — YAW_SPAN_DPS

Тот же масштаб пересчитывает µs в углы в канале SET_ATTITUDE_TARGET (att_convert.py), поэтому
PWM-эквивалент в статусе/HUD и угол, который получил полётник, — одно и то же число.
"""
import math

PWM_SPAN = 500.0
ANGLE_SPAN_DEG = 20.0                 # 500 µs → 20°
YAW_SPAN_DPS = 90.0                   # 500 µs → 90 °/с

DEG_PER_US = ANGLE_SPAN_DEG / PWM_SPAN            # 0.04 °/µs
RAD_PER_US = math.radians(ANGLE_SPAN_DEG) / PWM_SPAN
YAW_DPS_PER_US = YAW_SPAN_DPS / PWM_SPAN          # 0.18 °/с на µs
YAW_RAD_PER_US = math.radians(YAW_SPAN_DPS) / PWM_SPAN


def us_from_tilt(rad: float) -> float:
    """Наклон, рад → µs от центра (float, PWM-эквивалент)."""
    return rad / RAD_PER_US


def tilt_from_us(us: float) -> float:
    """µs от центра → наклон, рад."""
    return us * RAD_PER_US


def us_from_yaw(rad_s: float) -> float:
    """Темп курса, рад/с → µs от центра (float)."""
    return rad_s / YAW_RAD_PER_US


def yaw_from_us(us: float) -> float:
    """µs от центра → темп курса, рад/с."""
    return us * YAW_RAD_PER_US


def rc_off_yaw(rad_s: float) -> int:
    """Темп курса, рад/с → целое смещение канала, µs (усечение, как rc_off_tilt)."""
    return int(round(rad_s / YAW_RAD_PER_US, 9))


def rc_off_tilt(rad: float) -> int:
    """Наклон, рад → целое смещение канала от центра, µs — усечение к нулю, как прежнее
    int(out) стабилизаторов. Округление до 1e-9 µs снимает шум деления: потолок 6° обязан
    дать ровно 150, а не 149.99999999999997 → 149."""
    return int(round(rad / RAD_PER_US, 9))
