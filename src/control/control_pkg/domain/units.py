#!/usr/bin/env python3
"""Единицы домена управления: СИ внутри, PWM-эквивалент на границе — один масштаб на всех.

ЗАЧЕМ (перевод домена в СИ, 2026-09-30). Стабилизаторы считают в радианах (наклон) и рад/с
(курс), гейны в конфиге и профилях — в градусах (ключи с суффиксом _DEG), в коде — радианы.
Носитель команды — AttitudeCommand (СИ, domain/attitude.py; с фазы A 2026-09-30, до неё
RcCommand в µs). Переход СИ ↔ µs — ТОЛЬКО через этот модуль, фиксированным масштабом
варианта A (решение пилота 2026-09-30):

  наклон  20° на 500 µs  (0.040 °/µs) — ANGLE_SPAN_DEG
  курс    90 °/с на 500 µs (0.18 °/с на µs) — YAW_SPAN_DPS

Тот же масштаб пересчитывает µs в углы в канале SET_ATTITUDE_TARGET (att_convert.py), поэтому
PWM-эквивалент в статусе/HUD и угол, который получил полётник, — одно и то же число.
"""
import math

PWM_CENTER = 1500
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


# ── СЕТКА µs (фаза A переезда носителя в СИ) ──────────────────────────────────────────
# Выход стабилизаторов и стека до переезда был ЦЕЛЫМ числом µs (усечение int()), и
# ограничитель скорости стека считал в целых. Чтобы носитель в СИ дал проводу ровно те же
# µs, значения живут на той же сетке: q_tilt/q_yaw — усечение к целому µs, как прежнее
# int(). Снять сетку (выход без квантования) — отдельное осознанное изменение поведения.

def q_tilt(rad: float) -> float:
    """Наклон, рад → тот же наклон на сетке целых µs (усечение к нулю, как int())."""
    return rc_off_tilt(rad) * RAD_PER_US


def q_yaw(rad_s: float) -> float:
    """Темп курса, рад/с → на сетке целых µs (усечение к нулю)."""
    return rc_off_yaw(rad_s) * YAW_RAD_PER_US


def tilt_of_pwm(pwm: float) -> float:
    """Абсолютный PWM канала крена/тангажа (стик пилота) → наклон, рад."""
    return (pwm - PWM_CENTER) * RAD_PER_US


def yaw_of_pwm(pwm: float) -> float:
    """Абсолютный PWM канала курса (стик пилота) → темп, рад/с."""
    return (pwm - PWM_CENTER) * YAW_RAD_PER_US
