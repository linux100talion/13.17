#!/usr/bin/env python3
"""AttitudeCommand — выход домена в УГЛАХ (переезд с override на SET_ATTITUDE_TARGET).

Чистый домен: ноль импортов rclpy/mavros. Единицы СИ, соглашения ArduPilot (оси тела
FRD, как в логах полётника и в его коде): крен вправо +, тангаж нос ВВЕРХ + (стик
«вперёд» = отрицательный тангаж), курс по часовой +, набор вверх +. В оси ROS
(FLU/ENU) переводит адаптер MAVROS — домену рамы ROS знать не нужно.

Шаг 1 переезда (laptop_move.md §5.7): домен по-прежнему считает в PWM, а RcCommand →
AttitudeCommand пересчитывает infrastructure/att_convert.py так же, как ALT_HOLD
пересчитывал наш override (минус мёртвая зона — она стикам пилота, не стабилизатору).
"""
from dataclasses import dataclass


@dataclass
class AttitudeCommand:
    roll: float = 0.0        # рад, вправо +
    pitch: float = 0.0       # рад, нос вверх +
    yaw_rate: float = 0.0    # рад/с, по часовой +
    climb: float = 0.0       # м/с, вверх +
