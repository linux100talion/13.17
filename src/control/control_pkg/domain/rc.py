#!/usr/bin/env python3
"""RcCommand + RC-константы — базовый value object управления.

Чистый домен: ноль импортов rclpy/mavros/cv. Каналы 1..4 ArduCopter =
roll/pitch/throttle/yaw. Значения — «сырой» PWM (µs), центр 1500.
"""
from dataclasses import dataclass

# Центр стика; для throttle в ALT_HOLD центр = «держать высоту».
RC_CENTER = 1500
# Газ в минимум — нужно для арминга в ALT_HOLD.
RC_MIN_THR = 1000
# 65535 (UINT16_MAX) = «ИГНОРИРОВАТЬ канал» в OverrideRCIn (не оверрайдим канал).
RC_NOCHANGE = 65535


@dataclass
class RcCommand:
    roll: int = RC_CENTER
    pitch: int = RC_CENTER
    throttle: int = RC_CENTER
    yaw: int = RC_CENTER


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x
