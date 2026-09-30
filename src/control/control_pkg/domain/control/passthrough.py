#!/usr/bin/env python3
"""PilotPassthrough — легаси: сырые стики → RC. Выделен из stabilization.py."""
from ..attitude import AttitudeCommand
from ..setpoint import Setpoint
from ..state import DroneState
from ..units import tilt_of_pwm, yaw_of_pwm
from .base import StabilizationStrategy


class PilotPassthrough(StabilizationStrategy):
    """Легаси: сырые стики → RC (per-axis модель делает manual = ПУСТОЙ список)."""
    axes = frozenset({"roll", "pitch", "yaw"})

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> AttitudeCommand:
        return AttitudeCommand(roll=tilt_of_pwm(s.pilot_roll), pitch=tilt_of_pwm(s.pilot_pitch),
                               yaw_rate=yaw_of_pwm(s.pilot_yaw))
