#!/usr/bin/env python3
"""PilotPassthrough — легаси: сырые стики → RC. Выделен из stabilization.py."""
from ..rc import RC_CENTER, RcCommand
from ..setpoint import Setpoint
from ..state import DroneState
from .base import StabilizationStrategy


class PilotPassthrough(StabilizationStrategy):
    """Легаси: сырые стики → RC (per-axis модель делает manual = ПУСТОЙ список)."""
    axes = frozenset({"roll", "pitch", "yaw"})

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        return RcCommand(roll=s.pilot_roll, pitch=s.pilot_pitch,
                         throttle=RC_CENTER, yaw=s.pilot_yaw)
