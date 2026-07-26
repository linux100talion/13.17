#!/usr/bin/env python3
"""Arbiter — SAFETY-супервизор: пилот выхватывает управление БЕЗУСЛОВНО.

Стоит МЕЖДУ выходом миссии и RcOutput. Если тумблер пульта в MANUAL — отдаём сырые
стики пилота (включая throttle — пилоту нужна полная власть), что бы миссия ни
командовала. Независим от того, какие стратегии выбраны в ControlStack.

На БОЕВОМ борту это не единственный барьер: FLTMODE_CH остаётся ВКЛючённым (в отличие
от сима, где мы его обнулили), чтобы пилот мог сорвать наш override и на уровне FCU.
Arbiter — верхний, программный слой той же гарантии.
"""
from ..domain.rc import RcCommand
from ..domain.state import DroneState

PILOT_AUTO = 0
PILOT_MANUAL = 1


class Arbiter:
    def __init__(self, manual_value: int = PILOT_MANUAL):
        self.manual_value = manual_value
        self.last_manual = False        # для лога смены авто↔ручной (снаружи)

    def resolve(self, s: DroneState, autonomous: RcCommand) -> RcCommand:
        manual = (s.pilot_switch == self.manual_value)
        self.last_manual = manual
        if manual:
            # Полная власть пилоту, включая throttle.
            return RcCommand(roll=s.pilot_roll, pitch=s.pilot_pitch,
                             throttle=s.pilot_throttle, yaw=s.pilot_yaw)
        return autonomous
