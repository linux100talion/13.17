#!/usr/bin/env python3
"""Arbiter — SAFETY-супервизор: пилот выхватывает управление БЕЗУСЛОВНО.

Стоит МЕЖДУ выходом миссии и RcOutput. Если тумблер пульта в MANUAL — отдаём сырые
стики пилота (включая throttle — пилоту нужна полная власть), что бы миссия ни
командовала. Независим от того, какие стратегии выбраны в ControlStack.

Газ — через ThrottleLatch: на каждом входе в MANUAL защёлка взводится, и газ пилота
не проходит (держим центр = «висеть» в ALT_HOLD), пока стик впервые не побывает
в центре. Иначе seize с отклонённым газом = немедленный уход по высоте (см.
docstring защёлки — случай полёта 2026-08-16). Подпружиненный газ открывает
защёлку первым же тиком; roll/pitch/yaw идут сырыми всегда.

На БОЕВОМ борту это не единственный барьер: FLTMODE_CH остаётся ВКЛючённым (в отличие
от сима, где мы его обнулили), чтобы пилот мог сорвать наш override и на уровне FCU.
Arbiter — верхний, программный слой той же гарантии.
"""
from ..domain.control.throttle_latch import ThrottleLatch
from ..domain.rc import RC_CENTER, RcCommand
from ..domain.state import DroneState

PILOT_AUTO = 0
PILOT_MANUAL = 1


class Arbiter:
    def __init__(self, manual_value: int = PILOT_MANUAL, thr_deadzone: int = 30):
        self.manual_value = manual_value
        self.last_manual = False        # для лога смены авто↔ручной (снаружи)
        self._latch = ThrottleLatch(thr_deadzone)

    def resolve(self, s: DroneState, autonomous: RcCommand) -> RcCommand:
        manual = (s.pilot_switch == self.manual_value)
        if manual and not self.last_manual:
            self._latch.reset()         # каждый щелчок в MANUAL — газ снова заперт
        self.last_manual = manual
        if manual:
            # Полная власть пилоту; газ — через защёлку (None = держим центр).
            thr = self._latch.pass_through(s.pilot_throttle)
            return RcCommand(roll=s.pilot_roll, pitch=s.pilot_pitch,
                             throttle=RC_CENTER if thr is None else thr,
                             yaw=s.pilot_yaw)
        return autonomous
