#!/usr/bin/env python3
"""Стратегии возбуждения (Excitation) — экзогенный зонд для system-ID.

Срез 1: только NoExcitation (заглушка). Pulse/Chirp/Translate придут срезом 2/3;
контракт offset()→{ось:(pwm,policy)} уже позволяет их подмешать (ADDITIVE/REPLACE)
без правки ControlStack.
"""
from ..state import DroneState
from .base import ExcitationStrategy


class NoExcitation(ExcitationStrategy):
    def offset(self, s: DroneState, t: float) -> dict:
        return {}
