#!/usr/bin/env python3
"""Стратегии движения (Trajectory) — выдают НАМЕРЕНИЕ (смещение уставки, тело).

Срез 1:
- StaticSetpoint — чистый холд (нулевое смещение).
- Shuttle — челнок: сетпойнт едет const-V вдоль оси (боковой или продольный),
  gz-hold его отслеживает (дрон летит с нужной скоростью, позиц. ОС не даёт
  runaway). Профиль 0→−A→0 (плечо A/V сек) с паузой на −A. Малое ускорение →
  малый крен → чистая трансляция на крейсере (калибровка масштаба s).
"""
from ..setpoint import MotionIntent
from ..state import DroneState
from .base import TrajectoryStrategy


class StaticSetpoint(TrajectoryStrategy):
    def intent(self, s: DroneState, t: float) -> MotionIntent:
        return MotionIntent(0.0, 0.0)


class Shuttle(TrajectoryStrategy):
    def __init__(self, amplitude=5.0, velocity=1.5, pause=2.0, forward=False):
        self.a = amplitude
        self.v = max(0.1, velocity)
        self.pause = pause
        self.forward = forward   # True → продольный (looming/pitch-ID), иначе боковой

    def _offset(self, ts: float) -> float:
        """Смещение вдоль оси (м): 0→−A (плечо), пауза на −A, −A→0. Затем 0."""
        tleg = self.a / self.v
        if ts < tleg:                       # к −A
            return -self.v * ts
        if ts < tleg + self.pause:          # пауза на −A
            return -self.a
        if ts < 2.0 * tleg + self.pause:    # назад к 0
            return -self.a + self.v * (ts - (tleg + self.pause))
        return 0.0                          # готово

    def total(self) -> float:
        return 2.0 * (self.a / self.v) + 2.0 * self.pause   # 2 плеча + паузы

    def intent(self, s: DroneState, t: float) -> MotionIntent:
        d = self._offset(t)
        if self.forward:
            # ПРОДОЛЬНЫЙ: −d → вперёд-первым (d идёт 0→−A→0 → d_fwd 0→+A→0).
            return MotionIntent(d_fwd=-d, d_right=0.0)
        # боковой: 0→−A (влево), пауза, −A→0 (вправо).
        return MotionIntent(d_fwd=0.0, d_right=d)

    def done(self, t: float) -> bool:
        # +1.5 sim-сек успокоения после последовательности (как в монолите).
        return t > self.total() + 1.5
