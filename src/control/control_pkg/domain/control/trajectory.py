#!/usr/bin/env python3
"""Стратегии движения (Trajectory) — выдают НАМЕРЕНИЕ (смещение уставки, тело).

Срез 1:
- StaticSetpoint — чистый холд (нулевое смещение).
- Shuttle — челнок: сетпойнт едет const-V вдоль оси (боковой или продольный),
  gz-hold его отслеживает (дрон летит с нужной скоростью, позиц. ОС не даёт
  runaway). Профиль 0→−A→0 (плечо A/V сек) с паузой на −A. Малое ускорение →
  малый крен → чистая трансляция на крейсере (калибровка масштаба s).
"""
from ..rc import RC_CENTER
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


class RcTransmitter(TrajectoryStrategy):
    """СРЕЗ 2: реальный пульт как ИСТОЧНИК НАМЕРЕНИЯ (assisted-режим).

    Стик = СКОРОСТЬ уставки (как ArduPilot Loiter): держишь вперёд → уставка едет
    вперёд с const-V → стабилизатор (gz/vins-hold) ведёт дрон; отпустил в центр →
    уставка стоит → дрон тормозит и висит. Интегрируем нормированный стик по dt →
    смещение (тело). Переиспользует ВЕСЬ каркас среза 1 (ControlStack + стабилизатор):
    единственное отличие от Shuttle — смещение приходит от ЖИВЫХ стиков, не от профиля.

    Sim: стики от ScriptedPilot; боевой борт: те же поля DroneState от RosPilot
    (/mavros/rc/in, стики двигает человек) — домен идентичен.
    """

    def __init__(self, vel_gain=0.8, deadzone=30, full_pwm=400,
                 pitch_sign=1.0, roll_sign=1.0):
        self.vel_gain = vel_gain      # м/с при полном отклонении стика
        self.deadzone = deadzone      # мёртвая зона вокруг центра, PWM
        self.full = full_pwm          # полное отклонение стика от центра, PWM
        self.psign = pitch_sign       # знак «стик вперёд» (борт: сверить с радио)
        self.rsign = roll_sign
        self._d_fwd = 0.0
        self._d_right = 0.0
        self._prev_t = None

    def _axis(self, pwm: int) -> float:
        """PWM-стик → нормировка [-1..1] с мёртвой зоной вокруг центра."""
        e = pwm - RC_CENTER
        if abs(e) < self.deadzone:
            return 0.0
        e -= self.deadzone * (1 if e > 0 else -1)
        return max(-1.0, min(1.0, e / max(1.0, self.full - self.deadzone)))

    def intent(self, s: DroneState, t: float) -> MotionIntent:
        dt = 0.0 if self._prev_t is None else max(0.0, t - self._prev_t)
        self._prev_t = t
        fwd = self.psign * self._axis(s.pilot_pitch)
        rgt = self.rsign * self._axis(s.pilot_roll)
        self._d_fwd += fwd * self.vel_gain * dt      # интеграл стик→смещение уставки
        self._d_right += rgt * self.vel_gain * dt
        return MotionIntent(d_fwd=self._d_fwd, d_right=self._d_right)
