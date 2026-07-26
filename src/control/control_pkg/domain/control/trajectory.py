#!/usr/bin/env python3
"""Стратегии движения (Trajectory) — ПРОФИЛЬ-ГЕНЕРАТОРЫ.

PROFILE-ONLY: каждая выдаёт нормированный стик-уровень `MotionIntent(c_*)` — единый
«язык» движения. Метрики нет. Любой профиль → любой стек (позиц-холдер интегрирует,
демпфер берёт целью). Источник намерения взаимозаменяем: скрипт / пилот / (будущее) NN2.

- StaticSetpoint — нули (держать/гасить на месте).
- Shuttle — челнок как стик-профиль: ±level по плечам (const-стик), пауза, возврат.
- RcTransmitter — живой пилот = живой профиль (нормир. стики /mavros/rc/in).
- ProfileTrajectory — скриптовый профиль по sim-времени (сегменты уровень×длительность).
- ConstProfile — постоянная стик-команда c_* на dur сек (атом профиль-миссии mv_*/hover).
"""
from ..rc import RC_CENTER
from ..setpoint import MotionIntent
from ..state import DroneState
from .base import TrajectoryStrategy


def _norm_axis(pwm: int, deadzone: int, full: int) -> float:
    """PWM-стик → нормировка [-1..1] с мёртвой зоной вокруг центра."""
    e = pwm - RC_CENTER
    if abs(e) < deadzone:
        return 0.0
    e -= deadzone * (1 if e > 0 else -1)
    return max(-1.0, min(1.0, e / max(1.0, full - deadzone)))


class StaticSetpoint(TrajectoryStrategy):
    def intent(self, s: DroneState, t: float) -> MotionIntent:
        return MotionIntent()


class Shuttle(TrajectoryStrategy):
    """Челнок как стик-профиль: −level плечо `leg` сек, пауза, +level плечо, стоп.
    Симметрия ± → интеграл уставки возвращается к старту. forward → продольная ось."""

    def __init__(self, level=0.3, leg=3.0, pause=2.0, forward=False):
        self.level = level
        self.leg = leg
        self.pause = pause
        self.forward = forward

    def _cmd(self, ts: float) -> float:
        if ts < self.leg:
            return -self.level
        if ts < self.leg + self.pause:
            return 0.0
        if ts < 2.0 * self.leg + self.pause:
            return self.level
        return 0.0

    def total(self) -> float:
        return 2.0 * self.leg + self.pause

    def intent(self, s: DroneState, t: float) -> MotionIntent:
        v = self._cmd(t)
        return MotionIntent(c_fwd=v) if self.forward else MotionIntent(c_right=v)

    def done(self, t: float) -> bool:
        return t > self.total() + 1.5


class RcTransmitter(TrajectoryStrategy):
    """Живой пульт как источник намерения: нормир. стики → c_*. Интеграла БОЛЬШЕ НЕТ
    (его делает позиц-холдер). Sim (ScriptedPilot) и борт (RosPilot) — те же pilot_*."""

    def __init__(self, deadzone=30, full_pwm=400, pitch_sign=1.0, roll_sign=1.0):
        self.deadzone = deadzone
        self.full = full_pwm
        self.psign = pitch_sign
        self.rsign = roll_sign

    def intent(self, s: DroneState, t: float) -> MotionIntent:
        return MotionIntent(
            c_fwd=self.psign * _norm_axis(s.pilot_pitch, self.deadzone, self.full),
            c_right=self.rsign * _norm_axis(s.pilot_roll, self.deadzone, self.full),
            c_yaw=_norm_axis(s.pilot_yaw, self.deadzone, self.full))


class ProfileTrajectory(TrajectoryStrategy):
    """Скриптовый стик-профиль по sim-времени. segments: (t_until, roll, pitch, yaw) в
    PWM; первый сегмент с t_until > t выигрывает, после последнего — центр. Универсальный
    источник движения (как ScriptedPilot, но как Trajectory) — любой профиль → любой стек."""

    def __init__(self, segments, deadzone=30, full_pwm=400,
                 pitch_sign=1.0, roll_sign=1.0):
        self.seg = segments
        self.deadzone = deadzone
        self.full = full_pwm
        self.psign = pitch_sign
        self.rsign = roll_sign

    def _sticks(self, t: float):
        for tu, r, p, y in self.seg:
            if t < tu:
                return r, p, y
        return RC_CENTER, RC_CENTER, RC_CENTER

    def total(self) -> float:
        return self.seg[-1][0] if self.seg else 0.0

    def intent(self, s: DroneState, t: float) -> MotionIntent:
        r, p, y = self._sticks(t)
        return MotionIntent(
            c_fwd=self.psign * _norm_axis(p, self.deadzone, self.full),
            c_right=self.rsign * _norm_axis(r, self.deadzone, self.full),
            c_yaw=_norm_axis(y, self.deadzone, self.full))

    def done(self, t: float) -> bool:
        return t > self.total()


class ConstProfile(TrajectoryStrategy):
    """Постоянная стик-команда c_* в течение dur sim-сек, затем done. Атом профиль-миссии:
    один mv_*/hover = один Control-сегмент. Прямые НОРМИРОВАННЫЕ единицы (без PWM-round-
    trip): mv_fwd → c_fwd=+level, hover → нули (стабилизатор гасит снос/держит позицию)."""

    def __init__(self, dur: float, c_fwd: float = 0.0, c_right: float = 0.0,
                 c_yaw: float = 0.0):
        self.dur = dur
        self._intent = MotionIntent(c_fwd=c_fwd, c_right=c_right, c_yaw=c_yaw)

    def intent(self, s: DroneState, t: float) -> MotionIntent:
        return self._intent

    def total(self) -> float:
        return self.dur

    def done(self, t: float) -> bool:
        return t > self.dur
