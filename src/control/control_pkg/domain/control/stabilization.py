#!/usr/bin/env python3
"""Стратегии стабилизации. Срез 1: GzPositionHold (PID по истинной позе Gazebo).

Перенос закона из alt_hold_bootstrap.py (S_EXCITE / gz-hold). Ошибку и скорость из
world переводим в тело (по gt_yaw) → offset PWM по pitch(вперёд)/roll(вправо).
I-член интегрируется в WORLD (yaw-инвариантно), потом поворачивается в тело; знаки
psign/rsign=+1 выверены отладкой монолита (pitch_off<0 → ускорение ВПЕРЁД).
"""
import math

from ..rc import RC_CENTER, RcCommand, clamp
from ..setpoint import Setpoint
from ..state import DroneState
from .base import StabilizationStrategy


class GzPositionHold(StabilizationStrategy):
    axes = frozenset({"roll", "pitch"})   # yaw держит отдельная роль/центр

    def __init__(self, kp=40.0, kd=120.0, ki=8.0, imax=100.0, max_pwm=150.0,
                 psign=1.0, rsign=1.0):
        self.kp, self.kd, self.ki = kp, kd, ki
        self.imax, self.max = imax, max_pwm
        self.psign, self.rsign = psign, rsign
        self._ix = self._iy = 0.0          # интеграл ошибки позиции (world)
        self._it = None                    # пред. sim-время для dt интеграла

    def enter(self, s: DroneState) -> None:
        # Реюз hold-only-каркаса: сброс интегратора при входе в фазу.
        self._ix = self._iy = 0.0
        self._it = s.now_sim

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        ex = s.gt_x - sp.x
        ey = s.gt_y - sp.y
        # I-член: интегрируем в WORLD; anti-windup — кламп состояния так, чтобы
        # вклад ki*i не превышал imax PWM по каждой оси.
        now = s.now_sim
        if self.ki > 0 and self._it is not None and now > self._it:
            di = now - self._it
            self._ix += ex * di
            self._iy += ey * di
            cap = self.imax / self.ki
            self._ix = clamp(self._ix, -cap, cap)
            self._iy = clamp(self._iy, -cap, cap)
        self._it = now
        c = math.cos(s.gt_yaw)
        sn = math.sin(s.gt_yaw)
        e_fwd = ex * c + ey * sn
        e_rgt = -ex * sn + ey * c
        v_fwd = s.gt_vx * c + s.gt_vy * sn
        v_rgt = -s.gt_vx * sn + s.gt_vy * c
        i_fwd = self._ix * c + self._iy * sn
        i_rgt = -self._ix * sn + self._iy * c
        po = self.psign * (self.kp * e_fwd + self.kd * v_fwd + self.ki * i_fwd)
        ro = self.rsign * (self.kp * e_rgt + self.kd * v_rgt + self.ki * i_rgt)
        po = clamp(po, -self.max, self.max)
        ro = clamp(ro, -self.max, self.max)
        return RcCommand(roll=RC_CENTER + int(ro), pitch=RC_CENTER + int(po),
                         throttle=RC_CENTER, yaw=RC_CENTER)


class PilotPassthrough(StabilizationStrategy):
    """СРЕЗ 2: полный РУЧНОЙ режим — сырые стики пилота → RC, обратной связи НЕТ.

    «Стабилизация» вырождена: пилот сам в контуре (как ACRO/STABILIZE аппарата).
    Уставку (sp) игнорирует — отслеживать нечего. throttle центр (миссия держит
    высоту в EXCITE; при seize пилоту throttle отдаёт Arbiter). Читает pilot_* из
    DroneState — sim (ScriptedPilot) и борт (RosPilot) одинаково.
    """
    axes = frozenset({"roll", "pitch", "yaw"})

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        return RcCommand(roll=s.pilot_roll, pitch=s.pilot_pitch,
                         throttle=RC_CENTER, yaw=s.pilot_yaw)
