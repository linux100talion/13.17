#!/usr/bin/env python3
"""VinsHold — position-hold по VINS (после init, своя опора в vins-фрейме).

Выделен из stabilization.py (там — реэкспорт).
"""
import math

from ..rc import RC_CENTER, RcCommand, clamp
from ..setpoint import Setpoint
from ..state import DroneState
from .base import StabilizationStrategy


class VinsHold(StabilizationStrategy):
    """Position-hold по VINS — после init (рантайм switch Flow→Vins). Своя опора в
    vins-фрейме (захват в enter() на момент switch; ControlStack-origin в gt не годится,
    на борту gt=0). VINS-фрейм не выровнен к миру — для УДЕРЖАНИЯ неважно."""
    axes = frozenset({"roll", "pitch"})

    def __init__(self, kp=40.0, kd=120.0, ki=8.0, imax=100.0, max_pwm=150.0,
                 psign=1.0, rsign=1.0, cmd_gain=0.8):
        self.kp, self.kd, self.ki = kp, kd, ki
        self.imax, self.max = imax, max_pwm
        self.psign, self.rsign = psign, rsign
        self.cmd_gain = cmd_gain
        self._spx = self._spy = 0.0        # интеграл стик-команды → уставка (vins-опора)
        self._ix = self._iy = 0.0
        self._it = None

    def enter(self, s: DroneState) -> None:
        self._spx, self._spy = s.vins_x, s.vins_y
        self._ix = self._iy = 0.0
        self._it = s.now_sim

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        # проекция стик-команды по ТЕКУЩЕМУ vins-курсу (тело) — как в GzHold:
        # «вперёд» = куда сейчас смотрит нос, а не куда смотрел на входе в фазу
        c0 = math.cos(s.vins_yaw)
        s0 = math.sin(s.vins_yaw)
        self._spx += (sp.c_fwd * c0 + sp.c_right * s0) * self.cmd_gain * dt
        self._spy += (sp.c_fwd * s0 - sp.c_right * c0) * self.cmd_gain * dt
        ex = s.vins_x - self._spx
        ey = s.vins_y - self._spy
        now = s.now_sim
        if self.ki > 0 and self._it is not None and now > self._it:
            di = now - self._it
            self._ix += ex * di
            self._iy += ey * di
            cap = self.imax / self.ki
            self._ix = clamp(self._ix, -cap, cap)
            self._iy = clamp(self._iy, -cap, cap)
        self._it = now
        c = math.cos(s.vins_yaw)
        sn = math.sin(s.vins_yaw)
        e_fwd = ex * c + ey * sn
        e_rgt = -ex * sn + ey * c
        v_fwd = s.vins_vx * c + s.vins_vy * sn
        v_rgt = -s.vins_vx * sn + s.vins_vy * c
        i_fwd = self._ix * c + self._iy * sn
        i_rgt = -self._ix * sn + self._iy * c
        po = self.psign * (self.kp * e_fwd + self.kd * v_fwd + self.ki * i_fwd)
        ro = self.rsign * (self.kp * e_rgt + self.kd * v_rgt + self.ki * i_rgt)
        po = clamp(po, -self.max, self.max)
        ro = clamp(ro, -self.max, self.max)
        return RcCommand(roll=RC_CENTER + int(ro), pitch=RC_CENTER + int(po),
                         throttle=RC_CENTER, yaw=RC_CENTER)
