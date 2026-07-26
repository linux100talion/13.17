#!/usr/bin/env python3
"""ControlStack — композиция трёх ролей (Trajectory→Stabilization→Excitation) в RcCommand.

Сведённые StabilizationManager + MotionManager + Excitation-слот из наброска. Три
слота переключаются в рантайме (switch_*). Владеет ТОЧКОЙ ВХОДА (origin/yaw0/t0):
Trajectory отдаёт смещение относительно входа (тело), стабилизатор — абсолютную
world-уставку; сборка origin+intent здесь, а не размазана по стратегиям.

Самодостаточен — работает и БЕЗ MissionRunner (bare loiter-ассист).
"""
import math

from ..domain.rc import RC_CENTER, RcCommand
from ..domain.setpoint import AxisPolicy, MotionIntent, Setpoint


def _compose(rc: RcCommand, axis: str, off: int, policy: AxisPolicy) -> RcCommand:
    cur = getattr(rc, axis)
    if policy is AxisPolicy.ADDITIVE:
        setattr(rc, axis, cur + off)          # зонд ПОВЕРХ выхода стабилизатора
    elif policy is AxisPolicy.REPLACE:
        setattr(rc, axis, RC_CENTER + off)    # зонд ВЫТЕСНЯЕТ стабилизатор
    return rc


class ControlStack:
    def __init__(self, stabilization, trajectory, excitation):
        self.stab = stabilization
        self.traj = trajectory
        self.excite = excitation
        self._ox = self._oy = 0.0     # origin (world)
        self._yaw0 = 0.0              # yaw входа (ось проекции намерения)
        self._t0 = None              # sim-время входа в фазу
        self._prev_t = None          # для dt стабилизатора

    # --- горячая замена стратегий ---
    def switch_stabilization(self, s): self.stab = s
    def switch_trajectory(self, t): self.traj = t
    def switch_excitation(self, e): self.excite = e

    def enter(self, s):
        """Захват точки входа + сброс интеграторов стратегий (реюз hold-каркаса)."""
        self._ox, self._oy = s.gt_x, s.gt_y
        self._yaw0 = s.gt_yaw
        self._t0 = s.now_sim
        self._prev_t = s.now_sim
        self.stab.enter(s)

    def _origin_plus(self, intent: MotionIntent) -> Setpoint:
        # тело→world по yaw входа: fwd=(cos,sin), right=(sin,−cos).
        c = math.cos(self._yaw0)
        sn = math.sin(self._yaw0)
        dx = intent.d_fwd * c + intent.d_right * sn
        dy = intent.d_fwd * sn - intent.d_right * c
        return Setpoint(self._ox + dx, self._oy + dy)

    def update(self, s) -> RcCommand:
        if self._t0 is None:
            self.enter(s)
        t = s.now_sim - self._t0
        dt = max(0.0, s.now_sim - (self._prev_t if self._prev_t is not None else s.now_sim))
        self._prev_t = s.now_sim
        intent = self.traj.intent(s, t)
        sp = self._origin_plus(intent)
        rc = self.stab.update(s, sp, dt)
        for axis, (off, pol) in self.excite.offset(s, t).items():
            rc = _compose(rc, axis, off, pol)
        return rc

    def motion_done(self) -> bool:
        t = 0.0 if self._t0 is None else (self._prev_t - self._t0)
        return self.traj.done(t)

    def excite_done(self) -> bool:
        t = 0.0 if self._t0 is None else (self._prev_t - self._t0)
        return self.excite.done(t)
