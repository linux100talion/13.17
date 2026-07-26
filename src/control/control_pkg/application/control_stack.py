#!/usr/bin/env python3
"""ControlStack — композиция ролей (Trajectory→[Stabilization…]→Excitation) в RcCommand.

PER-AXIS модель (срез 3): стабилизаторов может быть НЕСКОЛЬКО, каждый владеет своими
осями (`axes`). Композиция:
  1) БАЗА = сырые стики пилота (незанятая ось → ручной наклон);
  2) каждый стабилизатор ПЕРЕЗАПИСЫВАЕТ свои оси (regulate/velocity-assist);
  3) Excitation подмешивается сверху (ADDITIVE/REPLACE).
Так «пульт + только yaw» = [DpYawHold] (yaw держит, roll/pitch пилот); «пульт + flow(roll)»
= [DpRollHold] (roll velocity-assist, pitch/yaw пилот). Manual = [] (всё пилоту).

Владеет ТОЧКОЙ ВХОДА (origin/yaw0/t0): Trajectory отдаёт смещение относительно входа
(тело), стек собирает абсолютную world-уставку + прокидывает скорость-команду в Setpoint.
Самодостаточен — работает и без MissionRunner.
"""
import math

from ..domain.rc import RC_CENTER, RcCommand
from ..domain.setpoint import AxisPolicy, MotionIntent, Setpoint


def _as_list(stab):
    if stab is None:
        return []
    return list(stab) if isinstance(stab, (list, tuple)) else [stab]


def _compose(rc: RcCommand, axis: str, off: int, policy: AxisPolicy) -> RcCommand:
    cur = getattr(rc, axis)
    if policy is AxisPolicy.ADDITIVE:
        setattr(rc, axis, cur + off)          # зонд ПОВЕРХ выхода стабилизатора
    elif policy is AxisPolicy.REPLACE:
        setattr(rc, axis, RC_CENTER + off)    # зонд ВЫТЕСНЯЕТ стабилизатор
    return rc


class ControlStack:
    def __init__(self, stabilization, trajectory, excitation):
        self.stabs = _as_list(stabilization)   # список стабилизаторов (может быть пуст)
        self.traj = trajectory
        self.excite = excitation
        self._ox = self._oy = 0.0
        self._yaw0 = 0.0
        self._t0 = None
        self._prev_t = None

    # --- горячая замена стратегий (per-axis: stabilization = один или список) ---
    def switch_stabilization(self, s): self.stabs = _as_list(s)
    def switch_trajectory(self, t): self.traj = t
    def switch_excitation(self, e): self.excite = e

    def enter(self, s):
        self._ox, self._oy = s.gt_x, s.gt_y
        self._yaw0 = s.gt_yaw
        self._t0 = s.now_sim
        self._prev_t = s.now_sim
        for st in self.stabs:
            st.enter(s)

    def _origin_plus(self, intent: MotionIntent) -> Setpoint:
        # тело→world по yaw входа: fwd=(cos,sin), right=(sin,−cos). Скорость-команда
        # (c_*) — в теле, прокидывается как есть (velocity-стабилизаторы её масштабируют).
        c = math.cos(self._yaw0)
        sn = math.sin(self._yaw0)
        dx = intent.d_fwd * c + intent.d_right * sn
        dy = intent.d_fwd * sn - intent.d_right * c
        return Setpoint(self._ox + dx, self._oy + dy,
                        c_fwd=intent.c_fwd, c_right=intent.c_right, c_yaw=intent.c_yaw)

    def update(self, s) -> RcCommand:
        if self._t0 is None:
            self.enter(s)
        t = s.now_sim - self._t0
        dt = max(0.0, s.now_sim - (self._prev_t if self._prev_t is not None else s.now_sim))
        self._prev_t = s.now_sim
        intent = self.traj.intent(s, t)
        sp = self._origin_plus(intent)
        # БАЗА — сырые стики пилота (незанятые оси = ручной наклон). throttle держит миссия.
        rc = RcCommand(roll=s.pilot_roll, pitch=s.pilot_pitch,
                       throttle=RC_CENTER, yaw=s.pilot_yaw)
        # каждый стабилизатор перезаписывает СВОИ оси
        for st in self.stabs:
            out = st.update(s, sp, dt)
            for ax in st.axes:
                setattr(rc, ax, getattr(out, ax))
        for axis, (off, pol) in self.excite.offset(s, t).items():
            rc = _compose(rc, axis, off, pol)
        return rc

    def motion_done(self) -> bool:
        t = 0.0 if self._t0 is None else (self._prev_t - self._t0)
        return self.traj.done(t)

    def excite_done(self) -> bool:
        t = 0.0 if self._t0 is None else (self._prev_t - self._t0)
        return self.excite.done(t)
