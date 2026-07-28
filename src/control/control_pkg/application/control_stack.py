#!/usr/bin/env python3
"""ControlStack — композиция ролей (Trajectory→[Stabilization…]→Excitation) в RcCommand.

PER-AXIS модель (срез 3): стабилизаторов может быть НЕСКОЛЬКО, каждый владеет своими
осями (`axes`). Композиция:
  1) БАЗА = НАМЕРЕНИЕ ТРАЕКТОРИИ (оператор): c_*→PWM, незанятая ось = открытый контур
     (сырой стик оператора). Оператор взаимозаменяем: скрипт (ConstProfile/Shuttle) /
     живой пульт (RcTransmitter читает s.pilot_*) / (будущее) NN2 — profile-only до конца;
  2) каждый стабилизатор ПЕРЕЗАПИСЫВАЕТ свои оси (regulate/velocity-assist);
  3) Excitation подмешивается сверху (ADDITIVE/REPLACE).
Так «пульт + только yaw» = [DpYawHold] (yaw держит, roll/pitch = профиль-оператор); боевой
пре-VINS = [DpRollHold,DpYawHold] (roll/yaw демпфер, pitch = оператор). Manual = [] (всё
оператору через RcTransmitter). Живой пилот входит ТОЛЬКО как Trajectory (RcTransmitter),
не как отдельная база — единый «язык» намерения c_*.

Владеет ТОЧКОЙ ВХОДА (origin/yaw0/t0): Trajectory отдаёт смещение относительно входа
(тело), стек собирает абсолютную world-уставку + прокидывает скорость-команду в Setpoint.
Самодостаточен — работает и без MissionRunner.
"""
from ..domain.rc import RC_CENTER, RcCommand, clamp
from ..domain.setpoint import AxisPolicy, Setpoint

_STICK_SPAN = 400   # PWM от центра при полном стике (c=±1) — конвенция pilot_full

# Знак продольного канала на ПРОВОДЕ. Домен говорит «c_fwd=+1 = лететь ВПЕРЁД», ArduPilot
# на RC2 говорит «PWM выше центра = стик на себя = нос ВВЕРХ = лететь НАЗАД». Значит
# намерение «вперёд» обязано уехать НИЖЕ центра. Здесь единственное место перевода
# намерения в провод, поэтому знак живёт здесь, а не размазан по стратегиям.
# Измерено (bag gzprobe_run4/nostab_run5): roll +0.060 °/PWM corr +0.97 — то есть
# c_right=+1 → PWM 1900 → крен вправо, знак верен; pitch шёл через ту же формулу и давал
# −0.037 °/PWM corr −0.58, то есть противоположно намерению. См. src/control/ToDo.md, факт 1.
_PITCH_RC_SIGN = -1.0


def _cmd_to_pwm(c: float) -> int:
    """Намерение оператора c_* [-1..1] → PWM открытого контура (наклон стика)."""
    return int(clamp(RC_CENTER + c * _STICK_SPAN,
                     RC_CENTER - _STICK_SPAN, RC_CENTER + _STICK_SPAN))


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
        self._t0 = None
        self._prev_t = None

    # --- горячая замена стратегий (per-axis: stabilization = один или список) ---
    def switch_stabilization(self, s): self.stabs = _as_list(s)
    def switch_trajectory(self, t): self.traj = t
    def switch_excitation(self, e): self.excite = e

    def enter(self, s):
        # Опору держит теперь КАЖДЫЙ позиц-холдер сам (в своём фрейме) — стек лишь
        # тактирует время траектории (t0) и раздаёт stab.enter.
        self._t0 = s.now_sim
        self._prev_t = s.now_sim
        for st in self.stabs:
            st.enter(s)

    def update(self, s) -> RcCommand:
        if self._t0 is None:
            self.enter(s)
        t = s.now_sim - self._t0
        dt = max(0.0, s.now_sim - (self._prev_t if self._prev_t is not None else s.now_sim))
        self._prev_t = s.now_sim
        intent = self.traj.intent(s, t)
        sp = Setpoint(intent.c_fwd, intent.c_right, intent.c_yaw)   # стик-команда → стабилизаторам
        # БАЗА — намерение траектории (оператор): c_*→PWM. Незанятая ось = открытый контур
        # (наклон оператора). throttle держит миссия. Живой пилот входит через RcTransmitter.
        rc = RcCommand(roll=_cmd_to_pwm(intent.c_right),
                       pitch=_cmd_to_pwm(_PITCH_RC_SIGN * intent.c_fwd),
                       throttle=RC_CENTER, yaw=_cmd_to_pwm(intent.c_yaw))
        # ⚠️ Стабилизаторы (Gz*/Dp*) пишут PWM НАПРЯМУЮ, минуя этот перевод: их знаки
        # (gz_psign, pitch_osign) заданы уже в проводной конвенции и здесь не участвуют.
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
