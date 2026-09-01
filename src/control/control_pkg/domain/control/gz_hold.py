#!/usr/bin/env python3
"""Gz* — position-hold по ground-truth Gazebo (sim-оракул для тюнинга).

Ошибку и скорость из world → в тело (по gt_yaw) → PWM по pitch(вперёд)/roll(вправо);
yaw — курс-холд к yaw входа. `GzHold(axes=…)` база; per-axis алиасы GzPosHold/
GzRollHold/GzPitchHold/GzYawHold. Арифметика roll/pitch выверена монолитом
(Δ=0 в test_gz_shuttle_equiv). Выделен из stabilization.py (там — реэкспорт).
"""
import math

from ..rc import RC_CENTER, RcCommand, clamp
from ..setpoint import Setpoint
from ..state import DroneState
from .base import StabilizationStrategy


class GzHold(StabilizationStrategy):
    """Position-hold по gt Gazebo, per-axis (axes задаёт, какие оси стек использует).

    roll/pitch — PID позиции (ошибка world→тело); yaw — курс-холд к yaw входа (P по
    heading; yaw-стик = rate → P даёт устойчивую сходимость). Незанятые оси стек
    игнорирует, поэтому вычисляем все три, а axes лишь маркирует владение.
    """

    def __init__(self, kp=40.0, kd=120.0, ki=8.0, imax=100.0, max_pwm=150.0,
                 psign=1.0, rsign=1.0, axes=frozenset({"roll", "pitch"}),
                 yaw_kp=80.0, yaw_sign=-1.0, cmd_gain=0.8, yaw_cmd_gain=0.5):
        self.kp, self.kd, self.ki = kp, kd, ki
        self.imax, self.max = imax, max_pwm
        self.psign, self.rsign = psign, rsign
        self.axes = axes
        self.yaw_kp, self.yaw_sign = yaw_kp, yaw_sign
        self.cmd_gain, self.yaw_cmd_gain = cmd_gain, yaw_cmd_gain
        self._ix = self._iy = 0.0          # интеграл ошибки позиции (world)
        self._it = None
        self._spx = self._spy = 0.0        # ИНТЕГРАЛ стик-команды → уставка (своя опора)
        self._yawsp = 0.0                  # интеграл yaw-стика → командный курс

    def enter(self, s: DroneState) -> None:
        self._ix = self._iy = 0.0
        self._it = s.now_sim
        self._spx, self._spy = s.gt_x, s.gt_y   # уставка стартует в опоре (gt на входе)
        self._yawsp = s.gt_yaw

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        # Стик-команду интегрируем в движущуюся уставку. Проекция — по ТЕКУЩЕМУ
        # курсу (тело): «стик от себя» = туда, куда СЕЙЧАС смотрит нос/камера, как
        # в Loiter реального ArduPilot. Была по курсу ВХОДА в фазу — после разворота
        # на 180° «вперёд» оставалось прибито к старому курсу (полёт 2026-08-16).
        c0 = math.cos(s.gt_yaw)
        s0 = math.sin(s.gt_yaw)
        self._spx += (sp.c_fwd * c0 + sp.c_right * s0) * self.cmd_gain * dt
        self._spy += (sp.c_fwd * s0 - sp.c_right * c0) * self.cmd_gain * dt
        # ⚠️ МИНУС, а не плюс. `c_yaw` — стик-конвенция (c_yaw>0 = стик вправо = разворот
        # ВПРАВО), а `_yawsp`/`gt_yaw` — ENU (влево = +yaw). Замер K1_slope: PWM ниже
        # центра = вращение в +yaw, значит открытый контур (control_stack: c_yaw → PWM
        # выше центра) на c_yaw>0 крутит в −yaw. Холдер обязан ехать туда же, иначе один
        # и тот же токен `mv_cw` означает разные стороны с холдером и без него.
        self._yawsp -= sp.c_yaw * self.yaw_cmd_gain * dt
        ex = s.gt_x - self._spx
        ey = s.gt_y - self._spy
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
        # yaw — курс-холд к КОМАНДНОМУ курсу (интеграл yaw-стика; c_yaw=0 → держит вход).
        # ⚠️ yaw_sign = −1, и это ПРОВЕРЕНО прогоном K1_slope, а не выведено из конвенции.
        # Там стоял дефолт +1, и борт раскрутило на 439° за 18 с (скорость курса всё
        # время одного знака, пики 73°/с — предел канала). Разбор: курс рос, значит
        # eyaw = yawsp − gt_yaw был отрицательным, значит команда шла НИЖЕ центра, и борт
        # продолжал вращаться туда же. Следовательно «ниже центра» = вращение в +yaw, и
        # гасить его надо командой ВЫШЕ центра при отрицательной ошибке → знак −1.
        # Ветка появилась при рефакторинге (в монолите gt-курс-холда не было, курс держал
        # DpYawHold по потоку) и до K1 в полёте не проверялась ни разу.
        eyaw = math.atan2(math.sin(self._yawsp - s.gt_yaw),
                          math.cos(self._yawsp - s.gt_yaw))
        yo = clamp(self.yaw_sign * self.yaw_kp * eyaw, -self.max, self.max)
        return RcCommand(roll=RC_CENTER + int(ro), pitch=RC_CENTER + int(po),
                         throttle=RC_CENTER, yaw=RC_CENTER + int(yo))


class GzPosHold(GzHold):
    def __init__(self, *a, **kw):
        kw['axes'] = frozenset({"roll", "pitch", "yaw"})
        super().__init__(*a, **kw)


class GzRollHold(GzHold):
    def __init__(self, *a, **kw):
        kw['axes'] = frozenset({"roll"})
        super().__init__(*a, **kw)


class GzPitchHold(GzHold):
    def __init__(self, *a, **kw):
        kw['axes'] = frozenset({"pitch"})
        super().__init__(*a, **kw)


class GzYawHold(GzHold):
    def __init__(self, *a, **kw):
        kw['axes'] = frozenset({"yaw"})
        super().__init__(*a, **kw)
