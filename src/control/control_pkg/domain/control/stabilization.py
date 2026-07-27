#!/usr/bin/env python3
"""Стратегии стабилизации — ДВА семейства + per-axis алиасы.

- **Gz\\*** — держит ПОЗИЦИЮ по ground-truth Gazebo (sim-оракул для тюнинга). Ошибку и
  скорость из world → в тело (по gt_yaw) → PWM по pitch(вперёд)/roll(вправо); yaw —
  курс-холд к yaw входа. `GzHold(axes=…)` база; алиасы GzPosHold/GzRollHold/GzPitchHold/
  GzYawHold. Арифметика roll/pitch выверена монолитом (Δ=0 в test_gz_shuttle_equiv).
- **Dp\\*** — ДЕМПФЕР: гонит СКОРОСТЬ к нулю по ОПТИЧЕСКОМУ ПОТОКУ (scale-free, боевой +
  sim через камеру), позицию НЕ держит. Источник — flow_lateral(roll)/flow_longitudinal(
  pitch)/flow_yaw(yaw) из FlowEstimator. Velocity-assist: цель = c_*·cmd_gain (стик).
  Покадровая интеграция (flow_seq), conf/stale-fade. DpRollHold/DpPitchHold/DpYawHold +
  DpHold (композит всех трёх). Законы roll/yaw — порт flow_hold/yaw_hold монолита.
- `VinsHold` — position-hold по VINS (после init, своя опора). `PilotPassthrough` — легаси.

Незанятые оси раздаёт пилоту сам ControlStack (per-axis база = сырые стики).
"""
import math

from ..rc import RC_CENTER, RcCommand, clamp
from ..setpoint import Setpoint
from ..state import DroneState
from .base import StabilizationStrategy


# ============================ Gz* — позиция по gazebo ============================

class GzHold(StabilizationStrategy):
    """Position-hold по gt Gazebo, per-axis (axes задаёт, какие оси стек использует).

    roll/pitch — PID позиции (ошибка world→тело); yaw — курс-холд к yaw входа (P по
    heading; yaw-стик = rate → P даёт устойчивую сходимость). Незанятые оси стек
    игнорирует, поэтому вычисляем все три, а axes лишь маркирует владение.
    """

    def __init__(self, kp=40.0, kd=120.0, ki=8.0, imax=100.0, max_pwm=150.0,
                 psign=1.0, rsign=1.0, axes=frozenset({"roll", "pitch"}),
                 yaw_kp=80.0, yaw_sign=1.0, cmd_gain=0.8, yaw_cmd_gain=0.5):
        self.kp, self.kd, self.ki = kp, kd, ki
        self.imax, self.max = imax, max_pwm
        self.psign, self.rsign = psign, rsign
        self.axes = axes
        self.yaw_kp, self.yaw_sign = yaw_kp, yaw_sign
        self.cmd_gain, self.yaw_cmd_gain = cmd_gain, yaw_cmd_gain
        self._ix = self._iy = 0.0          # интеграл ошибки позиции (world)
        self._it = None
        self._yaw0 = 0.0                   # курс входа (фрейм проекции стик-команды)
        self._spx = self._spy = 0.0        # ИНТЕГРАЛ стик-команды → уставка (своя опора)
        self._yawsp = 0.0                  # интеграл yaw-стика → командный курс

    def enter(self, s: DroneState) -> None:
        self._ix = self._iy = 0.0
        self._it = s.now_sim
        self._yaw0 = s.gt_yaw
        self._spx, self._spy = s.gt_x, s.gt_y   # уставка стартует в опоре (gt на входе)
        self._yawsp = s.gt_yaw

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        # стик-команду интегрируем в движущуюся уставку (в своём фрейме от опоры)
        c0 = math.cos(self._yaw0)
        s0 = math.sin(self._yaw0)
        self._spx += (sp.c_fwd * c0 + sp.c_right * s0) * self.cmd_gain * dt
        self._spy += (sp.c_fwd * s0 - sp.c_right * c0) * self.cmd_gain * dt
        self._yawsp += sp.c_yaw * self.yaw_cmd_gain * dt
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
        # yaw — курс-холд к КОМАНДНОМУ курсу (интеграл yaw-стика; c_yaw=0 → держит вход)
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


# ============================ прочие ============================

class PilotPassthrough(StabilizationStrategy):
    """Легаси: сырые стики → RC (per-axis модель делает manual = ПУСТОЙ список)."""
    axes = frozenset({"roll", "pitch", "yaw"})

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        return RcCommand(roll=s.pilot_roll, pitch=s.pilot_pitch,
                         throttle=RC_CENTER, yaw=s.pilot_yaw)


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
        self._yaw0 = 0.0                   # фрейм проекции стик-команды (vins-курс входа)
        self._spx = self._spy = 0.0        # интеграл стик-команды → уставка (vins-опора)
        self._ix = self._iy = 0.0
        self._it = None

    def enter(self, s: DroneState) -> None:
        self._spx, self._spy = s.vins_x, s.vins_y
        self._yaw0 = s.vins_yaw
        self._ix = self._iy = 0.0
        self._it = s.now_sim

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        c0 = math.cos(self._yaw0)
        s0 = math.sin(self._yaw0)
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


# ============================ Dp* — демпфер скорости по ОПТИЧЕСКОМУ ПОТОКУ ============================

def _blend(conf, conf_min, conf_full):
    """confidence (число треков) → авторитет демпфера [0..1] (плавный fade-out)."""
    return clamp((conf - conf_min) / max(1e-6, conf_full - conf_min), 0.0, 1.0)


class _FlowDamper1D(StabilizationStrategy):
    """Общий одноосевой флоу-демпфер: гасит визуальную скорость к цели по ОДНОЙ оси.
    Покадровая интеграция (flow_seq), conf/stale-fade. Подклассы задают: какой сигнал
    потока читать (_signal), какую c_* брать целью (_cmd), какую ось выдавать (_axis)."""
    _axis = "roll"

    def __init__(self, kp=8.0, ki=2.0, kd=0.0, imax=120.0, max_pwm=150.0,
                 conf_min=0.05, conf_full=0.20, osign=1.0, cmd_gain=10.0, stale_sec=0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.imax, self.max = imax, max_pwm
        self.conf_min, self.conf_full = conf_min, conf_full
        self.osign, self.cmd_gain, self.stale = osign, cmd_gain, stale_sec
        self._i = 0.0
        self._prev_err = 0.0
        self._last_seq = -1
        self._out = 0.0
        self._last_frame_sim = -1e9

    def _signal(self, s): raise NotImplementedError
    def _cmd(self, sp): raise NotImplementedError

    def enter(self, s: DroneState) -> None:
        self._i = 0.0
        self._prev_err = 0.0
        self._last_seq = -1
        self._out = 0.0
        self._last_frame_sim = -1e9

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        if s.flow_seq != self._last_seq:            # НОВЫЙ кадр → продвигаем PID
            self._last_seq = s.flow_seq
            fdt = max(1e-3, s.flow_dt)
            blend = _blend(s.flow_conf, self.conf_min, self.conf_full)
            err = self._signal(s) - self._cmd(sp) * self.cmd_gain   # velocity-assist
            self._i = clamp(self._i + self.ki * err * fdt, -self.imax, self.imax)
            d = self.kd * (err - self._prev_err) / fdt
            self._prev_err = err
            u = clamp(self.kp * err + self._i + d, -self.max, self.max)
            self._out = self.osign * blend * u
            self._last_frame_sim = s.now_sim
        fresh = (s.now_sim - self._last_frame_sim) < self.stale
        off = int(self._out) if fresh else 0         # протух → fade в центр
        rc = RcCommand(throttle=RC_CENTER)
        setattr(rc, self._axis, RC_CENTER + off)
        return rc


class DpRollHold(_FlowDamper1D):
    """Демпфер БОКОВОГО сноса по потоку → ROLL (был FlowDamper). Боевой пре-VINS.
    ⚠️ osign: drift_check подтвердил −1 (config.flow_osign=-1); класс-дефолт +1 (тесты)."""
    axes = frozenset({"roll"})
    _axis = "roll"

    def _signal(self, s): return s.flow_lateral
    def _cmd(self, sp): return sp.c_right


class DpPitchHold(_FlowDamper1D):
    """Демпфер ПРОДОЛЬНОГО сноса по потоку (looming) → PITCH. Сигнал flow_longitudinal
    (res['longitudinal'] FlowEstimator). ⚠️ НЕ проверен в полёте (looming менее зрел);
    знак osign не выверен."""
    axes = frozenset({"pitch"})
    _axis = "pitch"

    def _signal(self, s): return s.flow_longitudinal
    def _cmd(self, sp): return sp.c_fwd


class DpYawHold(StabilizationStrategy):
    """Демпфер визуального рыскания по потоку → YAW (был YawHold). Победитель свипа
    [[yaw-hold-tuning]] — ki=0 (интеграл вреден, bias yaw_flow). ∫err = курс-ошибка."""
    axes = frozenset({"yaw"})

    def __init__(self, kp=6.0, ki=0.0, imax=200.0, max_pwm=150.0,
                 conf_min=0.05, conf_full=0.20, osign=1.0, cmd_gain=10.0, stale_sec=0.5):
        self.kp, self.ki = kp, ki
        self.imax, self.max = imax, max_pwm
        self.conf_min, self.conf_full = conf_min, conf_full
        self.osign, self.cmd_gain, self.stale = osign, cmd_gain, stale_sec
        self._head = 0.0
        self._last_seq = -1
        self._out = 0.0
        self._last_frame_sim = -1e9

    def enter(self, s: DroneState) -> None:
        self._head = 0.0
        self._last_seq = -1
        self._out = 0.0
        self._last_frame_sim = -1e9

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        if s.flow_seq != self._last_seq:
            self._last_seq = s.flow_seq
            blend = _blend(s.flow_conf, self.conf_min, self.conf_full)
            target = sp.c_yaw * self.cmd_gain
            err = s.flow_yaw - target
            self._head = clamp(self._head + err, -self.imax, self.imax)
            yu = clamp(self.kp * err + self.ki * self._head, -self.max, self.max)
            self._out = self.osign * blend * yu
            self._last_frame_sim = s.now_sim
        fresh = (s.now_sim - self._last_frame_sim) < self.stale
        off = int(self._out) if fresh else 0
        return RcCommand(yaw=RC_CENTER + off, throttle=RC_CENTER)


class DpHold(StabilizationStrategy):
    """Демпфер по ВСЕМ трём осям — композит DpRollHold+DpPitchHold+DpYawHold. Каждая
    ось читает свой сигнал потока (разные единицы), поэтому композит, а не одна база."""
    axes = frozenset({"roll", "pitch", "yaw"})

    def __init__(self, roll=None, pitch=None, yaw=None):
        self._subs = [roll or DpRollHold(), pitch or DpPitchHold(), yaw or DpYawHold()]

    def enter(self, s: DroneState) -> None:
        for x in self._subs:
            x.enter(s)

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        rc = RcCommand(throttle=RC_CENTER)
        for x in self._subs:
            out = x.update(s, sp, dt)
            for ax in x.axes:
                setattr(rc, ax, getattr(out, ax))
        return rc
