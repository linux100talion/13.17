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

    NB: в per-axis модели (срез 3) этот класс избыточен — manual = ПУСТОЙ список
    стабилизаторов (база стека = сырые стики). Оставлен для совместимости.
    """
    axes = frozenset({"roll", "pitch", "yaw"})

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        return RcCommand(roll=s.pilot_roll, pitch=s.pilot_pitch,
                         throttle=RC_CENTER, yaw=s.pilot_yaw)


def _blend(conf, conf_min, conf_full):
    """confidence (число треков) → авторитет демпфера [0..1] (плавный fade-out)."""
    return clamp((conf - conf_min) / max(1e-6, conf_full - conf_min), 0.0, 1.0)


class FlowDamper(StabilizationStrategy):
    """СРЕЗ 3 (БОЕВОЙ пре-VINS): боковой демпфер сноса по оптическому потоку → ROLL.

    Наш простой стабилизатор ДО инициализации VINS (нет GPS/VINS/gt). Гасит боковую
    визуальную скорость (flow_lateral) к ЦЕЛИ. Velocity-assist: цель = c_right·cmd_gain
    (нормир. стик пилота) → пилот рулит боком, флоу убирает снос; стик в центре → цель 0
    → чистый демпф дрейфа. Порт закона из alt_hold_bootstrap._on_flow_image (flow_hold).

    PID интегрирует ПО КАДРАМ (flow_seq), не по тикам: на стоячем сигнале 20-Гц тик не
    должен накручивать интеграл. Между кадрами держим последний выход; протух (stale) →
    fade в центр. Confidence (flow_conf) → плавный авторитет. Читает flow_* из DroneState
    (наполняет RosPerception через FlowEstimator — sim и борт одинаково).
    """
    axes = frozenset({"roll"})

    def __init__(self, kp=8.0, ki=2.0, kd=0.0, imax=120.0, max_pwm=150.0,
                 conf_min=0.05, conf_full=0.20, osign=1.0, cmd_gain=0.0, stale_sec=0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.imax, self.max = imax, max_pwm
        self.conf_min, self.conf_full = conf_min, conf_full
        self.osign, self.cmd_gain, self.stale = osign, cmd_gain, stale_sec
        self._i = 0.0
        self._prev_err = 0.0
        self._last_seq = -1
        self._out = 0.0
        self._last_frame_sim = -1e9

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
            target = sp.c_right * self.cmd_gain      # velocity-assist: желаемый боковой поток
            err = s.flow_lateral - target
            self._i = clamp(self._i + self.ki * err * fdt, -self.imax, self.imax)
            d = self.kd * (err - self._prev_err) / fdt
            self._prev_err = err
            u = clamp(self.kp * err + self._i + d, -self.max, self.max)
            self._out = self.osign * blend * u
            self._last_frame_sim = s.now_sim
        fresh = (s.now_sim - self._last_frame_sim) < self.stale
        off = int(self._out) if fresh else 0         # протух → fade в центр
        return RcCommand(roll=RC_CENTER + off, throttle=RC_CENTER)


class YawHold(StabilizationStrategy):
    """СРЕЗ 3 (БОЕВОЙ пре-VINS): визуальный курс/рыскание по потоку → YAW.

    Гасит визуальную yaw-скорость (flow_yaw) к ЦЕЛИ. Velocity-assist: цель =
    c_yaw·cmd_gain (стик) → пилот рулит курсом, флоу гасит паразитное вращение; центр
    → удержание. Победитель свипа [[yaw-hold-tuning]] — ki=0 (чистый демпф yaw-rate;
    интеграл ВРЕДЕН, накручивает bias yaw_flow). Порт из _on_flow_image (yaw_hold).
    Depth-independent (курс не упирается в дальнюю сцену, в отличие от roll).
    """
    axes = frozenset({"yaw"})

    def __init__(self, kp=6.0, ki=0.0, imax=200.0, max_pwm=150.0,
                 conf_min=0.05, conf_full=0.20, osign=1.0, cmd_gain=0.0, stale_sec=0.5):
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
            self._head = clamp(self._head + err, -self.imax, self.imax)   # ∫err = курс-ошибка
            yu = clamp(self.kp * err + self.ki * self._head, -self.max, self.max)
            self._out = self.osign * blend * yu
            self._last_frame_sim = s.now_sim
        fresh = (s.now_sim - self._last_frame_sim) < self.stale
        off = int(self._out) if fresh else 0
        return RcCommand(yaw=RC_CENTER + off, throttle=RC_CENTER)
