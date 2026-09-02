#!/usr/bin/env python3
"""YawBankLimit — потолок крена виража через темп yaw: |ω| ≤ g·tan(φ_max)/v.

Выделен под ярус LOITER лесенки (путь 2 агро-профиля: «нос ведёт траекторию,
но крен ограничен»), 2026-09-01.
"""
import math

from ..rc import RC_CENTER, RcCommand, clamp
from ..setpoint import Setpoint
from ..state import DroneState
from .base import StabilizationStrategy

_G = 9.81
# Команда RC yaw → темп, °/с на PWM: PILOT_Y_RATE 202.5 / 400 (полный стик).
# Замеренный ЗАМКНУТЫЙ авторитет ниже (0.44 °/с на PWM, [[yaw-spring]]) — значит
# фактический крен при этом капе чуть МЕНЬШЕ φ_max: берём теоретический маппинг
# как консервативный. PILOT_Y_RATE не трогаем (от него посчитан яв-демпфер).
_PWM_TO_RATE = 202.5 / 400.0


class YawBankLimit(StabilizationStrategy):
    """Декоратор yaw-стаба: режет выходной PWM так, чтобы вираж не требовал
    крена больше φ_max.

    ЗАЧЕМ (разбор eagle/4, 2026-09-01). В LOITER «нос ведёт траекторию»: yaw на
    ходу вращает уставку скорости, дуга требует крена φ = atan(v·ω/g) — физика,
    контроллером не отменяется (замер сошёлся с формулой нос в нос). Значит
    единственный способ держать «нос ведёт» И «крен ≤ φ_max» — темп разворота
    по скорости: |ω| ≤ g·tan(φ_max)/v. На висении полный темп (ω_max → ∞),
    на 5 м/с при φ_max=8° — 15.8 °/с (разворот 180° за ~11 с). Это ЦЕНА пути 2;
    путь 1 (TrackHold, стики в осях мира) вместо этого убирает саму дугу.

    Оборачивает ЛЮБОЙ yaw-стаб (в ярусе LOITER — общий DpYawHold ярусов 0/1:
    прямая передача стика + демпфер) и капит его ВЫХОД: прямая передача,
    D-член — всё под одним потолком; внутреннее состояние не трогается (в
    прямой передаче контур и так обнуляется каждый тик — виндапа от капа нет).

    СКОРОСТЬ — по доступности, как ground_speed миссии: канал вида сверху
    (ipm_ok — метрический, живёт без VINS) → свежая одометрия VINS → истина
    Gazebo (сим-оракул). Ни одного источника (борт до VINS с ослепшим IPM) —
    капа нет, полный темп: честная деградация к прежнему поведению, а не
    залипание руля (ярус LOITER без свежего VINS всё равно распадается).
    Ниже v_floor кап заведомо шире max_pwm стаба — не считаем.

    Конверсия PWM→°/с — теоретическая PILOT_Y_RATE/400 (см. _PWM_TO_RATE):
    поменяли PILOT_Y_RATE в прошивке — поправить pwm_rate."""
    axes = frozenset({"yaw"})

    def __init__(self, inner, bank_max_deg=8.0, pwm_rate=_PWM_TO_RATE,
                 fresh_sec=2.0, v_floor=0.1):
        self.inner = inner
        self.bank = math.radians(float(bank_max_deg))
        self.pwm_rate = float(pwm_rate)
        self.fresh = float(fresh_sec)
        self.v_floor = float(v_floor)

    def __getattr__(self, name):
        # прозрачность декоратора: имя/диагностика/yaw_sub и т.п. — у inner
        inner = self.__dict__.get('inner')
        if inner is None:
            raise AttributeError(name)
        return getattr(inner, name)

    def enter(self, s: DroneState) -> None:
        self.inner.enter(s)

    def _speed(self, s: DroneState):
        if s.ipm_ok:
            return math.hypot(s.ipm_vfwd, s.ipm_vlat)
        if s.vins_valid and (s.now_sim - s.vins_last_sim) < self.fresh:
            return math.hypot(s.vins_vx, s.vins_vy)
        if s.gt_valid:
            return math.hypot(s.gt_vx, s.gt_vy)
        return None

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        rc = self.inner.update(s, sp, dt)
        v = self._speed(s)
        if v is None or v <= self.v_floor:
            return rc
        w_max = math.degrees(_G * math.tan(self.bank) / v)   # °/с
        cap = w_max / self.pwm_rate                          # PWM от центра
        rc.yaw = RC_CENTER + int(clamp(rc.yaw - RC_CENTER, -cap, cap))
        return rc
