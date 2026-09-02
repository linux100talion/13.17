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

    # защёлка трима (i_latch): пороги — мёртвая зона стика и «гвоздь» по скорости
    _I_DZ = 0.02       # |c_*| выше — стик живой, И-член замораживается
    _I_PIN_V = 0.3     # м/с: после отпускания И-член спит, пока борт не встал

    def __init__(self, kp=40.0, kd=120.0, ki=8.0, imax=100.0, max_pwm=150.0,
                 psign=1.0, rsign=1.0, cmd_gain=0.8, kd_err=False,
                 i_latch=False, pin_stop=False):
        self.kp, self.kd, self.ki = kp, kd, ki
        self.imax, self.max = imax, max_pwm
        self.psign, self.rsign = psign, rsign
        self.cmd_gain = cmd_gain
        # kd_err: D-член на ОШИБКЕ скорости (v − v_уставки), не на абсолютной v.
        # Со старым законом в движении kd·v — константный «тормоз» (4 м/с × 120 =
        # 480 PWM при потолке 150): его компенсируют позиционный долг ~9–12 м и
        # И-член в капе, борт летит «на растянутой пружине» и звенит ~1 Гц
        # (серия eagle 2026-09-02, разбор в docker/sim/doc/tmp/eagle/eagle.txt).
        # С kd_err D демпфирует вокруг ДВИЖУЩЕГОСЯ равновесия (как каскад
        # LOITER); на висении и при отпущенном стике v_уставки=0 → закон
        # бит-в-бит прежний (доказанное удержание 0.07 м не трогаем).
        self.kd_err = kd_err
        # i_latch — ЗАЩЁЛКА ТРИМА, аналог _TRIM_LATCH станции: И-член заморожен
        # от живого стика до «гвоздя» (борт встал после отпускания). В движении
        # ошибка позиции — это ЛАГ слежения, не ветер: интегрировать её — копить
        # мусорный трим до капа (100 PWM) и получать перелёт/звон на отпускании
        # (у станции в выбеге наматывалось +73 PWM при триме 0). Выученный на
        # висении ветровой трим НЕ сбрасывается — держится замороженным весь ход.
        self.i_latch = i_latch
        # pin_stop — ГВОЗДЬ ПО ОСТАНОВКЕ (пункт 2б, как штатный LOITER): на
        # отпускании стика, как только борт встал (|v_vins| < _I_PIN_V), уставка
        # перевязывается на ТОЧКУ ОСТАНОВКИ — один раз на отпускание. Без этого
        # уставка остаётся ~3–4 м позади места, где борт затормозил (перелёт
        # при быстром стопе kd_err), и kp тянет его назад (замер ab_ilatch:
        # возврат 3.6–5.3 м после каждого стопа). «Тормозим и держим где
        # встали», а не «возвращаемся туда, где была команда».
        self.pin_stop = pin_stop
        self._spx = self._spy = 0.0        # интеграл стик-команды → уставка (vins-опора)
        self._ix = self._iy = 0.0
        self._it = None
        self._i_frozen = False             # защёлка взведена (стик жил, гвоздя ещё нет)
        self._pin_pending = False          # гвоздь заказан (стик жил, ждём остановки)

    def enter(self, s: DroneState) -> None:
        self._spx, self._spy = s.vins_x, s.vins_y
        self._ix = self._iy = 0.0
        self._it = s.now_sim
        self._i_frozen = False
        self._pin_pending = False

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        # проекция стик-команды по ТЕКУЩЕМУ vins-курсу (тело) — как в GzHold:
        # «вперёд» = куда сейчас смотрит нос, а не куда смотрел на входе в фазу
        c0 = math.cos(s.vins_yaw)
        s0 = math.sin(s.vins_yaw)
        vspx = (sp.c_fwd * c0 + sp.c_right * s0) * self.cmd_gain
        vspy = (sp.c_fwd * s0 - sp.c_right * c0) * self.cmd_gain
        self._spx += vspx * dt
        self._spy += vspy * dt
        # общий «гвоздь»: стик отпущен И борт встал (|v| < _I_PIN_V) — здесь
        # снимается защёлка трима (i_latch) и перевязывается уставка (pin_stop)
        stick = abs(sp.c_fwd) > self._I_DZ or abs(sp.c_right) > self._I_DZ
        if stick:
            if self.i_latch:
                self._i_frozen = True      # стик живой — трим не трогаем
            if self.pin_stop:
                self._pin_pending = True   # гвоздь заказан на ближайший стоп
        elif math.hypot(s.vins_vx, s.vins_vy) < self._I_PIN_V:
            self._i_frozen = False         # гвоздь: встали — трим снова учится
            if self._pin_pending:
                self._spx, self._spy = s.vins_x, s.vins_y
                self._pin_pending = False  # один раз на отпускание — дальше держим
        ex = s.vins_x - self._spx
        ey = s.vins_y - self._spy
        now = s.now_sim
        if (self.ki > 0 and self._it is not None and now > self._it
                and not self._i_frozen):
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
        vx, vy = s.vins_vx, s.vins_vy
        if self.kd_err:
            vx -= vspx
            vy -= vspy
        v_fwd = vx * c + vy * sn
        v_rgt = -vx * sn + vy * c
        i_fwd = self._ix * c + self._iy * sn
        i_rgt = -self._ix * sn + self._iy * c
        po = self.psign * (self.kp * e_fwd + self.kd * v_fwd + self.ki * i_fwd)
        ro = self.rsign * (self.kp * e_rgt + self.kd * v_rgt + self.ki * i_rgt)
        po = clamp(po, -self.max, self.max)
        ro = clamp(ro, -self.max, self.max)
        return RcCommand(roll=RC_CENTER + int(ro), pitch=RC_CENTER + int(po),
                         throttle=RC_CENTER, yaw=RC_CENTER)
