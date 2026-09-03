#!/usr/bin/env python3
"""DpVins — velocity-каскад position-hold на опоре VINS (плавная замена VinsHold).

ЗАЧЕМ отдельный стабилизатор. VinsHold управлял ПОЗИЦИЕЙ: out = kp·e_pos + kd·v
+ ki·i — контур 2-го порядка, где kd демпфирует СЫРУЮ конечную разность 10 Гц
позы. Отсюда болезни серии eagle: звон ~1 Гц, пила команды (kd·шум скорости),
плавность σθ ~1.1° против 0.5° у демпфера (разбор doc/tmp/eagle/eagle.txt).
Причина СТРУКТУРНАЯ: демпфер плавен, потому что управляет СКОРОСТЬЮ (1-й порядок,
отклик наклона), а не позицией (2-й порядок + kd на шуме).

DpVins повторяет архитектуру демпфера/LOITER — velocity-каскад:
- ВНУТРЕННИЙ контур гонит скорость к цели: out = kp·(v_цель − v) + трим. Гейны
  в М/С — как rate-оси демпфера (DpRollRate kp 120, DpPitchRate kp 200), VINS-
  скорость в тех же единицах;
- ВНЕШНИЙ контур (стик отпущен) даёт цель скорости из ошибки позиции с √-капом
  (закон RETURN станции: |v_цель| ≤ √(2·acc·|e|), без перелёта) → плавный стоп
  и удержание, как штатный LOITER;
- ГВОЗДЬ по остановке (|v| < _PIN_V после отпускания) — уставка = точка стопа;
- ЛАТЧ ТРИМА: И-член (ветровой трим) заморожен от живого стика до гвоздя
  (_TRIM_LATCH демпфера / i_latch VinsHold);
- vsmooth: ФНЧ скорости для ВНУТРЕННЕГО контура — здесь это главная петля
  1-го порядка (аналог окна МНК 0.3 с демпфера), НЕ D-член 2-го порядка, где
  сглаживание съедало демпфирование (тупик BS_VINS_VSMOOTH в VinsHold).

Оси в раме курса: скорость/позиция VINS (мир) проецируются по vins_yaw. Трим —
пока в осях тела (разворот-во-время-удержания не разведён — как ранний демпфер
до StationFrame; для прямых галсов и стоп-удержания достаточно, TODO — мировой
трим). НЕ реализован полный автомат станции (BRAKE/фазы) — он был нужен ШУМНОМУ
дрейфующему каналу потока; у VINS позиция чистая, хватает √-капа RETURN.
"""
import math

from ..rc import RC_CENTER, RcCommand, clamp
from ..setpoint import Setpoint
from ..state import DroneState
from .base import StabilizationStrategy


class DpVins(StabilizationStrategy):
    axes = frozenset({"roll", "pitch"})

    _I_DZ = 0.02       # |c_*| выше — стик живой (трим замораживается, гвоздь снят)
    _PIN_V = 0.3       # м/с: гвоздь по остановке / порог «встал»

    def __init__(self, kp_fwd=200.0, kp_lat=120.0, ki=20.0, imax=100.0,
                 max_pwm=150.0, cmd_gain=4.0, pos_kp=0.3, pos_vmax=0.3,
                 pos_acc=0.15, psign=1.0, rsign=1.0, vsmooth=0.1,
                 i_latch=True):
        self.kp_fwd, self.kp_lat = kp_fwd, kp_lat
        self.ki, self.imax, self.max = ki, imax, max_pwm
        self.cmd_gain = cmd_gain
        # внешний позиционный контур (цель скорости при отпущенном стике)
        self.pos_kp, self.pos_vmax, self.pos_acc = pos_kp, pos_vmax, pos_acc
        self.psign, self.rsign = psign, rsign
        self.vsmooth = vsmooth
        self.i_latch = i_latch
        # состояние
        self._pinx = self._piny = None     # гвоздь (мир); None = стик жив / не встал
        self._pin_pending = False          # гвоздь заказан (стик жил, ждём стопа)
        self._itx = self._ity = 0.0        # И-член (трим) в осях МИРА (x, y)
        self._trim_armed = False           # ветровой трим выучен (первый стоп прошёл)
        self._vff = self._vfl = 0.0        # ФНЧ скорости внутреннего контура
        self._vf_init = False
        self._it = None

    def enter(self, s: DroneState) -> None:
        self._pinx = self._piny = None
        self._pin_pending = False
        self._itx = self._ity = 0.0
        self._trim_armed = False
        self._vff = self._vfl = 0.0
        self._vf_init = False
        self._it = s.now_sim

    def _return_target(self, e):
        """Цель скорости внешнего контура к гвоздю: линейный pos_kp + √-кап acc
        (тормозной путь без перелёта, как sqrt_controller ArduPilot / RETURN
        станции). Знак — к гвоздю."""
        mag = min(self.pos_kp * abs(e), self.pos_vmax)
        if self.pos_acc > 0.0:
            mag = min(mag, math.sqrt(2.0 * self.pos_acc * abs(e)))
        return math.copysign(mag, e)

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        c = math.cos(s.vins_yaw)
        sn = math.sin(s.vins_yaw)
        # скорость VINS (мир) → тело (fwd+, right+)
        v_fwd = s.vins_vx * c + s.vins_vy * sn
        v_rgt = -s.vins_vx * sn + s.vins_vy * c
        # ФНЧ скорости внутреннего контура (аналог окна МНК демпфера)
        if self.vsmooth > 0.0:
            if not self._vf_init:
                self._vff, self._vfl = v_fwd, v_rgt
                self._vf_init = True
            else:
                a = dt / (self.vsmooth + dt) if dt > 0.0 else 1.0
                self._vff += a * (v_fwd - self._vff)
                self._vfl += a * (v_rgt - self._vfl)
            v_fwd, v_rgt = self._vff, self._vfl

        stick = abs(sp.c_fwd) > self._I_DZ or abs(sp.c_right) > self._I_DZ

        # цель скорости (тело): стик → прямая; отпущен → внешний контур к гвоздю
        if stick:
            self._pinx = self._piny = None     # точка отпущена
            self._pin_pending = True           # гвоздь заказан на ближайший стоп
            # цель скорости тела: у VinsHold команда setpoint'а в осях тела
            # выходит (c_fwd·g, −c_right·g) — проекция мировой vsp(cmd) назад по
            # курсу; повторяем, иначе правый стик уводит борт влево
            tv_fwd = sp.c_fwd * self.cmd_gain
            tv_rgt = -sp.c_right * self.cmd_gain
        else:
            speed = math.hypot(s.vins_vx, s.vins_vy)
            if self._pin_pending and speed < self._PIN_V:
                self._pinx, self._piny = s.vins_x, s.vins_y   # ГВОЗДЬ по остановке
                self._pin_pending = False
                self._trim_armed = True      # первый стоп прошёл — ветер выучен
            if self._pinx is not None:
                ex = self._pinx - s.vins_x
                ey = self._piny - s.vins_y
                e_fwd = ex * c + ey * sn
                e_rgt = -ex * sn + ey * c
                tv_fwd = self._return_target(e_fwd)
                tv_rgt = self._return_target(e_rgt)
            else:
                tv_fwd = tv_rgt = 0.0          # тормозим к нулю до гвоздя

        # ошибка скорости = v − цель (конвенция демпфера: sig − target; при
        # движении вперёд быстрее цели err>0 → +out = торможение, знак сходится
        # с +kd·v VinsHold и osign=+1 rate-осей). ⚠️ обратный знак (цель − v) даёт
        # ПОЛОЖИТЕЛЬНУЮ ОС в удержании — унос (прогон ab_dpvins 2026-09-03).
        err_fwd = v_fwd - tv_fwd
        err_rgt = v_rgt - tv_rgt

        # И-член (ветровой трим). Дилемма (прогоны lv2_joy_065026 / ab_dpv_pinfix):
        # ТОРМОЖЕНИЕ (стик отпущен, гвоздя нет, цель 0) даёт ошибку = v вперёд.
        # Если ki мотает её ВСЕГДА — трим набирает «назад» на весь тормозной путь
        # → после стопа уносит борт назад (1.4–3.4 м). Если морозить до гвоздя —
        # без трима kp·v уравновешивает ветер на ~1 м/с, |v| не падает < pin_v,
        # гвоздь не вяжется, дрейф вечен (унос). Решение как _trim_armed/_BRAKE_TRIM
        # демпфера: на ПЕРВОМ торможении (ветер ещё не выучен) трим ИНТЕГРИРУЕТ —
        # это ловит ветер и даёт остановиться (цена — небольшой возврат на первом
        # стопе, обычно на висении зрелости при малой v); после первого гвоздя
        # (_trim_armed) трим на торможении ЗАМОРОЖЕН, учится только на удержании
        # → чистые стопы без возврата. На живом стике заморожен всегда.
        now = s.now_sim
        frozen = self.i_latch and (stick
                                   or (self._trim_armed and self._pinx is None))
        if self.ki > 0.0 and self._it is not None and now > self._it and not frozen:
            di = now - self._it
            cap = self.imax / self.ki
            # ⚠️ ТРИМ В ОСЯХ МИРА (как StationFrame DpHold): ветер — мировой,
            # трим тела устаревал после разворота (prog lv2_joy_075118: держит
            # при фикс. курсе, сносит 1.5 м/с при развороте «за/против ветра»).
            # Интегрируем ошибку скорости, повёрнутую в мир, храним (itx, ity),
            # ниже проецируем на ТЕКУЩИЙ курс — трим следует за курсом, гасит
            # тот же мировой ветер под любым разворотом.
            ex_w = err_fwd * c - err_rgt * sn
            ey_w = err_fwd * sn + err_rgt * c
            self._itx = clamp(self._itx + ex_w * di, -cap, cap)
            self._ity = clamp(self._ity + ey_w * di, -cap, cap)
        self._it = now

        i_fwd = self._itx * c + self._ity * sn        # мировой трим → тело (курс)
        i_rgt = -self._itx * sn + self._ity * c
        po = self.psign * (self.kp_fwd * err_fwd + self.ki * i_fwd)
        ro = self.rsign * (self.kp_lat * err_rgt + self.ki * i_rgt)
        po = clamp(po, -self.max, self.max)
        ro = clamp(ro, -self.max, self.max)
        return RcCommand(roll=RC_CENTER + int(ro), pitch=RC_CENTER + int(po),
                         throttle=RC_CENTER, yaw=RC_CENTER)
