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
- ГВОЗДЬ по остановке (|v| < _PIN_V после отпускания стика ЛИБО первый стоп
  после движения без стика — как станция/LOITER) — уставка = точка стопа;
- ЛАТЧ ТРИМА: И-член (ветровой трим) заморожен от живого стика до гвоздя
  (_TRIM_LATCH демпфера / i_latch VinsHold);
- vsmooth: ФНЧ скорости для ВНУТРЕННЕГО контура — здесь это главная петля
  1-го порядка (аналог окна МНК 0.3 с демпфера), НЕ D-член 2-го порядка, где
  сглаживание съедало демпфирование (тупик BS_VINS_VSMOOTH в VinsHold).

Оси в раме курса: скорость/позиция VINS (мир) проецируются по vins_yaw. Трим —
В ОСЯХ МИРА (как StationFrame демпфера), хранится в PWM: ветер мировой, трим
следует за курсом под разворотом (фикс lv2_joy_075118). Обучение трима
ДВУХСКОРОСТНОЕ: до первого ГВОЗДЯ — ki_trim (быстрый захват ветра: унос на
входе в ярус = нужный трим / ki обучения, при ветре 10 (~100 PWM) и ki 6 было
16–17.5 м на КАЖДОМ фронте яруса, от kp не зависит — замер wind_* 2026-09-03,
разбор doc/tmp/eagle/dpvins.txt), после — рабочий ki на удержании. Гвоздь при
этом вяжется и БЕЗ стика — первым стопом после движения: контур ki_trim
слабозатухающий (ζ≈0.26), длиться он должен секунды; когда его конец был
привязан к стику, голое висение раскачивалось (полёт lv2_joy_20260903_220204,
период 7.1 с — та же болезнь, что звон 10.7 с у демпфера). Трим
ПЕРЕЖИВАЕТ повторные enter() (trim_keep: ветер на переключении яруса не
исчезает, а дребезг гейта обнулял трим повторно — 24 м вместо 17); сброс —
только reset_trim(), его зовёт VinsHandover на фактическом /restart VINS
(мировая рама перерождается — вектор в старой раме недействителен).
ФАЗА BRAKE (2026-09-05, серия dphold_vs_dpvins + cmd/1–2): закон цели внешнего контура
взят у станции демпфера ЦЕЛИКОМ — два StationKeeper (оси вперёд/вбок), их target():
BRAKE — пока уходим от гвоздя быстрее brake_v, цель −brake·v (ошибка скорости
×(1+brake): при brake 3 kp 40/32 → 160/128 PWM на м/с — жёсткость демпфера, у
которого 360 ложатся на IPM с гейном 0.45–0.9); RETURN — прежний pos_kp/vmax/√-кап.
Без него DpVins пропускал порыв 8 м/с на 6–9 м против 2.5 у DpHold при том же
лаге: датчик лучше, закон мягче (разбор — память dphold-vs-dpvins-gusts). Трим на
торможении после первого гвоздя ЗАМОРОЖЕН (как _BRAKE_TRIM демпфера). brake 0 =
выкл: target() станции с brake 0 — та же формула, что прежний _return_target.
"""
import math

from ..rc import RC_CENTER, RcCommand, clamp
from ..setpoint import Setpoint
from ..state import DroneState
from .base import StabilizationStrategy
from .station_keeper import StationKeeper


class DpVins(StabilizationStrategy):
    axes = frozenset({"roll", "pitch"})

    _I_DZ = 0.02       # |c_*| выше — стик живой (трим замораживается, гвоздь снят)
    _PIN_V = 0.3       # м/с: гвоздь по остановке / порог «встал»

    def __init__(self, kp_fwd=200.0, kp_lat=120.0, ki=20.0, ki_trim=0.0,
                 imax=100.0, max_pwm=150.0, cmd_gain=4.0, pos_kp=0.3,
                 pos_vmax=0.3, pos_acc=0.15, psign=1.0, rsign=1.0, vsmooth=0.1,
                 i_latch=True, trim_keep=True, brake=0.0, brake_v=0.25,
                 brake_vmax=1.0):
        self.kp_fwd, self.kp_lat = kp_fwd, kp_lat
        self.ki, self.imax, self.max = ki, imax, max_pwm
        self.ki_trim = ki_trim             # скорость обучения ДО первого гвоздя (0 = ki)
        self.trim_keep = trim_keep         # трим переживает enter() (входы в ярус)
        self.cmd_gain = cmd_gain
        # внешний позиционный контур (цель скорости при отпущенном стике)
        self.pos_kp, self.pos_vmax, self.pos_acc = pos_kp, pos_vmax, pos_acc
        # ЗАКОН ЦЕЛИ внешнего контура — станция демпфера как есть (BRAKE/RETURN,
        # выход из тормоза, перевзвод): по экземпляру на ось тела. Гвоздь/трим —
        # свои (DpVins), у станции берём только target() и фазу braking.
        self.brake, self.brake_v, self.brake_vmax = brake, brake_v, brake_vmax
        self._st_fwd = StationKeeper(kp=pos_kp, vmax=pos_vmax, brake=brake,
                                     brake_vmax=brake_vmax, acc=pos_acc,
                                     brake_v=brake_v, pin_v=self._PIN_V)
        self._st_rgt = StationKeeper(kp=pos_kp, vmax=pos_vmax, brake=brake,
                                     brake_vmax=brake_vmax, acc=pos_acc,
                                     brake_v=brake_v, pin_v=self._PIN_V)
        self.psign, self.rsign = psign, rsign
        self.vsmooth = vsmooth
        self.i_latch = i_latch
        # состояние
        self._pinx = self._piny = None     # гвоздь (мир); None = стик жив / не встал
        self._pin_pending = False          # гвоздь заказан (стик жил, ждём стопа)
        self._moved = False                # ярус видел движение (|v| > _PIN_V)
        self._itx = self._ity = 0.0        # трим (PWM) в осях МИРА (x, y)
        self._trim_armed = False           # ветровой трим выучен (первый стоп прошёл)
        self._last_yaw = 0.0               # vins_yaw последнего rc() — для trim_pwm()
        self._vff = self._vfl = 0.0        # ФНЧ скорости внутреннего контура
        self._vf_init = False
        self._it = None

    def enter(self, s: DroneState) -> None:
        self._pinx = self._piny = None
        self._pin_pending = False
        self._moved = False
        self._st_fwd.reset()
        self._st_rgt.reset()
        # ТРИМ НЕ ТРОГАЕМ (trim_keep): ветер на переключении яруса не исчезает,
        # а обнуление на каждом входе = обучение заново = унос ≈ трим/ki по
        # ветру на каждом фронте яруса (wind_* 2026-09-03: 17 м; дребезг гейта
        # wind_back — 24). Сброс — только reset_trim() (handover, на /restart).
        if not self.trim_keep:
            self.reset_trim()
        self._vff = self._vfl = 0.0
        self._vf_init = False
        self._it = s.now_sim

    def reset_trim(self) -> None:
        """Обнулить ветровой трим и «ветер выучен». Зовёт VinsHandover на
        фактическом /restart VINS: мировая рама перерождается, хранимый мировой
        вектор в новой раме недействителен (при trim_keep=False — enter())."""
        self._itx = self._ity = 0.0
        self._trim_armed = False
        self._moved = False

    def seed_trim(self, pitch_off: float, roll_off: float, s: DroneState) -> bool:
        """ПОСЕВ трима от демпфера при передаче яруса 0→1 (п.5.3 dpvins.txt):
        установившийся И-член станции = тот же ветровой трим, снятый В ВАЛЮТЕ
        КАНАЛОВ (DpHold.trim_pwm, после osign). Обратно в пространство DpVins —
        обращением СОБСТВЕННОГО уравнения выхода: po = psign·(kp·err + i_fwd)
        → i_fwd = psign·pitch_off (psign² = 1), никаких рассуждений о
        конвенциях; тело → мир по vins_yaw — та же проекция, которой трим
        учится в update(). Сеем только ДЕВСТВЕННЫЙ трим (< 1 PWM и не armed):
        начатое обучение (дребезг гейта, trim_keep) и выученный ветер не
        перетираем — свой свежее. НЕ armed: посев — оценка (blend, протухание
        станции), ki_trim остаётся страховкой и быстро доучит остаток; при
        хорошем посеве ошибка мала и фаза — no-op до первого стопа."""
        if self._trim_armed or math.hypot(self._itx, self._ity) >= 1.0:
            return False
        i_fwd = self.psign * float(pitch_off)
        i_rgt = self.rsign * float(roll_off)
        c = math.cos(s.vins_yaw)
        sn = math.sin(s.vins_yaw)
        self._itx = clamp(i_fwd * c - i_rgt * sn, -self.imax, self.imax)
        self._ity = clamp(i_fwd * sn + i_rgt * c, -self.imax, self.imax)
        return True

    def trim_pwm(self, yaw=None):
        """Трим в валюте PWM КАНАЛОВ (pitch_off, roll_off) — ОБРАТНАЯ к
        seed_trim операция и зеркало DpHold.trim_pwm (общий интерфейс пулла:
        стрелка ветра HUD, посев). Мировой вектор проецируется в тело по yaw:
        без аргумента — vins_yaw ПОСЛЕДНЕГО update() (кэш _last_yaw; на
        активном ярусе свеж), с аргументом — заданный курс (LOITER: DpVins не
        тикает, кэш заморожен на выходе из яруса, а борт крутится — нода даёт
        текущий s.vins_yaw). Девственный трим честно отдаёт (0, 0)."""
        if yaw is None:
            yaw = self._last_yaw
        c = math.cos(yaw)
        sn = math.sin(yaw)
        i_fwd = self._itx * c + self._ity * sn
        i_rgt = -self._itx * sn + self._ity * c
        return (self.psign * i_fwd, self.rsign * i_rgt)

    @property
    def braking(self) -> bool:
        """Фаза BRAKE хотя бы на одной оси (станция)."""
        return self._st_fwd.braking or self._st_rgt.braking

    def _return_target(self, e):
        """Цель скорости внешнего контура к гвоздю: линейный pos_kp + √-кап acc
        (тормозной путь без перелёта, как sqrt_controller ArduPilot / RETURN
        станции). Знак — к гвоздю. С 2026-09-05 контур зовёт StationKeeper.target();
        при brake 0 он даёт ровно эту формулу — метод оставлен эталоном для теста."""
        mag = min(self.pos_kp * abs(e), self.pos_vmax)
        if self.pos_acc > 0.0:
            mag = min(mag, math.sqrt(2.0 * self.pos_acc * abs(e)))
        return math.copysign(mag, e)

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        c = math.cos(s.vins_yaw)
        sn = math.sin(s.vins_yaw)
        self._last_yaw = s.vins_yaw        # кэш для trim_pwm() (пулл без снапшота)
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
            self._st_fwd.release(False)        # брейк погашен (гвоздя нет)
            self._st_rgt.release(False)
            # цель скорости тела: у VinsHold команда setpoint'а в осях тела
            # выходит (c_fwd·g, −c_right·g) — проекция мировой vsp(cmd) назад по
            # курсу; повторяем, иначе правый стик уводит борт влево
            tv_fwd = sp.c_fwd * self.cmd_gain
            tv_rgt = -sp.c_right * self.cmd_gain
        else:
            speed = math.hypot(s.vins_vx, s.vins_vy)
            if speed > self._PIN_V:
                self._moved = True             # ярус видел движение
            # ГВОЗДЬ по остановке: после стика (pin_pending) ИЛИ первый стоп
            # после движения БЕЗ стика (_moved) — как станция демпфера/LOITER.
            # Раньше гвоздь вязался только после стика: на голом входе в ярус
            # (стики центр) фаза ki_trim не кончалась ВООБЩЕ — слабозатухающий
            # контур трима (ζ≈0.26 при ki 60, T≈8 с) с лагами канала раскачивал
            # борт (период 7.1 с, рост до 2.2 м/с — полёт lv2_joy_20260903_
            # 220204; демпфер той же болезнью болел — звон 10.7 с, лечился kp
            # 30→90 + автоматом станции). _moved отсекает ложный стоп на самом
            # входе: борт ещё не понесло, гвоздь тут съел бы фазу обучения.
            if (speed < self._PIN_V and self._pinx is None
                    and (self._pin_pending or self._moved)):
                self._pinx, self._piny = s.vins_x, s.vins_y   # ГВОЗДЬ по остановке
                self._pin_pending = False
                self._trim_armed = True      # первый стоп прошёл — ветер выучен
            if self._pinx is not None:
                ex = self._pinx - s.vins_x
                ey = self._piny - s.vins_y
                e_fwd = ex * c + ey * sn
                e_rgt = -ex * sn + ey * c
                # закон станции: BRAKE (−brake·v, пока уходим) / RETURN (√-кап)
                tv_fwd = self._st_fwd.target(e_fwd, v_fwd)
                tv_rgt = self._st_rgt.target(e_rgt, v_rgt)
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
        i_fwd = self._itx * c + self._ity * sn        # мировой трим (PWM) → тело (курс)
        i_rgt = -self._itx * sn + self._ity * c
        po = self.psign * (self.kp_fwd * err_fwd + i_fwd)
        ro = self.rsign * (self.kp_lat * err_rgt + i_rgt)
        # АНТИ-ВИНДАП: выход в упоре и ошибка толкает ГЛУБЖЕ — трим не мотать.
        # Так imax можно держать высоким (ветру нужен трим до ~ветра: при ветре
        # 10 ~100 PWM, кап 50 не держал — снос 1-1.3 м/с, унос 9-18 м,
        # lv2_joy_082437), а в насыщении торможения momentum не наматывается
        # (иначе перелёт первого стопа ∝ imax). Как anti_windup демпфера.
        sat = ((abs(po) >= self.max and po * err_fwd > 0.0)
               or (abs(ro) >= self.max and ro * err_rgt > 0.0))
        # заморозка: живой стик или ТОРМОЖЕНИЕ после стика (pin_pending) при
        # выученном ветре. Именно pin_pending, не «нет гвоздя»: после enter()
        # гвоздя нет, но и торможения нет — там трим ДОЛЖЕН учиться (рабочим
        # ki), иначе смена ветра между входами в ярус не доучивается никогда.
        frozen = self.i_latch and (stick
                                   or (self._trim_armed
                                       and (self._pin_pending or self.braking)))
        # СКОРОСТЬ ОБУЧЕНИЯ двухфазная: до первого гвоздя (ветер не выучен) —
        # ki_trim, быстрый захват (зеркало _POS_BRAKE_TRIM/ki_trim демпфера):
        # унос на входе в ярус = нужный трим / ki обучения, от kp не зависит
        # (при ветре 10 ≈ 100 PWM: ki 6 → 17 м, ki_trim 60 → ~2; замер wind_*
        # 2026-09-03). После гвоздя — рабочий ki на удержании. Трим хранится
        # в PWM, чтобы смена скорости не дёргала выход.
        ki = (self.ki_trim if self.ki_trim > 0.0 and not self._trim_armed
              else self.ki)
        if (ki > 0.0 and self._it is not None and now > self._it
                and not frozen and not sat):
            di = (now - self._it) * ki
            # ⚠️ ТРИМ В ОСЯХ МИРА (как StationFrame DpHold): ветер — мировой,
            # трим тела устаревал после разворота (prog lv2_joy_075118: держит
            # при фикс. курсе, сносит 1.5 м/с при развороте «за/против ветра»).
            # Интегрируем ошибку скорости, повёрнутую в мир, храним (itx, ity),
            # проецируем на ТЕКУЩИЙ курс — трим следует за курсом под разворотом.
            ex_w = err_fwd * c - err_rgt * sn
            ey_w = err_fwd * sn + err_rgt * c
            self._itx = clamp(self._itx + ex_w * di, -self.imax, self.imax)
            self._ity = clamp(self._ity + ey_w * di, -self.imax, self.imax)
            # пересчёт выхода со свежим тримом (для среза; в упоре кламп ниже)
            i_fwd = self._itx * c + self._ity * sn
            i_rgt = -self._itx * sn + self._ity * c
            po = self.psign * (self.kp_fwd * err_fwd + i_fwd)
            ro = self.rsign * (self.kp_lat * err_rgt + i_rgt)
        self._it = now
        po = clamp(po, -self.max, self.max)
        ro = clamp(ro, -self.max, self.max)
        return RcCommand(roll=RC_CENTER + int(ro), pitch=RC_CENTER + int(po),
                         throttle=RC_CENTER, yaw=RC_CENTER)
