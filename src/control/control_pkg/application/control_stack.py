#!/usr/bin/env python3
"""ControlStack — композиция ролей (Trajectory→[Stabilization…]→Excitation) в AttitudeCommand (СИ).

PER-AXIS модель (срез 3): стабилизаторов может быть НЕСКОЛЬКО, каждый владеет своими
осями (`axes`). Композиция:
  1) БАЗА = НАМЕРЕНИЕ ТРАЕКТОРИИ (оператор): c_*→PWM, незанятая ось = открытый контур
     (сырой стик оператора). Оператор взаимозаменяем: скрипт (ConstProfile/Shuttle) /
     живой пульт (RcTransmitter читает s.pilot_*) / (будущее) NN2 — profile-only до конца;
  2) каждый стабилизатор ПЕРЕЗАПИСЫВАЕТ свои оси (regulate/velocity-assist);
  3) Excitation подмешивается сверху (ADDITIVE/REPLACE).
Так «пульт + только yaw» = [DpYawHold] (yaw держит, roll/pitch = профиль-оператор); лётный
пре-VINS = [DpRollHold,DpYawHold] (roll/yaw демпфер, pitch = оператор). Manual = [] (всё
оператору через RcTransmitter). Живой пилот входит ТОЛЬКО как Trajectory (RcTransmitter),
не как отдельная база — единый «язык» намерения c_*.

Владеет ТОЧКОЙ ВХОДА (origin/yaw0/t0): Trajectory отдаёт смещение относительно входа
(тело), стек собирает абсолютную world-уставку + прокидывает скорость-команду в Setpoint.
Самодостаточен — работает и без MissionRunner.
"""
from ..domain.attitude import AttitudeCommand
from ..domain.rc import RC_CENTER, clamp
from ..domain.setpoint import AxisPolicy, Setpoint
from ..domain.units import rc_off_tilt, rc_off_yaw, tilt_from_us, tilt_of_pwm, yaw_from_us, yaw_of_pwm

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


def yaw_stabs(stabs):
    """Только yaw из стека: чистые yaw-стабы как есть, у композита (DpHold,
    оси roll+pitch+yaw) — его `yaw_sub`. Ярусы, где roll/pitch держит кто-то
    другой (ярус 1 — DpVins, ярус 2 — FCU), берут yaw ТАК, а не композитом
    целиком: иначе композит стоит в стеке «тенью» — выход крена/тангажа
    перезаписан, а оси считаются каждый тик с живыми побочными эффектами
    (два писателя в общий WindTrim — src/control/code_smells/shadow_composite_tier1.md)."""
    out = []
    for st in stabs or []:
        axes = getattr(st, "axes", frozenset())
        if "yaw" not in axes:
            continue
        st = st if axes == frozenset({"yaw"}) else getattr(st, "yaw_sub", None)
        if st is not None:
            out.append(st)
    return out


def shared_axes(stabs):
    """Оси, объявленные БОЛЕЕ чем одним стабом стека (пусто = инвариант цел).
    Инвариант: каждый стаб в стеке владеет каждой своей осью — правило «поздний
    перезаписывает» не должно прятать мёртвые оси раннего (тень композита)."""
    seen, dup = set(), set()
    for st in stabs or []:
        axes = set(getattr(st, "axes", ()))
        dup |= seen & axes
        seen |= axes
    return dup


def _compose(rc: AttitudeCommand, axis: str, off: float, policy: AxisPolicy) -> AttitudeCommand:
    cur = getattr(rc, axis)
    if policy is AxisPolicy.ADDITIVE:
        setattr(rc, axis, cur + off)          # зонд ПОВЕРХ выхода стабилизатора
    elif policy is AxisPolicy.REPLACE:
        setattr(rc, axis, off)                # зонд ВЫТЕСНЯЕТ стабилизатор
    return rc


# ось → (СИ → целые µs, целые µs → СИ): ограничитель скорости живёт на сетке µs (фаза A)
_GRID = {"roll": (rc_off_tilt, tilt_from_us), "pitch": (rc_off_tilt, tilt_from_us),
         "yaw": (rc_off_yaw, yaw_from_us)}


class ControlStack:
    """slew: ограничение СКОРОСТИ ИЗМЕНЕНИЯ выхода, µs канала в секунду (0 = выключено):
    для наклона 300 µs/с = 12 °/с, для курса та же ручка = 54 °/с² (масштаб units.py).
    Считается на сетке целых µs, как до переезда носителя в СИ (фаза A).


    Зачем. `OverrideRCIn` — это положение стика, то есть ЗАКАЗАННЫЙ УГОЛ, и борт выходит
    на него не мгновенно: измерено `τ = 0.27 ± 0.03 с` (шесть ступеней в A1/A2). Команда,
    удержанная время W, материализуется на `1 − e^(−W/τ)`:

        W = 33 мс (один кадр — так демпфер и командовал) →  11%
        W = 0.62 с                                       →  90%
        W = 1.5 с (полный ход при slew=100)              →  99.6%

    Прежний выход PID шёл в провод как есть: пересчёт каждый кадр, знак менялся раньше,
    чем угол успевал установиться (`+57, +131, −150, +144` PWM со средним ≈ 0). Борт
    отрабатывал огибающую, а не значения — факт 3 в src/control/ToDo.md.

    Величина: помеха растёт на 0.06 м/с³ = 5 PWM/с, так что 100 PWM/с даёт двадцатикратный
    запас на отслеживание. Побочно давит шум: остаточные 33 PWM на 30 Гц требовали бы
    ~1000 PWM/с, ограничитель режет их вдесятеро. Одна ручка чинит и реализуемость, и чаттер.

    Ставится ЗДЕСЬ, а не в стратегиях: это единственная точка, где формируется финальный
    PWM (тут же живёт `_PITCH_RC_SIGN`), и ограничение достаётся всем стабилизаторам разом.
    Throttle НЕ ограничивается — им управляет миссия, а не контур.
    """

    def __init__(self, stabilization, trajectory, excitation, slew=0.0):
        self.stabs = _as_list(stabilization)   # список стабилизаторов (может быть пуст)
        self.traj = trajectory
        self.excite = excitation
        self.slew = float(slew)
        self._prev_rc = None                   # предыдущий выход, для ограничения скорости
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
        self._prev_rc = None        # вход в сегмент = разрешаем стартовать с любого значения
        for st in self.stabs:
            st.enter(s)

    def update(self, s) -> AttitudeCommand:
        if self._t0 is None:
            self.enter(s)
        t = s.now_sim - self._t0
        dt = max(0.0, s.now_sim - (self._prev_t if self._prev_t is not None else s.now_sim))
        self._prev_t = s.now_sim
        intent = self.traj.intent(s, t)
        sp = Setpoint(intent.c_fwd, intent.c_right, intent.c_yaw)   # стик-команда → стабилизаторам
        # БАЗА — намерение траектории (оператор): c_*→PWM. Незанятая ось = открытый контур
        # (наклон оператора). throttle держит миссия. Живой пилот входит через RcTransmitter.
        # c_* → µs стика (конвенция pilot_full ±400) → СИ тем же масштабом (±16°, ±72 °/с)
        rc = AttitudeCommand(roll=tilt_of_pwm(_cmd_to_pwm(intent.c_right)),
                             pitch=tilt_of_pwm(_cmd_to_pwm(_PITCH_RC_SIGN * intent.c_fwd)),
                             yaw_rate=yaw_of_pwm(_cmd_to_pwm(intent.c_yaw)))
        # ⚠️ Стабилизаторы (Gz*/Dp*) пишут выход НАПРЯМУЮ, минуя этот перевод: их знаки
        # (gz_psign, pitch_osign) заданы уже в конвенции ArduPilot и здесь не участвуют.
        # каждый стабилизатор перезаписывает СВОИ оси
        for st in self.stabs:
            out = st.update(s, sp, dt)
            for ax in st.axes:
                setattr(rc, ax, getattr(out, ax))
        for axis, (off, pol) in self.excite.offset(s, t).items():
            rc = _compose(rc, axis, off, pol)
        return self._limit(rc, dt)

    def _limit(self, rc: AttitudeCommand, dt: float) -> AttitudeCommand:
        """Ограничить скорость изменения roll/pitch/yaw (см. docstring класса). На сетке
        целых µs: µs = центр + смещение, int() — как до переезда носителя в СИ."""
        if self.slew <= 0 or dt <= 0:
            self._prev_rc = rc
            return rc
        if self._prev_rc is not None:
            step = self.slew * dt
            for ax, (to_us, from_us) in _GRID.items():
                prev = RC_CENTER + to_us(getattr(self._prev_rc, ax))
                cur = RC_CENTER + to_us(getattr(rc, ax))
                setattr(rc, ax, from_us(int(clamp(cur, prev - step, prev + step)) - RC_CENTER))
        self._prev_rc = rc
        return rc

    def motion_done(self) -> bool:
        t = 0.0 if self._t0 is None else (self._prev_t - self._t0)
        return self.traj.done(t)

    def excite_done(self) -> bool:
        t = 0.0 if self._t0 is None else (self._prev_t - self._t0)
        return self.excite.done(t)
