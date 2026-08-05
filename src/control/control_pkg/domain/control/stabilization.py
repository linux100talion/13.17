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
  `DpPitchBack` — ЗОНД: та же команда, но выпрямленная назад (проверка канала, не holds).
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
                 yaw_kp=80.0, yaw_sign=-1.0, cmd_gain=0.8, yaw_cmd_gain=0.5):
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
    потока читать (_signal), какую c_* брать целью (_cmd), какую ось выдавать (_axis).

    КАК ОСЬ ЧИТАЕТ КОМАНДУ ПИЛОТА (`_cmd_mode`) — по ПОРЯДКУ сигнала, не по вкусу:

    - `rate` (сигнал = СКОРОСТЬ: flow_lateral у крена, flow_yaw у рыскания) —
      c_*·cmd_gain это ЦЕЛЬ скорости, вычитается из сигнала. Стик держат — борт едет,
      отпустили — встал. Так было всегда и для этих осей верно.
    - `pos` (сигнал = ПОЛОЖЕНИЕ: kf_logs у тангажа) — c_*·cmd_gain это СКОРОСТЬ УСТАВКИ,
      она ИНТЕГРИРУЕТСЯ в точку удержания (`_sp`), а ошибка считается до неё. Тот же
      механизм, что `_spx/_spy` у GzHold/VinsHold, только уставка живёт в единицах
      сигнала (log масштаба), а не в метрах.

    Почему `pos` нельзя заменить вычитанием: у холдера положения c_*·cmd_gain дало бы
    ПОСТОЯННОЕ СМЕЩЕНИЕ уставки — пилот жмёт «вперёд», борт уезжает на N метров и встаёт,
    отпускает — возвращается назад. Не движение, а параллакс ручки.

    Две детали режима `pos`, без которых он не летит:
    1. Уставка стартует С НУЛЯ, а не с захваченного значения сигнала. Ноль сигнала = сам
       опорный кадр, а шаг сбрасывает опору РОВНО при входе в сегмент (step.py зовёт
       reset_keyframe перед stack.enter) — значит ноль и есть точка входа. Захват «где
       стоим» пробовать нельзя: сброс опоры обнуляет накопитель, и кадр, посчитанный до
       сброса, дал бы уставку, смещённую на всю прошлую отсидку — не разовый пинок, как
       сейчас, а СТОЙКО ложную точку удержания. Пока сигнал негоден (набор высоты →
       kf_valid=False), уставка не едет: иначе на возврате достоверности она окажется
       впереди на весь простой.
    2. D-член вычитает скорость уставки: демпфируется отклонение от ЗАДАННОЙ скорости,
       а не сама заданная скорость. Иначе (kd=5000, крутизна 0.0145 log/м) команда 1 м/с
       рождает 72 PWM постоянного сопротивления, 2 м/с — 145 PWM при потолке 150, то есть
       контур душит собственную команду насыщением.
    """
    _axis = "roll"
    _cmd_mode = "rate"       # rate: c_* = цель скорости | pos: c_* = скорость уставки

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
        self._sp = 0.0           # pos-режим: точка удержания (0 = опорный кадр)
        self._sp_rate = 0.0      # её текущая скорость — для D-члена и отладки

    def _signal(self, s): raise NotImplementedError
    def _cmd(self, sp): raise NotImplementedError

    def _advance(self, s, fdt) -> None:
        """Хук: продвинуть внутреннее состояние сигнала РОВНО раз на новый кадр.

        Осям, чей сигнал приходит готовым (flow_lateral, kf_logs), не нужен — no-op.
        Нужен рысканию: его позиционный сигнал (визуальный курс) не измеряется, а
        НАКАПЛИВАЕТСЯ из flow_yaw, и накапливать его можно только по кадрам, а не по
        тикам ноды. Считать это внутри _signal() нельзя: тот вызывается не всегда
        (протухший сигнал коротит ветку) и накопитель разъехался бы с кадрами."""

    def _signal_dot(self, s):
        """Готовая производная сигнала, если оценщик её считает лучше нас.

        None → D-член берётся разностью соседних кадров (как было). Осям, чей сигнал
        — уже скорость (roll/yaw по потоку), разность и нужна. Продольной оси, чей
        сигнал — ПОЛОЖЕНИЕ, разность не годится: её corr с истиной +0.27 против +0.80
        у оконного наклона (замер J1b), см. DpPitchHold.
        """
        return None

    def enter(self, s: DroneState) -> None:
        self._i = 0.0
        self._prev_err = 0.0
        self._last_seq = -1
        self._out = 0.0
        self._last_frame_sim = -1e9
        # Уставка = опорный кадр (0), который шаг только что назначил сбросом опоры.
        # Захват текущего сигнала здесь был бы гонкой со сбросом — см. docstring класса.
        self._sp = 0.0
        self._sp_rate = 0.0

    def _signal_ok(self, s) -> bool:
        """Годен ли сигнал этой оси в этом кадре (переопределяется, где есть чем judge)."""
        return True

    def _fdt(self, s) -> float:
        """Шаг интегрирования = время С ПРОШЛОГО ПРОДВИЖЕНИЯ КОНТУРА, а не `flow_dt`.

        Домен тикает 20 Гц (`create_timer(0.05)`), камера на 960×540 даёт ~30 кадров/с:
        между двумя тиками успевает пройти больше одного кадра, `flow_seq` перескакивает,
        и контур видит ОДИН шаг там, где кадров было два. Беря `flow_dt` (промежуток
        между СОСЕДНИМИ кадрами), он засчитывал ~2/3 реального времени — замер Y3:
        уставка курса набрала 20.0° за 3.5 с команды вместо 30° (темп 5.71 против
        номинальных 8.60 °/с), и токен недодавал угол ровно на эту долю.

        Разница по осям невелика, потому что `flow_dt` стоял только там, где нужно
        «время с прошлого раза»: интеграл уставки, И-член и знаменатель Д-члена. У
        тангажа ki=0, а Д-член идёт через `_signal_dot` (без деления) — там правка
        трогает ровно уставку, то есть чинит тот же недобор. У крена ki=2, и его
        интегратор теперь копится в 1.5 раза быстрее — при возврате `DpRollHold`
        в стек это проверить.

        Потолок `stale`: после долгого провала сигнала не впрыскивать разом весь простой.
        """
        if self._last_frame_sim < -1e8:          # первый кадр сегмента — опереться не на что
            return max(1e-3, s.flow_dt)
        return clamp(s.now_sim - self._last_frame_sim, 1e-3, self.stale)

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        if s.flow_seq != self._last_seq and not self._signal_ok(s):
            # сигнал протух (у опоры — ушла высота): НЕ командуем и забываем производную,
            # иначе на возврате она даст пинок на всю накопленную разницу
            self._last_seq = s.flow_seq
            self._prev_err = 0.0
            self._out = 0.0
            self._last_frame_sim = s.now_sim
        elif s.flow_seq != self._last_seq:          # НОВЫЙ кадр → продвигаем PID
            self._last_seq = s.flow_seq
            fdt = self._fdt(s)
            blend = _blend(s.flow_conf, self.conf_min, self.conf_full)
            if self._cmd_mode == "pos":
                self._sp_rate = self._cmd(sp) * self.cmd_gain    # ед. сигнала в секунду
                self._sp += self._sp_rate * fdt                  # УСТАВКА ЕДЕТ
                # сигнал продвигаем ПОСЛЕ уставки: у рыскания `_advance` подтягивает
                # накопитель К УСТАВКЕ, и подтягивать его надо к свежей, а не к
                # прошлокадровой — иначе едущая уставка оставляет постоянный хвост
                # ошибки в один кадр команды
                self._advance(s, fdt)
                err = self._signal(s) - self._sp
            else:
                self._sp_rate = 0.0
                self._advance(s, fdt)
                err = self._signal(s) - self._cmd(sp) * self.cmd_gain   # velocity-assist
            self._i = clamp(self._i + self.ki * err * fdt, -self.imax, self.imax)
            dot = self._signal_dot(s)
            # разность соседних кадров уже считает производную ОШИБКИ (уставка в err),
            # готовой производной сигнала уставку надо вычесть руками
            d = (self.kd * (dot - self._sp_rate) if dot is not None
                 else self.kd * (err - self._prev_err) / fdt)
            self._prev_err = err
            u = clamp(self.kp * err + self._i + d, -self.max, self.max)
            self._out = self.osign * blend * u
            self._last_frame_sim = s.now_sim
        fresh = (s.now_sim - self._last_frame_sim) < self.stale
        off = int(self._out) if fresh else 0         # протух → fade в центр
        rc = RcCommand(throttle=RC_CENTER)
        setattr(rc, self._axis, RC_CENTER + off)
        return rc

    def hold_dbg(self):
        """(уставка, ошибка до неё, скорость уставки) для бэга — или None у rate-осей.

        Из телеметрии уставку не восстановить: она интеграл команды по КАДРАМ (fdt), а
        не по тикам ноды, и стартует с захваченного значения. Без неё в разборе нельзя
        отличить «борт не поехал» от «уставка не поехала»."""
        if self._cmd_mode != "pos":
            return None
        return (self._sp, self._prev_err, self._sp_rate)


class DpRollHold(_FlowDamper1D):
    """Демпфер БОКОВОГО сноса по потоку → ROLL (был FlowDamper). Боевой пре-VINS.
    ⚠️ osign: drift_check подтвердил −1 (config.flow_osign=-1); класс-дефолт +1 (тесты)."""
    axes = frozenset({"roll"})
    _axis = "roll"

    def _signal(self, s): return s.flow_lateral
    def _cmd(self, sp): return sp.c_right


class DpPitchHold(_FlowDamper1D):
    """УДЕРЖАНИЕ продольного положения по опорному кадру → PITCH.

    Сигнал — `kf_logs` (логарифм масштаба созвездия относительно опорного кадра), то
    есть СМЕЩЕНИЕ, а не скорость. Прежний `flow_longitudinal` (медиана покадрового
    вертикального сдвига) отброшен: замер по G1 показал связь с истинной скоростью
    corr −0.22, тогда как у `kf_logs` связь с истинным удалением −0.97 при крутизне
    −1.21% на метр. Причина — геометрия: камера смотрит почти горизонтально (наклон
    15°), ход вперёд идёт вдоль оптической оси и сдвига не даёт, он даёт МАСШТАБ.

    Из-за этого класс из ДЕМПФЕРА стал УДЕРЖАНИЕМ: kp работает по положению, kd — по
    его производной (та же скорость, но посчитанная из чистого сигнала). Единицы
    сменились полностью: раньше px, теперь log(масштаб) ≈ −0.0121 на метр, поэтому
    гейны несопоставимы со старыми (см. config.py).

    Знак: уехали назад → сцена дальше → масштаб меньше → kf_logs < 0 → выход < центра →
    нос вниз → летим вперёд, к опоре. osign=+1 (тот же, что был).

    ⚠️ Набор высоты тоже отдаляет землю (камера наклонена вниз) и читается как «уехал
    назад» → на подъёме контур будет толкать ВПЕРЁД. Компенсация по высоте не сделана;
    пока это заметно только вне висения, где z держится.

    КОМАНДА ПИЛОТА идёт через ИНТЕГРАТОР УСТАВКИ (`_cmd_mode='pos'`, см. базу): стик
    задаёт скорость, с которой едет точка удержания. `cmd_gain` поэтому меряется в
    log/с при полном стике, а не в единицах сигнала: полный стик = желаемые v_max м/с,
    то есть `cmd_gain = v_max · 0.0145` (крутизна канала 1.45-1.58 %/м, замер J2/K1s).
    Для v_max = 2 м/с это 0.029 — на три порядка меньше ролловых 10, потому что там
    ручка меряется в px/кадр. Дефолт 0.0 = команда не проходит (чистое удержание)."""
    axes = frozenset({"pitch"})
    _axis = "pitch"
    _cmd_mode = "pos"        # сигнал = ПОЛОЖЕНИЕ → команду интегрируем в уставку

    def __init__(self, kp=2000.0, ki=0.0, kd=1000.0, imax=120.0, max_pwm=150.0,
                 conf_min=0.05, conf_full=0.20, osign=1.0, cmd_gain=0.0, stale_sec=0.5):
        # Порядок аргументов — как у семейства (recipes зовёт позиционно), меняются
        # только ДЕФОЛТЫ: единицы сигнала другие (log масштаба вместо px), и общий
        # класс-дефолт kp=8 по пикселям дал бы выход ~0. Обоснование чисел — в
        # config.py (пересчёт от a₁=0.0115 м/с²/PWM и крутизны 1.21% на метр).
        super().__init__(kp, ki, kd, imax, max_pwm, conf_min, conf_full, osign,
                         cmd_gain, stale_sec)

    def _signal(self, s): return s.kf_logs
    def _cmd(self, sp): return sp.c_fwd

    def _signal_dot(self, s):
        """D-член по ОКОННОЙ скорости опоры, а не по разности соседних кадров.

        Чем это было: замер J1b (короткая опора с накоплением, hover40). Канал
        положения стал наконец рабочим — corr с истинным удалением +0.74 на 27 м,
        накопитель показал 31 м при истинных 28. И тем же прогоном борт ушёл на 34 м
        в АВТОКОЛЕБАНИЕ: +11 → −23 → +14 м, период 22 с. Знак верен (corr удаления с
        тангажом −0.91), гейна хватало (kp-член 192 PWM при потолке 150) — не хватало
        ДЕМПФИРОВАНИЯ. П-регулятор по положению на двойном интеграторе с задержкой
        1.04 с обязан звенеть, а kd работал по шуму: разность соседних кадров
        коррелирует с истинной скоростью на +0.27 (шаг сигнала p95 0.0134 против
        полезного приращения ~0.001 за кадр), выход её kd-члена бил в потолок 150 PWM
        на 21% кадров и переворачивал знак железной команды 9 раз за 5 с.
        Оконный наклон (kf_win=1 с) даёт corr +0.80 при той же крутизне.
        """
        return s.kf_vel

    def _signal_ok(self, s):
        # опора действительна только на постоянной высоте: на наборе и снижении
        # оценщик помечает кадр kf_valid=False (см. flow_estimator.kf_alt_max)
        return bool(s.kf_valid)


class DpPitchBack(DpPitchHold):
    """ЗОНД (не стабилизатор): расчёт DpPitchHold как есть, но выход ВЫПРЯМЛЕН НАЗАД
    (u = −|u|). Такт и амплитуда — демпферные (пересчёт по кадру, до max_pwm), знак
    всегда один. Отвечает на вопрос «доезжает ли рваная покадровая команда до борта и
    тащит ли она дрон в заданную сторону» отдельно от вопроса «верен ли знак сигнала»:
    направление тут задано руками, борт обязан поехать назад. Roll/yaw в таком прогоне
    держит gz-опора, так что продольная ось — единственная свободная."""

    is_probe = True        # НЕ удержание: план не ставит зонд на набор/посадку (_hold_stack)

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        rc = super().update(s, sp, dt)
        rc.pitch = RC_CENTER - abs(rc.pitch - RC_CENTER)
        return rc


class DpYawHold(_FlowDamper1D):
    """КУРС-ХОЛД по потоку → YAW. Команда идёт тем же механизмом, что у тангажа:
    интегратор уставки (`pos`), а не velocity-assist.

    ПОЧЕМУ `pos`, хотя flow_yaw — скорость. Позиционного сигнала у оси нет (опорного
    кадра для курса не существует), поэтому он НАКАПЛИВАЕТСЯ: визуальный курс
    `_head = ∫flow_yaw·dt`. Единицы: 1 ед. = 1/S градусов, S = 0.253 px/кадр на °/с
    (замер Y1s, corr +0.886 на ротации, стабильность между прогонами ±1.2% — это
    первая ось кампании, прошедшая ворота tune.md Фазы 2).

    СТАРЫЙ ЗАКОН НИКУДА НЕ ДЕЛСЯ, он переехал в Д-слот. В pos-режиме
        (err − prev_err)/fdt = (flow_yaw·fdt − _sp_rate·fdt)/fdt = flow_yaw − _sp_rate,
    то есть `kd·(…)` — это ровно прежнее `kp·(flow_yaw − цель)`. Победитель свипа
    [[yaw-hold-tuning]] kp=6 стал kd=6 БЕЗ переигрывания. При kp=0 поведение совпадает
    с прежним побитово; kp>0 включает курс-холд, ради которого всё и делалось.

    ⚠️ УТЕЧКА (`leak_sec`) — не украшение, а условие работоспособности. У тангажа
    позиционный сигнал меряется против опорного кадра, у нас он интеграл, поэтому
    смещение flow_yaw копится как ФАНТОМНЫЙ курс без предела. Это тот самый механизм,
    которым свип отверг ki (ki=2 → СКО 10.65°): П-член по накопленному курсу — то же
    самое. Замер Y1s дал число: смещение на висении −0.13/−0.09 px/кадр = −0.50/−0.34
    °/с, то есть 1.7–3.5° за командный сегмент (терпимо) и 14–20° за hover40 (нет).
    Утечка ограничивает фантом уровнем bias·T вместо роста: при T=8с это 3–4°.
    ЦЕНА, которую платим сознательно: на временах ≫T ось перестаёт быть абсолютным
    курс-холдом и работает демпфером скорости. Абсолютный курс — работа NN1/VINS.
    Течёт ОШИБКА (курс к уставке), а не курс к нулю — см. `_advance`.

    cmd_gain — в ЕДИНИЦАХ СИГНАЛА В СЕКУНДУ при полном стике, = S · (°/с). Дефолт 7.25
    = 0.253 · 28.65 °/с даёт полному стику тот же темп, что у GzHold (yaw_cmd_gain 0.5
    рад/с), — значит один и тот же токен mv_* крутит на одинаковый угол под обоими
    холдерами. Прежние 10.0 были в px/кадр (= 39.5 °/с, на 38% быстрее)."""
    axes = frozenset({"yaw"})
    _axis = "yaw"
    _cmd_mode = "pos"

    def __init__(self, kp=0.0, ki=0.0, kd=6.0, imax=200.0, max_pwm=150.0,
                 conf_min=0.05, conf_full=0.20, osign=1.0, cmd_gain=7.25,
                 stale_sec=0.5, leak_sec=8.0):
        super().__init__(kp, ki, kd, imax, max_pwm, conf_min, conf_full,
                         osign, cmd_gain, stale_sec)
        self.leak = leak_sec
        self._head = 0.0

    def enter(self, s: DroneState) -> None:
        super().enter(s)
        self._head = 0.0          # визуальный курс отсчитывается от входа в сегмент

    def _advance(self, s, fdt) -> None:
        self._head += s.flow_yaw * fdt
        if self.leak > 0.0:
            # утечка К УСТАВКЕ, а не к нулю. Ноль был опорой, пока уставка не ездила;
            # с командой он стал произвольной точкой, и утечка к нему СЪЕДАЛА разворот:
            # замер Y2 — на токене 60° (7с команды + 6с добора) накопитель просел с 82°
            # до 46°, борт добирал разницу и переворачивал на +25%. Утечка ошибки даёт
            # ровно то, ради чего заводилась: фантом от смещения по-прежнему ограничен
            # bias·T (на командах ошибка мала, утечке нечего есть).
            self._head += (self._sp - self._head) * min(1.0, fdt / self.leak)

    def _signal(self, s): return self._head

    def _cmd(self, sp):
        # ⚠️ МИНУС — тот же, что в GzHold._yawsp. `c_yaw>0` = стик ВПРАВО, а `flow_yaw`
        # (медиана горизонтального сдвига картинки) при развороте вправо ОТРИЦАТЕЛЕН:
        # сцена уезжает влево. Значит уставка накопленного курса обязана ехать в минус.
        # Без минуса ось отрабатывала команду ЗЕРКАЛЬНО — замер Y2 на всех четырёх
        # точках свипа: yaw_l30 давал −21…−28°, yaw_r60 давал +70…+76°.
        return -sp.c_yaw


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
