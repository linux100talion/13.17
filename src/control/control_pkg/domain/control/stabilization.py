#!/usr/bin/env python3
"""Стратегии стабилизации — ДВА семейства + per-axis алиасы.

- **Gz\\*** — держит ПОЗИЦИЮ по ground-truth Gazebo (sim-оракул для тюнинга). Ошибку и
  скорость из world → в тело (по gt_yaw) → PWM по pitch(вперёд)/roll(вправо); yaw —
  курс-холд к yaw входа. `GzHold(axes=…)` база; алиасы GzPosHold/GzRollHold/GzPitchHold/
  GzYawHold. Арифметика roll/pitch выверена монолитом (Δ=0 в test_gz_shuttle_equiv).
- **Dp\\*** — ДЕМПФЕР: гонит СКОРОСТЬ к нулю по ОПТИЧЕСКОМУ ПОТОКУ (scale-free, боевой +
  sim через камеру), позицию НЕ держит. Источник — flow_lateral(roll)/flow_longitudinal(
  pitch)/flow_yaw(yaw) из FlowEstimator. Velocity-assist: цель = c_*·cmd_gain (стик).
  Покадровая интеграция (flow_seq), conf-blend, hold+fade на провалах сигнала.
  DpRollHold/DpPitchHold/DpYawHold +
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
        self._spx = self._spy = 0.0        # интеграл стик-команды → уставка (vins-опора)
        self._ix = self._iy = 0.0
        self._it = None

    def enter(self, s: DroneState) -> None:
        self._spx, self._spy = s.vins_x, s.vins_y
        self._ix = self._iy = 0.0
        self._it = s.now_sim

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        # проекция стик-команды по ТЕКУЩЕМУ vins-курсу (тело) — как в GzHold:
        # «вперёд» = куда сейчас смотрит нос, а не куда смотрел на входе в фазу
        c0 = math.cos(s.vins_yaw)
        s0 = math.sin(s.vins_yaw)
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
    Покадровая интеграция (flow_seq), conf-blend; провал/протухание сигнала НЕ
    обнуляет выход — он держится stale и гаснет к нулю за ещё stale (см. update).
    Подклассы задают: какой сигнал
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
                 conf_min=0.05, conf_full=0.20, osign=1.0, cmd_gain=10.0, stale_sec=0.5,
                 pos_kp=0.0, pos_vmax=1.0, pos_brake=0.0, pos_brake_vmax=0.0,
                 pos_acc=0.0, anti_windup=False, pos_brake_v=0.0, pos_alt_band=0.0,
                 pos_alt_still=0.5, ki_trim=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        # ki_trim — скорость набора ТРИМА ВЕТРА в упоре ПЕРВОГО брейка (_BRAKE_TRIM),
        # отдельно от ki. Зачем две: ki по скорости — это скрытая ПРУЖИНА по положению
        # (I = ki·∫v = ki·Δx), и она же задаёт жёсткость звона станции (стенд
        # test_station_brake §9: ki 60 → 30 даёт ζ порыва 0.05 → 0.4), а трим на отрыве
        # в 10 м/с (104 PWM) при ki 30 набирался бы вдвое дольше (стоп 6 с вместо 3).
        # Упор первого брейка копит ∫v со скоростью ki_trim — отрыв бит-в-бит как при
        # ki 60, пружина вне упора вдвое мягче. 0 = ki (прежнее поведение).
        self.ki_trim = ki_trim if ki_trim > 0.0 else ki
        self.imax, self.max = imax, max_pwm
        self.conf_min, self.conf_full = conf_min, conf_full
        self.osign, self.cmd_gain, self.stale = osign, cmd_gain, stale_sec
        # --- СТАНЦИЯ-КИПИНГ поверх rate-оси (внешний P-контур по накопленному пути) ---
        # Скоростной демпфер не возвращает в точку: остаточные 0.2-0.5 м/с превращаются
        # в метры сноса, пока пилот не трогает стик. pos_kp > 0: стик В ЦЕНТРЕ →
        # захватывается текущий путь (_pos_signal) и цель скорости = pos_kp·(точка −
        # путь), кламп ±pos_vmax; стик ЖИВОЙ → точка отпускается, обычный velocity-
        # режим (пульт всегда главный, как LOITER). Точка живёт в BODY-осях пути —
        # при развороте на месте она поехала бы с курсом, поэтому уход курса больше
        # _POS_YAW_TOL от курса захвата перезахватывает точку (снос за время разворота
        # прощаем — честнее, чем тянуть в повёрнутую сторону). 0 = выкл (как было).
        self.pos_kp, self.pos_vmax = pos_kp, pos_vmax
        # --- ДВА ЗАКОНА СТАНЦИИ: «тормози жёстко, возвращайся мягко» ---
        # Одна линейная pos_kp обязана выбирать между стопом и звоном: она одинаково
        # жёстко тянет и ОТ точки (нужно — стоп), и К точке (даёт перелёт). Прогон
        # BS_ROLL_POS_KP/ab_pos13 (2026-08-28, pos_kp=1.3): стоп за 1 с в упоре 150,
        # но в момент стопа борт в 0.9 м от точки → цель −1.0 м/с → проходит точку
        # на −1.2 м/с → маятник ±1.2…1.8 м/с, период 5.4 с, ζ −0.08 (РАСТЁТ).
        # Механика: P по пути поверх PI по скорости = PID по позиции (P = kp·pos_kp
        # + ki, I = ki·pos_kp, D = kp) → при pos_kp 1.3 внешний контур той же
        # скорости, что внутренний (ω_n 0.9-1.2), лаг канала + контур FCU по углу
        # съедают всё демпфирование. Правило каскада: внешний в 3-5× медленнее.
        # Пилот «успокаивал руками» (ab_pos13_me) = стик отпускает точку + перезахват
        # в покое — то есть сброс накопленной ошибки, не демпфирование.
        # Решение — фазы. BRAKE: борт уходит ОТ точки быстрее _POS_PIN_V → цель
        # = −pos_brake·v_изм (кламп ±pos_brake_vmax): гасим СКОРОСТЬ, без
        # позиционного члена. Авторитет растёт со скоростью (k=3, канал видит 0.33
        # → цель −1.0 → внутренняя ошибка 1.3 м/с → упор, как при 1.3), а у нуля
        # скорости цель сама уходит в ноль — переход в RETURN без ступеньки.
        # ⚠️ Позиционный брейк (цель pos_kp_brake·err) пробовался первым и на стенде
        # test_station_brake дал предельный цикл ±0.6 м/с: на измеренном нуле цель
        # прыгала с −1.0 на −0.4, привод ещё толкал, борт проходил точку на 0.6,
        # уход > 0.3 снова будил брейк — бесконечно. Скоростной брейк заканчивается
        # сам (цель → 0 вместе со скоростью) и раскачаться не может.
        # RETURN: стоим или идём К точке → pos_kp/pos_vmax (мягко: по каскаду ≤0.3)
        # + √-кап pos_acc: |цель| ≤ √(2·pos_acc·|ошибка|) — тормозной путь до точки
        # без перелёта (как sqrt_controller ArduPilot). Гистерезис входа в BRAKE
        # (|v| > _POS_PIN_V) — иначе у точки дребезг фаз; малые уходы (<0.3) гасит
        # линейный RETURN сам.
        # pos_brake = 0 → одна ручка, как было (класс-дефолт: офлайн-репро);
        # pos_brake_vmax = 0 → потолок брейка = pos_vmax; pos_acc = 0 → без √-капа;
        # pos_brake_v — порог ВХОДА в BRAKE по |v_изм| (0 = _POS_PIN_V). Канал видит
        # 0.4-0.6 истины (ab_brake на 0.7 м: 0.28 при истинных 0.72, ab_brake_win5 на
        # 0.3 м: ровно 0.30 при 0.62 — брейк не проснулся), поэтому порог — ручка.
        # ⚠️ ВЫХОД из BRAKE — не только смена знака v (v·err ≥ 0), но и |v| <
        # _POS_BRAKE_EXIT: в ab_brake_win10 (10 м/с) канал после стопа 5 с показывал
        # +0.01…+0.12 «ухода» при истинном нуле (смещение прогноза по наклону в
        # ветер, пока acc_tau не сошёлся), брейк висел с целью −3·v ≈ −0.1 и держал
        # борт на месте в 2 м от точки, возврат начался лишь на 8-й секунде.
        self.pos_brake, self.pos_brake_vmax = pos_brake, pos_brake_vmax
        self.pos_acc = pos_acc
        self.pos_brake_v = pos_brake_v
        # --- СТАНЦИЯ ТОЛЬКО НА УСТАНОВИВШЕЙСЯ ВЫСОТЕ (pos_alt_band > 0) ---
        # Нужно ТАНГАЖУ: полоса земли лежит впереди, и ход по высоте канал читает как
        # ход вперёд — ФАНТОМ в пути ipm_fwd ~0.2-0.6 м на метр высоты (замер по
        # ab_brake_trim: +0.4 м за набор 0.1→0.3 м на отрыве, −0.5 м за посадку, до
        # 1.1 м за снижение с 3 м). Станция с гвоздём, пережившим набор, тянула бы борт
        # к фантомной точке на 0.3 м/с, а брейк бил бы по фантомной скорости набора
        # (0.8 м/с) на самом отрыве. Поэтому: пока высота идёт (вышла из полосы ±band
        # вокруг опорной, _AltSettled — без дифференциатора, см. её докстринг) или ещё
        # не постояла still секунд — гвоздь отпущен, цель 0 (чистый демпфер, как до
        # станции), брейк молчит; успокоилась — перезахват в покое, фантом прощён.
        # Крену не нужно (боковой фантом ×3 меньше, ipm_alt_band_lat по той же причине
        # выключен): 0 = без высотной логики, поведение прежнее. Гейт ОСИ
        # (_IpmGated.alt_band) при этом не трогаем — демпфер на наборе работает.
        self._pos_alt = _AltSettled(band=pos_alt_band, still=pos_alt_still) \
            if pos_alt_band > 0.0 else None
        self._pos_brake = False   # фаза BRAKE активна (только при pos_brake > 0)
        self._trim_armed = True   # ПЕРВЫЙ брейк после enter(): трим в упоре + порог
                                  # входа без множителя _POS_BRAKE_REFIRE
        # --- ANTI-WINDUP интегратора (условное интегрирование) ---
        # Кламп ±imax не спасает: пока выход в упоре ±max, интегратор копит дальше
        # (ab_pos13: за 1.5 с упора при ошибке ~1 м/с +90 PWM сверх ветра), и потом
        # обязан размотаться, толкая борт ЗА точку — заметная доля роста раскачки.
        # В фазе BRAKE упор — по замыслу, поэтому без anti-windup брейк не летит.
        # True: если выход в упоре и ошибка толкает глубже — И-член не растёт.
        # Класс-дефолт False (прежнее поведение бит-в-бит); лётный — config.
        self.anti_windup = bool(anti_windup)
        self._i = 0.0
        self._prev_err = 0.0
        self._last_seq = -1
        self._out = 0.0
        self._last_frame_sim = -1e9
        self._last_ok_sim = -1e9  # последний ГОДНЫЙ кадр — часы удержания выхода;
                                  # _last_frame_sim двигают и негодные кадры (он
                                  # задаёт шаг интегрирования), эти часы — нет
        self._sp = 0.0           # pos-режим: точка удержания (0 = опорный кадр)
        self._sp_rate = 0.0      # её текущая скорость — для D-члена и отладки
        self._pos_sp = None      # станция rate-оси: (путь в точке захвата, курс захвата)
        self._pos_wait_t = None  # начало торможения (для принудительного гвоздя)
        self._i_hold = False     # И-член заморожен: от живого стика до гвоздя (_TRIM_LATCH)
        self.frame = None        # StationFrame — общая рама крена/тангажа (оси курса);
                                 # None = станция в осях борта (как было). Вешает композит
        self._target = 0.0       # ЦЕЛЬ rate-оси (c_*·cmd_gain) — записи команды у этих
                                 # осей не было вовсе: /flow_dbg5 шлёт только pos-оси, и
                                 # калибровку R1 пришлось резать по истинной скорости,
                                 # где откат после торможения неотличим от команды

    _POS_YAW_TOL = 0.3       # рад (~17°): дальше точка станции перезахватывается
    _POS_PIN_V = 0.3         # м/с: гвоздь вяжется только когда борт затормозил
    _POS_BRAKE_EXIT = 0.1    # м/с: |v_изм| ниже — стоп состоялся, BRAKE → RETURN
                             # (см. pos_brake_v в __init__: смещённый канал может
                             # не пересечь ноль вовсе — ab_brake_win10)
    _POS_BRAKE_REFIRE = 2.0  # после ПЕРВОГО брейка порог входа ×2 (0.25 → 0.5 м/с):
                             # брейк — для настоящего ухода (толчок на отрыве, порыв,
                             # отпущенный стик на 1-2 м/с), не для качания у точки.
                             # В раскачке с амплитудой у порога входа он бил на каждом
                             # качке, а из-за лага канала — уже на развороте скорости,
                             # и качал (стенд, лаг 0.5: 0.29 → 0.74; рефрактерный
                             # период не помог — цикл подстраивался под него). Порыв
                             # слабее 0.5 м/с гасят RETURN + PI (прежний демпфер).
    _POS_BRAKE_REFIRE_DIST = 0.5  # м: ПЕРЕВЗВОД брейка (не первый) — только дальше
                             # этого от гвоздя. Стенд §10 (лаги 0.4/0.26, канал 1:1):
                             # брейк, разбуженный качанием у точки на |v| > 0.5,
                             # командует обратный ход −3v (до 1 м/с), лаги проносят
                             # борт сквозь гвоздь — и качание ±0.45 м живёт вечно
                             # (277 brake-кадров, |v| 0.82 в хвосте). Уход от гвоздя
                             # быстрее 0.5 м/с БЛИЖЕ 0.5 м — это качание контура, а не
                             # снос: ветер разгоняет плавно (порыв +5 м/с: брейк
                             # через 1.7 с и 1.05 м вместо 1.02 — цена ноль). Первый
                             # брейк (отрыв, гвоздь только что взят, err ≈ 0) —
                             # без ограничения. 0 = прежнее правило.
    _BRAKE_TRIM = True       # anti-windup В УПОРЕ БРЕЙКА: вместо заморозки И-член
                             # копит ВЕТРОВОЙ ТРИМ — интеграл чистой скорости (что
                             # нужно, чтобы держать ноль), а не ошибки до цели брейка
                             # (v+3v — её anti-windup и режет). Иначе в упоре
                             # интегратор стоит, и в 10 м/с (трим ~107 PWM) после
                             # стопа борт ~3 с набирает трим, прежде чем тронуться
                             # к точке (BS_ROLL_POS_BRAKE/3/hover: И-член −5…10 весь
                             # брейк, 31 на стопе, возврат с 6-й с; при слабом
                             # брейке 0.25 выход не в упоре, трим 113 к стопу —
                             # возврат сразу, что пилоту и понравилось). Вне упора
                             # правило прежнее (ошибка 4v учит трим быстрее всего).
                             # ⚠️ ТОЛЬКО ПЕРВЫЙ брейк после enter() (_trim_armed):
                             # трим учится один раз на взлёте; в раскачке каждый
                             # брейк-эпизод в упоре, и ∫v там копится В ФАЗЕ с
                             # колебанием — стенд на лаге 0.5 разнёс (0.29 → 0.85).
                             # Дальше в полёте трим уже в интеграторе, в упоре —
                             # прежняя заморозка. False — прежнее правило.
    _TRIM_LATCH = True       # ЗАЩЁЛКА ТРИМА НА ТОЛЧОК ПИЛОТА (станция, pos_kp > 0):
                             # И-член заморожен от живого стика до гвоздя. Полёт
                             # ab_brake_trim/win0 (крен, ветер 0), толчок 2.5 м/с:
                             # в выбеге после отпускания И-член намотал ki·путь
                             # торможения = +73 PWM при тримe 0 — это пружина с
                             # опорой ТАМ, ГДЕ НАЧАЛСЯ тормоз, а гвоздь станция
                             # вяжет там, где борт ВСТАЛ; две опоры дерутся: стоп →
                             # откат −0.93 м/с → брейк −124 → перелёт +0.65 → звон
                             # 1.0 → 0.33 м за 14 с. Во время самого толчка И-член
                             # тоже мусорил (−44 при нуле ветра): цель пилота в
                             # единицах канала (gain < 1) недостижима, интегратор
                             # «чинил» гейн. Трим ветра толчком пилота не меняется —
                             # его и держим; учится он в висении (гвоздь взят) и в
                             # первом брейке на отрыве (_BRAKE_TRIM, стик не жил —
                             # защёлка не ставится). False — прежнее правило.
    _POS_PIN_T = 3.0         # с: не затормозил за столько — гвоздь принудительно
                             # (иначе на злой рампе скорость никогда не падает ниже
                             # порога, гвоздь не вяжется вовсе и станция вырождается
                             # в чистый демпфер: прогон 2026-08-18 — fence за 18 с)

    def _signal(self, s): raise NotImplementedError
    def _cmd(self, sp): raise NotImplementedError

    def _pos_signal(self, s):
        """Накопленный ПУТЬ вдоль оси (для станции-кипинга) — None, если оси нечем."""
        return None

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
        self._last_ok_sim = -1e9
        # Уставка = опорный кадр (0), который шаг только что назначил сбросом опоры.
        # Захват текущего сигнала здесь был бы гонкой со сбросом — см. docstring класса.
        self._sp = 0.0
        self._sp_rate = 0.0
        self._pos_sp = None
        self._pos_wait_t = None
        self._pos_brake = False
        self._trim_armed = True
        self._i_hold = False
        if self._pos_alt is not None:
            self._pos_alt.reset()

    def _station_target(self, err, v):
        """Цель скорости станции по ошибке пути `err` (точка − путь) и скорости `v`.

        Одна ручка (pos_brake = 0): clamp(pos_kp·err, ±pos_vmax) — как было.
        Две фазы — см. комментарий в __init__: BRAKE, пока уходим от точки (v·err < 0)
        после входа по |v| > _POS_PIN_V, до измеренного нуля скорости — цель
        −pos_brake·v (скоростной брейк, без позиционного члена); RETURN — всё
        остальное, с √-капом тормозного пути при pos_acc > 0."""
        away = v * err < 0.0                     # скорость направлена ОТ точки
        if self.pos_brake > 0.0:
            if self._pos_brake:
                if not away or abs(v) < self._POS_BRAKE_EXIT:   # стоп состоялся
                    self._pos_brake = False
                    self._trim_armed = False      # первый брейк отработан: трим ветра
                                                  # выучен, дальше порог входа ×REFIRE
            elif away and abs(v) > ((self.pos_brake_v or self._POS_PIN_V)
                                    * (1.0 if self._trim_armed else self._POS_BRAKE_REFIRE)) \
                    and (self._trim_armed or abs(err) >= self._POS_BRAKE_REFIRE_DIST):
                self._pos_brake = True
        if self._pos_brake:
            vmax = self.pos_brake_vmax if self.pos_brake_vmax > 0.0 else self.pos_vmax
            return clamp(-self.pos_brake * v, -vmax, vmax)
        t = clamp(self.pos_kp * err, -self.pos_vmax, self.pos_vmax)
        if self.pos_acc > 0.0:
            cap = math.sqrt(2.0 * self.pos_acc * abs(err))
            t = clamp(t, -cap, cap)
        return t

    def _signal_ok(self, s) -> bool:
        """Годен ли сигнал этой оси в этом кадре (переопределяется, где есть чем judge)."""
        return True

    def _authority(self, s) -> float:
        """Авторитет оси [0..1] — множитель выхода по здоровью ЕЁ СОБСТВЕННОГО сигнала.

        База судит по flow_conf (число треков ПОЛНОКАДРОВОГО LK) — честно для осей,
        чей сигнал из него и считается (flow_lateral, flow_yaw, kf_logs). Ось с
        ДРУГИМ источником обязана переопределить: судить её по чужому сигналу —
        наследованная ошибка (см. _IpmGated._authority, прогон lv2 214015)."""
        return _blend(s.flow_conf, self.conf_min, self.conf_full)

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
            # сигнал негоден (у опоры — ушла высота): PID не двигаем и забываем
            # производную, иначе на возврате она даст пинок на всю накопленную разницу.
            # ВЫХОД ПРИ ЭТОМ ДЕРЖИМ, а не сбрасываем в 0. Сброс превращал команду в
            # пилу: полёт 2026-08-18 — на 3.5+ м/с провалы ipm_ok шли по 0.4–0.5 с
            # один за другим, каждый обнулял выход, slew=300 в ControlStack не успевал
            # вернуть его до следующего провала, и средний тормоз выходил ~30 PWM
            # вместо упора 150 при истинных 2.6 м/с — механизм всех разгонов STAB.
            # Гниение стоячей команды ограничено часами _last_ok_sim (см. ниже).
            self._last_seq = s.flow_seq
            self._prev_err = 0.0
            self._last_frame_sim = s.now_sim
        elif s.flow_seq != self._last_seq:          # НОВЫЙ кадр → продвигаем PID
            self._last_seq = s.flow_seq
            fdt = self._fdt(s)
            blend = self._authority(s)
            fr = None                    # рама станции — только у rate-осей со станцией
            if self._cmd_mode == "pos":
                self._sp_rate = self._cmd(sp) * self.cmd_gain    # ед. сигнала в секунду
                self._sp += self._sp_rate * fdt                  # УСТАВКА ЕДЕТ
                # сигнал продвигаем ПОСЛЕ уставки: у рыскания `_advance` подтягивает
                # накопитель К УСТАВКЕ, и подтягивать его надо к свежей, а не к
                # прошлокадровой — иначе едущая уставка оставляет постоянный хвост
                # ошибки в один кадр команды
                self._advance(s, fdt)
                sig = self._signal(s)
                err = sig - self._sp
            else:
                self._sp_rate = 0.0
                self._advance(s, fdt)
                cmd = self._cmd(sp)
                pos = self._pos_signal(s) if self.pos_kp > 0.0 else None
                fr = self.frame if (self.frame is not None and pos is not None) else None
                if fr is not None:
                    # общая рама: мировая позиция из приращений пути (раз на кадр,
                    # идемпотентно по flow_seq), стик этой оси — раме, И-член этой
                    # оси = компонента мирового вектора трима вдоль оси ТЕКУЩЕГО курса
                    fr.advance(s)
                    fr.stick(self._axis, cmd != 0.0)
                    self._i = fr.trim_body(self._axis)
                if pos is not None and cmd == 0.0 and self._pos_alt is not None \
                        and self._pos_alt.update(s.now_sim, s.rel_alt) is False:
                    # высота идёт / ещё не установилась — гвоздь отпущен, чистый
                    # демпфер (см. pos_alt_band в __init__: фантом набора в пути)
                    self._pos_sp = None
                    self._pos_wait_t = None
                    self._pos_brake = False
                    self._target = 0.0
                elif pos is not None and cmd == 0.0:
                    # СТАНЦИЯ: «СНАЧАЛА ТОРМОЗИ, ПОТОМ ГВОЗДЬ» (механика LOITER).
                    # Точка вяжется НЕ в момент отпускания стика: борт ещё несёт
                    # 2-3 м/с, выбег ~9 м, и станция тянула бы его назад к месту,
                    # которое пилот уже мысленно покинул — «рулю против резинки»
                    # (полёт 2026-08-18). Пока |скорость| ≥ _POS_PIN_V — цель 0
                    # (чистое торможение демпфером), гвоздь — где остановился.
                    # Перезахват при уходе курса: путь копится в body-осях.
                    # В осях курса (рама) точка мировая и с курсом не едет — перезахват
                    # по уходу курса нужен только пока ДРУГАЯ ось везёт пилота (эта
                    # держит «линию», и линия должна повернуться вместе с ходом).
                    if (self._pos_sp is not None
                            and (fr is None or fr.any_stick())
                            and abs(math.atan2(math.sin(s.att_yaw - self._pos_sp[1]),
                                               math.cos(s.att_yaw - self._pos_sp[1])))
                            > self._POS_YAW_TOL):
                        self._pos_sp = None
                    if self._pos_sp is None:
                        if self._pos_wait_t is None:
                            self._pos_wait_t = s.now_sim
                        if (abs(self._signal(s)) < self._POS_PIN_V
                                or s.now_sim - self._pos_wait_t > self._POS_PIN_T):
                            self._pos_sp = (pos, s.att_yaw)
                            self._pos_wait_t = None
                            self._i_hold = False      # гвоздь взят — трим снова учится
                            if fr is not None:
                                fr.set_pin()          # ОДИН 2D-гвоздь на обе оси
                    if self._pos_sp is not None:
                        err_pos = (fr.body_err(self._axis) if fr is not None
                                   else self._pos_sp[0] - pos)
                        if err_pos is None:           # рама без гвоздя (сброшен) — взять
                            fr.set_pin()
                            err_pos = 0.0
                        self._target = self._station_target(err_pos, self._signal(s))
                    else:
                        self._target = 0.0    # ещё тормозим — гвоздь позже
                        self._pos_brake = False
                else:
                    self._pos_sp = None       # стик живой → точка отпущена
                    self._pos_wait_t = None
                    self._pos_brake = False
                    self._target = cmd * self.cmd_gain
                    # трим ветра ЗАЩЁЛКНУТ на время толчка и выбега (_TRIM_LATCH)
                    self._i_hold = bool(self._TRIM_LATCH and pos is not None)
                sig = self._signal(s)
                err = sig - self._target                                # velocity-assist
            # первый брейк после enter() (трим ещё не выучен) — интегрируем со
            # скоростью ki_trim и вне упора: ошибка там 4v, набор трима быстрее всего
            ki = self.ki_trim if (self._pos_brake and self._trim_armed) else self.ki
            i_new = (self._i if self._i_hold
                     else clamp(self._i + ki * err * fdt, -self.imax, self.imax))
            dot = self._signal_dot(s)
            # разность соседних кадров уже считает производную ОШИБКИ (уставка в err),
            # готовой производной сигнала уставку надо вычесть руками
            d = (self.kd * (dot - self._sp_rate) if dot is not None
                 else self.kd * (err - self._prev_err) / fdt)
            self._prev_err = err
            if self.anti_windup:
                u_raw = self.kp * err + i_new + d
                if abs(u_raw) > self.max and u_raw * err > 0.0 and not self._i_hold:
                    # выход в упоре, ошибка толкает глубже — И-член не наматывать
                    # (см. __init__); в упоре БРЕЙКА — копить трим ветра (_BRAKE_TRIM)
                    i_new = (clamp(self._i + self.ki_trim * sig * fdt,
                                   -self.imax, self.imax)
                             if self._pos_brake and self._BRAKE_TRIM and self._trim_armed
                             else self._i)
            self._i = i_new
            if fr is not None:
                fr.set_trim_body(self._axis, self._i)   # компонента → мировой вектор
            u = clamp(self.kp * err + self._i + d, -self.max, self.max)
            self._out = self.osign * blend * u
            self._last_frame_sim = s.now_sim
            self._last_ok_sim = s.now_sim
        # Удержание выхода: полный авторитет `stale` секунд с последнего ГОДНОГО кадра
        # (типовой провал ipm_ok на скорости 0.3–0.5 с — переживается без потерь),
        # дальше линейный fade к нулю за ещё `stale` — команда старше 2·stale мертва.
        # Резать раньше нельзя (пила, см. выше), держать дольше — слепой полёт по
        # устаревшей команде.
        age = s.now_sim - self._last_ok_sim
        k = 1.0 if age < self.stale else clamp(2.0 - age / self.stale, 0.0, 1.0)
        off = int(self._out * k)
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

    def rate_dbg(self):
        """(цель, ошибка до неё, PWM на выходе) для бэга — или None у pos-осей.

        Зеркало `hold_dbg` для осей по СКОРОСТИ. Нужен по той же причине, по какой
        рысканию понадобился /flow_dbg6: без записи команды сегмент приходится искать
        по ИСТИННОЙ скорости, а там откат после торможения выглядит как ещё одна
        команда — калибровка R1 нашла три «проезда» на две команды миссии и дала
        разброс гейна 26%, где часть сегментов была не командой."""
        if self._cmd_mode == "pos":
            return None
        return (self._target, self._prev_err, self._out)


class DpRollHold(_FlowDamper1D):
    """Демпфер БОКОВОГО сноса по потоку → ROLL (был FlowDamper). Боевой пре-VINS.
    ⚠️ osign: drift_check подтвердил −1 (config.flow_osign=-1); класс-дефолт +1 (тесты)."""
    axes = frozenset({"roll"})
    _axis = "roll"

    def _signal(self, s): return s.flow_lateral

    def _cmd(self, sp):
        # ⚠️ МИНУС — тот же, что в GzHold._yawsp и DpYawHold._cmd, третья ось подряд.
        # Цепочка: `c_right>0` = стик ВПРАВО = борт должен разгоняться вправо; борт идёт
        # вправо → мир в кадре уезжает ВЛЕВО → `flow_lateral` ОТРИЦАТЕЛЕН (конвенция:
        # проекция на ось ВЛЕВО, см. roll_osign в config.py). Значит цель по сигналу при
        # стике вправо обязана быть отрицательной, а стояло `+c_right·gain`.
        # Замер R2 (первый прогон, где боковую команду вообще подали): токен mv_right →
        # цель +2.98 → PWM −32 → борт уехал ВЛЕВО на 4.38 м; mv_left → +15 → вправо на
        # 7.11 м. Петля при этом исправна — она честно ехала за своей зеркальной целью
        # (сигнал +2.00 на заказ +2.98). Баг дожил до сих пор потому, что боковую
        # команду не подавали ни разу: во всех сериях миссия была climb3,hover40,land.
        return -sp.c_right


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
    `_head = ∫flow_yaw·dt`. Единицы: 1 ед. = 1/S градусов, S = 0.324 px/кадр на °/с
    (замер Y4 — по СВОЕЙ уставке против истины; прежние 0.253 из регрессии flow_yaw на
    ω_z были мимо, там неизвестен покадровый шаг). S входит ТОЛЬКО в `cmd_gain`: ошибка
    контура живёт в сырых единицах сигнала, поэтому уточнение S гейнов не трогает.

    СТАРЫЙ ЗАКОН НИКУДА НЕ ДЕЛСЯ, он переехал в Д-слот: D-член считается по
    ЗАМЕРЕННОЙ скорости кадра (`_signal_dot` → `_dot`, принятый в накопитель step),
    d = kd·(flow_yaw − _sp_rate) — ровно прежнее `kp·(flow_yaw − цель)`. Победитель
    свипа [[yaw-hold-tuning]] kp=6 стал kd=6 БЕЗ переигрывания. При kp=0 поведение
    совпадает с прежним побитово; kp>0 включает курс-холд, ради которого всё и делалось.

    ⚠️ ПРУЖИНА (разбор yaw_ab_ki60_win03/spring, 2026-08-27). До `_dot` D-член брался
    разностью ОШИБКИ, куда утечка подмешивает −err/leak — при kp=0 это скрытый П-член
    kd/leak = 0.75 PWM/ед. Юнит-тест оценил его в «0.6% за кадр», но мерил при err≈0;
    в полёте контур не успевает за уставкой (заказ 50°/с, борт давал 11–14) и ошибка
    НАМАТЫВАЕТСЯ до 350–430° — тогда член утечки это 70–90 PWM, доминанта выхода. Итог
    двусторонний: во время команды намотанная ошибка ДУШИТ собственную команду (PWM
    пресса падал −47 → −29 от нажатия к нажатию), а после отпускания стика утечка
    сливает ошибку через D — контур сам РАЗМАТЫВАЕТ борт назад на 92–96% разворота
    (длинный пресс: +349° вправо → −330° обратно, τ≈10 с). Пик обратного PWM сошёлся
    с теорией kd/leak·S·err в пределах 3%. Лечение — `_dot`: утечка осталась жить в
    накопителе (для П-члена курс-холда), но производную больше не кормит.

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

    cmd_gain — в ЕДИНИЦАХ СИГНАЛА В СЕКУНДУ при полном стике, = S · (°/с). Дефолт 9.28
    = 0.324 · 28.65 °/с даёт полному стику тот же темп, что у GzHold (yaw_cmd_gain 0.5
    рад/с), — значит один и тот же токен mv_* крутит на одинаковый угол под обоими
    холдерами.

    ПРЯМАЯ ПЕРЕДАЧА СТИКА (`pilot_gain` > 0, PWM при полном стике). Пока yaw-стик жив,
    ось отдаёт пилоту провод: PWM = pilot_gain·c_yaw, зрительный контур ОБНУЛЯЕТСЯ
    каждый тик (`enter`) — уставке не из чего наматываться, отпускание = демпфер с
    чистого листа (после взведения arm_frames). Это тот же ход, что открытый контур
    ControlStack на незанятой оси (c_yaw → PWM выше центра при стике вправо), поэтому
    знак БЕЗ osign. Смысл двойной: (1) пружина невозможна по построению — стирается
    сам её накопитель; (2) полный стик получает честный темп FCU, а не хвост погони
    контура за уставкой (замер spring: заказ 0.86·60≈50°/с, борт давал 11–14°/с).
    Цена: пока стик жив, визуального демпфера нет — рыскание держит rate-контур FCU
    (RC yaw = команда скорости, центр = стоп), этого достаточно. 0 = выкл (токены
    yaw_l/yaw_r калиброваны через уставку — им передача не нужна и вредна).
    В /flow_dbg6 передача видна как (0, 0, PWM≠0)."""
    axes = frozenset({"yaw"})
    _axis = "yaw"
    _cmd_mode = "pos"

    def __init__(self, kp=0.0, ki=0.0, kd=6.0, imax=200.0, max_pwm=150.0,
                 conf_min=0.05, conf_full=0.20, osign=1.0, cmd_gain=9.28,
                 stale_sec=0.5, leak_sec=8.0, max_step=32.4, arm_frames=5,
                 pilot_gain=0.0, v_gate=0.0):
        super().__init__(kp, ki, kd, imax, max_pwm, conf_min, conf_full,
                         osign, cmd_gain, stale_sec)
        self.leak = leak_sec
        self.max_step = max_step
        self.arm_frames = arm_frames
        self.pilot_gain = pilot_gain
        # v_gate (м/с): на ходу быстрее этого картинке про курс НЕ ВЕРИТЬ — кадр
        # засчитывается как «вращения нет» (накопитель и D-член не двигаются), курс
        # держит сам FCU. Замер ab_frame (лобовой flow_yaw против истинной ω_z):
        # |v| < 0.3 м/с — наклон 0.90, corr 0.95, остаток 5 °/с; 0.3–1 — 0.96/0.96/6;
        # > 1 м/с — 0.74/0.58/21 °/с: параллакс хода читается как разворот, D-член
        # (kd=6) отвечает на него ±110 PWM, и борт на прямой в 2 м/с «ворочается»
        # ±40 °/с (scene_hud 25–31 с). Скорость — из канала вида сверху (ipm_v*),
        # он от разворота уже очищен деротом. 0 = без гейта.
        self.v_gate = v_gate
        self._gated = 0           # кадров, отброшенных гейтом хода (диагностика)
        self._head = 0.0
        self._dot = 0.0           # замеренная скорость последнего кадра — для D-члена
        self._armed = 0           # сколько подряд кадров картинке уже можно верить
        self._rejects = 0         # выброшенных кадров (диагностика, не управление)

    def enter(self, s: DroneState) -> None:
        super().enter(s)
        self._head = 0.0          # визуальный курс отсчитывается от входа в сегмент
        self._dot = 0.0
        self._armed = 0
        self._rejects = 0
        self._gated = 0

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        # ПРЯМАЯ ПЕРЕДАЧА (см. docstring): стик жив → PWM со стика, контур обнулён.
        # c_yaw=0.0 гарантирован мёртвой зоной траектории (PilotTrajectory._norm_axis).
        if self.pilot_gain > 0.0 and sp.c_yaw != 0.0:
            self.enter(s)
            off = int(clamp(self.pilot_gain * sp.c_yaw, -self.max, self.max))
            rc = RcCommand(throttle=RC_CENTER)
            setattr(rc, self._axis, RC_CENTER + off)
            return rc
        return super().update(s, sp, dt)

    def _signal_ok(self, s) -> bool:
        """ВЗВЕДЕНИЕ: не копить курс, пока картинке нельзя верить (замер YW1s1).

        Чем это было. На отрыве от земли (z 0.2→0.7) первый же кадр потока дал
        flow_yaw = −43.8 px/кадр при висенческих ±1: земля уходит, обдув винтов, масштаб
        сцены рвётся. Накопитель мгновенно набрал −32 ед. = −99° ФАНТОМНОГО разворота
        (S=0.324), контур упёрся в −150 PWM и честно довернул борт влево ровно на этот
        угол — 95° за полторы секунды, пик 81 °/с. Дальше накопитель отработался в ноль,
        команда пропала, и борт остался на новом курсе НАВСЕГДА: абсолютной опоры у оси
        нет, утечка тянет к уставке, а не к исходному курсу.

        Гейт `conf_min=0.05` это пропустил — на том кадре достоверность была 0.18.
        Поэтому накопителю нужен СВОЙ порог, строже выходного: `conf_full` подряд
        `arm_frames` кадров. Одиночный хороший кадр посреди срыва ничего не взводит.

        ⚠️ Метод МЕНЯЕТ СОСТОЯНИЕ — в отличие от чистого `DpPitchHold._signal_ok`. Так
        можно потому, что база зовёт его ровно раз на новый кадр (`flow_seq != _last_seq`),
        и это единственное место, где счётчику видно, что кадр НОВЫЙ. False здесь
        не даёт вызвать `_advance` — фантом не попадает в накопитель вовсе, а не
        гасится потом; НОВОЙ команды такой кадр тоже не рождает (старая держится и
        гаснет по часам `_last_ok_sim`, на отрыве она — ноль: ось ещё не взведена).
        """
        if s.flow_conf < self.conf_full:
            self._armed = 0
            return False
        if self._armed < self.arm_frames:
            self._armed += 1
            return False
        return True

    def _advance(self, s, fdt) -> None:
        step = s.flow_yaw
        if self.v_gate > 0.0 and math.hypot(s.ipm_vfwd, s.ipm_vlat) > self.v_gate:
            self._gated += 1
            step = 0.0            # ход: параллакс ≠ разворот (см. v_gate в __init__)
        # ОТСЕВ НЕВОЗМОЖНОГО: 43.8 px/кадр это 135 °/с (S=0.324) у висящего борта —
        # столько он не поворачивается. Полный стик даёт 28.65 °/с, так что потолок
        # 100 °/с (max_step = S·100 = 32.4) оставляет команде троекратный запас и при
        # этом режет взлётный выброс. Кадр ВЫБРАСЫВАЕТСЯ, а не подрезается: подрезка
        # всё равно влила бы в накопитель максимально допустимый фантом, то есть
        # заменила бы разворот на 99° разворотом на 75°.
        if self.max_step > 0.0 and abs(step) > self.max_step:
            self._rejects += 1
            step = 0.0
        # Принятый step и есть замеренная скорость курса этого кадра — им кормится
        # D-член (_signal_dot). Утечка ниже правит только накопитель: попади она в
        # производную, при kp=0 получается скрытый П-член kd/leak по накопленной
        # ошибке — та самая ПРУЖИНА (см. docstring класса).
        self._dot = step
        self._head += step * fdt
        if self.leak > 0.0:
            # утечка К УСТАВКЕ, а не к нулю. Ноль был опорой, пока уставка не ездила;
            # с командой он стал произвольной точкой, и утечка к нему СЪЕДАЛА разворот:
            # замер Y2 — на токене 60° (7с команды + 6с добора) накопитель просел с 82°
            # до 46°, борт добирал разницу и переворачивал на +25%. Утечка ошибки даёт
            # ровно то, ради чего заводилась: фантом от смещения по-прежнему ограничен
            # bias·T. ⚠️ «На командах ошибка мала» верно только при kp>0 (контур её
            # закрывает); при kp=0 команда через уставку наматывает err до сотен
            # градусов (замер spring) — потому D-член и кормится _dot, а не разностью.
            self._head += (self._sp - self._head) * min(1.0, fdt / self.leak)

    def _signal(self, s): return self._head

    def _signal_dot(self, s):
        """Замеренная скорость курса (принятый step кадра) — чтобы D-член видел только
        НАСТОЯЩЕЕ вращение. Разность err[k]−err[k−1] несла ещё и шаг утечки −err·fdt/leak,
        и на намотанной ошибке этот довесок вырастал в пружину (разбор spring)."""
        return self._dot

    def _cmd(self, sp):
        # ⚠️ МИНУС — тот же, что в GzHold._yawsp. `c_yaw>0` = стик ВПРАВО, а `flow_yaw`
        # (медиана горизонтального сдвига картинки) при развороте вправо ОТРИЦАТЕЛЕН:
        # сцена уезжает влево. Значит уставка накопленного курса обязана ехать в минус.
        # Без минуса ось отрабатывала команду ЗЕРКАЛЬНО — замер Y2 на всех четырёх
        # точках свипа: yaw_l30 давал −21…−28°, yaw_r60 давал +70…+76°.
        return -sp.c_yaw


class _AltSettled:
    """«ВЫСОТА УСПОКОИЛАСЬ» — по образцу заморозки опоры (flow_estimator.py:318).

    ⚠️ ЭТО ЗАМЕНА ДИФФЕРЕНЦИАТОРА, и замена не косметическая. Первая версия гейта
    (`_ClimbRate`) считала вертикальную скорость наклоном МНК по баро в окне 0.5 с и
    сравнивала с порогом 0.35 м/с — то есть делала ровно то, от чего предостерегает
    комментарий у опоры: «отличаем по ДЛИТЕЛЬНОСТИ, а не по величине: не нужен
    дифференциатор высоты со своим шумом». Серия IG1s показала цену: в висении |vz|
    болтается прямо ПО порогу, гейт дребезжал, тангаж молчал не 4 секунды набора, а
    весь полёт — борт разогнало ветром до 7 м/с и унесло на 39 м во всех трёх прогонах.

    Здесь производной нет вовсе. Есть опорная высота и счётчик: пока борт держится в
    полосе ±`band` вокруг опорной, копится «спокойное» время; вышел за полосу — счётчик
    в ноль, а опорной становится текущая высота (иначе после набора полоса осталась бы
    внизу и гейт не открылся бы никогда). Спокойно дольше `still` секунд → высота
    установившаяся, каналу можно верить.

    `band` берётся ВЫШЕ болтанки удержания (0.2–0.4 м на трёх метрах) и НИЖЕ размаха
    набора (3 м): у опоры порог 6% = 19 см на этой высоте срабатывал на штатном
    удержании — ту же ошибку тут повторять нельзя.
    """

    def __init__(self, band=0.5, still=0.5):
        self.band = band
        self.still = still
        # `moving` отделяет «высота ЕДЕТ» от «ещё коплю спокойное время»: обе ситуации
        # закрывают ось, но в разборе это разные диагнозы — набор против первых кадров
        # сегмента. Без разделения счётчик блокировок ловил бы старт и врал.
        self.moving = False
        self._a0 = None
        self._quiet = 0.0
        self._t = None

    def reset(self):
        self.moving = False
        self._a0 = None
        self._quiet = 0.0
        self._t = None

    def update(self, t, alt):
        """→ True, если высота установилась. None — «не знаю» (нет баро/первый кадр)."""
        if alt is None:
            self._t = None
            return None
        dt = 0.0 if self._t is None else max(0.0, t - self._t)
        self._t = t
        if self._a0 is None:
            self._a0 = alt
            self.moving = False
            return None
        if abs(alt - self._a0) > self.band:
            self._a0 = alt          # опорная едет за бортом: набор кончится — полоса с ним
            self._quiet = 0.0
            self.moving = True
            return False
        self._quiet += dt
        self.moving = False
        # СТРОГОЕ сравнение — не арифметическая мелочь. На монотонном наборе quiet
        # копится, пока высота ИДЁТ сквозь полосу, и при v = band/still (ровно 1 м/с
        # на дефолтах) добегает до порога В ТОМ ЖЕ кадре, где полоса ещё не пробита:
        # `>=` открывало гейт на один кадр каждые band/v секунд. Пока провал сигнала
        # обнулял выход, это был блип в 50 мс; с hold+fade такой кадр командовал бы
        # фантомом набора по полсекунды. Наборы МЕДЛЕННЕЕ band/still гейт не видит
        # по конструкции (порог корридора), это документированная цена.
        # Эпсилон — потому что quiet СУММА кадровых dt: десять сложений по 0.05
        # дают 0.5000000000000001, и голое строгое сравнение пробивается на тех же
        # кадрах, что и `>=`.
        return self._quiet > self.still + 1e-9


class _IpmGated(_FlowDamper1D):
    """ГЕЙТ ДОВЕРИЯ к каналу вида сверху — то же лечение, что получил курс на YW1s1,
    перенесённое на оси по скорости. Три независимых отказа, три разных гейта:

    1. `max_speed` — ПРАВДОПОДОБИЕ кадра. Скорость выше физически возможной для рамы =
       мусор, кадр не командует. Как и у курса, кадр ВЫБРАСЫВАЕТСЯ, а не подрезается:
       подрезка выдала бы максимально допустимую команду по мусорному кадру.
    2. `alt_band` — НАБОР/СНИЖЕНИЕ (`_AltSettled`). Полоса земли лежит ВПЕРЕДИ борта
       (X≈3…6 м), поэтому любая ошибка высоты двигает её вдоль X, и вертикальный ход
       читается каналом как продольный. Замер по 8 прогонам (`ipm_out.py`/`ipm_vz.py`):
       в окне взлёта канал показывал 4.5–7.2 м/с вперёд при истинных ≤1.3, наклон ошибки
       по vz +0.67 м/с на м/с, и тангаж СИДЕЛ В НАСЫЩЕНИИ 22–43% первых трёх секунд —
       гонялся за фантомом набора. Отдельный гейт нужен потому, что фантом ПЛАВНЫЙ: он
       проходит весь диапазон от 0 до 7 м/с, и потолок правдоподобия его не ловит.
       ⚠️ Судим по установившейся высоте, НЕ по вертикальной скорости — почему именно
       так, разобрано в `_AltSettled` (серия IG1s, ось молчала весь полёт).
    3. `arm_frames` — ВЗВЕДЕНИЕ. Ось не командует, пока не пришло N подряд кадров,
       прошедших всё выше. Любой сбой обнуляет счётчик.

    ⚠️ Метод МЕНЯЕТ СОСТОЯНИЕ — база зовёт его ровно раз на новый кадр
    (`flow_seq != _last_seq`); False замораживает PID (команда держится по часам
    `_last_ok_sim` и гаснет к 2·stale) и забывает производную.
    ⚠️ АВТОРИТЕТ ЭТИХ ОСЕЙ = 1 (`_authority` переопределён): базовый blend по
    flow_conf судил их по здоровью ЧУЖОГО сигнала и душил у земли — разбор в
    самом методе.
    ⚠️ Гейт по высоте — ПО ОСЯМ РАЗНЫЙ, и это не забывчивость: боковая ось геометрически
    почти не задета (полоса не смещается вбок при смене высоты — замер: наклон +0.25
    против +0.67, насыщения крена нет ни в одном прогоне), а слепой крен на наборе
    отдаёт борт ветру на все 4 секунды. Дефолт для крена — 0 (выключено).
    """

    def __init__(self, *a, max_speed=0.0, alt_band=0.0, alt_still=0.5,
                 arm_frames=0, **kw):
        super().__init__(*a, **kw)
        self.max_speed = max_speed
        self.alt_band = alt_band
        self.arm_frames = arm_frames
        self._alt = _AltSettled(band=alt_band, still=alt_still)
        self._armed = 0
        self._rejects = 0        # кадров отброшено (диагностика, не управление)
        self._alt_blocks = 0     # кадров закрыто по неустановившейся высоте

    def enter(self, s: DroneState) -> None:
        super().enter(s)
        self._alt.reset()
        self._armed = 0
        self._rejects = 0
        self._alt_blocks = 0

    def _signal_ok(self, s) -> bool:
        settled = self._alt.update(s.now_sim, s.rel_alt)
        if not s.ipm_ok:
            self._armed = 0
            return False
        if self.max_speed > 0.0 and (abs(s.ipm_vfwd) > self.max_speed
                                     or abs(s.ipm_vlat) > self.max_speed):
            self._rejects += 1
            self._armed = 0
            return False
        # settled is None = нет баро или первый кадр: судить не по чему, но и доверять
        # рано — взведение всё равно держит ось молча первые кадры сегмента.
        if self.alt_band > 0.0 and settled is False:
            self._alt_blocks += int(self._alt.moving)   # копим только НАСТОЯЩИЙ ход высоты
            self._armed = 0
            return False
        if self._armed < self.arm_frames:
            self._armed += 1
            return False
        return True

    def _authority(self, s) -> float:
        # АВТОРИТЕТ = 1: здоровье IPM-канала уже судят ipm_ok + гейты выше
        # (_signal_ok) с hold+fade базы. Базовый blend — доверие к ПОЛНОКАДРОВОМУ
        # потоку, то есть чужому сигналу: ниже ~1.2 м полнокадровый LK теряет
        # фичи (conf 0.04–0.31 по бинам высоты) при живом IPM-канале (годен 99%),
        # и оси при ошибке +5.4 м/с выдавали 0–50 PWM вместо упора 150 — «борт
        # не слушает стики у земли». Прогон lv2_joy_20260825_214015:
        # corr(|PWM|, blend·150)=+0.99, медианы совпали точно. На ≥1.5 м blend
        # и так был 1.0 → для отлётанных серий изменение нейтрально.
        return 1.0


class DpPitchRate(_IpmGated):
    """ДЕМПФЕР продольной СКОРОСТИ по виду сверху → PITCH. Ось `rate`, как крен.

    Зачем вместо DpPitchHold. Задача пре-VINS слоя — ОСТАНОВИТЬ борт, а не вернуть его
    в точку. Для остановки нужна СКОРОСТЬ, а у скорости нет накопления: значит физически
    не могут существовать отказы, которые съели кампанию E2-E7 — неверно засчитанный
    сегмент, стёртая память, квантование накопителя в ±0.03 (разбор в ToDo5.md).

    Подтверждение уже лежало в лётных данных, просто читалось не так: крен — ось ПО
    СКОРОСТИ, тангаж был ПО ПОЛОЖЕНИЮ, и уход по осям на одной раме и одних кадрах
    расходится втрое-всемеро:
        серия   вперёд (положение)   вбок (скорость)
        E2      +20.2 ± 13.7 м       +5.8 ± 7.5 м
        E3       −3.0 ± 27.3         −0.9 ± 3.2
        E6      +17.0 ± 23.8         −2.5 ± 5.9
        E7       +3.6 ± 29.7         −1.0 ± 3.7

    Почему скорость годится ТЕПЕРЬ, хотя когда-то её отвергли. Отвергали `flow_longitudinal`
    (медиана покадрового вертикального сдвига) — corr −0.22 с истинной скоростью, потому
    что 93% точек сидели на линии горизонта и почти не движутся. Здесь сигнал другой:
    `ipm_vfwd` — наклон МНК по МЕТРИЧЕСКОМУ пути из выпрямленной полосы земли. Замер по
    трём бэгам (ipm_flow_test.py): путь 0.97-1.03 при corr 0.99-1.00, скорость на окне
    0.5 с — 0.91-0.94 при corr 0.76-0.84 и ошибке 0.65-0.82 м/с (истинная СКО ~1.0).

    ⚠️ ЕДИНИЦЫ ДРУГИЕ: сигнал в М/С, а не в log-единицах. Гейны тангажа из кампании N
    (kp=1500, kd=1500) считались под log-масштаб и здесь НЕ годятся — их надо мерить
    заново. Отправная точка от крена: у него сигнал в px/кадр, тут метры в секунду.
    ⚠️ Демпфер не возвращает в точку: остаточная скорость 0.2 м/с за 20 с висения даёт
    4 м сноса. Для задачи «дожить до инициализации VINS» это приемлемо, для удержания —
    нет; абсолютную позицию даёт VINS/NN1."""
    axes = frozenset({"pitch"})
    _axis = "pitch"
    _cmd_mode = "rate"

    def _signal(self, s): return s.ipm_vfwd

    def _pos_signal(self, s):
        # путь вперёд, М (та же лево/вперёд-конвенция, что у скорости → знаки
        # станции сходятся автоматически: цель = pos_kp·(точка − путь))
        return s.ipm_fwd

    def _cmd(self, sp):
        # знак — как у DpPitchHold: c_fwd>0 = стик ВПЕРЁД = борт должен ехать вперёд,
        # и продольная скорость по виду сверху при ходе вперёд ПОЛОЖИТЕЛЬНА (крутизна
        # +1.00/+1.03/+0.97 по трём бэгам), поэтому цель прямая, без инверсии.
        return sp.c_fwd


class DpRollRate(_IpmGated):
    """ДЕМПФЕР бокового сноса по МЕТРИЧЕСКОЙ скорости вида сверху → ROLL.

    То же, что DpRollHold, но сигнал в М/С вместо px/кадр. Прежний `flow_lateral` —
    медиана горизонтального потока в пикселях, и цена метра в секунду у неё зависит от
    ГЛУБИНЫ точек ровно так же, как у убитого масштабного канала тангажа: замер по
    L1_scale2ax дал S_lat 0.40 px/(м/с) по всему кадру против расчётных 1.3-3.3, потому
    что 93% точек садились на линию горизонта, где потока почти нет. Под маской
    feat_lo=0.667 вышло 2.42.
    Ось от этого работала — демпферу хватает верного ЗНАКА и монотонности, а ошибку
    масштаба поглощал подобранный kp=48. Но это значит, что 48 — не «гейн контура», а
    «гейн контура, умноженный на неизвестную цену пикселя», и он поедет при любой смене
    высоты, наклона или сцены.

    Почему сейчас. После перевода тангажа на метры (DpPitchRate, серия E8: уход
    2.1-4.3 м, 100% времени внутри круга 10 м, пик 1.4-1.8 м/с) остаток ухода стал
    БОКОВЫМ: вперёд +1.05/+1.93/+2.80/+0.27 против вбок −2.27/−3.84/−2.54/−2.10.
    Ведущей осью впервые оказался крен — то есть теперь он и есть узкое место.

    Гейн: 48 px/кадр-единиц × 2.42 px/(м/с) ≈ 116 PWM на 1 м/с. Отсюда дефолт 120 —
    это ПЕРЕСЧЁТ уже подобранного значения в новые единицы, а не свежая догадка.
    Рядом стоит pitch_rate_kp=200, порядок тот же.
    ⚠️ Боковая крутизна канала мерилась хуже продольной: 0.91/1.06/1.09 при corr
    0.64/0.96/0.81 против 0.97-1.03 при corr 0.99-1.00 (ipm_flow_test, три бэга) —
    и со ЗНАКОМ, перепутанным в ground truth того замера (twist.y это «влево», не
    «вправо»: body-twist Gazebo — FLU). Правильный знак — в _cmd."""
    axes = frozenset({"roll"})
    _axis = "roll"
    _cmd_mode = "rate"

    def _signal(self, s): return s.ipm_vlat

    def _pos_signal(self, s):
        return s.ipm_lat        # путь вбок, М (лево+, как ipm_vlat)

    def _cmd(self, sp):
        # ⚠️ МИНУС — тот же, что у DpRollHold, и по той же геометрии: ipm_vlat
        # ЛЕВО-положителен (мир в выпрямленной полосе уезжает влево при ходе вправо,
        # как и flow_lateral). Первая редакция шла без минуса, поверив калибровке
        # «боковая крутизна +0.91/+1.06/+1.09 вправо» — та мерялась против twist.y в
        # предположении FRD, а body-twist Gazebo — FLU (y = ВЛЕВО). Полёт 2026-08-18
        # дал два независимых опровержения: (1) ipm_vlat ≈ −v_right с наклоном ~1
        # (v_right из производной МИРОВОЙ позиции + курс, не из twist); (2) демпфер
        # с osign=+1 устойчиво ТОРМОЗИТ — при право-положительном сигнале это была бы
        # положительная обратная связь. Симптом без минуса: стик вправо вёз борт
        # ВЛЕВО — контур сходился к цели идеально, но зеркальной.
        return -sp.c_right


class StationFrame:
    """РАМА СТАНЦИИ В ОСЯХ КУРСА — общая для крена и тангажа.

    Зачем. Станция и трим ветра жили в осях БОРТА: путь ipm_lat/ipm_fwd копится
    покомпонентно без поворота, гвоздь — точка на этом пути, И-член каждой оси —
    компонента трима вдоль оси борта. Разворот всё это ломает: полёт
    lv2_joy_20260829_153405, разворот 200° за 4 с в 5 м/с — стиков крена/тангажа
    нет, цели станции 0, а борт разгоняется с 0.06 до 1.38 м/с: трим (−50 PWM в
    тангаже) после разворота смотрит В ОБРАТНУЮ сторону и толкает ПО ветру, гвоздь
    сбрасывался каждые 17° курса, точка терялась. Здесь всё три вещи — мировые:
    - позиция (x, y): приращения пути IPM (тело: вперёд/влево) поворачиваются
      курсом ψ и суммируются — раз на кадр, идемпотентно по flow_seq;
    - гвоздь (px, py): ОДИН на обе оси, ставится осью, которая только что
      затормозила (set_pin); ошибка оси = компонента (гвоздь − позиция) вдоль
      её оси ТЕКУЩЕГО курса — после разворота точка на месте;
    - трим (tx, ty): вектор PWM в мировых осях; ось читает компоненту вдоль
      себя (trim_body), интегрирует по своему закону и пишет обратно
      (set_trim_body — другая компонента не тронута). После разворота трим
      сам поворачивается в оси борта — толчка по ветру нет.
    КУРС — ПОДКЛЮЧАЕМЫЙ ВХОД: `heading(s) → рад (ENU, как att_yaw)`. Сейчас —
    курс FCU (гиро + компас EKF; в симе идеален, им же считается порог 17°);
    для борта без компаса сюда встанет визуальный курс (лобовой или поворот
    полосы IPM) — станцию при этом переделывать не надо.
    Условности осей: тело = (вперёд, влево) — как ipm_vfwd/ipm_vlat; мир = ψ от
    оси x против часовой (ENU-курс). Сброс пути перцепцией (ipm_* = 0 на новом
    сегменте) распознаётся по точному нулю пары и приращения не даёт."""

    def __init__(self, heading=None):
        self.heading = heading if heading is not None else (lambda s: s.att_yaw)
        self.reset()

    def reset(self):
        self._seq = -1
        self._prev = None
        self.x = self.y = 0.0
        self.psi = 0.0
        self.pin = None
        self.trim = [0.0, 0.0]
        self._live = {}

    def _rot(self):
        return math.cos(self.psi), math.sin(self.psi)

    def advance(self, s) -> None:
        if s.flow_seq == self._seq:
            return
        self._seq = s.flow_seq
        self.psi = float(self.heading(s))
        cur = (float(s.ipm_fwd), float(s.ipm_lat))
        if self._prev is not None and not (cur[0] == 0.0 and cur[1] == 0.0):
            df, dl = cur[0] - self._prev[0], cur[1] - self._prev[1]
            c, si = self._rot()
            self.x += df * c - dl * si
            self.y += df * si + dl * c
        self._prev = cur
        # телеметрия рамы — в снапшот (→ /mission/status: sf/sx/sy/spx/spy)
        s.st_frame = 1
        s.st_x, s.st_y = self.x, self.y
        if self.pin is not None:
            s.st_px, s.st_py = self.pin
        else:
            s.st_px = s.st_py = float('nan')

    def set_pin(self) -> None:
        self.pin = (self.x, self.y)

    def drop_pin(self) -> None:
        self.pin = None

    def body_err(self, axis):
        """Компонента (гвоздь − позиция) вдоль оси тела: 'pitch' → вперёд, 'roll' → влево."""
        if self.pin is None:
            return None
        ex, ey = self.pin[0] - self.x, self.pin[1] - self.y
        c, si = self._rot()
        return ex * c + ey * si if axis == "pitch" else -ex * si + ey * c

    def trim_body(self, axis) -> float:
        tx, ty = self.trim
        c, si = self._rot()
        return tx * c + ty * si if axis == "pitch" else -tx * si + ty * c

    def set_trim_body(self, axis, value) -> None:
        f, l = self.trim_body("pitch"), self.trim_body("roll")
        if axis == "pitch":
            f = float(value)
        else:
            l = float(value)
        c, si = self._rot()
        self.trim = [f * c - l * si, f * si + l * c]

    def stick(self, axis, live: bool) -> None:
        self._live[axis] = bool(live)

    def any_stick(self) -> bool:
        return any(self._live.values())


class DpHold(StabilizationStrategy):
    """Демпфер по ВСЕМ трём осям — композит DpRollHold+DpPitchHold+DpYawHold. Каждая
    ось читает свой сигнал потока (разные единицы), поэтому композит, а не одна база.

    `frame` — StationFrame (станция в осях курса): вешается на rate-оси крена и
    тангажа; None — станция в осях борта, как было."""
    axes = frozenset({"roll", "pitch", "yaw"})

    def __init__(self, roll=None, pitch=None, yaw=None, frame=None):
        self._subs = [roll or DpRollHold(), pitch or DpPitchHold(), yaw or DpYawHold()]
        self.frame = frame
        if frame is not None:
            for x in self._subs:
                if hasattr(x, "frame") and getattr(x, "_axis", None) in ("roll", "pitch"):
                    x.frame = frame

    def enter(self, s: DroneState) -> None:
        if self.frame is not None:
            self.frame.reset()
        for x in self._subs:
            x.enter(s)

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        rc = RcCommand(throttle=RC_CENTER)
        for x in self._subs:
            out = x.update(s, sp, dt)
            for ax in x.axes:
                setattr(rc, ax, getattr(out, ax))
        return rc
