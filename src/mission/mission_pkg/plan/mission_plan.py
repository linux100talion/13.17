#!/usr/bin/env python3
"""compile_mission — компиляция плейлиста профиль-токенов в план (список Step'ов).

Полётное задание как ДАННЫЕ, ортогонально стабилизатору:
    BS_MISSION=[climb3, mv_fwd2, mv_bkwd4, landing3]   (что летим — движение)
    BS_STAB=DpRollHold+DpYawHold                        (как держимся — стабилизация)
→ шаги PlanRunner: prearm→arm→<токены>→land. Каждый Control-сегмент получает СВЕЖИЙ
ControlStack(build_stabilizers(spec) + профиль-сегмент): один стабилизатор-spec на всё
задание, разная траектория на сегмент.

Грамматика токена: глагол+число (climb=метры, mv/hover=секунды; уровень стика —
глобальный cfg.mv_level):
  climb<h>              — набор высоты h метров (Step Climb; абортит на 'land')
  mv_fwd/bkwd<t>        — продольный стик ±level на t sim-сек (ПОСТОЯННЫЙ наклон — уносит!)
  mv_right/left<t>      — боковой стик ±level
  mv_cw/ccw<t>          — yaw-стик ±level
  yaw_r/l<град>         — разворот на ЗАДАННЫЙ УГОЛ (yaw_l30 = 30° ВЛЕВО, yaw_r60 = вправо):
                          команда И добор ОДНИМ сегментом. В отличие от mv_cw<t> градусы
                          тут величина, за которую отвечает контур, а не производная от
                          того, сколько борт успел довернуть за отпущенное время
  sk_fwd/bkwd<τ>        — station-keeping оператор на pitch: +τ/−2τ/+τ, дрон У ТОЧКИ (не уносит)
  sk_right/left<τ>      — то же на roll
  hover<t>              — держать t сек (стик центр; стабилизатор гасит снос/держит)
  pilot[t]              — ЖИВОЙ ПИЛОТ t сек: стики = намерение (RcTransmitter), а
                          трёхпозиционник CH6 на лету выбирает стабилизацию:
                          −1 = BS_STAB (наш), 0 = чистый ALT_HOLD (стики-наклоны),
                          +1 = MANUAL (Арбитр, сырые стики). БЕЗ числа — бессрочно:
                          летаем до `make pilot-done` (топик /mission/pilot_done →
                          s.pilot_done), затем миссия идёт дальше (land). Требует
                          --pilot joy/ros; со scripted-пилотом токен = hover (центр,
                          стек BS_STAB), бессрочная форма scripted'у запрещена.
  land | landing<x>     — посадка (Step Land; число игнорируется)
  freefly               — СВОБОДНЫЙ ПОЛЁТ, миссия-одиночка ПО ОПРЕДЕЛЕНИЮ (другие
                          токены рядом — ошибка). Борт просто спавнится; пилот с
                          пульта сам армит (руддером), взлетает, летает, садится и
                          дизармит. Газ СЫРОЙ (без защёлки и AltHold), ГЕОЗАБОР
                          ИГНОРИРУЕТСЯ, селектор CH6 работает как в pilot.
                          Завершение миссии = дизарм. Требует --pilot joy/ros.

Реестр именованных заданий — MISSIONS (значение = список токенов или callable(cfg)).
Инлайн-список ('climb3,mv_fwd2 …') парсится напрямую (запятые/пробелы).
"""
import re

from control_pkg.application.control_stack import ControlStack
from control_pkg.domain.control.altitude import AltHold
from control_pkg.domain.control.excitation import NoExcitation
from control_pkg.domain.control.trajectory import (ConstProfile, RcTransmitter,
                                                   StationKeep, StaticSetpoint, YawTurn)
from control_pkg.domain.rc import RC_MIN_THR

from ..recipes import build_stabilizers
from .step import Arm, AwaitMode, Climb, Control, Freefly, Land

_TOKEN = re.compile(r'^([a-z_]+?)(-?\d+(?:\.\d+)?)?$')

# глагол движения → (c_fwd, c_right, c_yaw) единичного направления (× level)
_MV = {
    "mv_fwd":   (1.0, 0.0, 0.0), "mv_bkwd": (-1.0, 0.0, 0.0),
    "mv_right": (0.0, 1.0, 0.0), "mv_left":  (0.0, -1.0, 0.0),
    "mv_cw":    (0.0, 0.0, 1.0), "mv_ccw":   (0.0, 0.0, -1.0),
}

# разворот на угол → знак стика (c_yaw>0 = ВПРАВО, стик-конвенция; проверено Y1s)
_YAW = {"yaw_r": 1.0, "yaw_l": -1.0}

# station-keeping оператор → (ось, знак первого импульса): +τ/−2τ/+τ, возврат к точке
_SK = {
    "sk_fwd":  ("fwd", 1.0), "sk_bkwd": ("fwd", -1.0),
    "sk_right": ("right", 1.0), "sk_left": ("right", -1.0),
}


def parse_tokens(spec):
    """'climb3,mv_fwd2 mv_bkwd4' → ['climb3','mv_fwd2','mv_bkwd4'] (запятые/пробелы).
    Список/кортеж возвращается как есть."""
    if isinstance(spec, (list, tuple)):
        return list(spec)
    return [t for t in re.split(r'[,\s]+', str(spec).strip()) if t]


def _parse(tok):
    m = _TOKEN.match(tok)
    if not m:
        raise ValueError(f"нераспознан токен миссии {tok!r}")
    return m.group(1), (float(m.group(2)) if m.group(2) else None)


# ---- именованные задания (значение: список токенов | callable(cfg)->список) ----
MISSIONS = {
    # пример из диалога: взлёт 3м → вперёд 2с → назад 4с → посадка
    "Mission1": ["climb3", "mv_fwd2", "mv_bkwd4", "landing3"],
    # квадрат стик-профилем (уровень cfg.mv_level, 3с на сторону)
    "square":   ["climb3", "mv_fwd3", "mv_right3", "mv_bkwd3", "mv_left3", "land"],
    # боевой station-keeping: оператор гоняет pitch +τ/−2τ/+τ (у точки), флоу демпфит roll/yaw
    "sk_demo":  ["climb3", "sk_fwd3", "sk_fwd3", "land"],
    # bootstrap как профиль-миссия: взлёт → висеть (стабилизатор держит) → посадка
    "bootstrap": lambda cfg: [f"climb{cfg.alt:g}",
                              f"hover{max(1.0, cfg.flow_hold_sec):g}", "land"],
}


def resolve_mission(cfg, spec):
    """Имя из MISSIONS (в т.ч. callable(cfg)) ИЛИ инлайн-строка/уже-список токенов → список.
    Идемпотентна: приняв уже разрешённый список, возвращает его как есть (двойной резолв)."""
    if isinstance(spec, (list, tuple)):
        return list(spec)
    if spec in MISSIONS:
        val = MISSIONS[spec]
        return list(val(cfg)) if callable(val) else list(val)
    return parse_tokens(spec)


def compile_mission(cfg, mission, stab_spec, handover=None, keep="ALT_HOLD",
                    live_pilot=False):
    """mission: имя/список токенов; stab_spec: имя(+склейка) стабилизатора. → [Step]."""
    tokens = resolve_mission(cfg, mission)
    # --- freefly: миссия-одиночка, весь цикл руками пилота (см. грамматику) ---
    if any(_parse(t)[0] == "freefly" for t in tokens):
        if len(tokens) != 1:
            raise ValueError(f"freefly — миссия-одиночка по определению, другие "
                             f"токены рядом не имеют смысла: {tokens!r}")
        if not live_pilot:
            raise ValueError("freefly требует живого пилота (--pilot joy/ros): "
                             "арм/взлёт/посадка/дизарм — руками с пульта")
        stack = ControlStack(build_stabilizers(cfg, stab_spec),
                             RcTransmitter(cfg.pilot_deadzone, cfg.pilot_full,
                                           cfg.pilot_pitch_sign, cfg.pilot_roll_sign),
                             NoExcitation(), slew=cfg.slew)
        return [AwaitMode("prearm", keep, RC_MIN_THR, cfg.mode_budget),
                Freefly("freefly", stack, keep=keep,
                        pilot_stabs=build_stabilizers(cfg, stab_spec))]
    wait_gt = "Gz" in str(stab_spec)     # gz-семейство держит позицию по gt (sim-оракул)
    hold = cfg.throttle_hold
    lvl = cfg.mv_level
    # ОДИН контур высоты на всё задание: набор и удержание должны говорить с FCU одним
    # законом, иначе на стыке шагов высота прыгает (замер J1b: набор отдавал стик в
    # центр с vz=+1.6 м/с → перелёт 2.2 м, и висение узаконивало этот перелёт).
    alt_hold = AltHold(kp=cfg.alt_kp, rate_max=cfg.alt_rate_max, tol=cfg.alt_tol,
                       dz=cfg.alt_dz, span=cfg.alt_span, rate_full=cfg.alt_rate_full)
    alt_target = [None]        # высота последнего climb — уставка для Control-шагов
    steps = [
        AwaitMode("prearm", keep, RC_MIN_THR, cfg.mode_budget),
        Arm("arm", RC_MIN_THR, cfg.arm_budget),
    ]

    def _control(name, prof, max_sec=0.0, pilot_stabs=None):
        stack = ControlStack(build_stabilizers(cfg, stab_spec), prof, NoExcitation(),
                             slew=cfg.slew)
        # live_pilot: газ живого пульта через защёлку; при отпускании стика контур
        # AltHold перецеливается на текущую высоту (см. Control.tick).
        st = Control(name, stack, hold, keep=keep, handover=handover,
                     wait_gt=wait_gt, result="MISSION_DONE", max_sec=max_sec,
                     alt_hold=alt_hold, alt_target=alt_target[0],
                     pilot_thr=live_pilot, pilot_deadzone=cfg.pilot_deadzone,
                     pilot_stabs=pilot_stabs)
        st.fence = cfg.fence          # стендовая страховка: увод дальше fence → land
        return st

    def _hold_stack():
        # стабилизаторы держат ГОРИЗОНТ на месте (StaticSetpoint) — для набора/сброса.
        # ЗОНДЫ (is_probe: DpPitchBack и т.п.) сюда НЕ идут: они не удерживают, а гонят
        # ось в заданную сторону — на наборе/посадке это увезло бы дрон до начала опыта.
        hold = [st for st in build_stabilizers(cfg, stab_spec)
                if not getattr(st, 'is_probe', False)]
        return ControlStack(hold, StaticSetpoint(), NoExcitation(), slew=cfg.slew)

    for i, tok in enumerate(tokens):
        verb, num = _parse(tok)
        if verb == "climb":
            if num is None:
                raise ValueError(f"climb требует высоту(м): {tok!r}")
            alt_target[0] = num
            steps.append(Climb(f"climb{i}", num, cfg.throttle_climb, cfg.climb_budget,
                               land_step="land", keep=keep,
                               stack=_hold_stack(), wait_gt=wait_gt, alt_hold=alt_hold))
        elif verb in _MV:
            if num is None:
                raise ValueError(f"{verb} требует длительность(сек): {tok!r}")
            fx, rx, yx = _MV[verb]
            steps.append(_control(f"{verb}_{i}",
                                  ConstProfile(num, c_fwd=fx * lvl, c_right=rx * lvl,
                                               c_yaw=yx * lvl)))
        elif verb in _YAW:
            if num is None:
                raise ValueError(f"{verb} требует УГОЛ(градусы): {tok!r}")
            # градусы → длительность по общему темпу. Темп берётся из cfg.yaw_rate_full,
            # из которого выведены гейны команды У ОБОИХ холдеров (recipes) — поэтому
            # один и тот же токен даёт один угол и под Gz-оракулом, и под DpYawHold.
            rate = lvl * cfg.yaw_rate_full          # °/с на рабочем уровне стика
            if rate <= 0:
                raise ValueError(f"нулевой темп разворота: mv_level={lvl}, "
                                 f"yaw_rate_full={cfg.yaw_rate_full}")
            steps.append(_control(f"{verb}_{i}",
                                  YawTurn(level=_YAW[verb] * lvl,
                                          turn_sec=num / rate,
                                          settle_sec=cfg.yaw_settle)))
        elif verb in _SK:
            if num is None:
                raise ValueError(f"{verb} требует τ(сек): {tok!r}")
            ax, sgn = _SK[verb]
            steps.append(_control(f"{verb}_{i}",
                                  StationKeep(level=sgn * lvl, tau=num, axis=ax)))
        elif verb == "hover":
            if num is None:
                raise ValueError(f"hover требует длительность(сек): {tok!r}")
            steps.append(_control(f"hover_{i}", ConstProfile(num)))
        elif verb == "pilot":
            if num is None and not live_pilot:
                # scripted держит центр → бессрочный сегмент никогда не кончится
                raise ValueError(f"pilot без длительности — только живой пилот "
                                 f"(--pilot joy/ros): {tok!r}")
            # живые стики = намерение; селектор стабилизации — ТОЛЬКО живому пилоту
            # (scripted держит центр → фактически hover со стеком BS_STAB).
            # Без числа — БЕССРОЧНО: летаем до `make pilot-done` (s.pilot_done).
            steps.append(_control(
                f"pilot_{i}",
                RcTransmitter(cfg.pilot_deadzone, cfg.pilot_full,
                              cfg.pilot_pitch_sign, cfg.pilot_roll_sign),
                max_sec=num if num is not None else 0.0,
                pilot_stabs=build_stabilizers(cfg, stab_spec) if live_pilot else None))
        elif verb in ("land", "landing"):
            steps.append(Land("land", hold, cfg.ground_z, cfg.land_budget,
                              stack=_hold_stack(), wait_gt=wait_gt))
        else:
            raise ValueError(f"неизвестный глагол миссии {verb!r} (токен {tok!r}); допустимо: "
                             f"climb, {', '.join(_MV)}, {', '.join(_YAW)}, "
                             f"{', '.join(_SK)}, hover, pilot, land, freefly")

    # гарантируем посадочный шаг 'land' (Climb абортит на него; и эпилог, если не задан)
    if not any(st.name == "land" for st in steps):
        steps.append(Land("land", hold, cfg.ground_z, cfg.land_budget,
                          stack=_hold_stack(), wait_gt=wait_gt))
    return steps
