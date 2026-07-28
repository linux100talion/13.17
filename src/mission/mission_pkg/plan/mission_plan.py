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
  sk_fwd/bkwd<τ>        — station-keeping оператор на pitch: +τ/−2τ/+τ, дрон У ТОЧКИ (не уносит)
  sk_right/left<τ>      — то же на roll
  hover<t>              — держать t сек (стик центр; стабилизатор гасит снос/держит)
  land | landing<x>     — посадка (Step Land; число игнорируется)

Реестр именованных заданий — MISSIONS (значение = список токенов или callable(cfg)).
Инлайн-список ('climb3,mv_fwd2 …') парсится напрямую (запятые/пробелы).
"""
import re

from control_pkg.application.control_stack import ControlStack
from control_pkg.domain.control.excitation import NoExcitation
from control_pkg.domain.control.trajectory import ConstProfile, StationKeep, StaticSetpoint
from control_pkg.domain.rc import RC_MIN_THR

from ..recipes import build_stabilizers
from .step import Arm, AwaitMode, Climb, Control, Land

_TOKEN = re.compile(r'^([a-z_]+?)(-?\d+(?:\.\d+)?)?$')

# глагол движения → (c_fwd, c_right, c_yaw) единичного направления (× level)
_MV = {
    "mv_fwd":   (1.0, 0.0, 0.0), "mv_bkwd": (-1.0, 0.0, 0.0),
    "mv_right": (0.0, 1.0, 0.0), "mv_left":  (0.0, -1.0, 0.0),
    "mv_cw":    (0.0, 0.0, 1.0), "mv_ccw":   (0.0, 0.0, -1.0),
}

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


def compile_mission(cfg, mission, stab_spec, handover=None, keep="ALT_HOLD"):
    """mission: имя/список токенов; stab_spec: имя(+склейка) стабилизатора. → [Step]."""
    tokens = resolve_mission(cfg, mission)
    wait_gt = "Gz" in str(stab_spec)     # gz-семейство держит позицию по gt (sim-оракул)
    hold = cfg.throttle_hold
    lvl = cfg.mv_level
    steps = [
        AwaitMode("prearm", keep, RC_MIN_THR, cfg.mode_budget),
        Arm("arm", RC_MIN_THR, cfg.arm_budget),
    ]

    def _control(name, prof):
        stack = ControlStack(build_stabilizers(cfg, stab_spec), prof, NoExcitation(),
                             slew=cfg.slew)
        return Control(name, stack, hold, keep=keep, handover=handover,
                       wait_gt=wait_gt, result="MISSION_DONE")

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
            steps.append(Climb(f"climb{i}", num, cfg.throttle_climb, cfg.climb_budget,
                               land_step="land", keep=keep,
                               stack=_hold_stack(), wait_gt=wait_gt))
        elif verb in _MV:
            if num is None:
                raise ValueError(f"{verb} требует длительность(сек): {tok!r}")
            fx, rx, yx = _MV[verb]
            steps.append(_control(f"{verb}_{i}",
                                  ConstProfile(num, c_fwd=fx * lvl, c_right=rx * lvl,
                                               c_yaw=yx * lvl)))
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
        elif verb in ("land", "landing"):
            steps.append(Land("land", hold, cfg.ground_z, cfg.land_budget,
                              stack=_hold_stack(), wait_gt=wait_gt))
        else:
            raise ValueError(f"неизвестный глагол миссии {verb!r} (токен {tok!r}); допустимо: "
                             f"climb, {', '.join(_MV)}, {', '.join(_SK)}, hover, land")

    # гарантируем посадочный шаг 'land' (Climb абортит на него; и эпилог, если не задан)
    if not any(st.name == "land" for st in steps):
        steps.append(Land("land", hold, cfg.ground_z, cfg.land_budget,
                          stack=_hold_stack(), wait_gt=wait_gt))
    return steps
