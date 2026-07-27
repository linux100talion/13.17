#!/usr/bin/env python3
"""recipes — сборка стабилизаторов и ControlStack (per-axis, срез 3).

Два уровня API:
- `build_stabilizers(cfg, spec)` — ОРТОГОНАЛЬНЫЙ реестр стабилизаторов по ИМЕНИ
  (Gz*/Dp*/Vins), с '+'-склейкой (`DpRollHold+DpYawHold`). Это «ручка стабилизации»
  профиль-миссий (BS_STAB), независимая от движения (BS_MISSION).
- `build_control_stack(cfg)` — ЛЕГАСИ-ярлык по control_mode: слепляет конкретную пару
  «стабилизатор + траектория» (shuttle/assisted/manual/flow_assist). Оставлен для
  валидированных прогонов; профиль-миссии его не используют.

Стабилизаторы — СПИСОК (каждый владеет своими осями); незанятые оси → пилот.

Режимы (легаси):
  shuttle     — gz-hold(roll+pitch) + челнок (автономный system-ID, sim).
  assisted    — gz-hold(roll+pitch) + пульт-намерение (position). yaw пилот.
  manual      — [] : всё пилоту (per-axis база стека = сырые стики).
  flow_assist — БОЕВОЙ пре-VINS: [DpRollHold, DpYawHold] (демпфер по потоку) + пульт (velocity-assist);
                pitch — сырой стик пилота, throttle держит миссия.
"""
from control_pkg.application.control_stack import ControlStack
from control_pkg.domain.control.excitation import NoExcitation
from control_pkg.domain.control.stabilization import (
    DpHold, DpPitchHold, DpRollHold, DpYawHold, GzHold, GzPitchHold,
    GzPosHold, GzRollHold, GzYawHold, VinsHold)
from control_pkg.domain.control.trajectory import RcTransmitter, Shuttle

CONTROL_MODES = ("shuttle", "assisted", "manual", "flow_assist")


# ===================== ОРТОГОНАЛЬНЫЙ реестр стабилизаторов (BS_STAB) =====================

def _gz_alias(klass, cfg):
    """Gz*-алиас с gz-гейнами (ось задаёт сам алиас через kw['axes'])."""
    return klass(cfg.gz_kp, cfg.gz_kd, cfg.gz_ki, cfg.gz_imax, cfg.gz_max,
                 cfg.gz_psign, cfg.gz_rsign, cmd_gain=cfg.gz_cmd_gain)


def _dp_roll(cfg):
    return DpRollHold(cfg.flow_kp, cfg.flow_ki, cfg.flow_kd, cfg.flow_imax, cfg.flow_max,
                      cfg.flow_conf_min, cfg.flow_conf_full, cfg.flow_osign, cfg.flow_cmd_gain)


def _dp_pitch(cfg):
    return DpPitchHold(cfg.flow_kp, cfg.flow_ki, cfg.flow_kd, cfg.flow_imax, cfg.flow_max,
                       cfg.flow_conf_min, cfg.flow_conf_full, cfg.flow_osign, cfg.flow_cmd_gain)


def _dp_yaw(cfg):
    return DpYawHold(cfg.yaw_kp, cfg.yaw_ki, cfg.yaw_imax, cfg.yaw_max,
                     cfg.flow_conf_min, cfg.flow_conf_full, cfg.yaw_osign, cfg.yaw_cmd_gain)


def _vins(cfg):
    return VinsHold(cfg.gz_kp, cfg.gz_kd, cfg.gz_ki, cfg.gz_imax, cfg.gz_max,
                    cfg.gz_psign, cfg.gz_rsign, cfg.gz_cmd_gain)


# имя → билдер(cfg)→стратегия (None = «ничего», для manual/none)
_STAB = {
    "GzPosHold":   lambda cfg: _gz_alias(GzPosHold, cfg),
    "GzRollHold":  lambda cfg: _gz_alias(GzRollHold, cfg),
    "GzPitchHold": lambda cfg: _gz_alias(GzPitchHold, cfg),
    "GzYawHold":   lambda cfg: _gz_alias(GzYawHold, cfg),
    "DpHold":      lambda cfg: DpHold(_dp_roll(cfg), _dp_pitch(cfg), _dp_yaw(cfg)),
    "DpRollHold":  _dp_roll,
    "DpPitchHold": _dp_pitch,
    "DpYawHold":   _dp_yaw,
    "VinsHold":    _vins,
    "manual":      lambda cfg: None,
    "none":        lambda cfg: None,
}
STAB_NAMES = tuple(_STAB)


def build_stabilizers(cfg, spec):
    """spec: имя или '+'-склейка ('DpRollHold+DpYawHold') → список стратегий (может быть пуст).
    Так «пульт + только yaw» = 'DpYawHold', «пульт + flow(roll)» = 'DpRollHold', боевой
    пре-VINS = 'DpRollHold+DpYawHold', демпфер всех осей = 'DpHold'."""
    out = []
    for part in str(spec).split('+'):
        part = part.strip()
        if not part:
            continue
        if part not in _STAB:
            raise ValueError(f"неизвестный стабилизатор {part!r}; допустимо: {STAB_NAMES}")
        st = _STAB[part](cfg)
        if st is not None:
            out.append(st)
    return out


# ===================== ЛЕГАСИ ControlStack по control_mode =====================

def _gz(cfg):
    # горизонтальная позиция (roll+pitch), yaw — пилот; интегрирует стик-команду сам
    return GzHold(cfg.gz_kp, cfg.gz_kd, cfg.gz_ki, cfg.gz_imax,
                  cfg.gz_max, cfg.gz_psign, cfg.gz_rsign,
                  axes=frozenset({"roll", "pitch"}), cmd_gain=cfg.gz_cmd_gain)


def _rc_tx(cfg):
    return RcTransmitter(cfg.pilot_deadzone, cfg.pilot_full,
                         cfg.pilot_pitch_sign, cfg.pilot_roll_sign)


def build_control_stack(cfg) -> ControlStack:
    mode = cfg.control_mode
    if mode == "assisted":
        return ControlStack(_gz(cfg), _rc_tx(cfg), NoExcitation())
    if mode == "manual":
        return ControlStack([], _rc_tx(cfg), NoExcitation())   # всё оператору (пульт=траектория)
    if mode == "flow_assist":
        roll = DpRollHold(cfg.flow_kp, cfg.flow_ki, cfg.flow_kd, cfg.flow_imax,
                          cfg.flow_max, cfg.flow_conf_min, cfg.flow_conf_full,
                          cfg.flow_osign, cfg.flow_cmd_gain)
        yaw = DpYawHold(cfg.yaw_kp, cfg.yaw_ki, cfg.yaw_imax, cfg.yaw_max,
                        cfg.flow_conf_min, cfg.flow_conf_full, cfg.yaw_osign, cfg.yaw_cmd_gain)
        return ControlStack([roll, yaw], _rc_tx(cfg), NoExcitation())
    if mode == "shuttle":
        return ControlStack(
            _gz(cfg),
            Shuttle(cfg.gz_shuttle_level, cfg.gz_shuttle_leg, cfg.gz_shuttle_pause,
                    cfg.gz_shuttle_fwd),
            NoExcitation(),
        )
    raise ValueError(f"неизвестный control_mode={mode!r}; допустимо: {CONTROL_MODES}")
