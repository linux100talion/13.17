#!/usr/bin/env python3
"""recipes — сборка ControlStack по режиму управления (per-axis, срез 3).

Реестр «какие стратегии в какой комбинации» — замена лестниц if gz/elif flow/… .
Стабилизаторы теперь СПИСОК (каждый владеет своими осями); незанятые оси → пилот.

Режимы:
  shuttle     — gz-hold(roll+pitch) + челнок (автономный system-ID, sim).
  assisted    — gz-hold(roll+pitch) + пульт-намерение (position). yaw пилот.
  manual      — [] : всё пилоту (per-axis база стека = сырые стики).
  flow_assist — БОЕВОЙ пре-VINS: [FlowDamper(roll), YawHold(yaw)] + пульт (velocity-assist);
                pitch — сырой стик пилота, throttle держит миссия.
"""
from control_pkg.application.control_stack import ControlStack
from control_pkg.domain.control.excitation import NoExcitation
from control_pkg.domain.control.stabilization import (
    FlowDamper, GzPositionHold, YawHold)
from control_pkg.domain.control.trajectory import RcTransmitter, Shuttle, StaticSetpoint

CONTROL_MODES = ("shuttle", "assisted", "manual", "flow_assist")


def _gz(cfg):
    return GzPositionHold(cfg.gz_kp, cfg.gz_kd, cfg.gz_ki, cfg.gz_imax,
                          cfg.gz_max, cfg.gz_psign, cfg.gz_rsign)


def _rc_tx(cfg):
    return RcTransmitter(cfg.pilot_vel_gain, cfg.pilot_deadzone, cfg.pilot_full,
                         cfg.pilot_pitch_sign, cfg.pilot_roll_sign)


def build_control_stack(cfg) -> ControlStack:
    mode = cfg.control_mode
    if mode == "assisted":
        return ControlStack(_gz(cfg), _rc_tx(cfg), NoExcitation())
    if mode == "manual":
        return ControlStack([], StaticSetpoint(), NoExcitation())   # всё пилоту
    if mode == "flow_assist":
        flow = FlowDamper(cfg.flow_kp, cfg.flow_ki, cfg.flow_kd, cfg.flow_imax,
                          cfg.flow_max, cfg.flow_conf_min, cfg.flow_conf_full,
                          cfg.flow_osign, cfg.flow_cmd_gain)
        yaw = YawHold(cfg.yaw_kp, cfg.yaw_ki, cfg.yaw_imax, cfg.yaw_max,
                      cfg.flow_conf_min, cfg.flow_conf_full, cfg.yaw_osign, cfg.yaw_cmd_gain)
        return ControlStack([flow, yaw], _rc_tx(cfg), NoExcitation())
    if mode == "shuttle":
        return ControlStack(
            _gz(cfg),
            Shuttle(cfg.gz_shuttle_a, cfg.gz_shuttle_v, cfg.gz_shuttle_pause,
                    cfg.gz_shuttle_fwd),
            NoExcitation(),
        )
    raise ValueError(f"неизвестный control_mode={mode!r}; допустимо: {CONTROL_MODES}")
