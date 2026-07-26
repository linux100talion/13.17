#!/usr/bin/env python3
"""recipes — сборка ВАЛИДНОЙ тройки стратегий (ControlStack) по режиму управления.

Здесь живёт «какие стратегии в какой комбинации» — замена лестниц if gz/elif flow/…
монолита явным реестром. Добавить режим = добавить ветку, не трогая ControlStack/ноду.

Срез 1–2:
  shuttle  — gz-hold + челнок (автономный system-ID).
  assisted — пульт задаёт намерение (RcTransmitter) + gz-hold исполняет.
  manual   — пилот полностью (PilotPassthrough), без обратной связи.
"""
from control_pkg.application.control_stack import ControlStack
from control_pkg.domain.control.excitation import NoExcitation
from control_pkg.domain.control.stabilization import GzPositionHold, PilotPassthrough
from control_pkg.domain.control.trajectory import RcTransmitter, Shuttle, StaticSetpoint

CONTROL_MODES = ("shuttle", "assisted", "manual")


def _gz(cfg):
    return GzPositionHold(cfg.gz_kp, cfg.gz_kd, cfg.gz_ki, cfg.gz_imax,
                          cfg.gz_max, cfg.gz_psign, cfg.gz_rsign)


def build_control_stack(cfg) -> ControlStack:
    mode = cfg.control_mode
    if mode == "assisted":
        return ControlStack(
            _gz(cfg),
            RcTransmitter(cfg.pilot_vel_gain, cfg.pilot_deadzone, cfg.pilot_full,
                          cfg.pilot_pitch_sign, cfg.pilot_roll_sign),
            NoExcitation(),
        )
    if mode == "manual":
        return ControlStack(PilotPassthrough(), StaticSetpoint(), NoExcitation())
    if mode == "shuttle":
        return ControlStack(
            _gz(cfg),
            Shuttle(cfg.gz_shuttle_a, cfg.gz_shuttle_v, cfg.gz_shuttle_pause,
                    cfg.gz_shuttle_fwd),
            NoExcitation(),
        )
    raise ValueError(f"неизвестный control_mode={mode!r}; допустимо: {CONTROL_MODES}")
