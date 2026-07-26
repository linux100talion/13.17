#!/usr/bin/env python3
"""build_bootstrap_plan — bootstrap как ПЛАН (список Step'ов) вместо FSM-класса.

Это первый и эталонный план: PREARM→ARM→CLIMB→CONTROL→LAND. Новые задания оформляются
так же — список шагов + условия; PlanRunner их гоняет. Управляемую фазу (CONTROL) питает
готовый ControlStack (профиль-траектория + стабилизаторы по recipes.build_control_stack).
"""
from control_pkg.domain.rc import RC_MIN_THR

from .step import Arm, AwaitMode, Climb, Control, Land


def build_bootstrap_plan(cfg, stack, handover=None):
    hold = cfg.throttle_hold
    # gz-режимы держат ПОЗИЦИЮ по gt → ждём истинную позу перед стеком; флоу/manual — нет.
    wait_gt = cfg.control_mode in ('shuttle', 'assisted')
    return [
        AwaitMode("prearm", "ALT_HOLD", RC_MIN_THR, cfg.mode_budget),
        Arm("arm", RC_MIN_THR, cfg.arm_budget),
        Climb("climb", cfg.alt, cfg.throttle_climb, cfg.climb_budget, land_step="land"),
        Control("control", stack, hold, handover=handover,
                max_sec=cfg.excite_max_sec, wait_gt=wait_gt),
        Land("land", hold, cfg.ground_z, cfg.land_budget),
    ]
