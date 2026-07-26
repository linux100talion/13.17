#!/usr/bin/env python3
"""BootstrapConfig — конфиг миссии bootstrap (срез 1: gz-hold + shuttle).

Frozen-подобный контейнер параметров (замена argparse-namespace монолита в части,
нужной срезу). Имена/дефолты совместимы с флагами alt_hold_bootstrap.py, чтобы
обёртки (liftland.sh/bootstrap.sh) мапились 1:1 при переключении на --arch2.
"""
from dataclasses import dataclass


@dataclass
class BootstrapConfig:
    # фазы
    alt: float = 3.0
    throttle_climb: int = 1650
    throttle_hold: int = 1500
    ground_z: float = 0.3
    mode_budget: float = 40.0
    arm_budget: float = 40.0
    climb_budget: float = 60.0
    land_budget: float = 120.0
    # gz-hold (PID по истинной позе Gazebo)
    gz_kp: float = 40.0
    gz_kd: float = 120.0
    gz_ki: float = 8.0
    gz_imax: float = 100.0
    gz_max: float = 150.0
    gz_psign: float = 1.0
    gz_rsign: float = 1.0
    # shuttle (челнок): в срезе 1 фаза EXCITE = gz-hold + челнок
    gz_shuttle_a: float = 5.0
    gz_shuttle_v: float = 1.5
    gz_shuttle_pause: float = 2.0
    gz_shuttle_fwd: bool = False
