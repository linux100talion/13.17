#!/usr/bin/env python3
"""DroneState — снапшот телеметрии, который домен читает КАЖДЫЙ тик.

Адаптеры инфраструктуры наполняют его из ROS-топиков; домен (стратегии,
ControlStack, MissionRunner) читает только этот объект — про ROS не знает.

Замечание про источник позы: `gt_*` — истинная поза Gazebo (СИМ-костыль). На
боевом Orin её НЕТ (gt_valid=False) — там источник позы даст VINS-адаптер в те
же поля позы (или отдельная стратегия VinsHold). Домен от этого не меняется.
"""
from dataclasses import dataclass

from .rc import RC_CENTER


@dataclass
class DroneState:
    # --- FCU / телеметрия ---
    mode: str | None = None
    armed: bool = False
    rel_alt: float | None = None          # баро-высота, доступна БЕЗ origin/GPS
    rcin_throttle: int | None = None      # эхо throttle из /mavros/rc/in (диагностика)

    # --- VINS ---
    vins_odom_count: int = 0
    vins_last_sim: float = -1e9

    # --- Ground-truth Gazebo (СИМ; на Orin gt_valid=False) ---
    gt_valid: bool = False
    gt_x: float = 0.0
    gt_y: float = 0.0
    gt_yaw: float = 0.0
    gt_vx: float = 0.0                     # world-скорость (конечная разность в адаптере)
    gt_vy: float = 0.0

    # --- Поток (FlowEstimator): СЫРЫЕ агрегаты, PID теперь в домене ---
    flow_lateral: float = 0.0
    flow_yaw: float = 0.0
    flow_conf: float = 0.0
    flow_seq: int = 0                      # счётчик кадров: PID интегрирует ПО КАДРАМ
    flow_dt: float = 0.0                   # интервал последнего кадра

    # --- Пилот (пульт): сырой PWM стиков + тумблер режима ---
    # Адаптер PilotInput (ScriptedPilot в симе / RosPilot на борту) наполняет каждый
    # тик. Стратегии RcTransmitter/PilotPassthrough и Arbiter читают ОТСЮДА (как flow).
    pilot_roll: int = RC_CENTER
    pilot_pitch: int = RC_CENTER
    pilot_throttle: int = RC_CENTER
    pilot_yaw: int = RC_CENTER
    pilot_switch: int = 0                  # тумблер авто(0)/ручной(1) — для Arbiter

    # --- время ---
    now_sim: float = 0.0                   # проставляет адаптер из Clock (sim-время по /clock)
