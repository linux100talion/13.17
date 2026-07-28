#!/usr/bin/env python3
"""BootstrapConfig — конфиг миссии bootstrap (срез 1: gz-hold + shuttle).

Frozen-подобный контейнер параметров (замена argparse-namespace монолита в части,
нужной срезу). Имена/дефолты совместимы с флагами alt_hold_bootstrap.py, чтобы
обёртки (liftland.sh/bootstrap.sh) мапились 1:1 при переключении на --arch2.
"""
from dataclasses import dataclass


@dataclass
class BootstrapConfig:
    # режим управления фазы EXCITE: shuttle | assisted | manual (см. recipes.py).
    # ЛЕГАСИ-ярлык: слепляет стабилизатор+траекторию. Новый ортогональный путь — stab+mission.
    control_mode: str = "shuttle"
    # ОРТОГОНАЛЬНЫЙ путь (профиль-миссии): stab = стабилизатор(ы) '+'-склейкой
    # (GzPosHold|DpRollHold+DpYawHold|…, см. recipes.build_stabilizers), mission = плейлист
    # профиль-токенов (имя из MISSIONS или 'climb3,mv_fwd2,…', см. plan/mission_plan.py).
    # Пусто → идём легаси-путём по control_mode. Заданный mission → игнор control_mode.
    stab: str = ""
    mission: str = ""
    mv_level: float = 0.3            # глобальный уровень стика для профиль-миссий (mv_*), [-1..1]
    # предельная длительность EXCITE, sim-сек (0=выкл; для пилот-режимов = когда садиться,
    # т.к. RcTransmitter/manual сами не завершаются, в отличие от челнока с motion_done)
    excite_max_sec: float = 0.0
    # фазы
    alt: float = 3.0
    throttle_climb: int = 1650
    throttle_hold: int = 1500
    ground_z: float = 0.3
    mode_budget: float = 40.0
    arm_budget: float = 40.0
    climb_budget: float = 60.0
    land_budget: float = 45.0        # бэкстоп (sim-сек): касание ловит детект (баро/gt_z), не бюджет
    # gz-hold (PID по истинной позе Gazebo)
    gz_kp: float = 40.0
    gz_kd: float = 120.0
    gz_ki: float = 8.0
    gz_imax: float = 100.0
    gz_max: float = 150.0
    gz_psign: float = 1.0
    gz_rsign: float = 1.0
    # интегратор стик-команды → уставка в позиц-холдерах Gz*/Vins (setpoint-ед/с при полном стике)
    gz_cmd_gain: float = 0.8
    # shuttle (челнок) как стик-профиль: ±level по плечам leg сек
    gz_shuttle_level: float = 0.3    # уровень стика [-1..1] на плече
    gz_shuttle_leg: float = 3.0      # длительность плеча, sim-сек
    gz_shuttle_pause: float = 2.0
    gz_shuttle_fwd: bool = False
    # пульт (нормировка стика → c_*)
    pilot_deadzone: int = 30         # мёртвая зона вокруг центра, PWM
    pilot_full: int = 400            # полное отклонение стика от центра, PWM
    pilot_pitch_sign: float = -1.0   # сырой PWM пульта → намерение; парный к _PITCH_RC_SIGN
                                     # в ControlStack, вместе = pass-through (борт: сверить с радио)
    pilot_roll_sign: float = 1.0
    # ==== ДЕМПФЕР ПО ПОТОКУ (срез 3, БОЕВОЙ пре-VINS) — ТРИ НЕЗАВИСИМЫЕ ОСИ ====
    # Есть roll, pitch и yaw. Точка. У каждой оси СВОЙ полный набор — ничего не
    # шарится, дублирование НАМЕРЕННОЕ: тюнинг у осей разный (yaw ki=0 — интегратор
    # вреден, bias yaw_flow [[yaw-hold-tuning]]; roll ki=2 — победитель монолита),
    # и общий префикс «flow_» это только маскировал. Хочешь тронуть pitch — правь
    # pitch_*, roll не поедет.
    # cmd_gain (velocity-assist, у всех трёх): демпфер ПРОПУСКАЕТ намерение
    # (стик = цель скорости) и гасит только ОТКЛОНЕНИЕ от неё, а не саму команду.
    # Стик[-1..1] → целевой поток, px/кадр; полный стик ≈ 10 px/кадр (шум ~2).
    # c_*=0 → цель 0 → чистое удержание (hover как раньше).
    # ⚠️ масштаб 10 — первая оценка, калибруется в симе (flow_* ↔ gt-скорость).
    # smooth — медиана перцепта по N кадрам (FlowEstimator), тоже per-axis.
    # --- roll: боковой снос, сигнал flow_lateral → стик roll ---
    roll_kp: float = 8.0
    roll_ki: float = 2.0
    roll_kd: float = 0.0
    roll_imax: float = 120.0
    roll_max: float = 150.0
    roll_conf_min: float = 0.05
    roll_conf_full: float = 0.20
    roll_osign: float = -1.0         # ПОДТВЕРЖДЁН drift_check (arch2): +1 разгонял снос
                                     # (метрика 3.47), −1 гасит (0.49). Монолит помечал «TODO».
    roll_cmd_gain: float = 10.0
    roll_smooth: int = 1
    # --- pitch: продольный снос, сигнал flow_longitudinal → стик pitch ---
    # ⚠️ ОТДЕЛЬНО НЕ КАЛИБРОВАН: дефолты скопированы с roll (looming менее зрел,
    # в полёте ось не проверена). Свой набор — чтобы калибровать, не трогая roll.
    pitch_kp: float = 8.0
    pitch_ki: float = 2.0
    pitch_kd: float = 0.0
    pitch_imax: float = 120.0
    pitch_max: float = 150.0
    pitch_conf_min: float = 0.05
    pitch_conf_full: float = 0.20
    pitch_osign: float = -1.0        # ⚠️ НЕ ПРОВЕРЕН: демпфер пишет PWM мимо _PITCH_RC_SIGN,
                                     # знак выводится из знака flow_longitudinal (замер C1)
    pitch_cmd_gain: float = 10.0
    pitch_smooth: int = 1
    # --- yaw: визуальный курс, сигнал flow_yaw → стик yaw (kd нет: DpYawHold — PI) ---
    yaw_kp: float = 6.0
    yaw_ki: float = 0.0              # ВРЕДЕН (bias yaw_flow) — держим 0
    yaw_imax: float = 200.0
    yaw_max: float = 150.0
    yaw_conf_min: float = 0.05
    yaw_conf_full: float = 0.20
    yaw_osign: float = 1.0
    yaw_cmd_gain: float = 10.0
    yaw_smooth: int = 5              # победитель свипа [[yaw-hold-tuning]]
    flow_hold_sec: float = 30.0      # flow_assist: сколько держать (флоу гасит снос) до land
    flow_observe: bool = False       # ТОЛЬКО НАБЛЮДЕНИЕ: поднять зрение и писать /flow_dbg*,
                                     # когда демпфера в стеке НЕТ (Gz*/manual). Нужно, чтобы
                                     # мерить сигнал потока при ЗАДАННОМ движении (калибровка:
                                     # знаем истинную скорость → видим, что читает перцепт).
    # рантайм switch Flow→Vins по «VINS ready» (VinsHold юзает gz_* гейны)
    handover_vins: bool = False
    vins_min: int = 40               # сколько odom-сообщений считать сходимостью
    vins_fresh_sec: float = 2.0      # свежесть потока odom для «ready»
