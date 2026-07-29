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
    # Поза VINS (свой фрейд, НЕ выровнен к миру) — для VinsHold после init. Хватает
    # для УДЕРЖАНИЯ (держим текущую позу константной), выравнивание не нужно.
    vins_valid: bool = False
    vins_x: float = 0.0
    vins_y: float = 0.0
    vins_yaw: float = 0.0
    vins_vx: float = 0.0
    vins_vy: float = 0.0

    # --- Ground-truth Gazebo (СИМ; на Orin gt_valid=False) ---
    gt_valid: bool = False
    gt_x: float = 0.0
    gt_y: float = 0.0
    gt_z: float = 0.0                      # истинная высота (детект касания на посадке)
    gt_yaw: float = 0.0
    gt_vx: float = 0.0                     # world-скорость (конечная разность в адаптере)
    gt_vy: float = 0.0

    # --- Поток (FlowEstimator): СЫРЫЕ агрегаты, PID теперь в домене ---
    flow_lateral: float = 0.0        # боковой поток (DpRollHold)
    flow_longitudinal: float = 0.0   # продольный/looming (DpPitchHold)
    flow_yaw: float = 0.0            # визуальный yaw (DpYawHold)
    flow_conf: float = 0.0
    flow_seq: int = 0                      # счётчик кадров: PID интегрирует ПО КАДРАМ
    flow_dt: float = 0.0                   # интервал последнего кадра

    # --- Опорный кадр: СМЕЩЕНИЕ картинки от точки удержания (не скорость) ---
    # Фичи живут, пока их видно (замер: медиана 284 кадра), поэтому доступно не
    # только «как быстро уезжаем», но и «насколько уже уехали». kf_logs — логарифм
    # масштаба созвездия: −1.81% на метр удаления от опоры (замер keyframe_track.py
    # по G1; крутизна зависит от высоты и глубины сцены).
    kf_dx: float = 0.0               # сдвиг картинки от опоры, px (⚠️ включает поворот камеры)
    kf_dy: float = 0.0
    kf_logs: float = 0.0             # log(масштаб) — ПРОДОЛЬНОЕ смещение
    # Производная kf_logs, посчитанная оценщиком по окну (МНК, 1 с) — а НЕ разностью
    # соседних кадров: та даёт corr +0.27 с истинной скоростью против +0.80 у окна
    # (замер J1b). Демпфер продольной оси берёт D-член отсюда.
    kf_vel: float = 0.0              # log-единиц/с (≈ +0.0111 на м/с)
    kf_rot: float = 0.0              # поворот кадра от опоры, рад (диагностика)
    kf_n: int = 0                    # сколько точек опоры ещё видно
    kf_age: int = 0                  # кадров с посева опоры
    kf_reseeds: int = 0              # пересевов с ВЫБРОШЕННЫМ сегментом (высота/отбраковки)
    kf_segs: int = 0                 # сегментов ЗАЧТЕНО в накопитель (плановый рез/потеря точек)
    kf_rejects: int = 0              # кадров-выбросов отсечено (kf_max_step)
    kf_valid: bool = False

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
