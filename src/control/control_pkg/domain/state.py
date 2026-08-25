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
    vins_first_sim: float = -1e9          # sim-время ПЕРВОЙ одометрии (гейт
                                          # зрелости по времени потока)
    vins_res: float = -1.0                # EMA residual |Δp/Δt − twist|, м/с
                                          # (детектор зрелости; -1 = нет данных)
    vins_ratio: float = -1.0              # вертикальный ratio VINS/референс
                                          # (-1 = референс ещё не ушёл на dz)
    vins_ripe_det: bool = False           # детектор зрелости: residual тих
                                          # quiet_sec И ratio в полосе (2-я
                                          # ступень гейта, см. ripeness.py)
    # Поза VINS (свой фрейд, НЕ выровнен к миру) — для VinsHold после init. Хватает
    # для УДЕРЖАНИЯ (держим текущую позу константной), выравнивание не нужно.
    vins_valid: bool = False
    vins_x: float = 0.0
    vins_y: float = 0.0
    vins_yaw: float = 0.0
    vins_vx: float = 0.0
    vins_vy: float = 0.0
    # EKF полётника переведён на ExternalNav: очередь EK3_SRC1_* в ноде пройдена
    # (пары 3→6 применены по зрелости VINS). Гейт входа в ШТАТНЫЙ LOITER-на-VINS
    # (токен loiter<t>, freefly-центр CH6 при BS_FF_LOITER): Loiter'у нужна
    # позиционная оценка на FCU, и без extnav он жил бы на GPS — не тот опыт.
    extnav_ready: bool = False
    # Sim-время последнего /mavros/local_position/pose: топик публикуется РОВНО
    # пока EKF держит позицию (после GPS-kill замолкает — поэтому годится только
    # как ДО-полётный гейт WaitEkfPos, не как гейт LoiterHold). Урок LV4: арм с
    # ARMING_CHECK=0 проходит до захвата GPS EKF'ом, а начать aiding в воздухе
    # на манёврах EK3 не смог — полёт без позиции, Loiter refused.
    ekf_pos_last_sim: float = -1e9
    # z из того же /mavros/local_position/pose — «высота глазами EKF3» (fusion
    # баро+vision). Нужна HUD'у парой к rel_alt: расхождение баро/EKF по
    # вертикали — ровно та ошибка, что занижает масштаб IPM у земли (прогон
    # 174603: EKF z ниже истины на 0.27 м → гейт alt<0.5 душил демпфер).
    ekf_z: float | None = None

    # --- Ground-truth Gazebo (СИМ; на Orin gt_valid=False) ---
    gt_valid: bool = False
    gt_x: float = 0.0
    gt_y: float = 0.0
    gt_z: float = 0.0                      # истинная высота (детект касания на посадке)
    gt_yaw: float = 0.0
    gt_vx: float = 0.0                     # world-скорость (конечная разность в адаптере)
    gt_vy: float = 0.0

    # --- Ориентация FCU ---
    att_yaw: float = 0.0             # ENU-курс из /mavros/imu/data (кватернион EKF);
                                     # для поворота body→ENU при отдаче скорости IPM
                                     # обратно в EKF (vision_speed) — та же ориентация,
                                     # которой EKF сам и пользуется → поворот согласован

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
    # --- КАНАЛ ВИДА СВЕРХУ: то же движение, но в МЕТРАХ ---
    # Полоса земли перед бортом выпрямляется по высоте и углам, поток по ней метрический.
    # Замер по трём бэгам: путь против истины 0.97-1.09 при corr 0.64-1.00, скорость на
    # окне 0.5 с — 0.91-0.94 при corr 0.76-0.84. Масштабный канал на тех же данных давал
    # corr 0.40-0.83 и разброс цены метра в 14 раз.
    ipm_fwd: float = 0.0             # путь вперёд от точки удержания, М
    ipm_lat: float = 0.0             # путь вбок, М
    ipm_vfwd: float = 0.0            # продольная скорость, М/С
    ipm_vlat: float = 0.0            # боковая скорость, М/С
    ipm_ok: bool = False             # кадр выпрямился и поток посчитан
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
    # Трёхпозиционник CH6: +1 = MANUAL (Arbiter, сырые стики); 0 = ALT_HOLD (Control-шаг
    # убирает стабилизаторы, стики = наклоны); −1 = наш стабилизатор (BS_STAB-стек).
    # Схема «SF-мастер» (опт-ин BS_SF_MASTER): MANUAL сюда даёт ТОЛЬКО SF (CH7)
    # не-вверх (+1); SF вверх → −1 (авто), а CH6 переезжает в pilot_level.
    pilot_switch: int = 0
    # Потолок лесенки зрелости по SC (только схема SF-мастер, см. ros_pilot.joy_master):
    # 0 = только демпфер; 1 = +VinsHold по готовности VINS; 2 = +штатный LOITER по
    # зрелости extnav. Freefly._ladder_* держит борт на лучшей ДОСТУПНОЙ ступени.
    pilot_level: int = 0
    # «Хватит летать»: one-shot от оператора (/mission/pilot_done, make pilot-done) —
    # завершает БЕССРОЧНЫЙ pilot-сегмент (токен `pilot` без числа) → миссия идёт дальше
    # (land). Узел выставляет на ОДИН тик; вне живого пилот-сегмента игнорируется.
    pilot_done: bool = False

    # --- время ---
    now_sim: float = 0.0                   # проставляет адаптер из Clock (sim-время по /clock)
