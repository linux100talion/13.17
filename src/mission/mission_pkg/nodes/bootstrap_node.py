#!/usr/bin/env python3
"""bootstrap_node — точка входа арх2 (composition root).

Срез 1: control-mode=shuttle (gz-hold + челнок). Срез 2: assisted (пульт=намерение +
gz-hold) и manual (пилот полностью). Режим выбирает recipes.build_control_stack; пульт
— адаптер PilotInput (ScriptedPilot headless / JoyPilot живой пульт через /joy /
RosPilot легаси — см. ros_pilot.py про петлю rc/override→rc/in). Arbiter в контуре:
тумблер MANUAL → сырые стики (safety-seize), что бы миссия ни командовала.

Детерминизм override: точки СМЕНЫ значения задаёт sim-таймер (_tick, 20 Гц); wall-цикл
в main лишь РЕ-публикует неизменное между тиками значение для свежести на FCU.

Запуск (внутри nav-контейнера, после colcon build):
    ros2 run mission_pkg bootstrap_arch2 --alt 3 --gz-shuttle-a 5           # срез 1
    ros2 run mission_pkg bootstrap_arch2 --control-mode assisted            # срез 2 (пульт-намерение)
    ros2 run mission_pkg bootstrap_arch2 --control-mode manual              # срез 2 (ручной)
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Empty

from control_pkg.application.arbiter import Arbiter
from control_pkg.domain.control.wind_trim import WindTrim
from control_pkg.application.handover import VinsHandover
from control_pkg.application.hud import hud_status, wind_from_ekf
from control_pkg.application.rth_ready import RthReadiness
from control_pkg.domain.control.stabilization import VinsHold
from control_pkg.domain.rc import RC_CENTER, RcCommand

from control_pkg.infrastructure.mavros_actuator import MavrosActuator
from control_pkg.infrastructure.ros_clock import RosClock
from control_pkg.infrastructure.ros_io import RosDebugSink, RosLogger
from control_pkg.infrastructure.ros_perception import RosPerception
from control_pkg.infrastructure.ros_pilot import (JoyPilot, PressEdge, RosPilot,
                                                  ScriptedPilot)
from control_pkg.infrastructure.ros_telemetry import RosTelemetry

from ..config import BootstrapConfig
from ..plan.bootstrap_plan import build_bootstrap_plan
from ..plan.mission_plan import compile_mission, resolve_mission
from ..plan.runner import PlanRunner
from ..recipes import build_control_stack, build_vins_stab

# ПОТОКИ ТЕЛЕМЕТРИИ FCU, которые читает нода (сторож _telemetry_watch): id
# MAVLink, Гц, имя. Темпы = запросам nav_up.sh (стримы RAW_SENS 200 / POSITION 25 /
# EXTRA1 50 / EXT_STAT 2 / EXTRA2 5 + WIND 5), чтобы сторож, сработав, дал ровно
# те темпы, что летали. Не ручка полёта — инфраструктура; в профилях не живёт.
TEL_STREAMS = (
    (27, 200.0, 'RAW_IMU'),              # /mavros/imu/data_raw, гироскоп для IPM
    (29, 200.0, 'SCALED_PRESSURE'),      # /mavros/imu/static_pressure (баро, alt_src=baro)
    (30, 50.0, 'ATTITUDE'),              # /mavros/imu/data (ориентация → крен/тангаж/курс)
    (32, 25.0, 'LOCAL_POSITION_NED'),    # /mavros/local_position/pose (ekf=, гейт арма)
    (33, 25.0, 'GLOBAL_POSITION_INT'),   # /mavros/global_position/rel_alt
    (1, 2.0, 'SYS_STATUS'),              # /mavros/state расширения, батарея
    (245, 2.0, 'EXTENDED_SYS_STATE'),    # /mavros/extended_state (landed)
    (24, 2.0, 'GPS_RAW_INT'),            # /mavros/global_position/raw (LV=1)
    (74, 5.0, 'VFR_HUD'),                # /mavros/vfr_hud
    (168, 5.0, 'WIND'),                  # /mavros/wind_estimation (стрелка ветра HUD)
)
TEL_SILENT_SEC = 2.0     # IMU молчит дольше — телеметрия «мёртвая»
TEL_RETRY_SEC = 3.0      # период запросов, пока молчит

# Экстринсик камеры (R_cam_imu) + знак derotation — из sim.yaml/монолита, ПОДТВЕРЖДЕНЫ
# flow_derotation_check (остаток 0.55× baseline). Интринсики — из разрешения (см. RosPerception).
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
FLOW_ROTSIGN = 1.0

# Демо-профили стиков для ScriptedPilot (sim, без живого пульта). Кортежи
# (t_until, roll, pitch, yaw) в PWM; pitch>центр = вперёд (наш знак psign=+1).
ASSISTED_SCRIPT = [
    (6.0, 1500, 1650, 1500),   # вперёд (assisted: стик=скорость → gz-hold ведёт)
    (9.0, 1500, 1500, 1500),   # висеть
    (15.0, 1500, 1350, 1500),  # назад (возврат)
    (18.0, 1500, 1500, 1500),  # висеть
    (24.0, 1650, 1500, 1500),  # вправо
    (27.0, 1500, 1500, 1500),  # висеть
    (33.0, 1350, 1500, 1500),  # влево (возврат)
    (36.0, 1500, 1500, 1500),  # центр
]
# manual — ПОЛН. ручной без ОС (open-loop): мягкие СИММЕТРИЧНЫЕ тычки, чтобы дрон не
# уносило (позиц. обратной связи нет, как в ALT_HOLD-translate).
MANUAL_SCRIPT = [
    (2.0, 1500, 1560, 1500),   # чуть вперёд
    (4.0, 1500, 1440, 1500),   # чуть назад (тормоз)
    (6.0, 1500, 1500, 1500),
    (8.0, 1560, 1500, 1500),   # чуть вправо
    (10.0, 1440, 1500, 1500),  # чуть влево
    (12.0, 1500, 1500, 1500),
]


class BootstrapArch2Node(Node):
    def __init__(self, cfg: BootstrapConfig, pilot_kind: str):
        super().__init__('alt_hold_bootstrap_arch2')
        # Все бюджеты/таймеры — по sim-времени (/clock), RTF-независимо.
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.cfg = cfg

        # адаптеры (инфраструктура)
        self.clock = RosClock(self)
        self.telemetry = RosTelemetry(self, self.clock, alt_src=cfg.alt_src,
                                      vel_src=cfg.vins_vel_src)
        self.actuator = MavrosActuator(self)     # RcOutput + FlightMode + SetpointOutput
        self.logger = RosLogger(self)
        self.debug = RosDebugSink(self)
        self.pilot = self._make_pilot(cfg, pilot_kind)
        self.arbiter = Arbiter()

        # Путь: заданный mission → ОРТОГОНАЛЬНЫЙ (stab+mission); иначе ЛЕГАСИ (control_mode).
        use_mission = bool(cfg.mission)
        stab_spec = cfg.stab or ("GzPosHold" if use_mission else "")
        # Зрение нужно, если стабилизатор — флоу-демпфер (Dp*) или легаси flow_assist.
        # flow_observe — зрение БЕЗ демпфера: сигнал пишется в /flow_dbg*, но управление
        # не трогает (замер перцепта при заданном движении).
        need_flow = (cfg.control_mode == 'flow_assist') or cfg.flow_observe or \
                    (use_mission and 'Dp' in stab_spec) or cfg.vision_vel > 0
        self.perception = None
        if need_flow:
            w = float(os.environ.get('CAMERA_W', 1280))
            h = float(os.environ.get('CAMERA_H', 720))
            self.perception = RosPerception(self, w, h, FLOW_R, FLOW_ROTSIGN,
                                            roll_smooth_n=cfg.roll_smooth,
                                            pitch_smooth_n=cfg.pitch_smooth,
                                            yaw_smooth_n=cfg.yaw_smooth,
                                            kf_alt_max=cfg.kf_alt_max,
                                            kf_alt_hold=cfg.kf_alt_hold,
                                            yaw_trans_fix=cfg.yaw_trans_fix,
                                            kf_seg_min_sec=cfg.kf_seg_min_sec,
                                            kf_seg_frac=cfg.kf_seg_frac,
                                            att_extrap=cfg.att_extrap,
                                            att_interp=cfg.att_interp,
                                            att_latency=cfg.att_latency,
                                            att_wait_max=cfg.att_wait_max,
                                            ipm_model=cfg.ipm_model,
                                            ipm_derot=cfg.ipm_derot,
                                            ipm_wz_tau=cfg.ipm_wz_tau,
                                            ipm_wz_gate=cfg.ipm_wz_gate,
                                            ipm_wz_bias_max=cfg.ipm_wz_bias_max,
                                            ipm_win=cfg.ipm_win,
                                            ipm_adapt=cfg.ipm_adapt,
                                            ipm_vel_tau=cfg.ipm_vel_tau,
                                            ipm_alt_floor=cfg.ipm_alt_floor,
                                            ipm_scale_ref=cfg.ipm_scale_ref,
                                            ipm_acc_tau=cfg.ipm_acc_tau,
                                            alt_src=cfg.perc_alt_src,
                alt_stale=cfg.perc_alt_stale,
                                            alt_zero=cfg.perc_alt_zero > 0)
            # ⚠️ Высота перцепции — СВОЯ ручка (perc_alt_src), НЕ cfg.alt_src:
            # 4 прогона 2026-08-19 с баро в перцепции (сырой И EMA) дали улёты
            # 15-59 м при наборе. Кампания 2026-08-24 доказала механизм замером
            # (см. таблицу в RosPerception): сырой баро ломает IPM межкадровой
            # производной (25.6 см p95 → фантомные скорости), EMA чинит
            # производную, но лаг 0.35 с рушит масштаб/полосу на наборе. Для
            # GPS-denied — 'local' (EKF z: гладко И без лага, живёт без GPS,
            # вертикаль переживает смерть VINS); 'global' — дефолт GPS-профилей.

        # рантайм switch Flow→Vins: флаг + стабилизатор яруса 1 (VinsHold |
        # DpVins по cfg.vins_stab, строится в recipes.build_vins_stab)
        # ОБЩИЙ ВЕТРОВОЙ ТРИМ ярусов 0/1 (WindTrim; config.wind_trim): один вектор
        # в валюте PWM каналов по курсу AHRS — без посева между ярусами, переживает
        # переключения, LOITER и перерождение VINS; сброс на фронте арма
        self.wind = (WindTrim(imax=max(cfg.dpvins_imax, cfg.roll_imax, cfg.pitch_imax),
                              steady_sec=cfg.wind_steady_sec, steady_v=cfg.wind_steady_v)
                     if cfg.wind_trim > 0 else None)
        handover = None
        if cfg.handover_vins and (cfg.control_mode == 'flow_assist' or
                                  (use_mission and 'Dp' in stab_spec)):
            vins = build_vins_stab(cfg, wind=self.wind)
            handover = VinsHandover(vins, cfg.vins_min, cfg.vins_fresh_sec,
                                    v_max=cfg.vins_v_max, ipm_tol=cfg.vins_ipm_tol,
                                    sane_n=cfg.vins_sane_n,
                                    hover_v=cfg.vins_hover_v,
                                    hover_sec=cfg.vins_hover_sec,
                                    trim_seed=cfg.dpvins_trim_seed > 0,
                                    scale_ratio=cfg.vins_scale_ratio,
                                    scale_ipm_min=cfg.vins_scale_ipm_min,
                                    scale_sec=cfg.vins_scale_sec,
                                    scale_alt_max=cfg.vins_scale_alt_max,
                                    scale_hold=cfg.vins_scale_hold)
            if str(cfg.vins_stab).lower() == 'dpvins':
                note = (f", DpVins (velocity-каскад) kp {cfg.dpvins_kp_fwd:g}/"
                        f"{cfg.dpvins_kp_lat:g} ki {cfg.dpvins_ki:g} "
                        f"vsmooth {cfg.dpvins_vsmooth:g}")
            else:
                kd_note = ", kd на ошибке скорости" if cfg.vins_kd_err > 0 else ""
                il_note = ", защёлка трима" if cfg.vins_i_latch > 0 else ""
                ps_note = ", гвоздь по остановке" if cfg.vins_pin_stop > 0 else ""
                pr_note = ", предиктор позы" if cfg.vins_predict > 0 else ""
                vs_note = (f", сглаж.скорости τ={cfg.vins_vsmooth:g}"
                           if cfg.vins_vsmooth > 0 else "")
                note = f"{kd_note}{il_note}{ps_note}{pr_note}{vs_note}"
            self.logger.info(f"handover Flow→Vins ВКЛ: ready при ≥{cfg.vins_min} odom{note}")

        # домен/приложение: план по выбранному пути. live_pilot: газ живого пульта
        # проходит в Control-фазе через ThrottleLatch (scripted — нет: эталонные
        # прогоны не меняются, у них pilot_throttle всегда центр).
        live_pilot = pilot_kind in ('joy', 'ros')
        if use_mission:
            tokens = resolve_mission(cfg, cfg.mission)
            plan = compile_mission(cfg, tokens, stab_spec, handover, wind=self.wind,
                                   live_pilot=live_pilot)
            self.logger.info(f"MISSION={cfg.mission} stab={stab_spec} "
                             f"level={cfg.mv_level} токены={tokens}")
        else:
            plan = build_bootstrap_plan(cfg, build_control_stack(cfg), handover,
                                        live_pilot=live_pilot)
        self.runner = PlanRunner(plan, self.clock, self.actuator, self.logger,
                                 perception=self.perception,
                                 setpoints=self.actuator)

        # --- отдача скорости+позиции IPM в EKF (vision_vel, см. config) ---
        self._vision_pub = None
        self._vis_pose_pub = None
        self._vis_pos = [0.0, 0.0]       # интеграл фида → относительная ENU-позиция
        self._vis_pos_t = None
        # EK3-источники ставятся ПАРОЙ: прогон C (2026-08-18) показал, что без
        # позиционного источника EK3 вовсе не начинает aiding — фид честно говорил
        # «летишь 15 м/с» (corr +0.96, наклон +0.95 с истинной world-скоростью),
        # а EKF держал ложный горизонт до самого fence (уход 110 м).
        # Очередь параметров: (имя, значение, готовность|None). Готовность —
        # предикат по снапшоту; пока не истинен, запись НЕ отправляется (и
        # блокирует хвост очереди — порядок строгий).
        # При позе от VINS (extern) переключение источников EKF гейтится на
        # ЗРЕЛОСТЬ VINS (>50 сообщений ≈ 5 с после init): прогон 2026-08-19 —
        # переключение через 1.6 с после init, ровно на старте mv_fwd, дало
        # разгон-улёт (транзиент EKF на сыром масштабе + горизонтальный манёвр).
        # ⚠️ Зрелости МАЛО — нужна ещё СПОКОЙНАЯ ФАЗА, и её проверяет _calm_phase:
        # прогон LV2 (2026-08-19) — odom>50 наступил ПОСРЕДИ манёвра sk_fwd, пара
        # ушла 6 на манёвре → разгон-улёт за fence за 3.5 с (LV1 в том же месте
        # просто повезло). Комментарий «в миссии после climb нужен hover» был
        # знанием в голове, а не в коде — очередь фазу не спрашивала. Теперь
        # спрашивает: hover/loiter-шаг (ожидание гейта LoiterHold — тот же
        # стабилизированный ховер). В планах без hover/loiter (freefly, легаси)
        # гейт фазы выключен — там дисциплина на пилоте/операторе.
        # С интегралом IPM гейт не нужен — интеграл жив с земли (проверено).
        self._has_calm_steps = any(st.name.startswith(('hover', 'loiter'))
                                   for st in plan)
        # Зрелость VINS для EKF-свапа. В планах со спокойной фазой (hover/loiter)
        # порог низкий: к hover VINS давно зрел (LV5 — свап в ховере чист). БЕЗ
        # спокойной фазы (freefly) зрелость — ЕДИНСТВЕННАЯ защита, и она должна
        # быть жёстче хэндоверной: полёт 2026-08-20 №3 — свап на VINS возрастом
        # ~3 с (odom≈50-150): перемещение VIO врало до ×10 против истины (bag,
        # окна 2 с: ratio 0.11 на init → 10.0 сразу после), EKF словил «variance:
        # position lost» через 3 с после глушения GPS и НЕ восстановился до
        # посадки (in-flight restart aiding не работает — LV4). Стики пилота при
        # этом были В ЦЕНТРЕ — «спокойные стики» не гейт: скорость создают ветер
        # и демпфер. По замеру ratio выходит на ~1 к ≈21 с от init; хэндовер
        # нашего стека на 300 (BS_VINS_MIN) был на грани.
        # ГЕЙТ — ПО SIM-ВРЕМЕНИ ПОТОКА, а не по счётчику (2026-08-24, разбор
        # «моды 14.7 Гц»): freq-контроль feature_tracker держит длинное среднее
        # и после CPU-затыка на старте стека «выплачивает долг» кадрами —
        # /odometry разгоняется до 14.7-29.5 Гц, и прежние 600 сообщений
        # пролетали за 21-24 с (впритык к физике ~21 с, запас нулевой), а при
        # штатных 10.5 Гц тянулись 57-62 с (втрое консервативнее физики).
        # Время потока от моды частоты не зависит: ripe_sec=30 ≈ 21 с физики +
        # запас. Счётчик остаётся ПОЛОМ против дырявого потока (30 с при 5 Гц —
        # ещё не зрелость): vins_min (в LV-профилях 300). Слепое пятно то же,
        # что у счётчика: перезапуск VINS в полёте гейт не замечает
        # (first_sim/count не сбрасываются) — свежесть ловит vins_fresh.
        ripe_sec = 5.0 if self._has_calm_steps else cfg.ripe_sec
        ripe_min = 50 if self._has_calm_steps else cfg.vins_min
        # пороги — в /mission/status (rsec/rcnt): HUD рисует прогресс «extnav»
        self._ripe_sec, self._ripe_min = ripe_sec, ripe_min
        # порог яруса VinsHold (t1/vmin в статусе); 0 — хэндовера нет
        self._vins_min = cfg.vins_min if handover is not None else 0

        def _ripe_time(s):
            return (s.vins_first_sim > -1e8
                    and s.now_sim - s.vins_first_sim > ripe_sec
                    and s.vins_odom_count > ripe_min)
        # ВТОРАЯ СТУПЕНЬ — детектор зрелости (application/ripeness.py:
        # residual «поза/скорость» тих 4 с + вертикальный ratio к rel_alt в
        # полосе [0.8,1.25]) пускает РАНЬШЕ таймера в хороший день; время
        # остаётся страховкой плохого (детектор не защёлкнулся — ripe_sec
        # решает, как раньше). Пол детекторного пути СВОЙ, короткий (8 с /
        # 80 сообщ.): пол времени-гейта (vins_min=300 ≈ 28 с @10.5 Гц)
        # обесценил бы детектор. По bag'ам 041803/050600 зрелость на текущем
        # стеке — 2-4 с после init; 8 с = двукратный запас.
        self._ripe_det_logged = False

        def _ripe_det(s):
            if not (cfg.ripe_det > 0 and s.vins_ripe_det
                    and s.vins_first_sim > -1e8
                    and s.now_sim - s.vins_first_sim > 8.0
                    and s.vins_odom_count > 80):
                return False
            if not self._ripe_det_logged:
                self._ripe_det_logged = True
                self.logger.info(
                    f"зрелость VINS по ДЕТЕКТОРУ (res={s.vins_res:.2f} "
                    f"ratio={s.vins_ratio:.2f}) — раньше таймера {ripe_sec:g} с")
            return True
        _vins_ripe = (lambda s: (_ripe_time(s) or _ripe_det(s))
                      and self._calm_phase()) \
            if cfg.vision_pose_src == 'extern' else None
        # Глушение GPS — ещё позже: EKF должен пожить на extnav (см. очередь ниже).
        kill_sec = ripe_sec + (0.0 if self._has_calm_steps else 15.0)
        # ⚠️ САМОВОССТАНОВЛЕНИЕ eeprom ПЕРЕД АРМОМ: EK3_SRC1_* персистятся, и
        # после vision-прогона следующий бут стартует с extnav-источниками БЕЗ
        # якоря VINS — EKF фьюзит климб-фантом IPM-скорости → улёт на наборе
        # (серия 2026-08-19: 6 прогонов до диагноза). Возвращаем GPS до арма,
        # на extnav переходим по зрелости VINS (гейт выше).
        # В extern-профиле скорость IPM полётнику НЕ отдаём (VELXY=0, а не 6):
        # полёт 2026-08-20 №5 — в ветре 10 IPM шлёт фантомные 1.5-2.6 м/с при
        # истинных 0.1-0.2 (остатки деротации на порывах); в миг VELXY→6
        # скоростной варианс-ратио 1.72 → отбраковка → осцилляции варианса →
        # циклы EKF Failsafe/LAND. Поза VINS 10 Гц точна — EKF выводит скорость
        # из неё сам. LV5 (без ветра) не ловил: фантом и истина оба ~0.
        # IPM-скорость остаётся у профиля 'integral' (там она — единственные
        # данные) и в паблишере vision_speed (health VisOdom для арма).
        vel_ripe = 0.0 if cfg.vision_pose_src == 'extern' else 6.0
        if cfg.gps_denied > 0:
            # LV=2 «GPS отсутствует С БУТА»: eeprom уже стоит extnav-парой
            # (sitl_lv_profile.py 2: POSXY=6, VELXY=0, SIM_GPS1_ENABLE=0) — EKF
            # живёт на vision с земли (нулевая поза, мост в _pose_bridge).
            # Очередь тут не ПЕРЕКЛЮЧАЕТ источники, а (а) самовосстанавливает
            # eeprom, если его увёл чужой профиль (LV=0 вернул 3/3), и
            # (б) ДАТИРУЕТ extnav_ready зрелостью VINS: POSXY=6 переписывается
            # по _vins_ripe → гейт LOITER (и HUD READY) открывается на тех же
            # ~600 odom, что в LV=1 — семантика зрелости не меняется. GPS-ветки
            # (восстановление до арма / глушение в полёте) не нужны: глушить
            # нечего, gps_disable игнорируется.
            if cfg.gps_disable > 0:
                self.logger.warn("gps_denied>0: gps_disable игнорируется — "
                                 "GPS нет с бута, глушить нечего")
            if cfg.set_origin <= 0:
                self.logger.warn("gps_denied>0 БЕЗ set_origin: origin ставить "
                                 "некому — EKF не начнёт aiding (LOITER мёртв)")
            self._ekf_pending = [('EK3_SRC1_VELXY', vel_ripe, None),
                                 ('EK3_SRC1_POSXY', 6.0, _vins_ripe)]
        else:
            self._ekf_pending = [('EK3_SRC1_VELXY', 3.0, None),
                                 ('EK3_SRC1_POSXY', 3.0, None),
                                 ('EK3_SRC1_VELXY', vel_ripe, _vins_ripe),
                                 ('EK3_SRC1_POSXY', 6.0, _vins_ripe)]
        # GPS-denied профиль «GPS теряется В ПОЛЁТЕ»: глушим GPS только когда
        # VINS реально публикует одометрию И дрон в воздухе. Попытка глушить на
        # земле (прогон 2026-08-19) провалилась: VINS без параллакса не инитится,
        # EKF минуты без позиционных данных → z-оценка едет → AltHold (газ по
        # EKF-z) сам сажает дрон. Если VINS так и не оживёт — GPS не глушится
        # вовсе (миссия остаётся на GPS, безопасная деградация).
        if cfg.gps_disable > 0 and cfg.gps_denied <= 0:
            # Самовосстановление GPS ДО арма (голова очереди): SIM_GPS1_ENABLE=0
            # прошлого прогона ПЕРСИСТИТСЯ в eeprom — следующий бут остался бы
            # без GPS с земли (climb по замёрзшему global не видит взлёта).
            # Симметрично самовосстановлению EK3_SRC1_* выше; убирает ручной
            # pymavlink-шаг между прогонами (LV-серия делала его трижды).
            self._ekf_pending.insert(0, ('SIM_GPS1_ENABLE', 1.0, None))
            # Глушим ТОЛЬКО при живой позиции EKF (свежий local_position): если
            # EK3 так и не начал aiding (гонка бута, прогон 2026-08-20 — арм на
            # 17-й секунде, const_pos), убить GPS = добить полёт: в воздухе
            # aiding не стартует (LV4), LOITER невозможен до посадки. Без
            # позиции миссия остаётся на GPS — та же безопасная деградация,
            # что и при мёртвом VINS.
            # kill_sec > ripe_sec (+15 с): между свапом на extnav и глушением
            # EKF живёт на vision при живом GPS — если фьюжн развалится,
            # local_position протухнет и глушение не случится вовсе
            # (безопасная деградация).
            self._ekf_pending.append(
                ('SIM_GPS1_ENABLE', 0.0,
                 lambda s: s.vins_first_sim > -1e8
                 and s.now_sim - s.vins_first_sim > kill_sec
                 and s.vins_odom_count > ripe_min
                 and (s.rel_alt or 0.0) > 1.5
                 and (s.now_sim - s.ekf_pos_last_sim) < 2.0))
        self._ekf_src_last_try = 0.0
        if cfg.vision_vel > 0 and self.perception is not None:
            from geometry_msgs.msg import PoseStamped, TwistStamped
            self._vision_pub = self.create_publisher(
                TwistStamped, '/mavros/vision_speed/speed_twist', 10)
            # Поза: 'integral' — наш интеграл IPM (суррогат); 'extern' — позу
            # публикует ray_tracer (VINS, боевая архитектура), мы её НЕ дублируем
            # (два издателя vision_pose ломают фьюжн). Скорость+нули на земле
            # для арма шлём в любом случае.
            if cfg.vision_pose_src != 'extern':
                self._vis_pose_pub = self.create_publisher(
                    PoseStamped, '/mavros/vision_pose/pose', 10)
            elif cfg.gps_denied > 0:
                # Мост безжпсного бута: пока extern-издатель (ray_tracer) молчит,
                # позу взять неоткуда, а EK3 обязан начать aiding НА ЗЕМЛЕ (в
                # воздухе не стартует — LV4). Издаём (0,0,баро) сами (см.
                # _pose_bridge) и замолкаем НАВСЕГДА, когда МОСТ VINS→EKF
                # ОТКРОЕТСЯ (не «с первой одометрией»: с гейтом зрелости между
                # ними десятки секунд, и в эту дыру EKF оставался без позиции
                # вовсе — прогон 173941). Правило «два издателя позы недопустимы»
                # соблюдено во времени: перекрытия нет, стык гладкий (кадр
                # ray_tracer якорится на EKF, обе стороны ≈ (0,0,alt)).
                self._vis_pose_pub = self.create_publisher(
                    PoseStamped, '/mavros/vision_pose/pose', 10)
            from mavros_msgs.srv import ParamSetV2
            self._param_cli = self.create_client(ParamSetV2, '/mavros/param/set')
            self.logger.info("vision_vel: скорость IPM → EKF (external nav), поза: "
                             f"{cfg.vision_pose_src}; "
                             "ставлю EK3_SRC1_VELXY=6, POSXY=6 по готовности MAVROS")

        # --- SET_GPS_GLOBAL_ORIGIN (безжпсный бут, см. config.set_origin) ---
        self._origin_pub = None
        self._origin_ok = False
        self._origin_last = 0.0
        if cfg.set_origin > 0:
            from geographic_msgs.msg import GeoPointStamped
            self._origin_pub = self.create_publisher(
                GeoPointStamped, '/mavros/global_position/set_gp_origin', 1)
            self.create_subscription(
                GeoPointStamped, '/mavros/global_position/gp_origin',
                self._on_gp_origin, 1)
            self.logger.info("set_origin: шлю SET_GPS_GLOBAL_ORIGIN до подтверждения")

        self._last_rc = RcCommand()
        self._arb_seized = False
        # «хватит летать»: make pilot-done → one-shot в снапшот (завершает бессрочный
        # pilot-сегмент). Слать ВО ВРЕМЯ pilot-сегмента: в других фазах тик его съест.
        self._pilot_done = False
        self.create_subscription(Empty, '/mission/pilot_done',
                                 lambda _m: setattr(self, '_pilot_done', True), 1)
        # «кнопка SA» с хоста (make sa-land): one-shot → уровень pilot_land на
        # ОДИН тик → Freefly видит фронт как от пульта (тот же гейт)
        self._land_req = False
        self.create_subscription(Empty, '/mission/land',
                                 lambda _m: setattr(self, '_land_req', True), 1)
        # ВОЗВРАТ ДОМОЙ, два топика = два режима FCU (make rth / make smart-rth):
        # RTL — прямая на home, SMART_RTL — по крошкам пройденного пути. Импульс
        # one-shot, режим липкий (шаг rth читает его в enter, уже после гашения).
        self._pose_bridge_done = False   # мост позы бута замолчал навсегда (вышли из круга)
        self._rth_req = False
        self._rth_mode = ''
        self.create_subscription(Empty, '/mission/rth',
                                 lambda _m: self._on_rth('RTL'), 1)
        self.create_subscription(Empty, '/mission/smart_rth',
                                 lambda _m: self._on_rth('SMART_RTL'), 1)
        # ТА ЖЕ КНОПКА НА ПУЛЬТЕ (SD на TX12, cfg.rth_joy): пилот читает УРОВЕНЬ,
        # импульс делает фронт (PressEdge: зажатая на старте кнопка — не нажатие).
        # Режим — cfg.rth_joy_mode (дефолт SMART_RTL, возврат по следу).
        self._rth_edge = PressEdge()
        if cfg.rth_joy:
            self.logger.info(f"кнопка ВОЗВРАТА пульта: /joy {cfg.rth_joy} → "
                             f"{cfg.rth_joy_mode} (повторное нажатие отменяет)")
        self._hud_st = ''      # последний st= в /mission/status (лог переходов)
        self._armed_prev = False   # фронт armed → латч нуля высоты перцепции, сброс VINS
        # ФРОНТ ARMED → сброс VINS (/restart → restart_callback эстиматора). Пока
        # борт стоит на земле, эстиматор в INITIAL копит all_image_frame (не-ключевые
        # кадры не чистятся), и первая попытка инита после отрыва решает плотную
        # (3N+4)² LDLT — при 3 мин стояния ~109 с на попытку, /odometry не появляется
        # за весь полёт (odom_gets_borken, 2026-08-28). Подробно: config.vins_restart_arm.
        self._vins_restart_pub = None
        if cfg.vins_restart_arm > 0 or cfg.vins_restart_diverge > 0:
            self._vins_restart_pub = self.create_publisher(Bool, '/restart', 1)
        # восстановление VINS после демоута-по-разносу (см. config.vins_restart_diverge)
        self._handover = handover
        # ВЕРДИКТ ГЕЙТА ЗДОРОВЬЯ VINS → мосту ray_tracer (/vins/sane, каждый тик):
        # мост закрывает vision_pose и морозит якорь по нему (bridge_gate.py) —
        # полёт 142811: разнос VINS через мост отравил ориентацию EKF, DpHold унесло.
        self._sane_pub = (self.create_publisher(Bool, '/vins/sane', 10)
                          if handover is not None else None)
        # ЛАТЧ ДОВЕРИЯ К ВОЗВРАТУ + ГЕЙТ ЗРЕЛОСТИ МОСТА (rth_ready.py). Один объект
        # решает оба вопроса: «созрел ли VINS настолько, чтобы пускать его в EKF»
        # (ripe → /vins/bridge_ok, мост открывается только по нему) и «цела ли рама
        # с момента латча» (state → rth= в статусе, красный/зелёный баннер, отказ
        # кнопки SD). Разбор 073004: 2-4 с мусора от незрелого VINS хватило, чтобы
        # EKF уехал на 200 м и «возврат» сел в 52 м от старта.
        self._rth = RthReadiness(radius=cfg.rth_radius, heal_sec=cfg.rth_heal_sec,
                                 ripe_sec=cfg.rth_ripe_sec, min_count=int(cfg.rth_ripe_n),
                                 fresh_sec=cfg.vins_fresh_sec, track_m=cfg.rth_track_m,
                                 jump_m=cfg.rth_jump_m,
                                 home_settle=cfg.rth_home_settle,
                                 dyaw_tol=cfg.rth_dyaw_tol,
                                 dyaw_wz=cfg.rth_dyaw_wz)
        # ЧЕЙ КУРС ДЕРЖИТ EKF (src/nav/frames.md): 'compass' всегда, 'vins' —
        # переключаем EK3_SRC1_YAW 1 → 6 в момент латча возврата (рама уже спокойна,
        # борт висит в круге на демпфере). _yaw_want — чего мы хотим, _yaw_src — что
        # реально подтвердил FCU (в статус уходит второе).
        self._yaw_want = 'compass'
        self._yaw_src = 'compass'
        self._bridge_ok_pub = self.create_publisher(Bool, '/vins/bridge_ok', 10)
        # переставить дом полётника в точку латча (фолбэк RTL полетит туда же)
        from mavros_msgs.srv import CommandHome
        self._home_cli = (self.create_client(CommandHome, '/mavros/cmd/set_home')
                          if cfg.rth_set_home > 0 else None)
        self._home_req = CommandHome.Request
        self._restart_diverge = cfg.vins_restart_diverge > 0
        self._restart_cd = cfg.vins_restart_cd
        self._last_restart_t = -1e9
        self._rebirths_prev = 0    # детект перерождения VINS (VinsTrack) → трим/лог
        self._scale_trips_prev = 0 # срабатываний чека занижения (лог)
        # --- ПОТОКИ ТЕЛЕМЕТРИИ FCU — сторож (см. TEL_STREAMS / _telemetry_watch) ---
        from mavros_msgs.srv import MessageInterval
        self._tel_cli = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self._tel_last_req = -1e9          # wall-время последнего запроса
        self._tel_req_n = 0                # сколько раз запрашивали
        self._tel_silent = True            # текущее состояние «молчит»
        self._tel_silent_since = time.time()
        self.timer = self.create_timer(0.05, self._tick)
        self.logger.info(
            f"alt_hold_bootstrap ARCH2: mode={cfg.control_mode} alt={cfg.alt}м "
            f"pilot={pilot_kind} excite_max={cfg.excite_max_sec}s (sim)")

    def _make_pilot(self, cfg, kind):
        sf = cfg.sf_master > 0        # схема SF-мастер (CH7): см. config.sf_master
        if kind == 'joy':
            if cfg.joy_signs:
                return JoyPilot(self, signs=tuple(float(x) for x in cfg.joy_signs.split(',')),
                                sf_master=sf, land_src=cfg.land_joy,
                                rth_src=cfg.rth_joy)
            # знаки — JOY_SIGNS_DEFAULT (выверены полётом TX12)
            return JoyPilot(self, sf_master=sf, land_src=cfg.land_joy,
                            rth_src=cfg.rth_joy)
        if kind == 'ros':
            return RosPilot(self, sf_master=sf)
        # flow_assist — НЕЙТРАЛЬНЫЙ пилот (центр): флоу-демпфер держит снос сам;
        # изолирует боевой пре-VINS сценарий (аналог liftland --flow-hold монолита).
        script = {'assisted': ASSISTED_SCRIPT, 'manual': MANUAL_SCRIPT}.get(cfg.control_mode, [])
        return ScriptedPilot(self.clock, script)

    def _rth_tick(self, s) -> None:
        """Латч доверия к возврату + вердикт зрелости мосту. Вердикт гейта здоровья
        берём ОДИН раз за тик (vins_sane идемпотентен по now_sim — счётчики не
        двигаются дважды)."""
        sane = bool(self._handover.vins_sane(s)) if self._handover is not None else True
        prev = self._rth.state
        s.rth_state = self._rth.update(s, sane=sane)
        s.rth_why = self._rth.why
        s.rth_status = self._rth.status(s)
        # трек и дом — в снапшот (шаг RthTrack разматывает их уставками GUIDED);
        # список отдаётся ССЫЛКОЙ, копий на тик не делаем
        s.rth_track = self._rth.track
        s.rth_home = self._rth.home
        self._bridge_ok_pub.publish(Bool(data=bool(self._rth.ripe)))
        self._yaw_source_tick(s)
        if s.rth_state != prev:
            if s.rth_state == RthReadiness.READY:
                h = self._rth.home
                self.logger.info(
                    f"ВОЗВРАТ: дом залатчен ({h[0]:+.1f},{h[1]:+.1f}) м EKF, "
                    f"{self._rth.dist:.1f} м от арма по IPM — RTH разрешён")
                self._set_home()
            else:
                self.logger.warn(
                    f"ВОЗВРАТ ЗАПРЕЩЁН на этот полёт: {s.rth_why} "
                    f"(круг {self.cfg.rth_radius:g} м, лечение "
                    f"{self.cfg.rth_heal_sec:g} с) — домой ведёт пилот")

    def _yaw_source_tick(self, s) -> None:
        """Переключение источника курса EKF (BS_EKF_YAW_SRC=vins).

        ВПЕРЁД: в момент латча возврата — рама к этому мгновению зрелая, спокойная
        (home_settle) и с устойчивым Δyaw, борт висит в круге на демпфере, позиционного
        контура в петле нет. Скачка курса при переключении НЕ будет: мы публикуем
        ориентацию, уже повёрнутую якорем (yaw_VINS + Δyaw = yaw_AHRS), то есть отдаём
        полётнику его же текущий курс.
        НАЗАД: как только опора пропала — перерождение / не sane / закрытый мост /
        протухший поток. Иначе EKF остался бы без источника курса (голый гироскоп).
        ⚠️ Откат сегодня ЖЁСТКИЙ: EKF доберёт накопленное расхождение с компасом разом.
        Мягкий доворот перед переключением — следующий шаг (см. src/nav/frames.md)."""
        s.ekf_yaw_src = self._yaw_src
        if self.cfg.ekf_yaw_src != 'vins':
            return
        fresh = (s.now_sim - s.vins_last_sim) <= self.cfg.vins_fresh_sec
        bridge_ok = not (getattr(s, 'bridge_seen', False)
                         and not getattr(s, 'bridge_open', True))
        opora = (s.rth_state == RthReadiness.READY and fresh and bridge_ok
                 and (self._handover is None or self._handover.vins_sane(s)))
        want = 'vins' if opora else 'compass'
        if want == 'compass' and self.cfg.ekf_yaw_fallback <= 0:
            return                      # откат выключен ручкой — остаёмся как есть
        if want == self._yaw_want:
            # подтверждение: очередь параметров опустела от нашего запроса
            if self._yaw_src != want and not any(
                    n == 'EK3_SRC1_YAW' for n, _v, _r in self._ekf_pending):
                self._yaw_src = want
                self.logger.info(f"курс EKF: источник = {want} (EK3_SRC1_YAW "
                                 f"{'6' if want == 'vins' else '1'})")
            return
        self._yaw_want = want
        self._ekf_pending = [q for q in self._ekf_pending if q[0] != 'EK3_SRC1_YAW']
        self._ekf_pending.insert(0, ('EK3_SRC1_YAW', 6.0 if want == 'vins' else 1.0,
                                     None))
        self._ekf_src_last_try = 0.0    # не ждать 2 с — просим сейчас
        (self.logger.info if want == 'vins' else self.logger.warn)(
            f"курс EKF → {want}: " + ("рама зрелая и спокойная, сажаем полётник на "
                                      "курс VINS" if want == 'vins' else
                                      "опора пропала (мост/здоровье/поток) — "
                                      "возвращаем компас"))

    def _set_home(self) -> None:
        """Дом полётника = текущая точка (в момент латча мы ещё у места взлёта, но
        уже в свежей раме). Нужен только фолбэку RTL: наш возврат идёт по треку."""
        if self._home_cli is None:
            return
        if not self._home_cli.service_is_ready():
            self.logger.warn("set_home: сервис /mavros/cmd/set_home не готов — "
                             "дом полётника остался на точке арма")
            return
        req = self._home_req()
        req.current_gps = True
        self._home_cli.call_async(req)
        self.logger.info("set_home: дом полётника переставлен в точку латча")

    def _telemetry_watch(self, s):
        """Сторож потоков телеметрии FCU: пока /mavros/imu/data молчит дольше
        TEL_SILENT_SEC — раз в TEL_RETRY_SEC просим SET_MESSAGE_INTERVAL на всё,
        что читает нода (TEL_STREAMS). Первый запрос — сразу на старте (снапшот
        пуст), дальше до первого IMU; после — только если телеметрия пропадёт.

        Зачем: ArduPilot шлёт RAW_IMU/ATTITUDE/LOCAL_POSITION_NED только по
        запросу (SR0_* в eeprom нули, пока sitl_lv_profile их не записал), просил
        фоновый цикл nav_up.sh. Прогон lv2_joy_20260907_122716: «nav: готово»
        вышло до бута FCU, цикл дал потоки через 3.5 мин, а узел все 200 с ждал
        «EKF» при живом мосте позы (FCU: «is using external nav data» на 5-й с) и
        принял пустой IMU за «EKF не захватил позицию». Сторож делает узел
        независимым от порядка старта: потоки запросит тот, кто в них нуждается.
        Темпы = запросам nav_up.sh (те же, что летали все серии)."""
        silent = (s.now_sim - s.tel_last_sim) > TEL_SILENT_SEC
        if not silent:
            if self._tel_silent and self._tel_req_n:
                self.logger.info(
                    f"телеметрия FCU пошла (IMU) через "
                    f"{time.time() - self._tel_silent_since:.0f} с, "
                    f"запросов потоков: {self._tel_req_n}")
            self._tel_silent = False
            return
        if not self._tel_silent:
            self._tel_silent = True
            self._tel_silent_since = time.time()
            self.logger.warn("телеметрия FCU пропала (нет /mavros/imu/data > "
                             f"{TEL_SILENT_SEC:g} с) — перезапрашиваю потоки")
        if time.time() - self._tel_last_req < TEL_RETRY_SEC:
            return
        self._tel_last_req = time.time()
        if not self._tel_cli.service_is_ready():
            if self._tel_req_n == 0:
                self.logger.warn("телеметрия FCU молчит, а /mavros/set_message_interval "
                                 "ещё недоступен — MAVROS не поднялся? жду")
            return
        from mavros_msgs.srv import MessageInterval
        self._tel_req_n += 1
        n = self._tel_req_n
        if n <= 3 or n % 10 == 0:
            self.logger.warn(
                f"телеметрия FCU молчит {time.time() - self._tel_silent_since:.0f} с "
                f"(нет /mavros/imu/data) — это НЕ EKF: запрашиваю потоки "
                f"SET_MESSAGE_INTERVAL (попытка {n}): "
                + ' '.join(f"{name}@{hz:g}" for _id, hz, name in TEL_STREAMS))
        for mid, hz, name in TEL_STREAMS:
            req = MessageInterval.Request()
            req.message_id = int(mid)
            req.message_rate = float(hz)
            fut = self._tel_cli.call_async(req)

            def _done(f, name=name):
                r = f.result()
                if r is None or not r.success:
                    self.logger.debug(f"set_message_interval {name}: отказ MAVROS "
                                      "(FCU не подключён?) — ретрай сторожем")
            fut.add_done_callback(_done)

    def _tick(self):
        s = self.telemetry.snapshot()
        self._telemetry_watch(s)
        armed_front = bool(s.armed) and not self._armed_prev
        self._armed_prev = bool(s.armed)
        if armed_front and self.wind is not None:
            self.wind.reset()                        # новый полёт — ветер учим заново
        if armed_front and self._vins_restart_pub is not None:
            # сброс VINS по арму: окно инициализации, накопленное на земле, обнуляется
            # (см. config.vins_restart_arm); на земле одометрии ещё нет — терять нечего
            self._vins_restart_pub.publish(Bool(data=True))
            self.logger.info("арм: /restart → VINS (сброс окна инициализации, "
                             "накопленного на земле)")
            if self._handover is not None:
                self._handover.note_vins_restart()   # рама переродится → трим сбросить
            self.telemetry.reset_vins_stream()       # зрелость потока — заново
        # ПЕРЕРОЖДЕНИЕ потока VINS, замеченное телеметрией (дыра штампов/скачок:
        # переинициализация сама или наш /restart): рама и масштаб новые —
        # ветровой трим яруса 1 недействителен (как на /restart), зрелость уже
        # обнулена телеметрией → лесенка роняет ярус и ждёт vins_min заново.
        if s.vins_rebirths != self._rebirths_prev:
            self._rebirths_prev = s.vins_rebirths
            if self._handover is not None:
                self._handover.note_vins_restart()
            self.logger.warn(f"VINS переродился (#{s.vins_rebirths}): рама/масштаб "
                             f"новые → ярус 1 ждёт зрелость заново")
        # ЧЕК ЗАНИЖЕНИЯ сработал (Handover.vins_sane, третий канал): VINS видит
        # много меньше стойко годного IPM на висении низко — масштаб схлопнулся
        if (self._handover is not None
                and self._handover.scale_trips != self._scale_trips_prev):
            self._scale_trips_prev = self._handover.scale_trips
            self.logger.warn(f"гейт здоровья: VINS ЗАНИЖАЕТ скорость против IPM "
                             f"(#{self._handover.scale_trips}) → демпфер на "
                             f"{self.cfg.vins_scale_hold:g} с")
        if self.perception is not None:
            # ФРОНТ ARMED → ноль высоты перцепции здесь и сейчас (perc_alt_zero):
            # EKF local z смещён вниз на 0.2-0.3 м, и на низком полёте это больше
            # всей высоты — гейт земли IPM не открывается (разбор 183305/185921 в
            # config.perc_alt_zero). Латчим ДО merge: снапшот этого же тика уже
            # понесёт исправленную высоту.
            if armed_front:
                z0 = self.perception.latch_alt_zero()
                if z0 is not None:
                    self.logger.info(f"высота перцепции: ноль земли z0={z0:+.2f} м "
                                     f"(латч по арму)")
            self.perception.merge(s)          # камера → flow_* в снапшот
        # пилот → в снапшот (домен читает pilot_* как телеметрию)
        sticks = self.pilot.sticks()
        s.pilot_roll, s.pilot_pitch = sticks.roll, sticks.pitch
        s.pilot_throttle, s.pilot_yaw = sticks.throttle, sticks.yaw
        s.pilot_switch = self.pilot.mode_switch()
        s.pilot_level = self.pilot.stab_level()   # потолок лесенки SC (SF-мастер)
        s.pilot_done, self._pilot_done = self._pilot_done, False   # one-shot
        # кнопка посадки: пульт (уровень) ИЛИ one-shot /mission/land
        s.pilot_land = bool(self.pilot.land_switch()) or self._land_req
        self._land_req = False
        # возврат домой: кнопка пульта (фронт) ИЛИ one-shot /mission/rth|smart_rth;
        # режим — липкий (см. _on_rth)
        if self._rth_edge.pressed(self.pilot.rth_switch()):
            self.logger.info(f"кнопка ВОЗВРАТА нажата → {self.cfg.rth_joy_mode}")
            self._on_rth(self.cfg.rth_joy_mode)
        s.pilot_rth, self._rth_req = self._rth_req, False
        s.pilot_rth_mode = self._rth_mode
        s.extnav_ready = self._extnav_ready()    # гейт штатного LOITER-на-VINS
        self._rth_tick(s)                        # латч возврата + зрелость моста

        self._send_origin()              # безжпсный бут: origin до подтверждения
        rc = self.runner.tick(s)
        # восстановление после разноса: гейт здоровья демотнул ярус → /restart VINS
        # (переинициализация), с кулдауном (сброс окна VINS сам занимает время)
        if (self._restart_diverge and self._vins_restart_pub is not None
                and self._handover is not None
                and self._handover.pop_restart_request()
                and s.now_sim - self._last_restart_t >= self._restart_cd):
            self._last_restart_t = s.now_sim
            self._vins_restart_pub.publish(Bool(data=True))
            self._handover.note_vins_restart()   # рама переродится → трим сбросить
            self.telemetry.reset_vins_stream()   # зрелость потока — заново, ярус вниз сразу
            self.logger.warn("гейт здоровья: VINS разнёсся → демоут на демпфер + "
                             "/restart (переинициализация)")
        rc = self.arbiter.resolve(s, rc)          # safety-seize: MANUAL → сырые стики
        if self.arbiter.last_manual != self._arb_seized:
            self._arb_seized = self.arbiter.last_manual
            self.logger.warn("ПИЛОТ ВЗЯЛ УПРАВЛЕНИЕ (MANUAL)" if self._arb_seized
                             else "возврат в АВТО")
        self._last_rc = rc
        self._publish(rc)
        self._vision_feed(s)             # скорость IPM → EKF (если vision_vel включён)
        self.debug.publish_axes(s, rc)   # флоу-дамп в bag: /flow_dbg + /flow_dbg2 (sim-штамп)
        self.debug.publish_hold(self._hold_dbg('pitch'))       # /flow_dbg5: уставка тангажа
        # /flow_dbg7: цель крена по скорости + PWM (единственная запись roll-команды)
        self.debug.publish_rate_roll(self._rate_dbg('roll'))
        # /flow_dbg10: цель тангажа по скорости + PWM (зеркало /flow_dbg7 для станции на тангаж)
        self.debug.publish_rate_pitch(self._rate_dbg('pitch'))
        # /flow_dbg6: уставка курса + PWM рыскания (единственная запись yaw-команды)
        self.debug.publish_hold_yaw(self._hold_dbg('yaw'), rc.yaw - RC_CENTER)
        # /mission/status: лесенка SF-мастера (потолок/ярус/гейты ярусов) +
        # гейт LOITER-на-VINS для debug-HUD стримера + bag. Активный ярус —
        # правда шага Freefly (ladder_state), у прочих шагов лесенки нет.
        step = None if self.runner.finished else self.runner.steps[self.runner.i]
        ladder = (step.ladder_state(s)
                  if step is not None and hasattr(step, 'ladder_state') else None)
        # мягкая посадка (SoftLand.land_state): баннер LANDING в HUD
        land = (step.land_state()
                if step is not None and hasattr(step, 'land_state') else None)
        # рама станции: ПУЛЛ из StationFrame.dbg() → поля st_* снапшота (домен в
        # DroneState не пишет); рамы нет в текущем стеке / кадров не было — sf=0
        fdbg = next((d for d in (getattr(st_, 'frame', None) and st_.frame.dbg()
                                 for st_ in getattr(getattr(step, 'stack', None),
                                                    'stabs', ()))
                     if d is not None), None)
        s.st_frame = int(fdbg is not None)
        if fdbg is not None:
            s.st_x, s.st_y = fdbg[0], fdbg[1]
            s.st_px, s.st_py = (fdbg[2] if fdbg[2] is not None
                                else (float('nan'), float('nan')))
        # ── ОЦЕНКА ВЕТРА для стрелки HUD — источник ПО ЯРУСУ ─────────────────
        # Ярусы 0/1 (DpHold/DpVins) — НАШ расчётный ветер (трим активного
        # стабилизатора): контур держит вживую, трим гладко следит за ветром.
        # Ярус 2 (LOITER) — наш стек пуст, трим замерзает → ветер из EKF3
        # drag-фьюжна (/mavros/wind_estimation): фильтрован, физика, следит за
        # порывом. Наблюдаемость на VINS-external-nav доказана Ф0 (сошёлся к
        # истине 10 м/с в висении; на резком движении/разносе VINS дёргается —
        # точен как readout висенного ветра). Требует EK3_DRAG_BCOEF>0
        # (BS_EKF_DRAG) + WIND в стриме (nav_up); иначе в LOITER стрелки нет.
        s.wind_src = ''
        s.st_phase = s.st_ifz = ''
        tier = ladder.tier if ladder is not None else 0
        if tier >= 2:
            wage = s.now_sim - s.wind_ekf_sim
            if wage < 1.5 and math.hypot(s.wind_ekf_wx, s.wind_ekf_wy) >= 0.5:
                wr = wind_from_ekf(s.wind_ekf_wx, s.wind_ekf_wy, s.att_yaw)
                if wr is not None:
                    s.wind_p, s.wind_r, s.wind_src = wr[0], wr[1], 'ekf'
        else:
            # ПОСЛЕДНИЙ стаб с тримом (правило стека «поздний перезаписывает
            # оси»: ярус 1 — DpVins, не DpHold-композит). seed_trim есть только
            # у DpVins → он и различает vins/ipm.
            for st_ in getattr(getattr(step, 'stack', None), 'stabs', ()):
                tp = getattr(st_, 'trim_pwm', None)
                v = tp() if tp is not None else None
                if v is not None:
                    s.wind_p, s.wind_r = v
                    s.wind_src = ('vins' if getattr(st_, 'seed_trim', None)
                                  is not None else 'ipm')
                # фаза станции того же стаба (brk=/ifz= — см. hud._station_fields)
                sph = getattr(st_, 'station_phase', None)
                ph = sph() if sph is not None else None
                if ph is not None:
                    s.st_phase = f"{ph[0]}/{ph[1]}"
                    s.st_ifz = f"{int(ph[2])}/{int(ph[3])}"
        # общий трим WindTrim: устойчивость/вердикт входа/снимок/выучен — во ВСЕХ
        # ярусах (в LOITER показывает, что отдадим на выходе; wt= — hud._wind_trim_fields)
        if self.wind is not None:
            s.wt_state = self.wind.status(s.now_sim)
        # |скорость| борта АКТИВНОГО датчика вида (рядом с компасом): ярус 0 —
        # канал IPM (тело), ярусы 1/2 — одометрия VINS (мир). Не привязано к
        # источнику ветра: показывает, как быстро реально несёт по тому сенсору,
        # на котором борт летит.
        s.vel_mag = -1.0
        if tier >= 1 and (s.now_sim - s.vins_last_sim) < self.cfg.vins_fresh_sec:
            s.vel_mag = math.hypot(s.vins_vx, s.vins_vy)
        elif s.ipm_ok:
            s.vel_mag = math.hypot(s.ipm_vfwd, s.ipm_vlat)
        line = hud_status(s, self.cfg.vins_fresh_sec, self.cfg.loiter_alt,
                          ladder=ladder, vins_min=self._vins_min,
                          ripe_sec=self._ripe_sec, ripe_min=self._ripe_min,
                          land=land)
        # scl — срабатываний чека ЗАНИЖЕНИЯ |vins_v| против IPM (Handover.vins_sane):
        # состояние гейта, не датчика — поэтому здесь, а не в hud_status. В ленте
        # joy_timeline объясняет демоут яруса 1 при свежем и «медленном» VINS.
        if self._handover is not None:
            line += f" scl={self._handover.scale_trips}"
        if self._sane_pub is not None:
            # vins_sane идемпотентен в пределах тика (кэш по now_sim) — счётчики
            # не двигаются дважды, если лесенка уже спросила
            self._sane_pub.publish(Bool(data=bool(self._handover.vins_sane(s))))
        # переход гейта LOITER, яруса ИЛИ посадки — в лог (виден и в sim_nav.log)
        st = ' '.join(w for w in line.split()
                      if w.startswith(('st=', 'tier=', 'land=')))
        if st != self._hud_st:
            self._hud_st = st
            self.logger.info(f"HUD: {line}")
        self.debug.publish_status(line)

    def _calm_phase(self) -> bool:
        """Спокойная фаза для переключения источников EKF (урок LV2, см. очередь):
        текущий шаг плана — hover/loiter. В планах без таких шагов не гейтим."""
        if not self._has_calm_steps:
            return True
        r = self.runner
        if r.finished:
            return False
        return r.steps[r.i].name.startswith(('hover', 'loiter'))

    def _extnav_ready(self) -> bool:
        """EKF переведён на extnav: очередь EK3_SRC1_* пройдена (пары 3→6 применены
        по зрелости VINS). Гейт входа в штатный LOITER (токен loiter<t>, freefly-центр
        при ff_loiter): без extnav LOITER жил бы на GPS — не тот опыт. Вне vision-фида
        (vision_vel=0) очередь не обрабатывается вовсе → честный False."""
        if self._vision_pub is None:
            return False
        return not any(n.startswith('EK3_SRC1_') for n, _v, _r in self._ekf_pending)

    def _on_rth(self, mode):
        """Импульс возврата домой: /mission/rth → RTL, /mission/smart_rth → SMART_RTL."""
        self._rth_req = True
        self._rth_mode = mode
        self.logger.info(f"RTH: запрос возврата ({mode})")

    def _on_gp_origin(self, _m):
        if not self._origin_ok:
            self._origin_ok = True
            self.logger.info("set_origin: EKF-origin подтверждён (gp_origin)")

    def _send_origin(self):
        """SET_GPS_GLOBAL_ORIGIN раз в 2 с до подтверждения. Координаты — из
        cfg.origin_*: локальный фрейм борт строит от этой точки, но «условными»
        они быть НЕ могут — EKF выводит из origin модель магнитного поля (WMM)
        и сверяет с магнитометром. Магнитометр в SITL рисуется от ДОМА SITL,
        поэтому origin и дом обязаны совпадать: Киев при доме-CMAC давал
        «PreArm: Check mag field» и арма не было. С 2026-08-24 согласованы обе
        точки (дом SITL, origin) и начало координат мира Gazebo — все Киев,
        см. комментарий в config.origin_lat."""
        if self._origin_pub is None or self._origin_ok:
            return
        if time.time() - self._origin_last < 2.0:
            return
        self._origin_last = time.time()
        from geographic_msgs.msg import GeoPointStamped
        m = GeoPointStamped()
        m.header.frame_id = 'map'
        m.position.latitude = float(self.cfg.origin_lat)
        m.position.longitude = float(self.cfg.origin_lon)
        m.position.altitude = float(self.cfg.origin_alt)
        self._origin_pub.publish(m)

    def _vision_feed(self, s):
        """Скорость+позиция IPM → EKF (external nav): лечим ПРИЧИНУ A4-рампы.

        Публикуем только при живом фильтре (ipm_ok): слепые измерения хуже их
        отсутствия — EKF без данных просто не корректирует скорость (как сейчас),
        а с мусором уехал бы. Поворот body→ENU курсом самого EKF (att_yaw) —
        измерение согласовано с тем, кто его потребляет. Штамп WALL-временем:
        FCU в SITL живёт по wall (JSON no_time_sync), sim-штамп уехал бы на часы.
        EK3_SRC1_VELXY=6 ставится отсюда же (ретраи до успеха): рантайм-смена
        источника EKF штатная (механизм EK3_SRC_OPTIONS/RC-switch)."""
        if self._vision_pub is None:
            return
        if self._ekf_pending and time.time() - self._ekf_src_last_try > 2.0:
            self._ekf_src_last_try = time.time()
            name, val, ready = self._ekf_pending[0]
            if (ready is None or ready(s)) and self._param_cli.service_is_ready():
                from mavros_msgs.srv import ParamSetV2
                from rcl_interfaces.msg import ParameterValue
                req = ParamSetV2.Request()
                req.param_id = name
                req.value = ParameterValue(type=3, double_value=val)
                fut = self._param_cli.call_async(req)

                def _done(f, name=name):
                    ok = f.result() is not None and f.result().success
                    if ok and self._ekf_pending and self._ekf_pending[0][0] == name:
                        self._ekf_pending.pop(0)
                    (self.logger.info if ok else self.logger.warn)(
                        f"{name}: " + ("установлен" if ok else "отказ — ретраю"))
                fut.add_done_callback(_done)
        # мост позы безжпсного бута — ДО скоростных гейтов: в наборе высоты
        # ipm слеп (return ниже), а aiding EK3 рвать нельзя ни на секунду
        self._pose_bridge(s)
        if s.ipm_ok:
            vf, vl = s.ipm_vfwd, s.ipm_vlat
        elif s.rel_alt is not None and s.rel_alt >= 0.5:
            return    # в воздухе со слепым фильтром молчим: нули были бы ложью
        else:
            # на земле/отрыве стоим — нулевая скорость ЧЕСТНАЯ. И обязательная:
            # без данных AP_VisualOdom не даёт заармиться («Arm: VisOdom: not
            # healthy», прогон 2026-08-18 — ARM_FAIL весь бюджет), а IPM на земле
            # закрыт гейтом alt<0.5 — яйцо и курица рвутся именно здесь.
            vf = vl = 0.0
        from geometry_msgs.msg import TwistStamped
        m = TwistStamped()
        wall = time.time()
        m.header.stamp.sec = int(wall)
        m.header.stamp.nanosec = int((wall % 1.0) * 1e9)
        m.header.frame_id = 'map'
        cy, sy = math.cos(s.att_yaw), math.sin(s.att_yaw)
        # body: vfwd вперёд, vlat ВЛЕВО-положителен; ENU: fwd=(cy,sy), left=(-sy,cy)
        ve = vf * cy - vl * sy
        vn = vf * sy + vl * cy
        m.twist.linear.x = ve
        m.twist.linear.y = vn
        m.twist.linear.z = 0.0           # вертикаль не меряем: EK3_SRC1_VELZ не наш
        self._vision_pub.publish(m)
        # Позиция = интеграл фида (относительная ENU, старт в нуле). Дрейфует —
        # и пусть: EKF нужен ХОТЬ КАКОЙ-ТО позиционный источник, чтобы вообще
        # начать aiding (прогон C: без позиции скоростной источник игнорируется).
        # z = баро (EK3_SRC1_POSZ остаётся баро и это не потребляет).
        # При vision_pose_src='extern' позу даёт ray_tracer — блок ниже молчит
        # (мост gps_denied живёт в _pose_bridge, интеграл ему не нужен).
        if self._vis_pose_pub is None or self.cfg.vision_pose_src == 'extern':
            return
        from geometry_msgs.msg import PoseStamped
        dt = 0.0 if self._vis_pos_t is None else min(0.2, wall - self._vis_pos_t)
        self._vis_pos_t = wall
        self._vis_pos[0] += ve * dt
        self._vis_pos[1] += vn * dt
        pm = PoseStamped()
        pm.header.stamp = m.header.stamp
        pm.header.frame_id = 'map'
        pm.pose.position.x = self._vis_pos[0]
        pm.pose.position.y = self._vis_pos[1]
        pm.pose.position.z = float(s.rel_alt or 0.0)
        # ориентация — текущий курс EKF (кватернион вокруг z): yaw-источник EKF
        # остаётся компасом (EK3_SRC1_YAW не трогаем), эти углы он не потребляет
        pm.pose.orientation.z = math.sin(0.5 * s.att_yaw)
        pm.pose.orientation.w = math.cos(0.5 * s.att_yaw)
        self._vis_pose_pub.publish(pm)

    def _pose_bridge(self, s):
        """Мост позы безжпсного бута (gps_denied + pose extern): (0,0,баро) в
        /mavros/vision_pose/pose с земли и весь набор высоты, пока VINS молчит.

        Зачем: EK3 стартует aiding ТОЛЬКО на земле (в воздухе не начинает —
        LV4), а extern-издатель (ray_tracer) до init VINS в полёте не публикует
        ничего — без моста EKF остался бы без позиционного источника с бута и
        LOITER был бы мёртв весь полёт. x=y=0 на вертикальном взлёте — честное
        приближение (снос ветром за ~8 с набора ограничен); z — баро (alt_src
        профиля LV=2). С ПЕРВОЙ одометрией VINS замолкаем навсегда: топик
        переходит к ray_tracer, его кадр якорится на EKF (стык гладкий).
        Штамп wall-временем — как у всего vision-фида (FCU в SITL живёт по
        wall, см. _vision_feed).

        ⚠️ ПЕРЕДАЁМ ЭСТАФЕТУ НЕ ПО ПЕРВОЙ ОДОМЕТРИИ, А ПО ОТКРЫТИЮ МОСТА (2026-09-10).
        Раньше мы замолкали, едва VINS подал голос, — считалось, что топик тут же
        подхватит ray_tracer. С гейтом зрелости это перестало быть правдой: между
        первой одометрией и зрелостью проходят десятки секунд, и мост всё это время
        ЗАКРЫТ. Пока дыру в гейте затыкал случайный «open» на старте, стык держался
        сам собой; как только дыру закрыли (bridge_ready_required), EKF остался БЕЗ
        ЕДИНОГО источника позиции и не прогревался вовсе — прогон 173941.
        Условия молчания: мост ОТКРЫТ (публикует ray_tracer — двух издателей быть не
        должно) ИЛИ борт ушёл дальше круга лечения по счислению IPM (там (0,0) уже не
        «честное приближение», а ложь, и лучше не иметь позиции, чем иметь неверную).
        Эстафета ОДНОСТОРОННЯЯ: замолчав по любой из причин, больше не начинаем."""
        if self._vis_pose_pub is None or self.cfg.vision_pose_src != 'extern':
            return
        if self._pose_bridge_done:
            return
        if getattr(s, 'bridge_seen', False) and getattr(s, 'bridge_open', False):
            # мост открылся — топик ведёт ray_tracer. Эстафета ОДНОСТОРОННЯЯ: если
            # мост потом закроется, нулевую позу возобновлять нельзя (борт уже не
            # там, где был на взлёте), а двух издателей vision_pose быть не должно
            self._pose_bridge_done = True
            self.get_logger().info(
                "мост позы бута передал эстафету ray_tracer (мост VINS→EKF открыт)")
            return
        if self._rth.dist > self._rth.radius:
            self._pose_bridge_done = True
            self.get_logger().warn(
                f"мост позы бута ЗАМОЛК: ушли на {self._rth.dist:.1f} м по счислению "
                f"IPM (круг {self._rth.radius:g} м), а мост VINS→EKF так и не открылся — "
                "нулевая поза стала бы ложью")
            return
        from geometry_msgs.msg import PoseStamped
        wall = time.time()
        pm = PoseStamped()
        pm.header.stamp.sec = int(wall)
        pm.header.stamp.nanosec = int((wall % 1.0) * 1e9)
        pm.header.frame_id = 'map'
        pm.pose.position.z = float(s.rel_alt or 0.0)
        pm.pose.orientation.z = math.sin(0.5 * s.att_yaw)
        pm.pose.orientation.w = math.cos(0.5 * s.att_yaw)
        self._vis_pose_pub.publish(pm)

    def _rate_dbg(self, axis):
        """Цель rate-оси (режим `rate`) — или None. Зеркало `_hold_dbg`."""
        st = None if self.runner.finished else self.runner.steps[self.runner.i]
        h = self._find_hold(getattr(getattr(st, 'stack', None), 'stabs', ()), axis,
                            attr='rate_dbg')
        return h

    def _hold_dbg(self, axis):
        """Уставка холдера положения ЗАДАННОЙ оси (режим `pos`) — или None.

        Ищем в стеке ТЕКУЩЕГО шага: стеки живут по сегментам, и уставка у каждого своя
        (каждый сегмент = своя точка удержания). Ось спрашиваем ЯВНО: pos-осей теперь
        две (тангаж по kf_logs, рыскание по накопленному визуальному курсу), и прежнее
        «первый ответивший» отдало бы в /flow_dbg5 то из них, что раньше в списке."""
        st = None if self.runner.finished else self.runner.steps[self.runner.i]
        return self._find_hold(getattr(getattr(st, 'stack', None), 'stabs', ()), axis)

    def _find_hold(self, stabs, axis, attr='hold_dbg'):
        """Спуск в композиты: DpHold держит оси в `_subs`, снаружи у него одного
        `hold_dbg` нет — под ним уставка не писалась вообще (и до рыскания тоже).

        `attr` выбирает, ЧТО спрашиваем: `hold_dbg` у pos-осей (уставка) или
        `rate_dbg` у осей по скорости (цель). Каждый метод сам отдаёт None, если ось
        не его режима, поэтому один обход годится для обоих."""
        for stab in stabs:
            sub = getattr(stab, '_subs', None)
            if sub:
                got = self._find_hold(sub, axis, attr)
                if got is not None:
                    return got
            elif getattr(stab, '_axis', None) == axis:
                fn = getattr(stab, attr, None)
                if fn is not None and fn() is not None:
                    return fn()
        return None

    def _publish(self, rc: RcCommand):
        if self.runner.finished:          # план завершён — override не нужен
            return
        self.actuator.publish(rc)

    @property
    def finished(self) -> bool:
        return self.runner.finished


def _parse() -> tuple:
    """Конфиг ноды — ТОЛЬКО из env BS_<ПОЛЕ> (BootstrapConfig.from_env): его выставляет
    src/control/profiles/load.py в bootstrap_arch2.sh из профилей. Argparse на 196
    аргументов и проводка BS_FOO → --foo (192 строки скрипта) удалены 2026-09-07: у
    ручки было четыре места для значения (датакласс, argparse, скрипт, .env), и любое
    из них молча подменяло профиль (свип B3s отлетел с ki=0). Нет ключа → SystemExit
    с именем, незнакомый BS_* → SystemExit, значение не того типа → SystemExit."""
    cfg = BootstrapConfig.from_env()
    pilot_kind = cfg.pilot
    # Автотриггер land для пилот-режимов: садимся после демо-профиля (+2с успокоение).
    # Профиль-миссия (mission) сама секвенсит land — автотриггер не нужен.
    if not cfg.mission and cfg.excite_max_sec <= 0 and pilot_kind == 'scripted':
        if cfg.control_mode == 'flow_assist':
            cfg.excite_max_sec = cfg.flow_hold_sec        # держим, флоу гасит снос
        elif cfg.control_mode in ('assisted', 'manual'):
            total = {'assisted': ASSISTED_SCRIPT, 'manual': MANUAL_SCRIPT}[cfg.control_mode][-1][0]
            cfg.excite_max_sec = total + 2.0
    return cfg, pilot_kind


def main():
    cfg, pilot_kind = _parse()
    rclpy.init()
    node = BootstrapArch2Node(cfg, pilot_kind)
    try:
        last_pub = 0.0
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - last_pub >= 0.05:
                last_pub = now
                node._publish(node._last_rc)
    except KeyboardInterrupt:
        node.logger.info("Прервано — садимся вручную (make land).")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
