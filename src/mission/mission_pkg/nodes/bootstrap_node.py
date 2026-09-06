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
import argparse
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Empty

from control_pkg.application.arbiter import Arbiter
from control_pkg.application.handover import VinsHandover
from control_pkg.application.hud import hud_status, wind_from_ekf
from control_pkg.domain.control.stabilization import VinsHold
from control_pkg.domain.rc import RC_CENTER, RcCommand
from control_pkg.infrastructure.mavros_actuator import MavrosActuator
from control_pkg.infrastructure.ros_clock import RosClock
from control_pkg.infrastructure.ros_io import RosDebugSink, RosLogger
from control_pkg.infrastructure.ros_perception import RosPerception
from control_pkg.infrastructure.ros_pilot import JoyPilot, RosPilot, ScriptedPilot
from control_pkg.infrastructure.ros_telemetry import RosTelemetry

from ..config import BootstrapConfig
from ..plan.bootstrap_plan import build_bootstrap_plan
from ..plan.mission_plan import compile_mission, resolve_mission
from ..plan.runner import PlanRunner
from ..recipes import build_control_stack, build_vins_stab

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
        self.actuator = MavrosActuator(self)     # RcOutput + FlightMode
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
                                            ipm_win=cfg.ipm_win,
                                            ipm_adapt=cfg.ipm_adapt,
                                            ipm_vel_tau=cfg.ipm_vel_tau,
                                            ipm_alt_floor=cfg.ipm_alt_floor,
                                            ipm_scale_ref=cfg.ipm_scale_ref,
                                            ipm_acc_tau=cfg.ipm_acc_tau,
                                            alt_src=cfg.perc_alt_src,
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
        handover = None
        if cfg.handover_vins and (cfg.control_mode == 'flow_assist' or
                                  (use_mission and 'Dp' in stab_spec)):
            vins = build_vins_stab(cfg)
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
            plan = compile_mission(cfg, tokens, stab_spec, handover,
                                   live_pilot=live_pilot)
            self.logger.info(f"MISSION={cfg.mission} stab={stab_spec} "
                             f"level={cfg.mv_level} токены={tokens}")
        else:
            plan = build_bootstrap_plan(cfg, build_control_stack(cfg), handover,
                                        live_pilot=live_pilot)
        self.runner = PlanRunner(plan, self.clock, self.actuator, self.logger,
                                 perception=self.perception)

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
                # Мост безжпсного бута: до первой одометрии VINS позу extern-
                # издателя (ray_tracer) взять неоткуда, а EK3 обязан начать
                # aiding НА ЗЕМЛЕ (в воздухе не стартует — LV4). Издаём
                # (0,0,баро) сами (см. _pose_bridge) и замолкаем НАВСЕГДА с
                # первым /odometry — правило «два издателя позы недопустимы»
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
        self._restart_diverge = cfg.vins_restart_diverge > 0
        self._restart_cd = cfg.vins_restart_cd
        self._last_restart_t = -1e9
        self._rebirths_prev = 0    # детект перерождения VINS (VinsTrack) → трим/лог
        self._scale_trips_prev = 0 # срабатываний чека занижения (лог)
        self.timer = self.create_timer(0.05, self._tick)
        self.logger.info(
            f"alt_hold_bootstrap ARCH2: mode={cfg.control_mode} alt={cfg.alt}м "
            f"pilot={pilot_kind} excite_max={cfg.excite_max_sec}s (sim)")

    def _make_pilot(self, cfg, kind):
        sf = cfg.sf_master > 0        # схема SF-мастер (CH7): см. config.sf_master
        if kind == 'joy':
            if cfg.joy_signs:
                return JoyPilot(self, signs=tuple(float(x) for x in cfg.joy_signs.split(',')),
                                sf_master=sf, land_src=cfg.land_joy)
            # знаки — JOY_SIGNS_DEFAULT (выверены полётом TX12)
            return JoyPilot(self, sf_master=sf, land_src=cfg.land_joy)
        if kind == 'ros':
            return RosPilot(self, sf_master=sf)
        # flow_assist — НЕЙТРАЛЬНЫЙ пилот (центр): флоу-демпфер держит снос сам;
        # изолирует боевой пре-VINS сценарий (аналог liftland --flow-hold монолита).
        script = {'assisted': ASSISTED_SCRIPT, 'manual': MANUAL_SCRIPT}.get(cfg.control_mode, [])
        return ScriptedPilot(self.clock, script)

    def _tick(self):
        s = self.telemetry.snapshot()
        armed_front = bool(s.armed) and not self._armed_prev
        self._armed_prev = bool(s.armed)
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
        s.extnav_ready = self._extnav_ready()    # гейт штатного LOITER-на-VINS

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
        wall, см. _vision_feed)."""
        if self._vis_pose_pub is None or self.cfg.vision_pose_src != 'extern':
            return
        if s.vins_odom_count > 0:
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
    _D = BootstrapConfig()          # источник дефолтов для осевых гейнов (см. ниже)
    p = argparse.ArgumentParser()
    p.add_argument('--control-mode', dest='control_mode', default='shuttle',
                   choices=['shuttle', 'assisted', 'manual', 'flow_assist'],
                   help='ЛЕГАСИ-ярлык (стабилизатор+траектория). Игнор при заданном --mission')
    # ОРТОГОНАЛЬНЫЙ путь профиль-миссий (см. plan/mission_plan.py, recipes.build_stabilizers)
    p.add_argument('--stab', default='',
                   help="стабилизатор(ы): GzPosHold|DpRollHold+DpYawHold|DpHold|VinsHold|manual "
                        "('' → GzPosHold при --mission)")
    p.add_argument('--mission', default='',
                   help="плейлист профилей: имя из MISSIONS или 'climb3,mv_fwd2,mv_bkwd4,landing3' "
                        "('' → легаси bootstrap по --control-mode)")
    p.add_argument('--mv-level', dest='mv_level', type=float, default=0.3,
                   help='глобальный уровень стика для mv_* профиль-сегментов [-1..1]')
    p.add_argument('--pilot', default='scripted', choices=['scripted', 'joy', 'ros'],
                   help='источник стиков: scripted (sim-профиль) | joy (живой пульт '
                        'через /joy, мимо FCU) | ros (ЛЕГАСИ /mavros/rc/in — под '
                        'override это эхо собственной команды)')
    p.add_argument('--joy-signs', dest='joy_signs', default=_D.joy_signs,
                   help='знаки осей пульта "r,p,t,y", напр. "-1,1,1,-1" '
                        '(пусто = JOY_SIGNS_DEFAULT, выверены полётом TX12)')
    p.add_argument('--fence', type=float, default=_D.fence,
                   help='геозабор, м от старта: ушли дальше — сразу на посадку (0=выкл)')
    p.add_argument('--excite-max-sec', dest='excite_max_sec', type=float, default=0.0,
                   help='предел длительности EXCITE, sim-сек (0=авто для пилот-режимов)')
    p.add_argument('--alt', type=float, default=3.0)
    p.add_argument('--throttle-climb', dest='throttle_climb', type=int, default=1650)
    p.add_argument('--throttle-hold', dest='throttle_hold', type=int, default=RC_CENTER)
    p.add_argument('--ground-z', dest='ground_z', type=float, default=0.3)
    p.add_argument('--mode-budget', dest='mode_budget', type=float, default=40.0)
    p.add_argument('--arm-budget', dest='arm_budget', type=float, default=40.0)
    p.add_argument('--climb-budget', dest='climb_budget', type=float, default=60.0)
    p.add_argument('--land-budget', dest='land_budget', type=float, default=45.0)
    p.add_argument('--gz-kp', dest='gz_kp', type=float, default=40.0)
    p.add_argument('--gz-kd', dest='gz_kd', type=float, default=120.0)
    p.add_argument('--gz-ki', dest='gz_ki', type=float, default=8.0)
    p.add_argument('--gz-imax', dest='gz_imax', type=float, default=100.0)
    p.add_argument('--gz-max', dest='gz_max', type=float, default=150.0)
    p.add_argument('--gz-psign', dest='gz_psign', type=float, default=1.0)
    p.add_argument('--gz-rsign', dest='gz_rsign', type=float, default=1.0)
    p.add_argument('--gz-cmd-gain', dest='gz_cmd_gain', type=float, default=0.8)
    p.add_argument('--gz-shuttle-level', dest='gz_shuttle_level', type=float, default=0.3)
    p.add_argument('--gz-shuttle-leg', dest='gz_shuttle_leg', type=float, default=3.0)
    p.add_argument('--gz-shuttle-pause', dest='gz_shuttle_pause', type=float, default=2.0)
    p.add_argument('--gz-shuttle-fwd', dest='gz_shuttle_fwd', action='store_true')
    p.add_argument('--slew', dest='slew', type=float, default=_D.slew,
                   help='предел скорости изменения выхода, PWM/сек (0=выкл); τ борта 0.27с')
    p.add_argument('--roll-imax', dest='roll_imax', type=float, default=_D.roll_imax)
    p.add_argument('--pitch-imax', dest='pitch_imax', type=float, default=_D.pitch_imax)
    p.add_argument('--pilot-deadzone', dest='pilot_deadzone', type=int, default=30)
    p.add_argument('--pilot-full', dest='pilot_full', type=int, default=400)
    p.add_argument('--pilot-pitch-sign', dest='pilot_pitch_sign', type=float,
                   default=_D.pilot_pitch_sign)
    p.add_argument('--pilot-roll-sign', dest='pilot_roll_sign', type=float, default=1.0)
    # ---- демпфер по потоку (пре-VINS): ТРИ ОСИ, у каждой свой полный набор ----
    # Дефолты берём из BootstrapConfig (_D), а не хардкодом: parse_args всегда кладёт
    # свой default в d → BootstrapConfig(**d), т.е. хардкод здесь МОЛЧА перекрыл бы
    # дефолт датакласса (именно так velocity-assist один раз остался выключенным).
    p.add_argument('--roll-kp', dest='roll_kp', type=float, default=_D.roll_kp)
    p.add_argument('--roll-ki', dest='roll_ki', type=float, default=_D.roll_ki)
    p.add_argument('--roll-kd', dest='roll_kd', type=float, default=_D.roll_kd)
    p.add_argument('--roll-osign', dest='roll_osign', type=float, default=_D.roll_osign)
    p.add_argument('--roll-cmd-gain', dest='roll_cmd_gain', type=float, default=_D.roll_cmd_gain)
    p.add_argument('--roll-smooth', dest='roll_smooth', type=int, default=_D.roll_smooth)
    # --- ВЫСОТА (внешний контур AltHold; обоснование — control/altitude.py) ---
    p.add_argument('--alt-kp', dest='alt_kp', type=float, default=_D.alt_kp)
    p.add_argument('--alt-rate-max', dest='alt_rate_max', type=float, default=_D.alt_rate_max)
    p.add_argument('--alt-tol', dest='alt_tol', type=float, default=_D.alt_tol)
    p.add_argument('--pitch-kp', dest='pitch_kp', type=float, default=_D.pitch_kp)
    p.add_argument('--pitch-ki', dest='pitch_ki', type=float, default=_D.pitch_ki)
    p.add_argument('--pitch-kd', dest='pitch_kd', type=float, default=_D.pitch_kd)
    p.add_argument('--kf-alt-max', dest='kf_alt_max', type=float, default=_D.kf_alt_max)
    p.add_argument('--kf-alt-hold', dest='kf_alt_hold', type=float, default=_D.kf_alt_hold)
    p.add_argument('--yaw-trans-fix', dest='yaw_trans_fix', type=int, default=_D.yaw_trans_fix)
    p.add_argument('--att-extrap', dest='att_extrap', type=int, default=_D.att_extrap)
    p.add_argument('--att-interp', dest='att_interp', type=int, default=_D.att_interp)
    p.add_argument('--att-latency', dest='att_latency', type=float, default=_D.att_latency)
    p.add_argument('--att-wait-max', dest='att_wait_max', type=float, default=_D.att_wait_max)
    p.add_argument('--ipm-model', dest='ipm_model', default=_D.ipm_model,
                   choices=['legacy', 'rsign', 'exact'])
    p.add_argument('--ipm-derot', dest='ipm_derot', type=float, default=_D.ipm_derot)
    p.add_argument('--ipm-adapt', dest='ipm_adapt', type=float, default=_D.ipm_adapt)
    p.add_argument('--ipm-vel-tau', dest='ipm_vel_tau', type=float,
                   default=_D.ipm_vel_tau)
    p.add_argument('--ipm-alt-floor', dest='ipm_alt_floor', type=float,
                   default=_D.ipm_alt_floor)
    # масштабно-инвариантная полоса IPM (геометрия ∝ alt/h_ref; 0 = легаси)
    p.add_argument('--ipm-scale-ref', dest='ipm_scale_ref', type=float,
                   default=_D.ipm_scale_ref)
    p.add_argument('--vision-vel', dest='vision_vel', type=float,
                   default=_D.vision_vel)
    p.add_argument('--vision-pose-src', dest='vision_pose_src',
                   default=_D.vision_pose_src, choices=['integral', 'extern'])
    p.add_argument('--gps-disable', dest='gps_disable', type=float,
                   default=_D.gps_disable)
    p.add_argument('--gps-denied', dest='gps_denied', type=float,
                   default=_D.gps_denied)
    p.add_argument('--alt-src', dest='alt_src',
                   default=_D.alt_src, choices=['global', 'baro'])
    # высота ПЕРЦЕПЦИИ (масштаб IPM/гейты опоры) — отдельно от alt_src миссии
    p.add_argument('--perc-alt-src', dest='perc_alt_src',
                   default=_D.perc_alt_src, choices=['global', 'local', 'baro'])
    # ноль высоты перцепции по арму (см. config.perc_alt_zero): чинит смещение
    # EKF local z, из-за которого гейт земли IPM не открывался на низком полёте
    p.add_argument('--perc-alt-zero', dest='perc_alt_zero', type=float,
                   default=_D.perc_alt_zero)
    # сброс VINS по арму (см. config.vins_restart_arm): окно инициализации,
    # накопленное за стояние на земле, после отрыва не решается за полёт (O(N³))
    p.add_argument('--vins-restart-arm', dest='vins_restart_arm', type=float,
                   default=_D.vins_restart_arm)
    # ФВЧ прогноза ускорения в фильтре скорости IPM (см. config.ipm_acc_tau):
    # снимает балансирующий ветер наклон, из-за которого боковая ось смещена
    p.add_argument('--ipm-acc-tau', dest='ipm_acc_tau', type=float,
                   default=_D.ipm_acc_tau)
    p.add_argument('--set-origin', dest='set_origin', type=float,
                   default=_D.set_origin)
    # координаты origin (примерная РЕАЛЬНАЯ точка старта — см. config.origin_lat:
    # из них EKF строит модель магнитного поля; дефолт = дом SITL, с 2026-08-24 Киев)
    p.add_argument('--origin-lat', dest='origin_lat', type=float,
                   default=_D.origin_lat)
    p.add_argument('--origin-lon', dest='origin_lon', type=float,
                   default=_D.origin_lon)
    p.add_argument('--origin-alt', dest='origin_alt', type=float,
                   default=_D.origin_alt)
    p.add_argument('--ipm-wz-tau', dest='ipm_wz_tau', type=float, default=_D.ipm_wz_tau)
    p.add_argument('--ipm-wz-gate', dest='ipm_wz_gate', type=float, default=_D.ipm_wz_gate)
    p.add_argument('--ipm-win', dest='ipm_win', type=float, default=_D.ipm_win)
    p.add_argument('--ipm-max-speed', dest='ipm_max_speed', type=float,
                   default=_D.ipm_max_speed)
    p.add_argument('--ipm-alt-band-fwd', dest='ipm_alt_band_fwd', type=float,
                   default=_D.ipm_alt_band_fwd)
    p.add_argument('--pitch-soft-alt', dest='pitch_soft_alt', type=float,
                   default=_D.pitch_soft_alt)
    p.add_argument('--roll-soft-alt', dest='roll_soft_alt', type=float,
                   default=_D.roll_soft_alt)
    p.add_argument('--pitch-soft-noise', dest='pitch_soft_noise', type=float,
                   default=_D.pitch_soft_noise)
    p.add_argument('--roll-soft-noise', dest='roll_soft_noise', type=float,
                   default=_D.roll_soft_noise)
    p.add_argument('--ipm-alt-band-lat', dest='ipm_alt_band_lat', type=float,
                   default=_D.ipm_alt_band_lat)
    p.add_argument('--ipm-alt-still', dest='ipm_alt_still', type=float,
                   default=_D.ipm_alt_still)
    p.add_argument('--ipm-arm-frames', dest='ipm_arm_frames', type=int,
                   default=_D.ipm_arm_frames)
    p.add_argument('--kf-seg-min-sec', dest='kf_seg_min_sec', type=float,
                   default=_D.kf_seg_min_sec)
    p.add_argument('--kf-seg-frac', dest='kf_seg_frac', type=float, default=_D.kf_seg_frac)
    p.add_argument('--pitch-rate-kp', dest='pitch_rate_kp', type=float,
                   default=_D.pitch_rate_kp)
    p.add_argument('--roll-rate-kp', dest='roll_rate_kp', type=float,
                   default=_D.roll_rate_kp)
    # ⚠️ ki/kd rate-осей ДОЛГО СУЩЕСТВОВАЛИ БЕЗ АРГУМЕНТА: поля были в BootstrapConfig и
    # читались в recipes.py, но argparse их не знал, а BootstrapConfig(**vars(a)) собирается
    # ровно из разобранных аргументов — значит поле МОЛЧА получало дефолт датакласса (0.0).
    # Свип B3s (BS_ROLL_RATE_KI=5) из-за этого отлетел с ki=0 и выглядел как «интегратор не
    # помогает». Ручка без аргумента не отсутствует — она врёт, и врёт молча.
    p.add_argument('--roll-rate-ki', dest='roll_rate_ki', type=float,
                   default=_D.roll_rate_ki)
    p.add_argument('--roll-rate-kd', dest='roll_rate_kd', type=float,
                   default=_D.roll_rate_kd)
    p.add_argument('--pitch-rate-ki', dest='pitch_rate_ki', type=float,
                   default=_D.pitch_rate_ki)
    p.add_argument('--pitch-rate-kd', dest='pitch_rate_kd', type=float,
                   default=_D.pitch_rate_kd)
    p.add_argument('--roll-rate-ki-trim', dest='roll_rate_ki_trim', type=float,
                   default=_D.roll_rate_ki_trim)
    p.add_argument('--pitch-rate-ki-trim', dest='pitch_rate_ki_trim', type=float,
                   default=_D.pitch_rate_ki_trim)
    # cmd_gain rate-осей: стик пилота = ЦЕЛЕВАЯ СКОРОСТЬ демпфера (м/с при полном
    # стике), 0 = чистое удержание. Без аргумента поле повторяло судьбу ki/kd выше:
    # в DpHoldM стики roll/pitch молча игнорировались (полёт 2026-08-17 — полный
    # «на себя» не тормозил разгон, пилот был пассажиром до самого fence).
    p.add_argument('--roll-rate-cmd-gain', dest='roll_rate_cmd_gain', type=float,
                   default=_D.roll_rate_cmd_gain)
    p.add_argument('--pitch-rate-cmd-gain', dest='pitch_rate_cmd_gain', type=float,
                   default=_D.pitch_rate_cmd_gain)
    # станция-кипинг rate-осей: стик в центре = держим точку по накопленному пути
    p.add_argument('--pitch-pos-kp', dest='pitch_pos_kp', type=float,
                   default=_D.pitch_pos_kp)
    p.add_argument('--pitch-pos-vmax', dest='pitch_pos_vmax', type=float,
                   default=_D.pitch_pos_vmax)
    p.add_argument('--roll-pos-kp', dest='roll_pos_kp', type=float,
                   default=_D.roll_pos_kp)
    p.add_argument('--roll-pos-vmax', dest='roll_pos_vmax', type=float,
                   default=_D.roll_pos_vmax)
    # два закона станции + anti-windup (см. config.py, _FlowDamper1D.__init__)
    p.add_argument('--roll-pos-brake', dest='roll_pos_brake', type=float,
                   default=_D.roll_pos_brake)
    p.add_argument('--roll-pos-brake-vmax', dest='roll_pos_brake_vmax', type=float,
                   default=_D.roll_pos_brake_vmax)
    p.add_argument('--roll-pos-acc', dest='roll_pos_acc', type=float,
                   default=_D.roll_pos_acc)
    p.add_argument('--pitch-pos-brake', dest='pitch_pos_brake', type=float,
                   default=_D.pitch_pos_brake)
    p.add_argument('--pitch-pos-brake-vmax', dest='pitch_pos_brake_vmax', type=float,
                   default=_D.pitch_pos_brake_vmax)
    p.add_argument('--pitch-pos-acc', dest='pitch_pos_acc', type=float,
                   default=_D.pitch_pos_acc)
    p.add_argument('--roll-pos-brake-v', dest='roll_pos_brake_v', type=float,
                   default=_D.roll_pos_brake_v)
    p.add_argument('--pitch-pos-brake-v', dest='pitch_pos_brake_v', type=float,
                   default=_D.pitch_pos_brake_v)
    p.add_argument('--roll-pos-alt-band', dest='roll_pos_alt_band', type=float,
                   default=_D.roll_pos_alt_band)
    p.add_argument('--pitch-pos-alt-band', dest='pitch_pos_alt_band', type=float,
                   default=_D.pitch_pos_alt_band)
    p.add_argument('--rate-anti-windup', dest='rate_anti_windup', type=float,
                   default=_D.rate_anti_windup)
    p.add_argument('--station-frame', dest='station_frame', choices=('body', 'yaw'),
                   default=_D.station_frame)
    p.add_argument('--station-heading', dest='station_heading', choices=('fcu',),
                   default=_D.station_heading)
    p.add_argument('--pitch-osign', dest='pitch_osign', type=float, default=_D.pitch_osign)
    p.add_argument('--pitch-cmd-gain', dest='pitch_cmd_gain', type=float, default=_D.pitch_cmd_gain)
    p.add_argument('--pitch-smooth', dest='pitch_smooth', type=int, default=_D.pitch_smooth)
    p.add_argument('--yaw-kp', dest='yaw_kp', type=float, default=_D.yaw_kp)
    p.add_argument('--yaw-ki', dest='yaw_ki', type=float, default=_D.yaw_ki)
    p.add_argument('--yaw-kd', dest='yaw_kd', type=float, default=_D.yaw_kd)
    p.add_argument('--yaw-leak', dest='yaw_leak', type=float, default=_D.yaw_leak)
    p.add_argument('--yaw-osign', dest='yaw_osign', type=float, default=_D.yaw_osign)
    p.add_argument('--yaw-cmd-gain', dest='yaw_cmd_gain', type=float, default=_D.yaw_cmd_gain)
    p.add_argument('--yaw-smooth', dest='yaw_smooth', type=int, default=_D.yaw_smooth)
    # °/с при ПОЛНОМ стике — общий темп рыскания (гейн DpYawHold через S, темп
    # Gz-холдера и длительность yaw_l/r-токенов). Поле было недостижимо снаружи —
    # полный стик крутил только 28.65°/с (жалоба пилота freefly 2026-08-18).
    p.add_argument('--yaw-rate-full', dest='yaw_rate_full', type=float,
                   default=_D.yaw_rate_full)
    p.add_argument('--yaw-max-rate', dest='yaw_max_rate', type=float,
                   default=_D.yaw_max_rate)
    # ПРЯМАЯ ПЕРЕДАЧА yaw-стика (PWM при полном стике): лечение пружины курса —
    # разбор spring 2026-08-27, контур разматывал 92–96% разворота обратно
    p.add_argument('--yaw-pilot-gain', dest='yaw_pilot_gain', type=float,
                   default=_D.yaw_pilot_gain)
    p.add_argument('--yaw-v-gate', dest='yaw_v_gate', type=float, default=_D.yaw_v_gate)
    p.add_argument('--yaw-arm-frames', dest='yaw_arm_frames', type=int,
                   default=_D.yaw_arm_frames)
    p.add_argument('--flow-hold-sec', dest='flow_hold_sec', type=float, default=_D.flow_hold_sec)
    p.add_argument('--flow-observe', dest='flow_observe', action='store_true',
                   help='поднять зрение без демпфера: писать /flow_dbg* для замера перцепта')
    # рантайм switch Flow→Vins по «VINS ready» (только flow_assist)
    p.add_argument('--handover-vins', dest='handover_vins', action='store_true')
    p.add_argument('--vins-min', dest='vins_min', type=int, default=40)
    p.add_argument('--vins-fresh-sec', dest='vins_fresh_sec', type=float, default=2.0)
    # гейт здоровья VINS (авто-демоут яруса 1 при разносе; см. config.vins_v_max)
    p.add_argument('--vins-v-max', dest='vins_v_max', type=float, default=_D.vins_v_max)
    p.add_argument('--vins-ipm-tol', dest='vins_ipm_tol', type=float,
                   default=_D.vins_ipm_tol)
    p.add_argument('--vins-sane-n', dest='vins_sane_n', type=int,
                   default=_D.vins_sane_n)
    # источник скорости VINS: diff (разность позы + EMA) | twist (из одометрии) —
    # config.vins_vel_src
    p.add_argument('--vins-vel-src', dest='vins_vel_src', type=str,
                   choices=('diff', 'twist'), default=_D.vins_vel_src)
    p.add_argument('--vins-hover-v', dest='vins_hover_v', type=float,
                   default=_D.vins_hover_v)
    p.add_argument('--vins-hover-sec', dest='vins_hover_sec', type=float,
                   default=_D.vins_hover_sec)
    # чек занижения |vins_v| против IPM (коллапс масштаба; config.vins_scale_*)
    p.add_argument('--vins-scale-ratio', dest='vins_scale_ratio', type=float,
                   default=_D.vins_scale_ratio)
    p.add_argument('--vins-scale-ipm-min', dest='vins_scale_ipm_min', type=float,
                   default=_D.vins_scale_ipm_min)
    p.add_argument('--vins-scale-sec', dest='vins_scale_sec', type=float,
                   default=_D.vins_scale_sec)
    p.add_argument('--vins-scale-alt-max', dest='vins_scale_alt_max', type=float,
                   default=_D.vins_scale_alt_max)
    p.add_argument('--vins-scale-hold', dest='vins_scale_hold', type=float,
                   default=_D.vins_scale_hold)
    # рестарт VINS после демоута-по-разносу (см. config.vins_restart_diverge)
    p.add_argument('--vins-restart-diverge', dest='vins_restart_diverge',
                   type=float, default=_D.vins_restart_diverge)
    p.add_argument('--vins-restart-cd', dest='vins_restart_cd', type=float,
                   default=_D.vins_restart_cd)
    # зрелость VINS для EKF-свапа: sim-секунды от первой одометрии (см. config)
    p.add_argument('--ripe-sec', dest='ripe_sec', type=float, default=_D.ripe_sec)
    # 2-я ступень гейта — детектор residual+ratio (0 = только время)
    p.add_argument('--ripe-det', dest='ripe_det', type=float, default=_D.ripe_det)
    # схема «SF-мастер» селектора: SF (CH7) = мастер сырых стиков, SC (CH6) =
    # потолок лесенки зрелости (см. config.sf_master; ⚠️ не под старые реплеи)
    p.add_argument('--sf-master', dest='sf_master', type=float, default=_D.sf_master)
    # штатный LOITER-на-VINS: freefly-селектор (центр CH6) и бюджет гейта loiter<t>
    p.add_argument('--ff-loiter', dest='ff_loiter', type=float, default=_D.ff_loiter)
    # гейт «в воздухе» LOITER-на-VINS, м (одна правда: Freefly+LoiterHold+HUD)
    p.add_argument('--loiter-alt', dest='loiter_alt', type=float,
                   default=_D.loiter_alt)
    # ярус LOITER: стики = скорость в осях МИРА (TrackHold, см. config.loiter_track)
    p.add_argument('--loiter-track', dest='loiter_track', type=float,
                   default=_D.loiter_track)
    # ярус LOITER, путь 2: потолок крена виража, ° (YawBankLimit; 0 = выкл)
    p.add_argument('--loiter-bank-max', dest='loiter_bank_max', type=float,
                   default=_D.loiter_bank_max)
    # VinsHold: kd на ошибке скорости, не на абсолютной v (см. config.vins_kd_err)
    p.add_argument('--vins-kd-err', dest='vins_kd_err', type=float,
                   default=_D.vins_kd_err)
    # VinsHold: защёлка трима — И-член заморожен стик→гвоздь (config.vins_i_latch)
    p.add_argument('--vins-i-latch', dest='vins_i_latch', type=float,
                   default=_D.vins_i_latch)
    # VinsHold: гвоздь по остановке — уставка на точку стопа (config.vins_pin_stop)
    p.add_argument('--vins-pin-stop', dest='vins_pin_stop', type=float,
                   default=_D.vins_pin_stop)
    # VinsHold: предиктор позы между отсчётами VINS (config.vins_predict)
    p.add_argument('--vins-predict', dest='vins_predict', type=float,
                   default=_D.vins_predict)
    # VinsHold: сглаживание vins-скорости для D-члена, τ с (config.vins_vsmooth)
    p.add_argument('--vins-vsmooth', dest='vins_vsmooth', type=float,
                   default=_D.vins_vsmooth)
    # Ярус 1: стабилизатор на опоре VINS (config.vins_stab)
    p.add_argument('--vins-stab', dest='vins_stab', default=_D.vins_stab,
                   help="ярус VINS: 'vinshold' (2D position-PID) | 'dpvins' "
                        "(velocity-каскад)")
    # DpVins (velocity-каскад) гейны (config.dpvins_*)
    p.add_argument('--dpvins-kp-fwd', dest='dpvins_kp_fwd', type=float,
                   default=_D.dpvins_kp_fwd)
    p.add_argument('--dpvins-kp-lat', dest='dpvins_kp_lat', type=float,
                   default=_D.dpvins_kp_lat)
    p.add_argument('--dpvins-ki', dest='dpvins_ki', type=float,
                   default=_D.dpvins_ki)
    p.add_argument('--dpvins-cmd-gain', dest='dpvins_cmd_gain', type=float,
                   default=_D.dpvins_cmd_gain)
    p.add_argument('--dpvins-pos-kp', dest='dpvins_pos_kp', type=float,
                   default=_D.dpvins_pos_kp)
    p.add_argument('--dpvins-pos-vmax', dest='dpvins_pos_vmax', type=float,
                   default=_D.dpvins_pos_vmax)
    p.add_argument('--dpvins-pos-acc', dest='dpvins_pos_acc', type=float,
                   default=_D.dpvins_pos_acc)
    p.add_argument('--dpvins-vsmooth', dest='dpvins_vsmooth', type=float,
                   default=_D.dpvins_vsmooth)
    p.add_argument('--dpvins-imax', dest='dpvins_imax', type=float,
                   default=_D.dpvins_imax)
    p.add_argument('--dpvins-ki-trim', dest='dpvins_ki_trim', type=float,
                   default=_D.dpvins_ki_trim)
    p.add_argument('--dpvins-trim-keep', dest='dpvins_trim_keep', type=float,
                   default=_D.dpvins_trim_keep)
    p.add_argument('--dpvins-trim-seed', dest='dpvins_trim_seed', type=float,
                   default=_D.dpvins_trim_seed)
    # фаза BRAKE внешнего контура DpVins (config.dpvins_pos_brake*)
    p.add_argument('--dpvins-pos-brake', dest='dpvins_pos_brake', type=float,
                   default=_D.dpvins_pos_brake)
    p.add_argument('--dpvins-pos-brake-v', dest='dpvins_pos_brake_v', type=float,
                   default=_D.dpvins_pos_brake_v)
    p.add_argument('--dpvins-pos-brake-vmax', dest='dpvins_pos_brake_vmax', type=float,
                   default=_D.dpvins_pos_brake_vmax)
    p.add_argument('--dpvins-pos-brake-t', dest='dpvins_pos_brake_t', type=float,
                   default=_D.dpvins_pos_brake_t)
    # по-осевая защёлка трима DpVins (config.dpvins_latch_axis)
    p.add_argument('--dpvins-latch-axis', dest='dpvins_latch_axis', type=float,
                   default=_D.dpvins_latch_axis)
    # гвоздь сразу на входе при посеянном триме (config.dpvins_pin_armed)
    p.add_argument('--dpvins-pin-armed', dest='dpvins_pin_armed', type=float,
                   default=_D.dpvins_pin_armed)
    # прямая передача стика и линия на плече (config.dpvins_ff / dpvins_line_hold)
    p.add_argument('--dpvins-ff', dest='dpvins_ff', type=float, default=_D.dpvins_ff)
    p.add_argument('--dpvins-line-hold', dest='dpvins_line_hold', type=float,
                   default=_D.dpvins_line_hold)
    # тормоз с отпускания и гвоздь по таймауту (config.dpvins_settle_brake / dpvins_pin_t)
    p.add_argument('--dpvins-settle-brake', dest='dpvins_settle_brake', type=float,
                   default=_D.dpvins_settle_brake)
    p.add_argument('--dpvins-pin-t', dest='dpvins_pin_t', type=float, default=_D.dpvins_pin_t)
    # мягкая посадка по кнопке SA в freefly (см. config.ff_land)
    p.add_argument('--ff-land', dest='ff_land', type=float, default=_D.ff_land)
    p.add_argument('--land-alt-max', dest='land_alt_max', type=float,
                   default=_D.land_alt_max)
    p.add_argument('--land-v-max', dest='land_v_max', type=float,
                   default=_D.land_v_max)
    p.add_argument('--land-rate', dest='land_rate', type=float, default=_D.land_rate)
    p.add_argument('--land-joy', dest='land_joy', default=_D.land_joy,
                   help="кнопка посадки в /joy: 'b<i>' buttons[i] | 'a<i>' axes[i] | ''")
    p.add_argument('--loiter-gate-budget', dest='loiter_gate_budget', type=float,
                   default=_D.loiter_gate_budget)
    p.add_argument('--ekf-pos-budget', dest='ekf_pos_budget', type=float,
                   default=_D.ekf_pos_budget)
    a = p.parse_args()
    pilot_kind = a.pilot
    d = vars(a)
    d.pop('pilot')
    cfg = BootstrapConfig(**d)
    # Автотриггер land для пилот-режимов: садимся после демо-профиля (+2с успокоение).
    # Профиль-миссия (--mission) сама секвенсит land — автотриггер не нужен.
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
