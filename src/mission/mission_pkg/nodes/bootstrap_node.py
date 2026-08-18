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
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty

from control_pkg.application.arbiter import Arbiter
from control_pkg.application.handover import VinsHandover
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
from ..recipes import build_control_stack

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
        self.telemetry = RosTelemetry(self, self.clock)
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
                    (use_mission and 'Dp' in stab_spec)
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
                                            ipm_model=cfg.ipm_model,
                                            ipm_derot=cfg.ipm_derot,
                                            ipm_wz_tau=cfg.ipm_wz_tau,
                                            ipm_win=cfg.ipm_win,
                                            ipm_adapt=cfg.ipm_adapt,
                                            ipm_vel_tau=cfg.ipm_vel_tau)

        # рантайм switch Flow→Vins: флаг + флоу-стабилизатор (VinsHold на gz_* гейнах)
        handover = None
        if cfg.handover_vins and (cfg.control_mode == 'flow_assist' or
                                  (use_mission and 'Dp' in stab_spec)):
            vins = VinsHold(cfg.gz_kp, cfg.gz_kd, cfg.gz_ki, cfg.gz_imax,
                            cfg.gz_max, cfg.gz_psign, cfg.gz_rsign, cfg.gz_cmd_gain)
            handover = VinsHandover(vins, cfg.vins_min, cfg.vins_fresh_sec)
            self.logger.info(f"handover Flow→Vins ВКЛ: ready при ≥{cfg.vins_min} odom")

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

        self._last_rc = RcCommand()
        self._arb_seized = False
        # «хватит летать»: make pilot-done → one-shot в снапшот (завершает бессрочный
        # pilot-сегмент). Слать ВО ВРЕМЯ pilot-сегмента: в других фазах тик его съест.
        self._pilot_done = False
        self.create_subscription(Empty, '/mission/pilot_done',
                                 lambda _m: setattr(self, '_pilot_done', True), 1)
        self.timer = self.create_timer(0.05, self._tick)
        self.logger.info(
            f"alt_hold_bootstrap ARCH2: mode={cfg.control_mode} alt={cfg.alt}м "
            f"pilot={pilot_kind} excite_max={cfg.excite_max_sec}s (sim)")

    def _make_pilot(self, cfg, kind):
        if kind == 'joy':
            if cfg.joy_signs:
                return JoyPilot(self, signs=tuple(float(x) for x in cfg.joy_signs.split(',')))
            return JoyPilot(self)     # знаки — JOY_SIGNS_DEFAULT (выверены полётом TX12)
        if kind == 'ros':
            return RosPilot(self)
        # flow_assist — НЕЙТРАЛЬНЫЙ пилот (центр): флоу-демпфер держит снос сам;
        # изолирует боевой пре-VINS сценарий (аналог liftland --flow-hold монолита).
        script = {'assisted': ASSISTED_SCRIPT, 'manual': MANUAL_SCRIPT}.get(cfg.control_mode, [])
        return ScriptedPilot(self.clock, script)

    def _tick(self):
        s = self.telemetry.snapshot()
        if self.perception is not None:
            self.perception.merge(s)          # камера → flow_* в снапшот
        # пилот → в снапшот (домен читает pilot_* как телеметрию)
        sticks = self.pilot.sticks()
        s.pilot_roll, s.pilot_pitch = sticks.roll, sticks.pitch
        s.pilot_throttle, s.pilot_yaw = sticks.throttle, sticks.yaw
        s.pilot_switch = self.pilot.mode_switch()
        s.pilot_done, self._pilot_done = self._pilot_done, False   # one-shot

        rc = self.runner.tick(s)
        rc = self.arbiter.resolve(s, rc)          # safety-seize: MANUAL → сырые стики
        if self.arbiter.last_manual != self._arb_seized:
            self._arb_seized = self.arbiter.last_manual
            self.logger.warn("ПИЛОТ ВЗЯЛ УПРАВЛЕНИЕ (MANUAL)" if self._arb_seized
                             else "возврат в АВТО")
        self._last_rc = rc
        self._publish(rc)
        self.debug.publish_axes(s, rc)   # флоу-дамп в bag: /flow_dbg + /flow_dbg2 (sim-штамп)
        self.debug.publish_hold(self._hold_dbg('pitch'))       # /flow_dbg5: уставка тангажа
        # /flow_dbg7: цель крена по скорости + PWM (единственная запись roll-команды)
        self.debug.publish_rate_roll(self._rate_dbg('roll'))
        # /flow_dbg6: уставка курса + PWM рыскания (единственная запись yaw-команды)
        self.debug.publish_hold_yaw(self._hold_dbg('yaw'), rc.yaw - RC_CENTER)

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
    p.add_argument('--ipm-model', dest='ipm_model', default=_D.ipm_model,
                   choices=['legacy', 'rsign', 'exact'])
    p.add_argument('--ipm-derot', dest='ipm_derot', type=float, default=_D.ipm_derot)
    p.add_argument('--ipm-adapt', dest='ipm_adapt', type=float, default=_D.ipm_adapt)
    p.add_argument('--ipm-vel-tau', dest='ipm_vel_tau', type=float,
                   default=_D.ipm_vel_tau)
    p.add_argument('--ipm-wz-tau', dest='ipm_wz_tau', type=float, default=_D.ipm_wz_tau)
    p.add_argument('--ipm-win', dest='ipm_win', type=float, default=_D.ipm_win)
    p.add_argument('--ipm-max-speed', dest='ipm_max_speed', type=float,
                   default=_D.ipm_max_speed)
    p.add_argument('--ipm-alt-band-fwd', dest='ipm_alt_band_fwd', type=float,
                   default=_D.ipm_alt_band_fwd)
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
    # cmd_gain rate-осей: стик пилота = ЦЕЛЕВАЯ СКОРОСТЬ демпфера (м/с при полном
    # стике), 0 = чистое удержание. Без аргумента поле повторяло судьбу ki/kd выше:
    # в DpHoldM стики roll/pitch молча игнорировались (полёт 2026-08-17 — полный
    # «на себя» не тормозил разгон, пилот был пассажиром до самого fence).
    p.add_argument('--roll-rate-cmd-gain', dest='roll_rate_cmd_gain', type=float,
                   default=_D.roll_rate_cmd_gain)
    p.add_argument('--pitch-rate-cmd-gain', dest='pitch_rate_cmd_gain', type=float,
                   default=_D.pitch_rate_cmd_gain)
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
    p.add_argument('--yaw-max-rate', dest='yaw_max_rate', type=float,
                   default=_D.yaw_max_rate)
    p.add_argument('--yaw-arm-frames', dest='yaw_arm_frames', type=int,
                   default=_D.yaw_arm_frames)
    p.add_argument('--flow-hold-sec', dest='flow_hold_sec', type=float, default=_D.flow_hold_sec)
    p.add_argument('--flow-observe', dest='flow_observe', action='store_true',
                   help='поднять зрение без демпфера: писать /flow_dbg* для замера перцепта')
    # рантайм switch Flow→Vins по «VINS ready» (только flow_assist)
    p.add_argument('--handover-vins', dest='handover_vins', action='store_true')
    p.add_argument('--vins-min', dest='vins_min', type=int, default=40)
    p.add_argument('--vins-fresh-sec', dest='vins_fresh_sec', type=float, default=2.0)
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
