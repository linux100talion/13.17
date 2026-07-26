#!/usr/bin/env python3
"""MissionRunner — конечный автомат миссии bootstrap. Потребитель ControlStack.

Срез 1: PREARM → ARM → CLIMB → EXCITE(gz-hold+shuttle) → LAND → DONE. Логика фаз/
бюджетов перенесена из alt_hold_bootstrap.py дословно; фаза EXCITE делегирована в
ControlStack (законы управления — там). Зависит только от портов (Clock/FlightMode/
Logger) и доменных типов — ни строчки rclpy. Возвращает RcCommand, публикует нода.

Зависимость строго mission → control (control о миссии не знает).
"""
from control_pkg.domain.rc import RC_MIN_THR, RcCommand

S_PREARM, S_ARM, S_CLIMB, S_EXCITE, S_LAND, S_DONE = range(6)
S_NAME = {S_PREARM: "PREARM", S_ARM: "ARM", S_CLIMB: "CLIMB",
          S_EXCITE: "EXCITE", S_LAND: "LAND", S_DONE: "DONE"}


class MissionRunner:
    def __init__(self, cfg, clock, flight_mode, stack, log):
        self.cfg = cfg
        self.clock = clock
        self.mode = flight_mode      # порт FlightMode (set_mode/arm)
        self.stack = stack           # ControlStack (фаза EXCITE)
        self.log = log
        self.state = S_PREARM
        self.state_t0 = None         # ленивое базирование по первому тику с живым /clock
        self.last_cmd = -1e9         # троттлинг вызовов сервисов (~1/sim-сек)
        self.last_mode_assert = -1e9
        self.result = "?"
        self.finished = False
        self._entered_excite = False
        self._rcin_logged = False

    # --- утилиты (sim-время) ---
    def _now(self):
        return self.clock.now_sim()

    def _elapsed(self):
        if self.state_t0 is None:
            self.state_t0 = self._now()
            return 0.0
        return self._now() - self.state_t0

    def _goto(self, st):
        self.log.info(f">>> {S_NAME[self.state]} → {S_NAME[st]}")
        self.state = st
        self.state_t0 = None
        self.last_cmd = -1e9

    def _try(self, fn):
        if self._now() - self.last_cmd >= 1.0:
            self.last_cmd = self._now()
            fn()

    def _hold_alt_hold(self, s):
        # Защитный ре-ассерт ALT_HOLD на ARM/CLIMB/EXCITE (страховка на смену режима
        # полётником). '' — транзиент /mavros/state до первого heartbeat.
        if s.mode not in (None, "", "ALT_HOLD") and \
                self._now() - self.last_mode_assert >= 2.0:
            self.last_mode_assert = self._now()
            self.log.warn(f"режим={s.mode} ≠ ALT_HOLD — ре-ассерт")
            self.mode.set_mode("ALT_HOLD")

    # --- автомат: один тик, возвращает RcCommand ---
    def tick(self, s) -> RcCommand:
        cfg = self.cfg
        rc = RcCommand()   # дефолт: все стики в центре, throttle=центр
        st = self.state

        if st == S_PREARM:
            rc.throttle = RC_MIN_THR                       # газ в минимум для арминга
            self._try(lambda: self.mode.set_mode("ALT_HOLD"))
            if s.mode == "ALT_HOLD":
                self._goto(S_ARM)
            elif self._elapsed() > cfg.mode_budget:
                self.log.warn(f"⚠️ ALT_HOLD не залатчился (mode={s.mode}) — пробуем дальше")
                self._goto(S_ARM)

        elif st == S_ARM:
            rc.throttle = RC_MIN_THR
            self._hold_alt_hold(s)
            self._try(self.mode.arm)
            if s.armed:
                self._goto(S_CLIMB)
            elif self._elapsed() > cfg.arm_budget:
                self.log.error(f"⚠️ арм не прошёл (armed={s.armed}) — аборт")
                self.result = "ARM_FAIL"
                self._goto(S_DONE)

        elif st == S_CLIMB:
            rc.throttle = cfg.throttle_climb               # газ вверх → подъём
            self._hold_alt_hold(s)
            if not self._rcin_logged and s.rcin_throttle is not None and self._elapsed() > 2:
                self.log.info(f"    rc/in throttle={s.rcin_throttle} "
                              f"(override проходит, если ≈{cfg.throttle_climb})")
                self._rcin_logged = True
            if s.rel_alt is not None and s.rel_alt >= cfg.alt:
                self.log.info(f"    набрали {s.rel_alt:.1f}м (цель {cfg.alt}м)")
                self._goto(S_EXCITE)
            elif self._elapsed() > cfg.climb_budget:
                if s.rel_alt is not None and s.rel_alt >= 0.5:
                    self.log.warn(f"⚠️ climb-бюджет вышел, высота {s.rel_alt:.1f}м — раскачиваем как есть")
                    self._goto(S_EXCITE)
                else:
                    self.log.error(f"⚠️ не взлетели (rel_alt={s.rel_alt}) — RC override не принят? аборт→LAND")
                    self.result = "CLIMB_FAIL"
                    self._goto(S_LAND)

        elif st == S_EXCITE:
            rc.throttle = cfg.throttle_hold                # держим высоту (центр)
            self._hold_alt_hold(s)
            if not self._entered_excite:
                if not s.gt_valid:
                    return rc                              # ждём истинную позу перед захватом origin
                self.stack.enter(s)
                self._entered_excite = True
                self.log.info(
                    f"    gz-hold+shuttle: центр=({s.gt_x:.2f},{s.gt_y:.2f}) "
                    f"kp={cfg.gz_kp} kd={cfg.gz_kd} ki={cfg.gz_ki} "
                    f"челнок a={cfg.gz_shuttle_a} v={cfg.gz_shuttle_v} fwd={cfg.gz_shuttle_fwd}")
            ctrl = self.stack.update(s)                    # роль-композиция → roll/pitch/yaw
            rc.roll, rc.pitch, rc.yaw = ctrl.roll, ctrl.pitch, ctrl.yaw
            if self.stack.motion_done():
                self.log.info("    челнок завершён — садимся")
                self.result = "HOLD_DONE"
                self._goto(S_LAND)

        elif st == S_LAND:
            rc.throttle = cfg.throttle_hold                # LAND сам снижает, throttle игнор
            self._try(lambda: self.mode.set_mode("LAND"))
            touched = (s.rel_alt is not None and s.rel_alt <= cfg.ground_z)
            if touched or (s.mode == "LAND" and not s.armed and self._elapsed() > 3):
                self.log.info(f"    касание (rel_alt={s.rel_alt}, armed={s.armed})")
                self._goto(S_DONE)
            elif self._elapsed() > cfg.land_budget:
                self.log.warn(f"⚠️ касание не подтверждено (rel_alt={s.rel_alt}) — выходим")
                self._goto(S_DONE)

        elif st == S_DONE:
            self.log.info(f">>> ИТОГ: {self.result} (mode={s.mode}, armed={s.armed}, "
                          f"rel_alt={s.rel_alt}, odom={s.vins_odom_count})")
            self.finished = True

        return rc
