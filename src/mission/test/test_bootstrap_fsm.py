#!/usr/bin/env python3
"""Оффлайн smoke-тест миссии bootstrap (срез 1): MissionRunner + ControlStack на
ФЕЙКАХ-портах, без ROS/Gazebo. Проверяет проводку и прохождение автомата фаз
PREARM → ARM → CLIMB → EXCITE(gz-hold+shuttle) → LAND → DONE.

Фейковый «мир» связывает FlightMode (команды меняют режим/арм) и Telemetry (домен
читает эти изменения) + простая физика высоты по фазе. Ловит регрессии переходов,
вызовов сервисов и триггера land по завершении челнока — до дорогого прогона в симе.

Запуск:  python3 src/mission/test/test_bootstrap_fsm.py
"""
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_here, "..", "..", "..", "src", "control"))   # control_pkg
sys.path.insert(0, os.path.join(_here, ".."))                                 # mission_pkg (../)
sys.path.insert(0, os.path.join(_here, "..", ".."))                           # src/mission

from control_pkg.application.control_stack import ControlStack          # noqa: E402
from control_pkg.domain.control.excitation import NoExcitation          # noqa: E402
from control_pkg.domain.control.stabilization import GzPositionHold     # noqa: E402
from control_pkg.domain.control.trajectory import Shuttle               # noqa: E402
from control_pkg.domain.state import DroneState                         # noqa: E402
from mission_pkg.application.mission_runner import (                    # noqa: E402
    S_CLIMB, S_DONE, S_EXCITE, S_LAND, MissionRunner)
from mission_pkg.config import BootstrapConfig                          # noqa: E402


class FakeWorld:
    def __init__(self):
        self.t = 10.0
        self.mode = ""
        self.armed = False
        self.rel_alt = 0.0
        self.set_mode_calls = []
        self.arm_calls = 0
        self.gt_x = self.gt_y = self.gt_yaw = 0.0
        self.gt_vx = self.gt_vy = 0.0


class FakeClock:
    def __init__(self, w): self._w = w
    def now_sim(self): return self._w.t


class FakeFlightMode:
    def __init__(self, w): self._w = w
    def set_mode(self, m):
        self._w.set_mode_calls.append(m)
        if m in ("ALT_HOLD", "LAND"):
            self._w.mode = m
    def arm(self):
        self._w.arm_calls += 1
        self._w.armed = True
    def ready(self): return True


class FakeTelemetry:
    def __init__(self, w, clock): self._w = w; self._c = clock
    def snapshot(self):
        w = self._w
        return DroneState(mode=w.mode, armed=w.armed, rel_alt=w.rel_alt,
                          rcin_throttle=1650, gt_valid=True,
                          gt_x=w.gt_x, gt_y=w.gt_y, gt_yaw=w.gt_yaw,
                          gt_vx=w.gt_vx, gt_vy=w.gt_vy, now_sim=self._c.now_sim())


class FakeLog:
    def __init__(self): self.lines = []
    def info(self, m): self.lines.append(m)
    def warn(self, m): self.lines.append("WARN " + m)
    def error(self, m): self.lines.append("ERR " + m)


def _physics(w, state, cfg):
    """Минимальная физика высоты по фазе (эмулирует отклик FCU на override)."""
    if state == S_CLIMB and w.armed:
        w.rel_alt = min(cfg.alt + 0.2, w.rel_alt + 0.12)   # набор ~2.4 м/с sim
    elif state == S_LAND:
        w.rel_alt = max(0.0, w.rel_alt - 0.12)             # снижение
        if w.rel_alt <= cfg.ground_z:
            w.armed = False


def main():
    w = FakeWorld()
    clock = FakeClock(w)
    mode = FakeFlightMode(w)
    tele = FakeTelemetry(w, clock)
    log = FakeLog()
    cfg = BootstrapConfig()   # дефолты: alt=3, челнок a=5 v=1.5 pause=2

    stack = ControlStack(
        GzPositionHold(cfg.gz_kp, cfg.gz_kd, cfg.gz_ki, cfg.gz_imax,
                       cfg.gz_max, cfg.gz_psign, cfg.gz_rsign),
        Shuttle(cfg.gz_shuttle_a, cfg.gz_shuttle_v, cfg.gz_shuttle_pause,
                cfg.gz_shuttle_fwd),
        NoExcitation(),
    )
    runner = MissionRunner(cfg, clock, mode, stack, log)

    seen_states = set()
    excite_rc_offsets = 0
    GUARD = 20000
    i = 0
    while not runner.finished and i < GUARD:
        s = tele.snapshot()
        st_before = runner.state
        rc = runner.tick(s)
        seen_states.add(st_before)
        if st_before == S_EXCITE and (rc.roll != 1500 or rc.pitch != 1500):
            excite_rc_offsets += 1       # стек реально командует в EXCITE
        _physics(w, runner.state, cfg)
        w.t += 0.05
        i += 1

    # --- проверки ---
    checks = []
    checks.append(("достигнут DONE", runner.finished and runner.state == S_DONE))
    checks.append(("не упёрлись в guard", i < GUARD))
    checks.append(("result == HOLD_DONE", runner.result == "HOLD_DONE"))
    checks.append(("ALT_HOLD запрошен", "ALT_HOLD" in w.set_mode_calls))
    checks.append(("арм вызван", w.arm_calls >= 1))
    checks.append(("LAND запрошен", "LAND" in w.set_mode_calls))
    checks.append(("прошли CLIMB", S_CLIMB in seen_states))
    checks.append(("прошли EXCITE", S_EXCITE in seen_states))
    checks.append(("прошли LAND", S_LAND in seen_states))
    checks.append(("стек командовал в EXCITE", excite_rc_offsets > 0))

    print(f"Оффлайн smoke-тест FSM bootstrap (итераций: {i}):")
    ok_all = True
    for name, ok in checks:
        print(f"  [{'OK ' if ok else 'FAIL'}] {name}")
        ok_all = ok_all and ok
    print("ИТОГ:", "✅ АВТОМАТ ПРОШЁЛ" if ok_all else "❌ СБОЙ")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
