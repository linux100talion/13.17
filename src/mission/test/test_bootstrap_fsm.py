#!/usr/bin/env python3
"""Оффлайн smoke-тест плана bootstrap (PlanRunner + build_bootstrap_plan) на ФЕЙКАХ-
портах, без ROS/Gazebo. Проверяет проводку и прохождение шагов
prearm → arm → climb → control(gz-hold+shuttle) → land → финиш.

Запуск:  python3 src/mission/test/test_bootstrap_fsm.py
"""
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_here, "..", "..", "..", "src", "control"))
sys.path.insert(0, os.path.join(_here, ".."))
sys.path.insert(0, os.path.join(_here, "..", ".."))

from mission_pkg.config import BootstrapConfig                          # noqa: E402
from mission_pkg.plan.bootstrap_plan import build_bootstrap_plan        # noqa: E402
from mission_pkg.plan.runner import PlanRunner                          # noqa: E402
from mission_pkg.recipes import build_control_stack                     # noqa: E402
from control_pkg.domain.state import DroneState                         # noqa: E402


class FakeWorld:
    def __init__(self):
        self.t = 10.0
        self.mode = ""
        self.armed = False
        self.rel_alt = 0.0
        self.set_mode_calls = []
        self.arm_calls = 0


class FakeClock:
    def __init__(self, w): self._w = w
    def now_sim(self): return self._w.t


class FakeMode:
    def __init__(self, w): self._w = w
    def set_mode(self, m):
        self._w.set_mode_calls.append(m)
        if m in ("ALT_HOLD", "LAND"): self._w.mode = m
    def arm(self): self._w.arm_calls += 1; self._w.armed = True
    def ready(self): return True


class FakeLog:
    def __init__(self): self.lines = []
    def info(self, m): self.lines.append(m)
    def warn(self, m): self.lines.append("WARN " + m)
    def error(self, m): self.lines.append("ERR " + m)


def _physics(w, step_name, cfg):
    if step_name == "climb" and w.armed:
        w.rel_alt = min(cfg.alt + 0.2, w.rel_alt + 0.12)
    elif step_name == "land":
        w.rel_alt = max(0.0, w.rel_alt - 0.12)
        if w.rel_alt <= cfg.ground_z:
            w.armed = False


def main():
    w = FakeWorld()
    clock = FakeClock(w)
    mode = FakeMode(w)
    log = FakeLog()
    cfg = BootstrapConfig()   # shuttle по умолчанию
    stack = build_control_stack(cfg)
    runner = PlanRunner(build_bootstrap_plan(cfg, stack), clock, mode, log)

    seen = set()
    control_offsets = 0
    GUARD = 20000
    i = 0
    while not runner.finished and i < GUARD:
        s = DroneState(mode=w.mode, armed=w.armed, rel_alt=w.rel_alt,
                       gt_valid=True, now_sim=clock.now_sim())
        step_name = runner.steps[runner.i].name
        rc = runner.tick(s)
        seen.add(step_name)
        if step_name == "control" and (rc.roll != 1500 or rc.pitch != 1500):
            control_offsets += 1
        _physics(w, runner.steps[runner.i].name, cfg)
        w.t += 0.05
        i += 1

    checks = [
        ("план завершён", runner.finished and i < GUARD),
        ("result == HOLD_DONE", runner.result == "HOLD_DONE"),
        ("ALT_HOLD запрошен", "ALT_HOLD" in w.set_mode_calls),
        ("арм вызван", w.arm_calls >= 1),
        ("LAND запрошен", "LAND" in w.set_mode_calls),
        ("прошли prearm", "prearm" in seen),
        ("прошли arm", "arm" in seen),
        ("прошли climb", "climb" in seen),
        ("прошли control", "control" in seen),
        ("прошли land", "land" in seen),
        ("стек командовал в control", control_offsets > 0),
    ]
    print(f"Оффлайн smoke план bootstrap (итераций: {i}):")
    ok_all = True
    for name, ok in checks:
        print(f"  [{'OK ' if ok else 'FAIL'}] {name}")
        ok_all = ok_all and ok
    print("ИТОГ:", "✅ ПЛАН ПРОШЁЛ" if ok_all else "❌ СБОЙ")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
