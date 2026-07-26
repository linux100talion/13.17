#!/usr/bin/env python3
"""Оффлайн smoke плана в пилот-режимах: assisted + manual + seize Arbiter, на ФЕЙКАХ-
портах со ScriptedPilot, без ROS/Gazebo. Зеркалит композицию тика ноды (pilot→снапшот →
runner.tick → arbiter.resolve). Проверяет прохождение плана и что пилот реально управляет.

Запуск:  python3 src/mission/test/test_pilot_fsm.py
"""
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_here, "..", "..", "..", "src", "control"))
sys.path.insert(0, os.path.join(_here, ".."))
sys.path.insert(0, os.path.join(_here, "..", ".."))

from control_pkg.application.arbiter import Arbiter                    # noqa: E402
from control_pkg.domain.state import DroneState                       # noqa: E402
from control_pkg.infrastructure.ros_pilot import ScriptedPilot        # noqa: E402
from mission_pkg.config import BootstrapConfig                        # noqa: E402
from mission_pkg.plan.bootstrap_plan import build_bootstrap_plan      # noqa: E402
from mission_pkg.plan.runner import PlanRunner                        # noqa: E402
from mission_pkg.recipes import build_control_stack                   # noqa: E402


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


def run(mode, script, switch=None):
    w = FakeWorld()
    clock = FakeClock(w)
    mode_port = FakeMode(w)
    log = FakeLog()
    cfg = BootstrapConfig(control_mode=mode, excite_max_sec=(script[-1][0] + 1.0))
    stack = build_control_stack(cfg)
    runner = PlanRunner(build_bootstrap_plan(cfg, stack), clock, mode_port, log)
    pilot = ScriptedPilot(clock, script, switch_segments=switch)
    arb = Arbiter()

    seen = set()
    control_offsets = 0
    manual_match = 0
    i = 0
    while not runner.finished and i < 20000:
        s = DroneState(mode=w.mode, armed=w.armed, rel_alt=w.rel_alt, gt_valid=True,
                       now_sim=clock.now_sim())
        st = pilot.sticks()
        s.pilot_roll, s.pilot_pitch, s.pilot_throttle, s.pilot_yaw = \
            st.roll, st.pitch, st.throttle, st.yaw
        s.pilot_switch = pilot.mode_switch()
        step_name = runner.steps[runner.i].name
        rc = runner.tick(s)
        rc = arb.resolve(s, rc)
        seen.add(step_name)
        if step_name == "control":
            if rc.roll != 1500 or rc.pitch != 1500:
                control_offsets += 1
            if arb.last_manual and rc.roll == s.pilot_roll and rc.pitch == s.pilot_pitch:
                manual_match += 1
        _physics(w, runner.steps[runner.i].name, cfg)
        w.t += 0.05
        i += 1
    return runner, seen, control_offsets, manual_match, i


def main():
    checks = []

    # 1. ASSISTED: пульт=намерение, gz-hold ведёт. Стек должен командовать в control.
    r, seen, off, _, it = run('assisted', [(2.0, 1500, 1650, 1500), (4.0, 1500, 1500, 1500)])
    checks += [
        ("assisted: завершён", r.finished),
        ("assisted: HOLD_DONE", r.result == "HOLD_DONE"),
        ("assisted: прошли control→land", "control" in seen and "land" in seen),
        ("assisted: gz-hold командовал (стик→уставка→оффсет)", off > 0),
    ]

    # 2. MANUAL: passthrough. rc в control = стики пилота.
    r, seen, off, _, it = run('manual', [(2.0, 1560, 1500, 1500), (4.0, 1500, 1500, 1500)])
    checks += [
        ("manual: завершён", r.finished),
        ("manual: прошли control→land", "control" in seen and "land" in seen),
        ("manual: passthrough дал оффсет", off > 0),
    ]

    # 3. SEIZE: тумблер MANUAL всю миссию → Arbiter отдаёт сырые стики в control.
    r, seen, off, mmatch, it = run(
        'shuttle', [(3.0, 1580, 1500, 1520), (6.0, 1500, 1500, 1500)], switch=[(999.0, 1)])
    checks += [
        ("seize: завершён", r.finished),
        ("seize: Arbiter отдал стики пилоту в control", mmatch > 0),
    ]

    print("Оффлайн smoke пилот-режимов (план):")
    ok_all = True
    for name, ok in checks:
        print(f"  [{'OK ' if ok else 'FAIL'}] {name}")
        ok_all = ok_all and ok
    print("ИТОГ:", "✅ ПИЛОТ-ПАЙПЛАЙН OK" if ok_all else "❌ СБОЙ")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
