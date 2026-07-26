#!/usr/bin/env python3
"""Оффлайн-гейт ортогонального пути stab×mission (чистый python, без ROS/Gazebo).

Проверяет:
  1. build_stabilizers — реестр + '+'-склейка + оси + manual=[] + ошибка на мусор.
  2. токен-грамматика _parse (climb3 / mv_fwd2 / land).
  3. compile_mission — Mission1 компилится в prearm→arm→climb→mv→mv→land, имена уникальны.
  4. прогон Mission1 через PlanRunner на фейках: план завершается MISSION_DONE, шаги
     пройдены по порядку, стабилизатор (GzPosHold) реально командует в mv-сегментах.
  5. Dp-миссия: wait_gt=False, стабилизаторы — флоу-демпфер.

Запуск:  python3 src/mission/test/test_mission_plan.py
"""
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_here, "..", "..", "..", "src", "control"))
sys.path.insert(0, os.path.join(_here, ".."))
sys.path.insert(0, os.path.join(_here, "..", ".."))

from control_pkg.domain.state import DroneState                        # noqa: E402
from mission_pkg.config import BootstrapConfig                         # noqa: E402
from mission_pkg.plan.mission_plan import (                            # noqa: E402
    _parse, compile_mission, resolve_mission)
from mission_pkg.plan.runner import PlanRunner                         # noqa: E402
from mission_pkg.recipes import build_stabilizers                     # noqa: E402


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
    if step_name.startswith("climb") and w.armed:
        w.rel_alt = min(cfg.alt + 0.2, w.rel_alt + 0.12)
    elif step_name == "land":
        w.rel_alt = max(0.0, w.rel_alt - 0.12)
        if w.rel_alt <= cfg.ground_z:
            w.armed = False


def run_plan(cfg, steps):
    w = FakeWorld()
    clock = FakeClock(w)
    runner = PlanRunner(steps, clock, FakeMode(w), FakeLog())
    seen, ctrl_off, i = set(), 0, 0
    while not runner.finished and i < 40000:
        s = DroneState(mode=w.mode, armed=w.armed, rel_alt=w.rel_alt,
                       gt_valid=True, now_sim=clock.now_sim())
        name = runner.steps[runner.i].name
        rc = runner.tick(s)
        seen.add(name)
        if name.startswith(("mv_", "hover")) and (rc.roll != 1500 or rc.pitch != 1500
                                                   or rc.yaw != 1500):
            ctrl_off += 1
        _physics(w, name, cfg)
        w.t += 0.05
        i += 1
    return runner, seen, ctrl_off, w, i


def main():
    checks = []
    cfg = BootstrapConfig(mv_level=0.3)

    # 1. build_stabilizers -----------------------------------------------------
    combo = build_stabilizers(cfg, "DpRollHold+DpYawHold")
    axes_combo = set().union(*[st.axes for st in combo])
    checks.append(("stab: '+'-склейка → 2 стратегии", len(combo) == 2))
    checks.append(("stab: склейка roll+yaw", axes_combo == {"roll", "yaw"}))
    dphold = build_stabilizers(cfg, "DpHold")
    checks.append(("stab: DpHold = 1 композит (roll+pitch+yaw)",
                   len(dphold) == 1 and dphold[0].axes == {"roll", "pitch", "yaw"}))
    gz = build_stabilizers(cfg, "GzPosHold")
    checks.append(("stab: GzPosHold держит все 3 оси",
                   len(gz) == 1 and gz[0].axes == {"roll", "pitch", "yaw"}))
    checks.append(("stab: manual → []", build_stabilizers(cfg, "manual") == []))
    try:
        build_stabilizers(cfg, "Nonsense")
        bad = False
    except ValueError:
        bad = True
    checks.append(("stab: мусор → ValueError", bad))

    # 2. токен-грамматика ------------------------------------------------------
    checks.append(("token: climb3 → ('climb',3)", _parse("climb3") == ("climb", 3.0)))
    checks.append(("token: mv_fwd2 → ('mv_fwd',2)", _parse("mv_fwd2") == ("mv_fwd", 2.0)))
    checks.append(("token: mv_bkwd4 → ('mv_bkwd',4)", _parse("mv_bkwd4") == ("mv_bkwd", 4.0)))
    checks.append(("token: land → ('land',None)", _parse("land") == ("land", None)))

    # 3. compile Mission1 ------------------------------------------------------
    steps = compile_mission(cfg, "Mission1", "GzPosHold")
    names = [st.name for st in steps]
    checks.append(("compile: prearm+arm пролог", names[:2] == ["prearm", "arm"]))
    checks.append(("compile: climb-сегмент присутствует",
                   any(n.startswith("climb") for n in names)))
    checks.append(("compile: 2 mv-сегмента", sum(n.startswith("mv_") for n in names) == 2))
    checks.append(("compile: последний шаг land", names[-1] == "land"))
    checks.append(("compile: имена уникальны", len(names) == len(set(names))))

    # 4. прогон Mission1 (GzPosHold) ------------------------------------------
    r, seen, off, w, it = run_plan(cfg, compile_mission(cfg, "Mission1", "GzPosHold"))
    checks.append(("run: план завершён", r.finished and it < 40000))
    checks.append(("run: MISSION_DONE", r.result == "MISSION_DONE"))
    checks.append(("run: ALT_HOLD+арм+LAND", "ALT_HOLD" in w.set_mode_calls
                   and w.arm_calls >= 1 and "LAND" in w.set_mode_calls))
    checks.append(("run: прошли climb→mv→land",
                   any(n.startswith("climb") for n in seen)
                   and any(n.startswith("mv_") for n in seen) and "land" in seen))
    checks.append(("run: GzPosHold командовал в mv-сегментах", off > 0))

    # 5. Dp-миссия: wait_gt=False, стабилизаторы флоу ------------------------
    dp_steps = compile_mission(cfg, "climb3,mv_fwd2,land", "DpRollHold+DpYawHold")
    mv = next(st for st in dp_steps if st.name.startswith("mv_"))
    checks.append(("dp-mission: mv-сегмент wait_gt=False", mv.wait_gt is False))
    checks.append(("dp-mission: 2 флоу-стабилизатора в стеке", len(mv.stack.stabs) == 2))

    # 6. bootstrap как именованная миссия (callable-значение) ----------------
    boot = resolve_mission(cfg, "bootstrap")
    checks.append(("mission: bootstrap раскрылся в токены (climb…hover…land)",
                   boot[0].startswith("climb") and boot[-1] == "land"))

    # 7. resolve_mission идемпотентна (узел резолвит → compile_mission резолвит ещё раз) --
    toks = resolve_mission(cfg, "Mission1")
    checks.append(("resolve: список токенов не ре-резолвится (list unhashable)",
                   resolve_mission(cfg, toks) == toks))
    st2 = compile_mission(cfg, toks, "GzPosHold")   # приняв уже-список — как из ноды
    checks.append(("compile: принимает уже-резолвленный список",
                   [x.name for x in st2] == names))

    print("Оффлайн-гейт stab×mission:")
    ok_all = True
    for name, ok in checks:
        print(f"  [{'OK ' if ok else 'FAIL'}] {name}")
        ok_all = ok_all and ok
    print("ИТОГ:", "✅ STAB×MISSION OK" if ok_all else "❌ СБОЙ")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
