#!/usr/bin/env python3
"""Оффлайн-тест ВОЗВРАТА ПО СВОЕМУ ТРЕКУ (шаг RthTrack: GUIDED + поток уставок).

Проверяет:
- отказ, если латча не было (трек пуст / rth≠ready) — возвращаться некуда;
- GUIDED просится раз в sim-секунду, после латча повторных set_mode нет;
- уставки идут КАЖДЫЙ тик (GUID_TIMEOUT полётника 3 с) в координатах трека;
- разматывание НАЗАД: старт с ближайшей точки, индекс убывает по мере подхода;
- КУРС ПО ТРЕКУ (нос на следующую точку), вблизи точки курс не командуется;
- дома (точка 0) → прыжок на шаг мягкой посадки (RTH_HOME);
- отмена импульсом, MANUAL, guard (закрытый мост), выход FCU из GUIDED, бюджет.

Чистый python, без ROS.  Запуск: python3 src/mission/test/test_rth_track.py
"""
import math
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_here, "..", "..", "..", "src", "control"))
sys.path.insert(0, os.path.join(_here, ".."))
sys.path.insert(0, os.path.join(_here, "..", ".."))

from control_pkg.domain.rc import RC_CENTER                            # noqa: E402
from control_pkg.domain.state import DroneState                        # noqa: E402
from mission_pkg.plan.runner import PlanRunner                         # noqa: E402
from mission_pkg.plan.step import RthTrack                             # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


class FakeStack:
    def __init__(self):
        self.stabs = ['damper']

    def switch_stabilization(self, stabs):
        self.stabs = list(stabs)

    def enter(self, s):
        pass

    def update(self, s):
        from control_pkg.domain.rc import RcCommand
        return RcCommand()


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def now_sim(self):
        return self.t


class FakeMode:
    def __init__(self):
        self.modes = []

    def set_mode(self, m):
        self.modes.append(m)

    def arm(self, v=True):
        pass

    def ready(self):
        return True


class FakeSp:
    def __init__(self):
        self.pts = []

    def publish_pos(self, x, y, z, yaw=None):
        self.pts.append((round(x, 3), round(y, 3), round(z, 3), yaw))


class FakeLog:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    def warn(self, m):
        self.lines.append("WARN " + m)

    def error(self, m):
        self.lines.append("ERR " + m)

    def count(self, sub):
        return sum(1 for ln in self.lines if sub in ln)


class Done:
    """Шаг-приёмник (вместо SoftLand / freefly): фиксирует, что до него дошли."""

    def __init__(self, name="land"):
        self.name = name
        self.entered = False

    def enter(self, ctx, s):
        self.entered = True

    def tick(self, ctx, s):
        from mission_pkg.plan.step import _run
        from control_pkg.domain.rc import RcCommand
        return _run(RcCommand())


# трек: дом (0,0) → (5,0) → (10,0) → (15,0), высота 3 м
TRACK = [(0.0, 0.0, 3.0), (5.0, 0.0, 3.0), (10.0, 0.0, 3.0), (15.0, 0.0, 3.0)]


def make(track=TRACK, guard=False, speed=2.0, land_home=True, wp_r=1.5):
    clock, mode, log, sp = FakeClock(), FakeMode(), FakeLog(), FakeSp()
    stack = FakeStack()
    land, ff = Done("land"), Done("freefly")
    rth = RthTrack("rth", stack, wp_r=wp_r, speed=speed, guard=guard,
                   land_home=land_home)
    runner = PlanRunner([land, ff, rth], clock, mode, log, setpoints=sp)
    runner.i = 2                      # входим сразу в rth
    return runner, clock, mode, log, sp, stack, land, rth


def snap(t, x=15.0, y=0.0, z=3.0, mode="GUIDED", armed=True, sw=-1, rth=False,
         state="ready", track=TRACK, brg_seen=False, brg_open=True):
    return DroneState(now_sim=t, armed=armed, mode=mode, pilot_switch=sw,
                      pilot_rth=rth, rth_state=state, rth_track=track,
                      ekf_x=x, ekf_y=y, ekf_z=z, rel_alt=z,
                      bridge_seen=brg_seen, bridge_open=brg_open,
                      pilot_roll=RC_CENTER, pilot_pitch=RC_CENTER,
                      pilot_throttle=RC_CENTER, pilot_yaw=RC_CENTER)


def run(r, clock, dur, dt=0.05, **kw):
    t_end = clock.t + dur
    while clock.t < t_end and not r.finished:
        clock.t += dt
        r.tick(snap(clock.t, **kw))


# --- 1. нет латча → отказ ---
r, clock, mode, log, sp, stack, land, rth = make(track=[])
run(r, clock, 0.2, track=[])
check("трека нет → RTH_REFUSED (возвращаться некуда)", r.result == "RTH_REFUSED")
r, clock, mode, log, sp, stack, land, rth = make()
run(r, clock, 0.2, state="heal")
check("rth=heal (дом не залатчен) → RTH_REFUSED", r.result == "RTH_REFUSED")

# --- 2. GUIDED просится, стек пуст, уставки идут каждый тик ---
r, clock, mode, log, sp, stack, land, rth = make()
run(r, clock, 0.2, mode="ALT_HOLD")
check("стек ПУСТ (ведёт FCU)", stack.stabs == [])
check("послан GUIDED", "GUIDED" in mode.modes)
n_before = len(sp.pts)
run(r, clock, 0.5, mode="ALT_HOLD")
check("уставки идут ДО латча (GUIDED без цели просто висит)", len(sp.pts) > n_before)
run(r, clock, 2.0, mode="GUIDED")
n_mode = mode.modes.count("GUIDED")
run(r, clock, 2.0, mode="GUIDED")
check("после латча повторных set_mode нет", mode.modes.count("GUIDED") == n_mode)
check("латч отмечен в логе", log.count("GUIDED залатчен") == 1)

# --- 3. разматывание НАЗАД от ближайшей точки + курс по треку ---
r, clock, mode, log, sp, stack, land, rth = make()
rth._track = TRACK                    # _nearest читает трек, снятый на входе в шаг
check("ближайшая точка трека к позе x=12 — индекс 2 (10,0)",
      rth._nearest(snap(0.0, x=12.0)) == 2)
run(r, clock, 0.2, x=15.0, mode="GUIDED")
check("вошли на конце трека (x=15): точка пройдена, цель — следующая К ДОМУ",
      sp.pts[-1][:3] == (10.0, 0.0, 3.0) and rth._i == 2)
yaw = sp.pts[-1][3]
check("КУРС ПО ТРЕКУ: нос на точку (−x → yaw ≈ 180°)",
      yaw is not None and abs(abs(yaw) - math.pi) < 1e-6)
run(r, clock, 0.2, x=9.0, mode="GUIDED")    # в радиусе 1.5 от (10,0) → цель (5,0)
check("идём дальше к дому", sp.pts[-1][:3] == (5.0, 0.0, 3.0) and rth._i == 1)
# при мелком радиусе приёмки курс у самой точки не командуется (дрожал бы)
r2, clock2, mode2, log2, sp2, stack2, land2, rth2 = make(wp_r=0.2)
run(r2, clock2, 0.2, x=10.3, mode="GUIDED")
check("вблизи точки (< 0.5 м) курс НЕ командуем", sp2.pts[-1][3] is None)

# --- 4. дошли домой → мягкая посадка ---
r, clock, mode, log, sp, stack, land, rth = make()
run(r, clock, 0.2, x=15.0, mode="GUIDED")
for x in (14.0, 9.0, 4.0, 0.5):
    run(r, clock, 0.1, x=x, mode="GUIDED")
check("дома → прыжок на шаг мягкой посадки (RTH_HOME)",
      r.result == "RTH_HOME" and land.entered)
check("в логе сказано, что дома и садимся", log.count("ДОМА (точка 0 трека)") == 1)

# --- 4б. land_home=0 → висим дома, шаг не завершается ---
r, clock, mode, log, sp, stack, land, rth = make(land_home=False)
run(r, clock, 0.2, x=15.0, mode="GUIDED")
for x in (14.0, 9.0, 4.0, 0.5):
    run(r, clock, 0.1, x=x, mode="GUIDED")
check("land_home=0: остаёмся в шаге (висим дома)",
      not land.entered and r.result != "RTH_HOME")

# --- 5. выходы: отмена, MANUAL, guard, вылет FCU из GUIDED ---
r, clock, mode, log, sp, stack, land, rth = make()
run(r, clock, 1.0, mode="GUIDED")
run(r, clock, 0.05, mode="GUIDED", rth=True)
check("повторный импульс → RTH_CANCEL + keep", r.result == "RTH_CANCEL"
      and mode.modes[-1] == "ALT_HOLD")
r, clock, mode, log, sp, stack, land, rth = make()
run(r, clock, 1.0, mode="GUIDED")
run(r, clock, 0.05, mode="GUIDED", sw=1)
check("MANUAL → RTH_MANUAL", r.result == "RTH_MANUAL")
r, clock, mode, log, sp, stack, land, rth = make(guard=True)
run(r, clock, 1.0, mode="GUIDED")
run(r, clock, 0.5, mode="GUIDED", brg_seen=True, brg_open=False)
check("мост закрыт < GUARD_SEC → ещё летим", r.result != "RTH_GUARD")
run(r, clock, 1.0, mode="GUIDED", brg_seen=True, brg_open=False)
check("мост закрыт > GUARD_SEC → RTH_GUARD", r.result == "RTH_GUARD")
r, clock, mode, log, sp, stack, land, rth = make()
run(r, clock, 1.0, mode="GUIDED")
run(r, clock, 0.1, mode="LOITER")
check("FCU сам вышел из GUIDED → RTH_EJECT", r.result == "RTH_EJECT")

# --- 6. GUIDED не залатчился / бюджет ---
r, clock, mode, log, sp, stack, land, rth = make()
run(r, clock, 4.0, mode="ALT_HOLD")
check("GUIDED не залатчился за 3 с → RTH_REFUSED", r.result == "RTH_REFUSED")
r, clock, mode, log, sp, stack, land, rth = make(speed=2.0)
run(r, clock, 0.2, x=15.0, mode="GUIDED")
check("бюджет считается от остатка пути (15 м / 2 м/с × 2.5 → минимум 60 с)",
      abs(rth._budget - 60.0) < 1e-6)
run(r, clock, 61.0, x=15.0, mode="GUIDED")
check("не дошли за бюджет → RTH_TIMEOUT", r.result == "RTH_TIMEOUT")

# --- 7. дизарм в возврате (сел сам / пилот) ---
r, clock, mode, log, sp, stack, land, rth = make()
run(r, clock, 1.0, mode="GUIDED")
run(r, clock, 0.1, mode="GUIDED", armed=False)
check("дизарм → RTH_DONE", r.finished and r.result == "RTH_DONE")

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ RTH TRACK OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
