#!/usr/bin/env python3
"""Оффлайн-тест страховки дизарма Freefly (урок lv1_replay_20260823_191230:
FCU отказал в руддер-дизарме → нода ждала дизарм бессрочно, bag 41 ГБ).

Проверяет: жест дизарма (газ min + yaw влево) НА ЗЕМЛЕ дольше 8 с → нода
дизармит сервисом cmd/arming; ещё через 4 с — force_disarm; отпущенный жест
сбрасывает таймер; в воздухе жест страховку НЕ взводит; после дизарма миссия
завершается FREEFLY_DONE. Чистый python, без ROS.

Запуск:  python3 src/mission/test/test_freefly_disarm_guard.py
"""
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_here, "..", "..", "..", "src", "control"))
sys.path.insert(0, os.path.join(_here, ".."))
sys.path.insert(0, os.path.join(_here, "..", ".."))

from control_pkg.domain.rc import RC_CENTER, RcCommand                # noqa: E402
from control_pkg.domain.state import DroneState                       # noqa: E402
from mission_pkg.plan.runner import PlanRunner                        # noqa: E402
from mission_pkg.plan.step import Freefly                             # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


class FakeStack:
    stabs = []
    def enter(self, s): pass
    def update(self, s): return RcCommand()
    def switch_stabilization(self, stabs): pass


class FakeClock:
    def __init__(self): self.t = 100.0
    def now_sim(self): return self.t


class FakeMode:
    """arm(value) считает штатные дизармы, force_disarm — принудительные."""
    def __init__(self):
        self.disarm_calls = 0
        self.force_calls = 0
    def set_mode(self, m): pass
    def arm(self, value=True):
        if not value:
            self.disarm_calls += 1
    def force_disarm(self):
        self.force_calls += 1
    def ready(self): return True


class FakeLog:
    def __init__(self): self.lines = []
    def info(self, m): self.lines.append(m)
    def warn(self, m): self.lines.append("WARN " + m)
    def error(self, m): self.lines.append("ERR " + m)


def make(clock, mode):
    step = Freefly("freefly", FakeStack())
    return PlanRunner([step], clock, mode, FakeLog())


def snap(t, armed=True, alt=0.1, thr=1100, yaw=1100, gt=None):
    return DroneState(mode="LOITER", armed=armed, rel_alt=alt, now_sim=t,
                      pilot_throttle=thr, pilot_yaw=yaw,
                      pilot_roll=RC_CENTER, pilot_pitch=RC_CENTER,
                      gt_valid=gt is not None, gt_z=gt if gt is not None else 0.0)


def tick_until(runner, clock, mode, dur, dt=0.05, **kw):
    t_end = clock.t + dur
    while clock.t < t_end:
        clock.t += dt
        runner.tick(snap(clock.t, **kw))


# --- 1. жест на земле: до 8 с тишина, после — сервис, после 12 с — force ---
clock, mode = FakeClock(), FakeMode()
r = make(clock, mode)
r.tick(snap(clock.t))                     # арм замечен (armed=True с первого тика)
tick_until(r, clock, mode, 7.5)
check("7.5 с жеста — дизарм ещё НЕ дёргали", mode.disarm_calls == 0)
tick_until(r, clock, mode, 3.0)
check("после 8 с — дизарм сервисом пошёл", mode.disarm_calls >= 1)
check("force до 12 с не трогали", mode.force_calls == 0)
tick_until(r, clock, mode, 3.0)
check("после 12 с — force_disarm пошёл", mode.force_calls >= 1)
r.tick(snap(clock.t, armed=False))        # FCU дизармился
check("после дизарма миссия FREEFLY_DONE",
      r.finished and r.result == "FREEFLY_DONE")

# --- 2. отпущенный жест (пауза > 3 с) сбрасывает таймер ---
clock, mode = FakeClock(), FakeMode()
r = make(clock, mode)
r.tick(snap(clock.t))
tick_until(r, clock, mode, 6.0)                       # 6 с жеста
tick_until(r, clock, mode, 3.5, yaw=RC_CENTER)        # отпустил yaw > 3 с
tick_until(r, clock, mode, 6.0)                       # снова 6 с жеста
check("сброс таймера: 6+6 с с разрывом >3 с — дизарм НЕ дёргали",
      mode.disarm_calls == 0 and mode.force_calls == 0)

# --- 2b. ИМПУЛЬСНЫЙ жест реплея (4с yaw-влево / 2с отпуск, joy_replay) —
#         короткие отпуски таймер НЕ сбрасывают (урок lv2_replay_20260824_040722:
#         страховка молчала все 60 с дизарм-якоря, борт простоял 9 мин) ---
clock, mode = FakeClock(), FakeMode()
r = make(clock, mode)
r.tick(snap(clock.t))
for _ in range(3):                                    # 3 цикла = 18 с
    tick_until(r, clock, mode, 4.0)                   # 4 с жеста
    tick_until(r, clock, mode, 2.0, yaw=RC_CENTER)    # 2 с отпуск
check("импульсы 4с/2с: сервис-дизарм сработал", mode.disarm_calls >= 1)
check("импульсы 4с/2с: дошло и до force", mode.force_calls >= 1)

# --- 3. жест В ВОЗДУХЕ страховку не взводит (газ min + yaw влево на снижении) ---
clock, mode = FakeClock(), FakeMode()
r = make(clock, mode)
r.tick(snap(clock.t))
tick_until(r, clock, mode, 20.0, alt=3.0, gt=3.0)
check("в воздухе (alt=3, gt=3) 20 с жеста — ничего не дёргали",
      mode.disarm_calls == 0 and mode.force_calls == 0)

# --- 3b. сценарий прогона 2 серии 2026-08-23: баро ЗАСТРЯЛ на 1.4 м, но
#         gt=0.0 (дрон на земле) — гейт по gt обязан взвести страховку ---
clock, mode = FakeClock(), FakeMode()
r = make(clock, mode)
r.tick(snap(clock.t))
tick_until(r, clock, mode, 10.0, alt=1.4, gt=0.0)
check("баро застрял (1.4), gt=0 → страховка сработала по gt",
      mode.disarm_calls >= 1)

# --- 4. rel_alt=None (нет баро) — страховка молчит, не крэш ---
clock, mode = FakeClock(), FakeMode()
r = make(clock, mode)
r.tick(snap(clock.t))
tick_until(r, clock, mode, 20.0, alt=None)
check("rel_alt=None — молчим, не крэш", mode.disarm_calls == 0)

# --- 4b. арм НЕ СЛУЧИЛСЯ (урок lv2_replay_20260824_034433: FCU отверг руддер-
#         арм по PreArm mag — 59 ГБ земли): 300 с без арма → FREEFLY_NOARM ---
clock, mode = FakeClock(), FakeMode()
r = make(clock, mode)
tick_until(r, clock, mode, 290.0, armed=False)
check("290 с без арма — миссия ещё ждёт пилота", not r.finished)
tick_until(r, clock, mode, 15.0, armed=False)
check("после 300 с без арма → FREEFLY_NOARM",
      r.finished and r.result == "FREEFLY_NOARM")

# --- 5. FCU игнорирует и сервис, и force (реплей №1: опрокинутый борт) —
#        жест дольше 30 с завершает миссию FREEFLY_STUCK (запись ограничена) ---
clock, mode = FakeClock(), FakeMode()          # FakeMode armed не меняет
r = make(clock, mode)
r.tick(snap(clock.t))
tick_until(r, clock, mode, 31.0)
check("32 с жеста при глухом FCU → FREEFLY_STUCK",
      r.finished and r.result == "FREEFLY_STUCK")
check("до сдачи пробовали и сервис, и force",
      mode.disarm_calls >= 1 and mode.force_calls >= 1)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ FREEFLY DISARM GUARD OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
