#!/usr/bin/env python3
"""Оффлайн-тест лесенки SF-мастера в Freefly (схема BS_SF_MASTER).

Проверяет автомат «потолок SC × готовность» (_ladder_*):
- потолок 0 (SC вверх): только демпфер — свапа на VinsHold НЕТ даже при зрелом VINS;
- потолок 1: демпфер → VinsHold по готовности; вниз — с гистерезисом 3×fresh
  (мигание свежести у порога стек не дёргает, настоящий протух — честный демпфер);
- потолок 2: VinsHold держит, нода шлёт LOITER; стек пустеет ТОЛЬКО после
  фактического латча режима FCU (урок LoiterHold); протух в LOITER → откат
  на ярус ниже (не в голый ALT_HOLD);
- MANUAL (SF не-вверх → pilot_switch=+1): режим ALT_HOLD; возврат из MANUAL —
  пересев опор (держим от текущей точки, а не тянем в точку до перехвата);
- потолок вниз в полёте (2→0): стек и режим возвращаются;
- LAND-failsafe FCU уважается (тир LOITER распадается, LAND не ре-ассертится).
Чистый python, без ROS.

Запуск:  python3 src/mission/test/test_freefly_ladder.py
"""
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_here, "..", "..", "..", "src", "control"))
sys.path.insert(0, os.path.join(_here, ".."))
sys.path.insert(0, os.path.join(_here, "..", ".."))

from control_pkg.application.handover import VinsHandover              # noqa: E402
from control_pkg.domain.rc import RC_CENTER, RcCommand                 # noqa: E402
from control_pkg.domain.state import DroneState                        # noqa: E402
from mission_pkg.plan.runner import PlanRunner                         # noqa: E402
from mission_pkg.plan.step import Freefly                              # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


class FakeStab:
    def __init__(self, name, axes):
        self.name = name
        self.axes = frozenset(axes)

    def enter(self, s):
        pass


class FakeVins(FakeStab):
    def __init__(self):
        super().__init__('vins', ('roll', 'pitch'))
        self.enters = 0

    def enter(self, s):
        self.enters += 1


class FakeStack:
    def __init__(self, stabs):
        self.stabs = list(stabs)
        self.enters = 0

    def enter(self, s):
        self.enters += 1

    def update(self, s):
        return RcCommand()

    def switch_stabilization(self, stabs):
        self.stabs = list(stabs)


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

    def arm(self, value=True):
        pass

    def ready(self):
        return True


class FakeLog:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    def warn(self, m):
        self.lines.append("WARN " + m)

    def error(self, m):
        self.lines.append("ERR " + m)


DAMPER = FakeStab('damper', ('roll', 'pitch'))
YAWD = FakeStab('yawd', ('yaw',))


def make(loiter_center=True):
    clock, mode = FakeClock(), FakeMode()
    vins = FakeVins()
    stack = FakeStack([DAMPER, YAWD])
    step = Freefly("freefly", stack, pilot_stabs=[DAMPER, YAWD],
                   handover=VinsHandover(vins, min_count=40, fresh_sec=2.0),
                   loiter_center=loiter_center, vins_fresh=2.0, sf_master=True)
    runner = PlanRunner([step], clock, mode, FakeLog())
    return runner, clock, mode, stack, vins


def snap(t, lvl=0, sw=-1, alt=3.0, odom=0, vins_age=None, extnav=False,
         mode="ALT_HOLD"):
    """vins_age: возраст последней одометрии, с (None = одометрии не было)."""
    return DroneState(mode=mode, armed=True, rel_alt=alt, now_sim=t,
                      pilot_switch=sw, pilot_level=lvl,
                      pilot_roll=RC_CENTER, pilot_pitch=RC_CENTER,
                      pilot_throttle=RC_CENTER, pilot_yaw=RC_CENTER,
                      vins_odom_count=odom,
                      vins_last_sim=(t - vins_age) if vins_age is not None else -1e9,
                      extnav_ready=extnav)


def tick_until(runner, clock, dur, dt=0.05, **kw):
    t_end = clock.t + dur
    s = None
    while clock.t < t_end:
        clock.t += dt
        s = snap(clock.t, **kw)
        runner.tick(s)
    return s


def names(stack):
    return [st.name for st in stack.stabs]


# --- 1. потолок 0 (SC вверх): зрелый VINS свап НЕ вызывает ---
r, clock, mode, stack, vins = make()
tick_until(r, clock, 5.0, lvl=0, odom=500, vins_age=0.1)
check("потолок 0: стек остался демпферным", names(stack) == ['damper', 'yawd'])
check("потолок 0: VinsHold не входил", vins.enters == 0)
check("потолок 0: LOITER не слался", "LOITER" not in mode.modes)

# --- 2. потолок 1: демпфер → VinsHold по готовности ---
r, clock, mode, stack, vins = make()
tick_until(r, clock, 3.0, lvl=1, odom=10, vins_age=0.1)     # мало odom
check("потолок 1 до готовности: демпфер", names(stack) == ['damper', 'yawd'])
tick_until(r, clock, 1.0, lvl=1, odom=100, vins_age=0.1)    # VINS готов
check("потолок 1: свап на VinsHold (yaw-стаб сохранён)",
      names(stack) == ['yawd', 'vins'])
check("потолок 1: VinsHold взял опору (enter)", vins.enters >= 1)

# --- 3. гистерезис вниз: age в (fresh, 3×fresh] держит, дольше — демпфер ---
tick_until(r, clock, 1.0, lvl=1, odom=100, vins_age=4.0)    # 2 < 4 ≤ 6
check("гистерезис: age=4с (< 3×fresh) VinsHold держится",
      names(stack) == ['yawd', 'vins'])
tick_until(r, clock, 1.0, lvl=1, odom=100, vins_age=7.0)    # > 3×fresh
check("протух (age=7с): откат на демпфер", names(stack) == ['damper', 'yawd'])

# --- 4. потолок 2: VinsHold держит, LOITER шлётся; стек пустеет ПОСЛЕ латча ---
r, clock, mode, stack, vins = make()
tick_until(r, clock, 2.0, lvl=2, odom=700, vins_age=0.1, extnav=True)
check("потолок 2 до латча: LOITER запрошен", "LOITER" in mode.modes)
check("потолок 2 до латча: стек ещё VinsHold (законы не бросаем)",
      names(stack) == ['yawd', 'vins'])
tick_until(r, clock, 1.0, lvl=2, odom=700, vins_age=0.1, extnav=True,
           mode="LOITER")
check("LOITER залатчен: стек пуст (стики = уставки скорости)",
      names(stack) == [])

# --- 5. VINS протух в LOITER → откат на ярус ниже (демпфер: VinsHold мёртв) ---
tick_until(r, clock, 3.0, lvl=2, odom=700, vins_age=8.0, extnav=True,
           mode="LOITER")
check("протух в LOITER: ALT_HOLD ре-ассертится", "ALT_HOLD" in mode.modes)
check("протух в LOITER: стек — демпфер (не голый ALT_HOLD)",
      names(stack) == ['damper', 'yawd'])

# --- 6. MANUAL (SF не-вверх) и возврат: пересев опор от текущей точки ---
r, clock, mode, stack, vins = make()
tick_until(r, clock, 2.0, lvl=2, odom=700, vins_age=0.1, extnav=True,
           mode="LOITER")
check("подготовка: в LOITER, стек пуст", names(stack) == [])
tick_until(r, clock, 2.0, lvl=2, sw=1, odom=700, vins_age=0.1, extnav=True,
           mode="LOITER")
check("MANUAL при LOITER: ALT_HOLD ре-ассертится (seize в «стик = наклон»)",
      "ALT_HOLD" in mode.modes)
enters_before = stack.enters
tick_until(r, clock, 1.0, lvl=2, sw=-1, odom=700, vins_age=0.1, extnav=True)
check("возврат из MANUAL: пересев опор (stack.enter)",
      stack.enters > enters_before)

# --- 7. потолок вниз в полёте (2 → 0): стек и режим возвращаются ---
r, clock, mode, stack, vins = make()
tick_until(r, clock, 2.0, lvl=2, odom=700, vins_age=0.1, extnav=True,
           mode="LOITER")
mode.modes.clear()
tick_until(r, clock, 3.0, lvl=0, odom=700, vins_age=0.1, extnav=True,
           mode="LOITER")
check("потолок 2→0: ALT_HOLD ре-ассертится", "ALT_HOLD" in mode.modes)
check("потолок 2→0: стек — демпфер", names(stack) == ['damper', 'yawd'])

# --- 8. LAND-failsafe FCU уважается: LOITER не ре-ассертится, тир распадается ---
r, clock, mode, stack, vins = make()
tick_until(r, clock, 2.0, lvl=2, odom=700, vins_age=0.1, extnav=True,
           mode="LOITER")
mode.modes.clear()
tick_until(r, clock, 3.0, lvl=2, odom=700, vins_age=0.1, extnav=True,
           mode="LAND")
check("FCU в LAND: не ре-ассертим ничего", mode.modes == [])
check("FCU в LAND: стек вернулся на VinsHold (VINS жив)",
      names(stack) == ['yawd', 'vins'])

# --- 9. без loiter_center потолок 2 ведёт себя как 1 (LOITER-механики нет) ---
r, clock, mode, stack, vins = make(loiter_center=False)
tick_until(r, clock, 3.0, lvl=2, odom=700, vins_age=0.1, extnav=True)
check("без ff_loiter: LOITER не слался", "LOITER" not in mode.modes)
check("без ff_loiter: потолок 2 живёт на VinsHold", names(stack) == ['yawd', 'vins'])

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ FREEFLY LADDER OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
