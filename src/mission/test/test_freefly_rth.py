#!/usr/bin/env python3
"""Оффлайн-тест ВОЗВРАТА ДОМОЙ ПО RTL (Freefly.rth + шаг Rth).

Проверяет:
- импульс /mission/rth в freefly → ПРЫЖОК ПО ИМЕНИ на шаг rth (FREEFLY_RTH), а
  «следующий индекс» после freefly по-прежнему посадка кнопкой SA (инвариант
  порядка плана: rth стоит ПОСЛЕ land и достижим только goto);
- в MANUAL импульс отвергается с предупреждением (Арбитр отдал бы оси сырыми);
- шаг rth: стек ПУСТ, стики/газ в центре, set_mode RTL раз в sim-секунду до
  латча; после латча повторных set_mode нет;
- дизарм после посадки RTL → RTH_DONE;
- RTL не залатчился за 3 с → RTH_REFUSED + error, борт обратно пилоту (freefly);
- повторный импульс → RTH_CANCEL (keep послан), MANUAL → RTH_MANUAL, выход FCU
  из RTL после латча → RTH_EJECT — все три возвращают в freefly;
- бюджет шага → RTH_TIMEOUT;
- SMART_RTL: режим берётся из снапшота (какой топик дёрнули) — шлём SMART_RTL,
  латч/дизарм → RTH_DONE, отказ латча → RTH_REFUSED; пустое поле = дефолт RTL;
- режимы, которых не знает MAVROS (domain/modes.py, разбор полёта 200909): имя
  уходит номером ('21'), а латч ловится по безымянному 'CMODE(21)' из /mavros/state.
Чистый python, без ROS.

Запуск:  python3 src/mission/test/test_freefly_rth.py
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
from control_pkg.domain import modes as fcu_modes                      # noqa: E402
from mission_pkg.plan.step import Freefly, Rth, SoftLand               # noqa: E402

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


class FakeStack:
    def __init__(self, stabs):
        self.stabs = list(stabs)

    def enter(self, s):
        pass

    def update(self, s):
        return RcCommand(roll=1520, pitch=1480, yaw=1510)

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
        self.arms = []

    def set_mode(self, m):
        self.modes.append(m)

    def arm(self, value=True):
        self.arms.append(value)

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

    def count(self, sub):
        return sum(1 for ln in self.lines if sub in ln)


DAMPER = FakeStab('damper', ('roll', 'pitch'))
YAWD = FakeStab('yawd', ('yaw',))
VINS = FakeStab('vins', ('roll', 'pitch'))


def make(budget=180.0, guard=False, ho=None):
    """План freefly как в mission_plan: [freefly, land, rth] — rth ПОСЛЕДНИМ."""
    clock, mode, log = FakeClock(), FakeMode(), FakeLog()
    stack = FakeStack([DAMPER, YAWD])
    pilot_stabs = [DAMPER, YAWD]
    ho = ho or VinsHandover(VINS, min_count=40, fresh_sec=2.0)
    ff = Freefly("freefly", stack, pilot_stabs=pilot_stabs, handover=ho,
                 loiter_center=True, vins_fresh=2.0, sf_master=True,
                 land_gate=(1.0, 0.3), rth=True)
    land = SoftLand("land", stack, 0.3, 45.0, pilot_stabs=pilot_stabs, handover=ho)
    rth = Rth("rth", stack, budget=budget, handover=ho, guard=guard)
    runner = PlanRunner([ff, land, rth], clock, mode, log)
    return runner, clock, mode, log, stack, ff, rth


def snap(t, alt=4.0, mode="ALT_HOLD", armed=True, sw=-1, lvl=0, rth=False, sa=False,
         rth_mode="", bridge_seen=False, bridge_open=True, vins_vx=0.0):
    """rth_mode — ЛИПКОЕ поле снапшота (нода держит его после импульса), поэтому в
    сценариях SMART_RTL его передают и на тиках после pulse. bridge_* — состояние
    моста VINS→EKF из /nn1/bridge (guard шага rth)."""
    return DroneState(mode=mode, armed=armed, rel_alt=alt, now_sim=t,
                      pilot_switch=sw, pilot_level=lvl, pilot_land=sa, pilot_rth=rth,
                      pilot_rth_mode=rth_mode,
                      bridge_seen=bridge_seen, bridge_open=bridge_open,
                      bridge_why="ext" if not bridge_open else "-",
                      vins_valid=True, vins_odom_count=300, vins_last_sim=t,
                      vins_vx=vins_vx,
                      pilot_roll=RC_CENTER, pilot_pitch=RC_CENTER,
                      pilot_throttle=RC_CENTER, pilot_yaw=RC_CENTER)


def tick_until(runner, clock, dur, dt=0.05, **kw):
    t_end = clock.t + dur
    rc = None
    while clock.t < t_end and not runner.finished:
        clock.t += dt
        rc = runner.tick(snap(clock.t, **kw))
    return rc


def pulse(runner, clock, **kw):
    """Один тик с импульсом /mission/rth|smart_rth (one-shot, как выставляет нода);
    kw прокидывается в snap — в т.ч. rth_mode для SMART_RTL."""
    clock.t += 0.05
    return runner.tick(snap(clock.t, rth=True, **kw))


def cur(runner):
    return runner.steps[runner.i].name if not runner.finished else "<fin>"


# --- 1. импульс в freefly → шаг rth; порядок плана не сломан ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
check("до импульса: шаг freefly", cur(r) == "freefly")
pulse(r, clock)
check("импульс /mission/rth → шаг rth, результат FREEFLY_RTH",
      cur(r) == "rth" and r.result == "FREEFLY_RTH")
tick_until(r, clock, 0.2, mode="ALT_HOLD")
check("шаг rth: стек ПУСТ (позицию ведёт FCU)", stack.stabs == [])
rc = tick_until(r, clock, 0.2, mode="ALT_HOLD")
check("шаг rth: стики и газ в центре",
      (rc.roll, rc.pitch, rc.yaw, rc.throttle) == (RC_CENTER,) * 4)
check("шаг rth: послан RTL", "RTL" in mode.modes)
n_rtl = mode.modes.count("RTL")
tick_until(r, clock, 3.0, mode="RTL")
check("RTL залатчен: повторных set_mode нет, лог о латче",
      mode.modes.count("RTL") == n_rtl and log.count("RTL залатчен") == 1)
tick_until(r, clock, 0.2, mode="RTL", armed=False)
check("дизарм после посадки RTL → RTH_DONE", r.finished and r.result == "RTH_DONE")

# --- 2. инвариант порядка: кнопка SA из freefly по-прежнему ведёт на land ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, sa=True)
check("SA из freefly → шаг land (rth стоит после и достижим только goto)",
      cur(r) == "land" and r.result == "FREEFLY_LAND")

# --- 3. MANUAL: импульс отвергается ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0, sw=1)
pulse(r, clock, sw=1)
check("импульс в MANUAL: остаёмся в freefly, одно предупреждение, RTL не послан",
      cur(r) == "freefly" and log.count("RTH: отказ — MANUAL") == 1
      and "RTL" not in mode.modes)

# --- 4. RTL не латчится за 3 с → RTH_REFUSED, борт пилоту ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
pulse(r, clock)
tick_until(r, clock, 4.0, mode="ALT_HOLD")
check("RTL не залатчился за 3 с → RTH_REFUSED + error, возврат в freefly",
      cur(r) == "freefly" and r.result == "RTH_REFUSED"
      and log.count("ERR") >= 1 and log.count("не залатчился") == 1)
check("повторные попытки не чаще 1/с (try_cmd): за 3 с не больше 4 set_mode RTL",
      mode.modes.count("RTL") <= 4)

# --- 5. повторный импульс = отмена ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
pulse(r, clock)
tick_until(r, clock, 2.0, mode="RTL")
n0 = len(mode.modes)
pulse(r, clock, mode="RTL")
check("повторный импульс → RTH_CANCEL, keep послан, шаг freefly",
      cur(r) == "freefly" and r.result == "RTH_CANCEL"
      and "ALT_HOLD" in mode.modes[n0:])

# --- 6. пилот забрал борт в MANUAL из шага rth ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
pulse(r, clock)
tick_until(r, clock, 2.0, mode="RTL")
tick_until(r, clock, 0.2, mode="RTL", sw=1)
check("MANUAL из шага rth → RTH_MANUAL, шаг freefly",
      cur(r) == "freefly" and r.result == "RTH_MANUAL")

# --- 7. FCU сам вышел из RTL после латча ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
pulse(r, clock)
tick_until(r, clock, 2.0, mode="RTL")
tick_until(r, clock, 0.2, mode="LAND")
check("FCU вышел из RTL → RTH_EJECT, шаг freefly",
      cur(r) == "freefly" and r.result == "RTH_EJECT")

# --- 8. бюджет шага ---
r, clock, mode, log, stack, ff, rth = make(budget=20.0)
tick_until(r, clock, 1.0)
pulse(r, clock)
tick_until(r, clock, 25.0, mode="RTL")
check("не сели за бюджет → RTH_TIMEOUT (борт в воздухе, сажает пилот)",
      r.finished and r.result == "RTH_TIMEOUT")

# --- 9. SMART_RTL: режим приходит со снапшотом (топик /mission/smart_rth) ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
pulse(r, clock, rth_mode="SMART_RTL")
check("импульс smart_rth → шаг rth (тот же)", cur(r) == "rth")
tick_until(r, clock, 0.5, mode="ALT_HOLD", rth_mode="SMART_RTL")
check("шлём SMART_RTL, а не RTL",
      "SMART_RTL" in mode.modes and "RTL" not in mode.modes)
check("лог называет режим", log.count("SMART_RTL полётника") == 1)
tick_until(r, clock, 2.0, mode="SMART_RTL", rth_mode="SMART_RTL")
check("латч SMART_RTL", log.count("SMART_RTL залатчен") == 1)
tick_until(r, clock, 0.2, mode="SMART_RTL", armed=False, rth_mode="SMART_RTL")
check("дизарм после SmartRTL → RTH_DONE", r.finished and r.result == "RTH_DONE")

# --- 10. SMART_RTL не залатчился (пустой буфер крошек) → борт пилоту ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
pulse(r, clock, rth_mode="SMART_RTL")
tick_until(r, clock, 4.0, mode="ALT_HOLD", rth_mode="SMART_RTL")
check("SMART_RTL не залатчился за 3 с → RTH_REFUSED, шаг freefly",
      cur(r) == "freefly" and r.result == "RTH_REFUSED")

# --- 11. пустой режим в снапшоте = дефолт шага (RTL) ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
pulse(r, clock)
tick_until(r, clock, 0.5)
check("без режима в снапшоте шлём дефолтный RTL",
      "RTL" in mode.modes and "SMART_RTL" not in mode.modes)

# --- 12. режимы, которых не знает MAVROS (разбор полёта 200909) ---
check("modes.to_fcu: SMART_RTL уходит номером '21' (иначе MAVROS съедает запрос)",
      fcu_modes.to_fcu('SMART_RTL') == '21')
check("modes.to_fcu: известные имена не трогаем",
      (fcu_modes.to_fcu('RTL'), fcu_modes.to_fcu('LOITER')) == ('RTL', 'LOITER'))
check("modes.matches: 'CMODE(21)' из /mavros/state = SMART_RTL",
      fcu_modes.matches('CMODE(21)', 'SMART_RTL')
      and fcu_modes.matches('SMART_RTL', 'SMART_RTL')
      and not fcu_modes.matches('CMODE(21)', 'RTL'))

# --- 13. латч SMART_RTL по безымянному CMODE(21) (как отдаёт MAVROS) ---
r, clock, mode, log, stack, ff, rth = make()
tick_until(r, clock, 1.0)
pulse(r, clock, rth_mode="SMART_RTL")
tick_until(r, clock, 2.5, mode="CMODE(21)", rth_mode="SMART_RTL")
check("MAVROS отдаёт CMODE(21) — шаг считает режим залатченным, а не отказывает",
      cur(r) == "rth" and r.result != "RTH_REFUSED"
      and log.count("SMART_RTL залатчен") == 1)
tick_until(r, clock, 0.2, mode="CMODE(21)", armed=False, rth_mode="SMART_RTL")
check("дизарм в CMODE(21) → RTH_DONE", r.finished and r.result == "RTH_DONE")

# --- 14. GUARD ЗДОРОВЬЯ: мост VINS→EKF закрылся посреди возврата (разбор 044105) ---
r, clock, mode, log, stack, ff, rth = make(guard=True)
tick_until(r, clock, 1.0)
pulse(r, clock, rth_mode="SMART_RTL")
tick_until(r, clock, 3.0, mode="CMODE(21)", rth_mode="SMART_RTL")
check("guard: возврат идёт, мост открыт — шаг rth", cur(r) == "rth")
# мигок вердикта короче GUARD_SEC возврат НЕ рвёт
tick_until(r, clock, 0.5, mode="CMODE(21)", rth_mode="SMART_RTL",
           bridge_seen=True, bridge_open=False)
check("guard: мост закрыт 0.5 с (< GUARD_SEC 1 с) — возврат продолжается",
      cur(r) == "rth" and r.result != "RTH_GUARD")
tick_until(r, clock, 1.0, mode="CMODE(21)", rth_mode="SMART_RTL", bridge_seen=True)
check("guard: мост открылся обратно — таймер сброшен, возврат идёт", cur(r) == "rth")
# закрыт дольше GUARD_SEC → отмена
tick_until(r, clock, 1.2, mode="CMODE(21)", rth_mode="SMART_RTL",
           bridge_seen=True, bridge_open=False)
check("guard: мост закрыт > GUARD_SEC → RTH_GUARD, борт пилоту в freefly",
      cur(r) == "freefly" and r.result == "RTH_GUARD")
check("guard: послан keep (ALT_HOLD), а не оставлен режим FCU",
      mode.modes[-1] == "ALT_HOLD")
check("guard: причина в логе — закрытый мост",
      log.count("МОСТ VINS→EKF ЗАКРЫТ") == 1)

# --- 15. guard по вердикту гейта здоровья (разнос VINS) ---
ho = VinsHandover(VINS, min_count=40, fresh_sec=2.0, v_max=12.0)
r, clock, mode, log, stack, ff, rth = make(guard=True, ho=ho)
tick_until(r, clock, 1.0)
pulse(r, clock)
tick_until(r, clock, 2.0, mode="RTL")
check("guard: здоровый VINS — возврат идёт", cur(r) == "rth")
tick_until(r, clock, 1.2, mode="RTL", vins_vx=20.0)     # |v| 20 > потолок 12
check("guard: гейт здоровья (|v|=20) → RTH_GUARD",
      cur(r) == "freefly" and r.result == "RTH_GUARD"
      and log.count("VINS РАЗНЁССЯ") == 1)

# --- 16. guard выключен (BS_RTH_GUARD=0) — старое поведение ---
r, clock, mode, log, stack, ff, rth = make(guard=False)
tick_until(r, clock, 1.0)
pulse(r, clock, rth_mode="SMART_RTL")
tick_until(r, clock, 5.0, mode="CMODE(21)", rth_mode="SMART_RTL",
           bridge_seen=True, bridge_open=False)
check("guard выкл: закрытый мост возврат не рвёт (как до 2026-09-09)",
      cur(r) == "rth" and r.result != "RTH_GUARD")

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ FREEFLY RTH OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
