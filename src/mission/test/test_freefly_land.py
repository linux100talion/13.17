#!/usr/bin/env python3
"""Оффлайн-тест МЯГКОЙ ПОСАДКИ ПО КНОПКЕ SA (Freefly.land_gate + SoftLand).

Проверяет:
- фронт кнопки через гейт «низко и почти стоим»: пускает при rel_alt ≤ alt_max и
  |v| ≤ v_max (источник IPM → VINS → gt), отказывает по высоте / скорости / MANUAL
  с одним предупреждением на нажатие; зажатая кнопка — один фронт; без источника
  скорости — пускает с предупреждением; one-shot /mission/land (уровень на один
  тик) — тот же фронт;
- ветка pos (FCU в LOITER): LAND шлётся, стек ПУСТ, касание → газ в пол,
  самодизарм FCU → LAND_DONE; LAND не залатчился за 3 с / VINS протух → ветка alt;
- ветка alt (ALT_HOLD, «сесть до VINS»): LAND НЕ шлётся, газ снижения = центр −
  dz − rate/rate_full·span (в тесте rate 0.3 явно → 1362; дефолт конфига 0.15 →
  1381, это проверяет test_mission_plan), стек = демпфер (VinsHold при
  готовности, вниз при протухании), опора пересеяна; касание (баро | gt |
  детектор FCU) → газ в пол, дизарм сервисом через 1 с, force через 5 с;
  дизарм → LAND_DONE; бюджет → LAND_TIMEOUT; 30 с без дизарма → LAND_STUCK.
Чистый python, без ROS.

Запуск:  python3 src/mission/test/test_freefly_land.py
"""
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_here, "..", "..", "..", "src", "control"))
sys.path.insert(0, os.path.join(_here, ".."))
sys.path.insert(0, os.path.join(_here, "..", ".."))

from control_pkg.application.handover import VinsHandover              # noqa: E402
from control_pkg.domain.rc import RC_CENTER, RC_MIN_THR, RcCommand     # noqa: E402
from control_pkg.domain.state import DroneState                        # noqa: E402
from mission_pkg.plan.runner import PlanRunner                         # noqa: E402
from mission_pkg.plan.step import Freefly, SoftLand, ground_speed      # noqa: E402

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
        self.enters = 0

    def enter(self, s):
        self.enters += 1

    def update(self, s):
        return RcCommand(roll=1520, pitch=1480, yaw=1500)

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
        self.forces = 0

    def set_mode(self, m):
        self.modes.append(m)

    def arm(self, value=True):
        self.arms.append(value)

    def force_disarm(self):
        self.forces += 1

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


def make(budget=45.0, land_in_loiter=True):
    clock, mode, log = FakeClock(), FakeMode(), FakeLog()
    stack = FakeStack([DAMPER, YAWD])
    pilot_stabs = [DAMPER, YAWD]
    ho = VinsHandover(VINS, min_count=40, fresh_sec=2.0)
    ff = Freefly("freefly", stack, pilot_stabs=pilot_stabs, handover=ho,
                 loiter_center=True, vins_fresh=2.0, sf_master=True,
                 land_gate=(1.0, 0.3), land_in_loiter=land_in_loiter)
    land = SoftLand("land", stack, 0.3, budget, pilot_stabs=pilot_stabs,
                    handover=ho, rate=0.3, fresh_sec=2.0)
    runner = PlanRunner([ff, land], clock, mode, log)
    return runner, clock, mode, log, stack, ff, land


def snap(t, alt=0.8, sa=False, sw=-1, lvl=0, mode="ALT_HOLD", armed=True,
         ipm=None, vins=None, gt=None, odom=0, vins_age=None, extnav=False,
         fcu_landed=-1):
    """ipm/vins/gt — (vx, vy) источника скорости; None = источника нет."""
    s = DroneState(mode=mode, armed=armed, rel_alt=alt, now_sim=t,
                   pilot_switch=sw, pilot_level=lvl, pilot_land=sa,
                   pilot_roll=RC_CENTER, pilot_pitch=RC_CENTER,
                   pilot_throttle=RC_CENTER, pilot_yaw=RC_CENTER,
                   vins_odom_count=odom,
                   vins_last_sim=(t - vins_age) if vins_age is not None else -1e9,
                   extnav_ready=extnav, fcu_landed=fcu_landed, fcu_landed_sim=t)
    if ipm is not None:
        s.ipm_ok, s.ipm_vfwd, s.ipm_vlat = True, ipm[0], ipm[1]
    if vins is not None:
        s.vins_valid, s.vins_vx, s.vins_vy = True, vins[0], vins[1]
        if vins_age is None:
            s.vins_last_sim = t - 0.1
    if gt is not None:
        s.gt_valid, s.gt_vx, s.gt_vy, s.gt_z = True, gt[0], gt[1], alt
    return s


def tick_until(runner, clock, dur, dt=0.05, **kw):
    t_end = clock.t + dur
    rc, s = None, None
    while clock.t < t_end and not runner.finished:
        clock.t += dt
        s = snap(clock.t, **kw)
        rc = runner.tick(s)
    return rc, s


def cur(runner):
    return runner.steps[runner.i].name if not runner.finished else "<fin>"


def names(stack):
    return [st.name for st in stack.stabs]


# --- 0. ground_speed: приоритет источников ---
s = snap(0.0, ipm=(0.3, 0.4), vins=(1.0, 0.0), gt=(2.0, 0.0))
check("ground_speed: IPM первым (0.5 м/с)", ground_speed(s, 2.0) == (0.5, 'ipm'))
s = snap(0.0, vins=(0.6, 0.8), gt=(2.0, 0.0))
check("ground_speed: без IPM — свежий VINS (1.0)", ground_speed(s, 2.0) == (1.0, 'vins'))
s = snap(0.0, vins=(0.6, 0.8), gt=(2.0, 0.0), vins_age=5.0)
check("ground_speed: VINS протух → gt", ground_speed(s, 2.0) == (2.0, 'gt'))
check("ground_speed: источников нет → (None, None)",
      ground_speed(snap(0.0), 2.0) == (None, None))

# --- 1. кнопка через гейт: низко и стоим → FREEFLY_LAND → шаг land ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)                              # арм → стек с этой точки
check("до кнопки: шаг freefly", cur(r) == "freefly")
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.1, 0.1), sa=True)
check("кнопка при 0.8 м, |v|=0.14 (ipm): переход на land, результат FREEFLY_LAND",
      cur(r) == "land" and r.result == "FREEFLY_LAND")
check("лог: «SA: ПОСАДКА» с источником ipm",
      log.count("SA: ПОСАДКА") == 1 and any("(ipm)" in ln for ln in log.lines))

# --- 2. гейт закрыт по высоте: остаёмся, одно предупреждение на нажатие ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 1.0, alt=2.3, ipm=(0.0, 0.0), sa=True)   # зажата 1 с = 20 тиков
check("2.3 м > 1.0: остаёмся в freefly", cur(r) == "freefly")
check("зажатая кнопка — ОДИН фронт → одно предупреждение (не 20)",
      log.count("SA: гейт закрыт") == 1)
tick_until(r, clock, 0.3, alt=0.7, ipm=(0.0, 0.0), sa=False)  # отпустили, снизились
tick_until(r, clock, 0.3, alt=0.7, ipm=(0.0, 0.0), sa=True)   # нажали снова
check("отпустил → снизился → нажал: пускает", cur(r) == "land")

# --- 3. гейт закрыт по скорости (VINS как источник) ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, vins=(0.5, 0.0), sa=True)
check("|v|=0.5 (vins) > 0.3: отказ с причиной (vins)",
      cur(r) == "freefly" and any("0.50 > 0.3" in ln and "(vins)" in ln
                                  for ln in log.lines))

# --- 4. MANUAL (SF не вверх): отказ ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.0, 0.0), sa=True, sw=1)
check("MANUAL: кнопка отвергнута с подсказкой про SF",
      cur(r) == "freefly" and log.count("SA: MANUAL") == 1)

# --- 5. скорости судить нечем → пускаем с предупреждением ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, sa=True)
check("без IPM/VINS/gt: пускает, в логе «неизвестна»",
      cur(r) == "land" and any("неизвестна" in ln for ln in log.lines))

# --- 6. one-shot /mission/land: уровень на ОДИН тик — тот же фронт ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
clock.t += 0.05
r.tick(snap(clock.t, alt=0.5, gt=(0.0, 0.0), sa=True))     # один тик с кнопкой
tick_until(r, clock, 0.2, alt=0.5, gt=(0.0, 0.0), sa=False)
check("импульс на один тик: посадка запущена (источник gt)", cur(r) == "land")

# --- 7. ветка pos: FCU в LOITER → LAND, стек пуст ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 3.0, alt=3.0, odom=700, vins_age=0.1, extnav=True, lvl=2,
           mode="LOITER")                                   # ярус 2 залатчен
check("подготовка: ярус 2, стек = yaw-стаб",
      names(stack) == ['yawd'] and ff._tier == 2)
tick_until(r, clock, 0.2, alt=0.9, ipm=(0.0, 0.0), odom=700, vins_age=0.1,
           extnav=True, lvl=2, mode="LOITER", sa=True)
check("кнопка в LOITER: шаг land", cur(r) == "land")
n_modes = len(mode.modes)
rc, _ = tick_until(r, clock, 0.3, alt=0.9, odom=700, vins_age=0.1, extnav=True,
                   lvl=2, mode="LOITER")
# LAND уходит первым же тиком шага (в том же окне, что и нажатие) — Freefly сам
# LAND не шлёт (только LOITER/ALT_HOLD), поэтому «LAND в списке» = SoftLand
check("ветка pos: LAND послан, стек пуст, land_state=pos",
      "LAND" in mode.modes and names(stack) == [] and land.land_state() == "pos")
check("ветка pos: стики центр, газ центр (в LAND игнорируется)",
      (rc.roll, rc.pitch, rc.yaw, rc.throttle) == (RC_CENTER,) * 4)
tick_until(r, clock, 2.0, alt=0.5, odom=700, vins_age=0.1, extnav=True, lvl=2,
           mode="LAND")
check("LAND залатчен: ветка pos держится (в alt не ушли)",
      land.land_state() == "pos" and "ALT_HOLD" not in mode.modes[n_modes:])
rc, _ = tick_until(r, clock, 0.5, alt=0.2, odom=700, vins_age=0.1, extnav=True,
                   lvl=2, mode="LAND")
check("касание по баро (0.2 ≤ 0.3): land_state=touch, газ в пол",
      land.land_state() == "touch" and rc.throttle == RC_MIN_THR)
check("ветка pos сразу после касания: дизарм сервисом НЕ шлём (LAND сам)",
      mode.arms == [])
tick_until(r, clock, 0.5, alt=0.2, odom=700, vins_age=0.1, extnav=True, lvl=2,
           mode="LAND", armed=False)
check("FCU дизармил → LAND_DONE, миссия завершена",
      r.finished and r.result == "LAND_DONE")

# --- 8. ветка pos: LAND не залатчился за 3 с → ветка alt под стеком ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 3.0, alt=3.0, odom=700, vins_age=0.1, extnav=True, lvl=2,
           mode="LOITER")
tick_until(r, clock, 0.2, alt=0.9, ipm=(0.0, 0.0), odom=700, vins_age=0.1,
           extnav=True, lvl=2, mode="LOITER", sa=True)
rc, _ = tick_until(r, clock, 4.0, alt=0.9, odom=700, vins_age=0.1, extnav=True,
                   lvl=2, mode="LOITER")                    # FCU так и не в LAND
check("LAND не латчится 3 с: ветка alt, VinsHold (VINS готов), газ снижения 1362",
      land.land_state() == "vinshold" and names(stack) == ['yawd', 'vins']
      and rc.throttle == 1362)
check("ветка alt: ре-ассерт ALT_HOLD пошёл", "ALT_HOLD" in mode.modes)

# --- 9. ветка alt с нуля («сесть до VINS»): демпфер, опора пересеяна ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
enters0 = stack.enters
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.05, 0.0), sa=True)
n_modes = len(mode.modes)
rc, _ = tick_until(r, clock, 0.5, alt=0.8, ipm=(0.05, 0.0))
check("ветка alt: LAND НЕ послан, land_state=damper, стек демпфер",
      "LAND" not in mode.modes and land.land_state() == "damper"
      and names(stack) == ['damper', 'yawd'])
check("ветка alt: газ 1362, крен/тангаж от стека (стик = наклон)",
      (rc.throttle, rc.roll, rc.pitch) == (1362, 1520, 1480))
check("ветка alt: опора пересеяна на входе (stack.enter)", stack.enters > enters0)
# касание по детектору FCU (баро «застрял» на 1.4 м — урок 2026-08-23)
rc, _ = tick_until(r, clock, 0.3, alt=1.4, ipm=(0.05, 0.0), fcu_landed=1)
check("касание по детектору FCU при баро 1.4 м: touch, газ в пол, стики центр",
      land.land_state() == "touch" and rc.throttle == RC_MIN_THR
      and rc.roll == RC_CENTER)
tick_until(r, clock, 1.5, alt=1.4, fcu_landed=1)
check("ветка alt: через 1 с после касания — дизарм сервисом", False in mode.arms)
check("force ещё рано (< 5 с)", mode.forces == 0)
tick_until(r, clock, 4.0, alt=1.4, fcu_landed=1)
check("5 с без дизарма → force_disarm + предупреждение",
      mode.forces >= 1 and log.count("force") >= 1)
tick_until(r, clock, 0.3, alt=1.4, fcu_landed=1, armed=False)
check("дизарм → LAND_DONE", r.finished and r.result == "LAND_DONE")

# --- 10. ветка alt: VinsHold → протух → демпфер ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.0, 0.0), odom=100, vins_age=0.1,
           sa=True)
tick_until(r, clock, 0.3, alt=0.8, odom=100, vins_age=0.1)
check("VINS готов (100 ≥ 40, свежий): ветка alt под VinsHold",
      land.land_state() == "vinshold" and names(stack) == ['yawd', 'vins'])
tick_until(r, clock, 0.3, alt=0.8, odom=100, vins_age=7.0)
check("протух > 3×fresh: ярус ДЕМПФЕР",
      land.land_state() == "damper" and names(stack) == ['damper', 'yawd'])

# --- 11. ветка pos: VINS протух в LAND → ветка alt (семантика стиков неизвестна) ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 3.0, alt=3.0, odom=700, vins_age=0.1, extnav=True, lvl=2,
           mode="LOITER")
tick_until(r, clock, 0.2, alt=0.9, ipm=(0.0, 0.0), odom=700, vins_age=0.1,
           extnav=True, lvl=2, mode="LOITER", sa=True)
tick_until(r, clock, 1.0, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2,
           mode="LAND")
check("подготовка: pos залатчен", land.land_state() == "pos")
tick_until(r, clock, 0.5, alt=0.9, odom=700, vins_age=8.0, extnav=True, lvl=2,
           mode="LAND")
check("VINS протух в LAND → ветка alt под демпфером, ALT_HOLD ре-ассерт",
      land.land_state() == "damper" and "ALT_HOLD" in mode.modes)

# --- 12. бюджет: касания нет → LAND_TIMEOUT (error) ---
r, clock, mode, log, stack, ff, land = make(budget=5.0)
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.0, 0.0), sa=True)
tick_until(r, clock, 6.0, alt=0.8, ipm=(0.0, 0.0))
check("бюджет 5 с без касания → LAND_TIMEOUT, error в логе",
      r.finished and r.result == "LAND_TIMEOUT" and log.count("ERR") >= 1)

# --- 13. касание без дизарма 30 с → LAND_STUCK ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.0, 0.0), sa=True)
tick_until(r, clock, 31.0, alt=0.1)
check("30 с после касания заармлен → LAND_STUCK (громкий error)",
      r.finished and r.result == "LAND_STUCK"
      and any("ЗААРМЛЕННЫМ" in ln for ln in log.lines))

# --- 14. без land_gate (ff_land=0) кнопка игнорируется ---
clock, mode, log = FakeClock(), FakeMode(), FakeLog()
stack = FakeStack([DAMPER, YAWD])
ff0 = Freefly("freefly", stack, pilot_stabs=[DAMPER, YAWD], sf_master=True)
r0 = PlanRunner([ff0], clock, mode, log)
tick_until(r0, clock, 1.0)
tick_until(r0, clock, 0.5, alt=0.5, ipm=(0.0, 0.0), sa=True)
check("ff_land=0: кнопка не делает ничего", cur(r0) == "freefly"
      and log.count("SA:") == 0)

# --- 10. ОТМЕНА повторным нажатием, ветка alt: назад в freefly, стек демпфер ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.05, 0.0), sa=True)           # нажали, зажали
tick_until(r, clock, 0.5, alt=0.8, ipm=(0.05, 0.0), sa=True)           # ещё держим — не отмена
check("отмена: зажатая с входа кнопка — не отмена (шаг land)", cur(r) == "land")
tick_until(r, clock, 0.3, alt=0.8, ipm=(0.05, 0.0), sa=False)          # отпустили
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.05, 0.0), sa=True)           # второй фронт
check("отмена (ветка alt): второй фронт → шаг freefly, результат LAND_CANCEL",
      cur(r) == "freefly" and r.result == "LAND_CANCEL")
rc, _ = tick_until(r, clock, 0.5, alt=0.8, ipm=(0.05, 0.0), sa=True)
check("после отмены: стек демпфер, газ = стик пилота (не 1362), «возврат» в логе",
      names(stack) == ['damper', 'yawd'] and rc.throttle == RC_CENTER
      and any("возврат в свободный полёт" in ln for ln in log.lines))
tick_until(r, clock, 0.3, alt=0.8, ipm=(0.05, 0.0), sa=False)
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.05, 0.0), sa=True)           # третий фронт — снова посадка
check("третье нажатие — снова посадка (шаг land)", cur(r) == "land")

# --- 11. ОТМЕНА в ветке pos (LOITER → LAND): keep послан сразу, стек перепринят ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 3.0, alt=3.0, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LOITER")
tick_until(r, clock, 0.2, alt=0.9, ipm=(0.0, 0.0), odom=700, vins_age=0.1, extnav=True,
           lvl=2, mode="LOITER", sa=True)
tick_until(r, clock, 1.0, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LAND", sa=False)
check("подготовка: ветка pos, LAND залатчен, стек пуст", land.land_state() == "pos" and names(stack) == [])
n_modes = len(mode.modes)
tick_until(r, clock, 0.2, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LAND", sa=True)
check("отмена (ветка pos): ALT_HOLD послан сразу, шаг freefly, LAND_CANCEL",
      "ALT_HOLD" in mode.modes[n_modes:] and cur(r) == "freefly" and r.result == "LAND_CANCEL")
tick_until(r, clock, 0.5, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="ALT_HOLD", sa=False)
check("после отмены из LAND: стек перепринят лесенкой (не пуст)", names(stack) != [])

# --- 12. после касания отмены нет; cancel=False — отмена выключена ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.05, 0.0), sa=True)
tick_until(r, clock, 0.3, alt=0.2, ipm=(0.05, 0.0), sa=False)          # касание по баро
check("касание: land_state=touch", land.land_state() == "touch")
tick_until(r, clock, 0.2, alt=0.2, sa=True)
check("SA после касания: остаёмся в land, предупреждение",
      cur(r) == "land" and any("ПОСЛЕ касания" in ln for ln in log.lines))
r, clock, mode, log, stack, ff, land = make()
land.cancel = False
tick_until(r, clock, 1.0)
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.05, 0.0), sa=True)
tick_until(r, clock, 0.3, alt=0.8, ipm=(0.05, 0.0), sa=False)
tick_until(r, clock, 0.2, alt=0.8, ipm=(0.05, 0.0), sa=True)
check("cancel=False: второе нажатие не отменяет (шаг land), предупреждение",
      cur(r) == "land" and any("отмена выключена" in ln for ln in log.lines))

# --- 13. отмена из LAND, FCU медлит: keep ре-ассертится, стек молчит, потом всё штатно ---
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 3.0, alt=3.0, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LOITER")
tick_until(r, clock, 0.2, alt=0.9, ipm=(0.0, 0.0), odom=700, vins_age=0.1, extnav=True,
           lvl=2, mode="LOITER", sa=True)
tick_until(r, clock, 1.0, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LAND", sa=False)
n0 = len(mode.modes)
tick_until(r, clock, 0.2, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LAND", sa=True)
rc, _ = tick_until(r, clock, 2.5, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LAND", sa=False)
asserts = [m for m in mode.modes[n0:] if m == "ALT_HOLD"]
check(f"FCU медлит в LAND 2.5 с: ALT_HOLD ре-ассертится ({len(asserts)} раз ≥ 2), шаг freefly",
      len(asserts) >= 2 and cur(r) == "freefly")
check("пока FCU в LAND: стики в центре (стек молчит — в position-LAND стик = скорость)",
      (rc.roll, rc.pitch) == (RC_CENTER, RC_CENTER))
rc, _ = tick_until(r, clock, 0.5, alt=0.9, ipm=(0.05, 0.0), odom=700, vins_age=0.1, extnav=True,
                   lvl=2, mode="ALT_HOLD", sa=False)
check("FCU вышел в ALT_HOLD: стек снова рулит", rc.roll != RC_CENTER or rc.pitch != RC_CENTER)
r, clock, mode, log, stack, ff, land = make()
tick_until(r, clock, 1.0)
tick_until(r, clock, 3.0, alt=3.0, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LOITER")
tick_until(r, clock, 0.2, alt=0.9, ipm=(0.0, 0.0), odom=700, vins_age=0.1, extnav=True,
           lvl=2, mode="LOITER", sa=True)
tick_until(r, clock, 1.0, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LAND", sa=False)
tick_until(r, clock, 0.2, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LAND", sa=True)
tick_until(r, clock, 6.0, alt=0.9, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LAND", sa=False)
check("FCU не вышел из LAND за 5 с: один error в логе, дальше LAND уважаем",
      sum(1 for ln in log.lines if "не вышел из LAND" in ln) == 1 and cur(r) == "freefly")

# --- 14. land_in_loiter=0 (дефолт ноды): на ярусе LOITER кнопка отвергается ---
r, clock, mode, log, stack, ff, land = make(land_in_loiter=False)
tick_until(r, clock, 1.0)
tick_until(r, clock, 3.0, alt=3.0, odom=700, vins_age=0.1, extnav=True, lvl=2, mode="LOITER")
check("подготовка: ярус 2", ff._tier == 2)
tick_until(r, clock, 0.2, alt=0.9, ipm=(0.0, 0.0), odom=700, vins_age=0.1, extnav=True,
           lvl=2, mode="LOITER", sa=True)
check("SA на ярусе LOITER: отказ с подсказкой, шаг freefly, LAND не послан",
      cur(r) == "freefly" and any("ярус LOITER — посадка кнопкой ОТКЛЮЧЕНА" in ln for ln in log.lines)
      and "LAND" not in mode.modes)
tick_until(r, clock, 0.3, alt=0.9, ipm=(0.0, 0.0), odom=700, vins_age=0.1, extnav=True,
           lvl=1, mode="ALT_HOLD", sa=False)                       # CH6 вниз → ярус 1
tick_until(r, clock, 0.2, alt=0.9, ipm=(0.0, 0.0), odom=700, vins_age=0.1, extnav=True,
           lvl=1, mode="ALT_HOLD", sa=True)
check("CH6 вниз → ярус 1 → SA: посадка (ветка alt под VinsHold)",
      cur(r) == "land" and land.land_state() in ("vinshold", "damper"))

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ FREEFLY SOFT-LAND OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
