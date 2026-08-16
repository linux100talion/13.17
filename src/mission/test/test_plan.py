#!/usr/bin/env python3
"""Юнит-тест механики PlanRunner (чистый python): NEXT/GOTO/FINISH, именованные прыжки,
ленивое базирование elapsed(), троттлинг try_cmd. Dummy-шаги, без домена управления.

Запуск:  python3 src/mission/test/test_plan.py
"""
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_here, "..", "..", "..", "src", "control"))
sys.path.insert(0, os.path.join(_here, ".."))
sys.path.insert(0, os.path.join(_here, "..", ".."))

from control_pkg.domain.rc import RcCommand                            # noqa: E402
from mission_pkg.plan.runner import PlanRunner                         # noqa: E402
from mission_pkg.plan.step import Step, _finish, _goto, _next, _run    # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


class Clock:
    def __init__(self): self.t = 0.0
    def now_sim(self): return self.t


class Mode:
    def __init__(self): self.calls = []
    def set_mode(self, m): self.calls.append(m)
    def arm(self): self.calls.append("arm")
    def ready(self): return True


class Log:
    def info(self, m): pass
    def warn(self, m): pass
    def error(self, m): pass


class S:
    """Минимальный dummy-снапшот (лог финиша PlanRunner читает эти поля)."""
    mode = None; armed = False; rel_alt = None; vins_odom_count = 0


class Once(Step):
    """Один тик → NEXT (отмечает, что тикнул)."""
    def __init__(self, name, log): self.name = name; self._log = log
    def tick(self, ctx, s):
        self._log.append(self.name)
        return _next(RcCommand(), result=self.name)


class AbortTo(Step):
    """Сразу GOTO по имени с меткой result."""
    def __init__(self, name, target, res): self.name = name; self.t = target; self.r = res
    def tick(self, ctx, s):
        return _goto(RcCommand(), self.t, result=self.r)


class End(Step):
    """Финиш БЕЗ метки result (как реальный Land — не перетирает исход миссии)."""
    def __init__(self, name, log): self.name = name; self._log = log
    def tick(self, ctx, s):
        self._log.append(self.name)
        return _finish(RcCommand())


class Cmd(Step):
    """Зовёт mode.set_mode каждый разрешённый тик (проверка троттлинга), N тиков → NEXT."""
    def __init__(self, name, n): self.name = name; self.n = n; self._k = 0
    def tick(self, ctx, s):
        ctx.try_cmd(lambda: ctx.mode.set_mode("X"))
        self._k += 1
        return _next(RcCommand()) if self._k >= self.n else _run(RcCommand())


# --- 1. Последовательность NEXT проходит все шаги и финиширует ---
clk = Clock(); md = Mode(); log = []
r = PlanRunner([Once("a", log), Once("b", log), Once("c", log)], clk, md, Log())
for _ in range(10):
    if r.finished: break
    clk.t += 0.05
    r.tick(S())
check("NEXT: все шаги пройдены по порядку", log == ["a", "b", "c"])
check("NEXT: финиш после последнего", r.finished)
check("NEXT: result = последний исход", r.result == "c")

# --- 2. GOTO по имени прыгает на нужный шаг ---
clk = Clock(); log = []
r = PlanRunner([AbortTo("s0", "land", "CLIMB_FAIL"),
                Once("mid", log), End("land", log)], clk, Mode(), Log())
for _ in range(10):
    if r.finished: break
    clk.t += 0.05
    r.tick(S())
check("GOTO: прыжок через 'mid' сразу на 'land'", log == ["land"])
check("GOTO: result сохранён (CLIMB_FAIL)", r.result == "CLIMB_FAIL")

# --- 3. try_cmd троттлит вызовы до ~1/sim-сек ---
clk = Clock(); md = Mode()
r = PlanRunner([Cmd("cmd", 40)], clk, md, Log())   # 40 тиков по 0.05с = 2 sim-сек
for _ in range(45):
    if r.finished: break
    clk.t += 0.05
    r.tick(S())
# за ~2 sim-сек троттл ~1/сек → 2-3 вызова (t=0, ~1.0, ~2.0)
check("try_cmd троттлит (≤3 вызова за 2 sim-сек)", 1 <= len(md.calls) <= 3)

# --- 4. FINISH завершает немедленно ---
clk = Clock(); log = []
class Fin(Step):
    name = "fin"
    def tick(self, ctx, s): return _finish(RcCommand(), result="DONE")
r = PlanRunner([Fin(), Once("never", log)], clk, Mode(), Log())
clk.t += 0.05
r.tick(S())
check("FINISH: завершает не доходя до следующего", r.finished and log == [] and r.result == "DONE")

# --- 5. Control + pilot_thr: газ живого пилота через ThrottleLatch ---
from mission_pkg.plan.step import Control                              # noqa: E402


class Ctx:
    """Минимальный ctx для Control.tick (без PlanRunner)."""
    def __init__(self): self.log = Log()
    def keep_mode(self, s, m): pass
    def reset_keyframe(self): pass
    def elapsed(self): return 0.0


class Stack:
    """ControlStack-мок: r/p/y фиксированные, фаза не завершается."""
    def enter(self, s): pass
    def update(self, s): return RcCommand(roll=1600, pitch=1610, yaw=1620)
    def motion_done(self): return False
    def excite_done(self): return False


class SP(S):
    """Снапшот с газом пилота и высотой."""
    def __init__(self, thr, alt=3.0):
        self.pilot_throttle = thr
        self.rel_alt = alt


class AltHoldMock:
    def __init__(self): self.target = None
    def set_target(self, a): self.target = a
    def throttle(self, s): return 1580        # «контур что-то командует»


ctx = Ctx()
c = Control("c", Stack(), 1500, pilot_thr=True)
c.enter(ctx, SP(1700))
rc = c.tick(ctx, SP(1700)).rc               # вход с ОТКЛОНЁННЫМ газом
check("Control+latch: газ отклонён на входе → заперт (hold)", rc.throttle == 1500)
check("Control+latch: r/p/y при этом от стека", rc.roll == 1600 and rc.yaw == 1620)
c.tick(ctx, SP(1500))                        # стик побывал в центре
rc = c.tick(ctx, SP(1700)).rc
check("Control+latch: после центра газ пилота проходит", rc.throttle == 1700)
rc = c.tick(ctx, SP(1500)).rc
check("Control+latch: газ отпущен → снова hold", rc.throttle == 1500)

# с контуром AltHold: отпускание газа перецеливает контур на ТЕКУЩУЮ высоту
ah = AltHoldMock()
c2 = Control("c2", Stack(), 1500, pilot_thr=True, alt_hold=ah, alt_target=3.0)
c2.enter(ctx, SP(1500))
check("Control+alt_hold: вход — уставка плана", ah.target == 3.0)
c2.tick(ctx, SP(1500))                       # защёлка открыта
rc = c2.tick(ctx, SP(1800, alt=4.6)).rc      # пилот набирает
check("Control+alt_hold: газ пилота вытесняет контур", rc.throttle == 1800)
rc = c2.tick(ctx, SP(1500, alt=5.2)).rc      # отпустил на 5.2 м
check("Control+alt_hold: отпустил → контур снова в проводе", rc.throttle == 1580)
check("Control+alt_hold: контур перецелен на текущую высоту", ah.target == 5.2)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ PLANRUNNER OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
