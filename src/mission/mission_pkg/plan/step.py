#!/usr/bin/env python3
"""Step — примитивы фаз полётного задания (декларативные кирпичи плана).

Полётное задание = список Step'ов; `PlanRunner` гоняет их по порядку. Каждый Step —
маленькое поведение: на тик отдаёт RcCommand + статус (продолжать / следующий / прыжок
по имени / финиш). Логика фаз перенесена дословно из бывшего MissionRunner FSM.

Step зависит только от портов через `ctx` (PlanRunner): ctx.mode (FlightMode), ctx.log,
ctx.elapsed()/try_cmd()/keep_mode() — ни строчки rclpy.
"""
from control_pkg.domain.rc import RC_CENTER, RcCommand

# статусы результата шага
RUN, NEXT, GOTO, FINISH = "run", "next", "goto", "finish"


class StepResult:
    def __init__(self, rc, status=RUN, goto="", result=""):
        self.rc = rc
        self.status = status
        self.goto = goto          # имя целевого шага (для GOTO)
        self.result = result      # метка исхода (накапливается в runner.result)


def _run(rc): return StepResult(rc, RUN)
def _next(rc, result=""): return StepResult(rc, NEXT, result=result)
def _goto(rc, name, result=""): return StepResult(rc, GOTO, goto=name, result=result)
def _finish(rc, result=""): return StepResult(rc, FINISH, result=result)


def _overlay_stack(step, ctx, s, rc) -> None:
    """Наложить roll/pitch/yaw от ControlStack ПОВЕРХ rc (throttle НЕ трогаем) — так набор
    (Climb) и сброс (Land) тоже стабилизируются горизонтально, а не летят с центр-стиками.
    Ленивый enter с wait_gt (gz-опоре нужна истинная поза). step имеет .stack/.wait_gt/
    ._entered_stack. stack=None → шаг горизонтально НЕ стабилизируется (легаси-поведение).
    Активацию ЛОГИРУЕМ (как Control): иначе по логу прогона climb/land выглядят «без
    стабилизатора», хотя стек там живой — на этом уже один раз построили ложный диагноз."""
    if step.stack is None:
        return
    if not step._entered_stack:
        if step.wait_gt and not s.gt_valid:
            return                       # ждём истинную позу (gz-опора) перед активацией
        step.stack.enter(s)
        step._entered_stack = True
        ctx.log.info(f"    {step.name}: стек активирован")
    ctrl = step.stack.update(s)
    rc.roll, rc.pitch, rc.yaw = ctrl.roll, ctrl.pitch, ctrl.yaw


class Step:
    name = "step"

    def enter(self, ctx, s) -> None:
        """Вызывается один раз при входе в шаг (сброс внутреннего состояния)."""

    def tick(self, ctx, s) -> StepResult:
        raise NotImplementedError


class AwaitMode(Step):
    """Ждать латча режима (PREARM): слать set_mode, держать стики центр. Бюджет вышел →
    идём дальше (не абортим — режим мог не отдать heartbeat, но арм попробуем)."""

    def __init__(self, name, mode, throttle, budget):
        self.name = name
        self.mode = mode
        self.throttle = throttle
        self.budget = budget

    def tick(self, ctx, s) -> StepResult:
        rc = RcCommand(throttle=self.throttle)
        ctx.try_cmd(lambda: ctx.mode.set_mode(self.mode))
        if s.mode == self.mode:
            return _next(rc)
        if ctx.elapsed() > self.budget:
            ctx.log.warn(f"⚠️ {self.mode} не залатчился (mode={s.mode}) — пробуем дальше")
            return _next(rc)
        return _run(rc)


class Arm(Step):
    """Армирование. Не прошло за бюджет → финиш с ARM_FAIL (садиться нечем — не в воздухе)."""

    def __init__(self, name, throttle, budget, keep="ALT_HOLD"):
        self.name = name
        self.throttle = throttle
        self.budget = budget
        self.keep = keep

    def tick(self, ctx, s) -> StepResult:
        rc = RcCommand(throttle=self.throttle)
        ctx.keep_mode(s, self.keep)
        ctx.try_cmd(ctx.mode.arm)
        if s.armed:
            return _next(rc)
        if ctx.elapsed() > self.budget:
            ctx.log.error(f"⚠️ арм не прошёл (armed={s.armed}) — аборт")
            return _finish(rc, "ARM_FAIL")
        return _run(rc)


class Climb(Step):
    """Набор высоты до alt по баро. Бюджет вышел: если оторвались (>0.5м) — дальше как
    есть; иначе аборт → прыжок на посадочный шаг (RC override не принят / не взлетели)."""

    def __init__(self, name, alt, throttle, budget, land_step="land", keep="ALT_HOLD",
                 stack=None, wait_gt=False):
        self.name = name
        self.alt = alt
        self.throttle = throttle
        self.budget = budget
        self.land_step = land_step
        self.keep = keep
        self.stack = stack            # ControlStack (StaticSetpoint+стабилизаторы) — держит горизонт
        self.wait_gt = wait_gt
        self._entered_stack = False

    def enter(self, ctx, s) -> None:
        self._entered_stack = False

    def tick(self, ctx, s) -> StepResult:
        rc = RcCommand(throttle=self.throttle)
        ctx.keep_mode(s, self.keep)
        _overlay_stack(self, ctx, s, rc)   # набор стабилизирован: горизонт держит стек с отрыва
        if s.rel_alt is not None and s.rel_alt >= self.alt:
            ctx.log.info(f"    набрали {s.rel_alt:.1f}м (цель {self.alt}м)")
            return _next(rc)
        if ctx.elapsed() > self.budget:
            if s.rel_alt is not None and s.rel_alt >= 0.5:
                ctx.log.warn(f"⚠️ climb-бюджет вышел, высота {s.rel_alt:.1f}м — дальше")
                return _next(rc)
            ctx.log.error(f"⚠️ не взлетели (rel_alt={s.rel_alt}) — аборт→LAND")
            return _goto(rc, self.land_step, "CLIMB_FAIL")
        return _run(rc)


class Control(Step):
    """Управляемая фаза: держит высоту (throttle=hold), roll/pitch/yaw — от ControlStack
    (trajectory-профиль + стабилизаторы). Опц. VinsHandover (Flow→Vins при VINS ready).
    wait_gt — ждать истинную позу перед активацией стека (gz-режимы; на борту/флоу — нет).
    Завершается по traj.done / excite.done / max_sec."""

    def __init__(self, name, stack, throttle, keep="ALT_HOLD", handover=None,
                 max_sec=0.0, wait_gt=False, result="HOLD_DONE"):
        self.name = name
        self.stack = stack
        self.throttle = throttle
        self.keep = keep
        self.handover = handover
        self.max_sec = max_sec
        self.wait_gt = wait_gt
        self.result = result
        self._entered_stack = False

    def enter(self, ctx, s) -> None:
        self._entered_stack = False

    def tick(self, ctx, s) -> StepResult:
        rc = RcCommand(throttle=self.throttle)
        ctx.keep_mode(s, self.keep)
        if not self._entered_stack:
            if self.wait_gt and not s.gt_valid:
                return _run(rc)                      # ждём истинную позу (gz-опора)
            self.stack.enter(s)
            self._entered_stack = True
            ctx.log.info(f"    {self.name}: стек активирован")
        if self.handover is not None and self.handover.maybe_switch(self.stack, s):
            ctx.log.info(f"    ✅ VINS сошёлся ({s.vins_odom_count} odom) → Flow→Vins (hot-swap)")
        ctrl = self.stack.update(s)
        rc.roll, rc.pitch, rc.yaw = ctrl.roll, ctrl.pitch, ctrl.yaw
        done = self.stack.motion_done() or self.stack.excite_done()
        if not done and self.max_sec > 0 and ctx.elapsed() > self.max_sec:
            done = True
        if done:
            ctx.log.info(f"    {self.name} завершён — дальше")
            return _next(rc, self.result)
        return _run(rc)


class Land(Step):
    """Посадка: режим LAND, ждём касание ПО ФАКТУ. Касание = баро rel_alt<=ground_z ИЛИ
    истинная высота gt_z<=ground_z (сим-оракул: ловит посадку ГДЕ УГОДНО, в т.ч. за краем
    сцены, когда баро врёт/лагает) ИЛИ дизарм после LAND. Бюджет — лишь бэкстоп (в sim-сек):
    при рабочем детекте не достигается, поэтому держим его вменяемым, а не 180."""

    def __init__(self, name, throttle, ground_z, budget, stack=None, wait_gt=False):
        self.name = name
        self.throttle = throttle
        self.ground_z = ground_z
        self.budget = budget
        self.stack = stack            # держит горизонт на сбросе (на борту пре-VINS у LAND нет position-hold)
        self.wait_gt = wait_gt
        self._entered_stack = False

    def enter(self, ctx, s) -> None:
        self._entered_stack = False

    def tick(self, ctx, s) -> StepResult:
        rc = RcCommand(throttle=self.throttle)
        ctx.try_cmd(lambda: ctx.mode.set_mode("LAND"))
        _overlay_stack(self, ctx, s, rc)   # сброс стабилизирован: горизонт держит стек до касания
        # касание по ФАКТУ: баро ИЛИ истинная высота (ловит посадку за краем сцены)
        touched = (s.rel_alt is not None and s.rel_alt <= self.ground_z) or \
                  (s.gt_valid and s.gt_z <= self.ground_z)
        # Land — эпилог: НЕ перетирает исход миссии (HOLD_DONE/CLIMB_FAIL…) своей меткой.
        if touched or (s.mode == "LAND" and not s.armed and ctx.elapsed() > 3):
            ctx.log.info(f"    касание (rel_alt={s.rel_alt}, armed={s.armed})")
            return _finish(rc)
        if ctx.elapsed() > self.budget:
            ctx.log.warn(f"⚠️ касание не подтверждено (rel_alt={s.rel_alt}) — выходим")
            return _finish(rc)
        return _run(rc)


class Hover(Step):
    """Держать высоту sim-сек (стики центр). Простой самодостаточный шаг (observe/hover)."""

    def __init__(self, name, throttle, sec, keep="ALT_HOLD"):
        self.name = name
        self.throttle = throttle
        self.sec = sec
        self.keep = keep

    def tick(self, ctx, s) -> StepResult:
        rc = RcCommand(throttle=self.throttle)
        ctx.keep_mode(s, self.keep)
        return _next(rc) if ctx.elapsed() > self.sec else _run(rc)
