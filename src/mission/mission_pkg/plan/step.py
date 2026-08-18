#!/usr/bin/env python3
"""Step — примитивы фаз полётного задания (декларативные кирпичи плана).

Полётное задание = список Step'ов; `PlanRunner` гоняет их по порядку. Каждый Step —
маленькое поведение: на тик отдаёт RcCommand + статус (продолжать / следующий / прыжок
по имени / финиш). Логика фаз перенесена дословно из бывшего MissionRunner FSM.

Step зависит только от портов через `ctx` (PlanRunner): ctx.mode (FlightMode), ctx.log,
ctx.elapsed()/try_cmd()/keep_mode() — ни строчки rclpy.
"""
import math

from control_pkg.domain.control.throttle_latch import ThrottleLatch
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
        ctx.reset_keyframe()             # начало сегмента = точка удержания
        step.stack.enter(s)              # СТРОГО после сброса опоры: холдер положения
        step._entered_stack = True       # берёт точку с первого кадра ОТ НОВОЙ опоры
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
                 stack=None, wait_gt=False, alt_hold=None):
        self.name = name
        self.alt = alt
        self.throttle = throttle
        # AltHold (или None = прежнее поведение: постоянный throttle_climb до цели).
        # С контуром шаг ТОРМОЗИТ на подходе: без него он выходил с vz=+1.6 м/с и
        # ALT_HOLD доносил борт ещё на 2.2 м вверх (замер J1b: 3.0 → 5.2 м).
        self.alt_hold = alt_hold
        self.budget = budget
        self.land_step = land_step
        self.keep = keep
        self.stack = stack            # ControlStack (StaticSetpoint+стабилизаторы) — держит горизонт
        self.wait_gt = wait_gt
        self._entered_stack = False

    def enter(self, ctx, s) -> None:
        self._entered_stack = False
        if self.alt_hold is not None:
            self.alt_hold.set_target(self.alt)

    def tick(self, ctx, s) -> StepResult:
        thr = self.alt_hold.throttle(s) if self.alt_hold is not None else self.throttle
        rc = RcCommand(throttle=thr)
        ctx.keep_mode(s, self.keep)
        _overlay_stack(self, ctx, s, rc)   # набор стабилизирован: горизонт держит стек с отрыва
        # С контуром цель считается достигнутой по ДОПУСКУ, а не по «перешли черту»:
        # иначе шаг закрывается на подлёте, когда контур ещё тормозит.
        reached = (s.rel_alt is not None
                   and (s.rel_alt >= self.alt if self.alt_hold is None
                        else abs(s.rel_alt - self.alt) <= self.alt_hold.tol))
        if reached:
            ctx.log.info(f"    набрали {s.rel_alt:.2f}м (цель {self.alt}м)")
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
    pilot_thr — газ ЖИВОГО пилота (через ThrottleLatch): стик вне центра = сырой газ
    (вертикальная скорость в ALT_HOLD), в центре/до открытия защёлки = hold/контур.
    Завершается по traj.done / excite.done / max_sec."""

    # ГЕОЗАБОР задаётся планом (compile_mission); 0 = выкл. Проверяется по ИСТИННОЙ позе,
    # то есть это стендовая страховка, а не бортовая функция: на борту gt нет.
    fence = 0.0

    def __init__(self, name, stack, throttle, keep="ALT_HOLD", handover=None,
                 max_sec=0.0, wait_gt=False, result="HOLD_DONE", alt_hold=None,
                 alt_target=None, pilot_thr=False, pilot_deadzone=30,
                 pilot_stabs=None):
        self.name = name
        self.stack = stack
        self.throttle = throttle
        # Высоту держит контур, а НЕ «стик в центре»: центр в ALT_HOLD означает лишь
        # «не менять высоту», и любую уже накопленную ошибку он консервирует. Уставка
        # приходит от плана (высота последнего climb), а НЕ снимается с борта на входе —
        # иначе шаг узаконил бы перелёт набора как рабочую высоту.
        self.alt_hold = alt_hold
        self.alt_target = alt_target
        self.keep = keep
        self.handover = handover
        self.max_sec = max_sec
        self.wait_gt = wait_gt
        self.result = result
        self._entered_stack = False
        # Газ живого пилота (только --pilot joy/ros): защёлка = нельзя войти в фазу
        # с отклонённым стиком; scripted-пилотам НЕ даётся, чтобы не менять
        # воспроизводимость эталонных прогонов (там pilot_throttle всегда центр).
        self._latch = ThrottleLatch(pilot_deadzone) if pilot_thr else None
        self._pilot_flying_thr = False
        # СЕЛЕКТОР СТАБИЛИЗАЦИИ трёхпозиционником CH6 (только живой пилот):
        # −1 → pilot_stabs (наш стабилизатор, BS_STAB); 0 → [] (чистый ALT_HOLD:
        # стики = наклоны); +1 → MANUAL перехватывает Арбитр ВЫШЕ шага, стек не
        # трогаем. None = селектор выключен (scripted-миссии, стек фиксирован).
        self._pilot_stabs = pilot_stabs
        self._stab_pos = None            # применённое положение (−1/0)

    def enter(self, ctx, s) -> None:
        self._entered_stack = False
        if self.alt_hold is not None and self.alt_target is not None:
            self.alt_hold.set_target(self.alt_target)
        if self._latch is not None:
            self._latch.reset()
            self._pilot_flying_thr = False
        self._stab_pos = None

    def tick(self, ctx, s) -> StepResult:
        thr = self.alt_hold.throttle(s) if self.alt_hold is not None else self.throttle
        if self._latch is not None:
            p = self._latch.pass_through(s.pilot_throttle)
            if p is not None:
                thr = p                          # пилот командует вертикалью
                self._pilot_flying_thr = True
            elif self._pilot_flying_thr:
                self._pilot_flying_thr = False
                # пилот отпустил газ: контур перецеливается на ТЕКУЩУЮ высоту,
                # иначе он потащит борт обратно на уставку, заданную до вмешательства
                if self.alt_hold is not None and s.rel_alt is not None:
                    self.alt_hold.set_target(s.rel_alt)
                    ctx.log.info(f"    газ отпущен — держим {s.rel_alt:.1f}м")
        rc = RcCommand(throttle=thr)
        ctx.keep_mode(s, self.keep)
        if not self._entered_stack:
            if self.wait_gt and not s.gt_valid:
                return _run(rc)                      # ждём истинную позу (gz-опора)
            ctx.reset_keyframe()         # начало сегмента = точка удержания
            self.stack.enter(s)          # СТРОГО после сброса опоры (см. _enter_stack)
            self._entered_stack = True
            ctx.log.info(f"    {self.name}: стек активирован")
        if self.fence > 0 and s.gt_valid:
            d = math.hypot(s.gt_x, s.gt_y)
            if d > self.fence:
                ctx.log.warn(f"    ⛔ ГЕОЗАБОР: ушли на {d:.1f} м > {self.fence:.0f} — на посадку")
                return _goto(rc, "land", result="FENCE")
        if self._pilot_stabs is not None:
            # применяем положение тумблера (−1/0); +1 игнорируем — там правит Арбитр
            pos = s.pilot_switch if s.pilot_switch in (-1, 0) else self._stab_pos
            if pos is not None and pos != self._stab_pos:
                self._stab_pos = pos
                self.stack.switch_stabilization(self._pilot_stabs if pos == -1 else [])
                self.stack.enter(s)      # пересев опор: держим ОТ ТЕКУЩЕЙ точки
                ctx.log.info("    тумблер: {}".format(
                    "НАШ СТАБИЛИЗАТОР" if pos == -1 else "чистый ALT_HOLD (стики=наклоны)"))
        if self.handover is not None and self.handover.maybe_switch(self.stack, s):
            ctx.log.info(f"    ✅ VINS сошёлся ({s.vins_odom_count} odom) → Flow→Vins (hot-swap)")
        ctrl = self.stack.update(s)
        rc.roll, rc.pitch, rc.yaw = ctrl.roll, ctrl.pitch, ctrl.yaw
        done = self.stack.motion_done() or self.stack.excite_done()
        if not done and self.max_sec > 0 and ctx.elapsed() > self.max_sec:
            done = True
        # «хватит летать» от оператора (make pilot-done) — только живому пилоту
        # (_latch как маркер live): бессрочный `pilot` иначе не завершить штатно
        if not done and self._latch is not None and s.pilot_done:
            ctx.log.info("    оператор завершил pilot-сегмент (pilot_done)")
            done = True
        if done:
            ctx.log.info(f"    {self.name} завершён — дальше")
            return _next(rc, self.result)
        return _run(rc)


class Freefly(Step):
    """СВОБОДНЫЙ ПОЛЁТ: полный ручной цикл с пульта — арм (руддером), взлёт, полёт,
    посадка, дизарм. Миссия-одиночка по определению (гейт в compile_mission).

    Отличия от pilot-сегмента Control — осознанные, по одному на строку:
    - ГАЗ СЫРОЙ, без ThrottleLatch и AltHold: пилот владеет вертикалью с нулевой
      секунды. Защёлка заводилась против ОПАСНОГО МОМЕНТА ПЕРЕДАЧИ управления в
      воздухе — здесь такого момента не существует (модель «обычного пульта»), а
      руддер-арм требует честного НИЗКОГО газа, защёлка держала бы центр.
    - ГЕОЗАБОР НЕ ПРОВЕРЯЕТСЯ ВОВСЕ — по определению миссии.
    - ДО ПЕРВОГО АРМА все оси сырые независимо от тумблера: руддер-арм требует
      живого yaw, а демпферы на земле выдают центр (сигналов нет).
    - Завершение = ДИЗАРМ после хотя бы одного арма; make pilot-done не нужен,
      auto-Land эпилога нет — сажает пилот.
    Селектор CH6 после арма работает как в pilot-сегменте (−1 = наш стек, 0 = чистый
    ALT_HOLD; +1 = MANUAL правит Арбитр выше шага).
    ⚠️ Тумблер +1 (MANUAL) до арма: газ идёт через защёлку АРБИТРА (центр, пока стик
    не побывал в центре) — армить проще с тумблером вверх/в центре, либо сперва
    качнуть газ через центр."""

    def __init__(self, name, stack, keep="ALT_HOLD", pilot_stabs=None):
        self.name = name
        self.stack = stack
        self.keep = keep
        self._pilot_stabs = pilot_stabs
        self._greeted = False
        self._was_armed = False
        self._stab_pos = None

    def enter(self, ctx, s) -> None:
        self._greeted = False
        self._was_armed = False
        self._stab_pos = None

    def tick(self, ctx, s) -> StepResult:
        rc = RcCommand(throttle=s.pilot_throttle)
        ctx.keep_mode(s, self.keep)
        if not self._greeted:
            self._greeted = True
            ctx.log.info(f"    {self.name}: борт у пилота — арм руддером (газ вниз + "
                         f"yaw вправо); дизарм завершает миссию, геозабора нет")
        if s.armed and not self._was_armed:
            self._was_armed = True
            # опора и стек — с момента арма: точка отсчёта там, где армили
            ctx.reset_keyframe()
            self.stack.enter(s)
            ctx.log.info("    пилот заармил — свободный полёт")
        if not self._was_armed:
            rc.roll, rc.pitch, rc.yaw = s.pilot_roll, s.pilot_pitch, s.pilot_yaw
            return _run(rc)
        if self._pilot_stabs is not None:
            pos = s.pilot_switch if s.pilot_switch in (-1, 0) else self._stab_pos
            if pos is not None and pos != self._stab_pos:
                self._stab_pos = pos
                self.stack.switch_stabilization(self._pilot_stabs if pos == -1 else [])
                self.stack.enter(s)      # пересев опор: держим ОТ ТЕКУЩЕЙ точки
                ctx.log.info("    тумблер: {}".format(
                    "НАШ СТАБИЛИЗАТОР" if pos == -1 else "чистый ALT_HOLD (стики=наклоны)"))
        ctrl = self.stack.update(s)
        rc.roll, rc.pitch, rc.yaw = ctrl.roll, ctrl.pitch, ctrl.yaw
        if not s.armed:
            ctx.log.info("    пилот дизармил — freefly завершён")
            return _finish(rc, "FREEFLY_DONE")
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
