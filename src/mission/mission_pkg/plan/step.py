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


class WaitEkfPos(Step):
    """Ждать, пока EKF полётника ДЕРЖИТ позицию (свежий /mavros/local_position),
    ПЕРЕД армом. Профиль «взлёт на GPS → Loiter» (вставляется compile_mission
    только при loiter-токене в миссии).

    Урок LV4 против LV3 (dataflash XKF4.SS): арм с ARMING_CHECK=0 проходит ДО
    того, как EK3 захватил GPS — гонка бута (LV3: aiding на t=11.7, за 4 с до
    арма; LV4: EKF замешкался на выравнивании яу, борт оторвался без позиции,
    и начать aiding уже в воздухе на манёврах EK3 не смог — const_pos весь
    полёт, Loiter refused). Старый GUIDED-флоу ждал НЕЯВНО (GUIDED не латчится
    без позиции) — ALT_HOLD-флоу должен ждать явно. Бюджет вышел → warn и
    дальше (как AwaitMode): миссия продолжается, но loiter потом честно
    пропустится своим гейтом."""

    def __init__(self, name, throttle, budget, fresh_sec=2.0, keep="ALT_HOLD"):
        self.name = name
        self.throttle = throttle
        self.budget = budget
        self.fresh_sec = fresh_sec
        self.keep = keep

    def tick(self, ctx, s) -> StepResult:
        rc = RcCommand(throttle=self.throttle)
        ctx.keep_mode(s, self.keep)
        if (s.now_sim - s.ekf_pos_last_sim) < self.fresh_sec:
            ctx.log.info(f"    {self.name}: EKF держит позицию — к арму")
            return _next(rc)
        if ctx.elapsed() > self.budget:
            ctx.log.warn(f"⚠️ {self.name}: EKF не захватил позицию за "
                         f"{self.budget:g} с — дальше (loiter пропустится гейтом)")
            return _next(rc)
        return _run(rc)


class LoiterHold(Step):
    """ШТАТНЫЙ LOITER на sec sim-сек: позицию держит контроллер САМОГО FCU по EKF,
    скормленному VINS через vision_pose (extnav). Оракул/фолбэк для VinsHold:
    тот же VINS, но держит прошивка, а не наш стек.

    Три фазы, по одной причине на каждую:
      WAIT  — гейт закрыт: ведём себя как стабилизированный hover (ALT_HOLD,
              hold-стек держит горизонт, газ — контур высоты). Гейт: extnav_ready
              (пара EK3_SRC1_*=6 применена очередью ноды) + свежий VINS + в воздухе
              (>loiter_alt — на земле без GPS позиции нет, VINS без параллакса не
              инитится, latch невозможен по построению). Не открылся за
              gate_budget → шаг ПРОПУСКАЕТСЯ (LOITER_SKIP): деградация безопасная,
              миссия продолжается на нашем стеке.
      LATCH — шлём LOITER; пока FCU не подтвердил mode, продолжаем стабилизировать
              как в WAIT: наши законы живут в семантике ALT_HOLD (стик = наклон),
              бросать их ДО фактической смены режима нельзя — центр-стики в
              ALT_HOLD никого не держат. Отказ за mode_budget → LOITER_REFUSED.
      HOLD  — mode=LOITER: ВСЕ стики в центр (центр в Loiter = «стоять», override
              ch1..4 для FCU и есть стики пилота), позицию и высоту держит FCU.
              Режим НЕ ре-ассертим: выпал из LOITER — это решение полётника
              (EKF-failsafe), уважаем и выходим (LOITER_EJECT). VINS протух
              дольше 3×fresh → выходим сами (LOITER_STALE), не дожидаясь
              расхождения EKF. Отлетали sec → LOITER_DONE.
    """

    def __init__(self, name, sec, throttle, gate_budget, keep="ALT_HOLD",
                 stack=None, wait_gt=False, alt_hold=None, alt_target=None,
                 fresh_sec=2.0, mode_budget=20.0, loiter_alt=1.5):
        self.name = name
        self.sec = sec
        self.throttle = throttle
        self.gate_budget = gate_budget
        self.keep = keep
        self.stack = stack            # hold-стек: держит горизонт, пока ждём гейт
        self.wait_gt = wait_gt
        self.alt_hold = alt_hold
        self.alt_target = alt_target
        self.fresh_sec = fresh_sec
        self.mode_budget = mode_budget
        self.loiter_alt = loiter_alt  # гейт «в воздухе» (см. config.loiter_alt)
        self._entered_stack = False
        self._gated = False
        self._t_gate = None
        self._t_loiter = None

    def enter(self, ctx, s) -> None:
        self._entered_stack = False
        self._gated = False
        self._t_gate = None
        self._t_loiter = None
        if self.alt_hold is not None and self.alt_target is not None:
            self.alt_hold.set_target(self.alt_target)

    def tick(self, ctx, s) -> StepResult:
        if self._t_loiter is not None or (self._gated and s.mode == "LOITER"):
            # --- HOLD: держит FCU, мы только считаем время и следим за VINS ---
            if self._t_loiter is None:
                self._t_loiter = s.now_sim
                ctx.log.info(f"    {self.name}: LOITER залатчен — стики центр, "
                             f"позицию держит FCU (extnav)")
            rc = RcCommand()                       # всё в центре = «стоять»
            if s.mode != "LOITER":
                ctx.log.warn(f"    {self.name}: FCU вышел из LOITER (mode={s.mode}) "
                             f"— уважаем, дальше")
                return _next(rc, "LOITER_EJECT")
            if (s.now_sim - s.vins_last_sim) > 3.0 * self.fresh_sec:
                ctx.log.warn(f"    {self.name}: VINS протух — выходим из LOITER")
                return _next(rc, "LOITER_STALE")
            if s.now_sim - self._t_loiter >= self.sec:
                ctx.log.info(f"    {self.name}: {self.sec:g} с в LOITER — дальше")
                return _next(rc, "LOITER_DONE")
            return _run(rc)
        # --- WAIT / LATCH: стабилизированный hover в семантике ALT_HOLD ---
        thr = self.alt_hold.throttle(s) if self.alt_hold is not None else self.throttle
        rc = RcCommand(throttle=thr)
        if not self._gated:
            ctx.keep_mode(s, self.keep)
            _overlay_stack(self, ctx, s, rc)
            gate = (s.extnav_ready
                    and (s.now_sim - s.vins_last_sim) < self.fresh_sec
                    and (s.rel_alt or 0.0) > self.loiter_alt)
            if gate:
                self._gated = True
                self._t_gate = s.now_sim
                ctx.log.info(f"    {self.name}: extnav+VINS готовы "
                             f"({s.vins_odom_count} odom) — шлю LOITER")
            elif ctx.elapsed() > self.gate_budget:
                ctx.log.warn(f"    {self.name}: гейт не открылся за "
                             f"{self.gate_budget:g} с (extnav_ready={s.extnav_ready}, "
                             f"odom={s.vins_odom_count}) — пропуск")
                return _next(rc, "LOITER_SKIP")
            return _run(rc)
        ctx.try_cmd(lambda: ctx.mode.set_mode("LOITER"))
        _overlay_stack(self, ctx, s, rc)   # до подтверждения режима держим по-старому
        if s.now_sim - self._t_gate > self.mode_budget:
            ctx.log.warn(f"    {self.name}: LOITER не залатчился (mode={s.mode}) "
                         f"— пропуск")
            return _next(rc, "LOITER_REFUSED")
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
    - СТРАХОВКА ДИЗАРМА (урок lv1_replay_20260823_191230): FCU может молча
      отказывать в руддер-дизарме (детектор посадки не взводится), а freefly
      ждёт дизарм БЕССРОЧНО — прогон завис на земле, bag раздуло до 41 ГБ.
      Пилот при этом ЯВНО держит жест (газ min + yaw влево; руддеру хватает
      ~2 с). Жест на земле (баро ≤0.3 м) дольше 8 с → дизармим сервисом
      cmd/arming сами; ещё через 4 с — force (MAV_CMD 400, param2=21196):
      раз штатный путь отвергнут, сломан как раз детектор посадки. На земле
      с газом в полу это безопасно и повторяет аварийный тумблер реального
      пульта. Отпустил жест — таймер сбрасывается. Последний рубеж: реплей №1
      (2026-08-22, опрокинутый борт) показал, что FCU может отвергнуть ДАЖЕ
      force — жест дольше 30 с завершает миссию FREEFLY_STUCK: запись обязана
      остановиться, bag ограничен (борт остаётся заармленным — громкий error).
    Селектор CH6 после арма работает как в pilot-сегменте (−1 = наш стек, 0 = чистый
    ALT_HOLD; +1 = MANUAL правит Арбитр выше шага).
    ⚠️ Тумблер +1 (MANUAL) до арма: газ идёт через защёлку АРБИТРА (центр, пока стик
    не побывал в центре) — армить проще с тумблером вверх/в центре, либо сперва
    качнуть газ через центр.

    loiter_center (BS_FF_LOITER): центр CH6 = ШТАТНЫЙ LOITER-на-VINS вместо чистого
    ALT_HOLD — позицию держит контроллер FCU по EKF-от-VINS, стики (наш passthrough-
    override) для него уставки скорости. Гейт тот же, что у LoiterHold: extnav_ready
    + свежий VINS + в воздухе; закрыт → честный ALT_HOLD с одним предупреждением.
    Выход из LOITER — с гистерезисом (3×fresh): мигание свежести у порога не должно
    дёргать режим под пилотом. MANUAL (+1) всегда возвращает ALT_HOLD — safety-seize
    отдаёт сырые стики в предсказуемой семантике «стик = наклон», а не «стик =
    скорость». Стек селектора не меняется (центр и так = пустой список).

    sf_master (BS_SF_MASTER) — схема «SF-мастер» вместо легаси-селектора: MANUAL
    даёт ТОЛЬКО SF (CH7) не-вверх (pilot_switch=+1, Арбитр выше шага), а SC (CH6)
    задаёт ПОТОЛОК лесенки зрелости (s.pilot_level): 0 = демпфер, 1 = +VinsHold,
    2 = +штатный LOITER. Борт живёт на лучшей ДОСТУПНОЙ ступени ≤ потолка
    (_ladder_*): позиция «loiter» до зрелости — это демпфер/VinsHold, а не голый
    ALT_HOLD (в легаси-центре борт при закрытом гейте дрейфовал с ветром — прогон
    2026-08-20); деградация симметрична (VINS протух в LOITER → демпфер, VinsHold
    с протухшим VINS мёртв так же). Вход/выход LOITER ведёт тот же _mode_target
    (гейт + гистерезис легаси-центра, ярусу нужен loiter_center=True)."""

    def __init__(self, name, stack, keep="ALT_HOLD", pilot_stabs=None,
                 handover=None, loiter_center=False, vins_fresh=2.0,
                 sf_master=False, loiter_alt=1.5):
        self.name = name
        self.stack = stack
        self.keep = keep
        self._pilot_stabs = pilot_stabs
        self.loiter_alt = loiter_alt   # гейт «в воздухе» (см. config.loiter_alt)
        # handover Flow→Vins: срабатывает ТОЛЬКО в позиции селектора «наш стек»
        # (−1 или тумблер не трогали) — «вверх» = лучший доступный стек (демпфер
        # до готовности VINS, VinsHold после); центр/MANUAL свапом не трогаем.
        # В схеме SF-мастер одноразовый свап заменён лесенкой (_ladder_apply).
        self.handover = handover
        self.loiter_center = loiter_center
        self.vins_fresh = vins_fresh
        self.sf_master = sf_master
        self._greeted = False
        self._was_armed = False
        self._stab_pos = None
        self._in_loiter = False
        self._loiter_warned = False
        self._loiter_since = 0.0
        self._latch_warned = False
        self._land_warned = False
        self._level = None             # применённый потолок SC (0/1/2, sf_master)
        self._tier = 0                 # активный ярус лесенки (0/1/2)
        self._was_manual = False       # для пересева опор на выходе из MANUAL
        self._disarm_since = None      # sim-старт удержания жеста дизарма на земле
        self._gesture_last = 0.0       # sim-время последнего тика с жестом
        self._disarm_warned = False

    def enter(self, ctx, s) -> None:
        self._greeted = False
        self._was_armed = False
        self._stab_pos = None
        self._in_loiter = False
        self._loiter_warned = False
        self._loiter_since = 0.0
        self._latch_warned = False
        self._land_warned = False
        self._level = None
        self._tier = 0
        self._was_manual = False
        self._disarm_since = None
        self._gesture_last = 0.0
        self._disarm_warned = False

    def _loiter_selected(self) -> bool:
        """Позиция селектора «хочу штатный LOITER»: легаси-центр или потолок 2."""
        return (self._level == 2) if self.sf_master else (self._stab_pos == 0)

    def _mode_target(self, ctx, s) -> str:
        """Режим FCU под селектор (см. docstring про loiter_center)."""
        if (not self.loiter_center or not self._loiter_selected()
                or s.pilot_switch == 1):        # MANUAL-seize всегда в ALT_HOLD
            self._in_loiter = False
            self._loiter_warned = False
            self._latch_warned = False
            return self.keep
        if s.mode == "LAND":
            # FCU сам ушёл в LAND (EKF-failsafe, FS_EKF_ACTION) — с failsafe
            # не воюем (полёт 2026-08-20 №5: ре-ассерт LOITER бился с LAND
            # весь остаток полёта). Центр = уважаем посадку; пилот забирает
            # борт тумблером вверх/вниз (ветка выше — сознательное действие).
            if not self._land_warned:
                self._land_warned = True
                ctx.log.warn("    LOITER: FCU в LAND (failsafe) — уважаем, не "
                             "ре-ассертим; тумблер вверх вернёт наш стек")
            self._in_loiter = False
            return "LAND"
        self._land_warned = False
        fresh_age = s.now_sim - s.vins_last_sim
        if self._in_loiter:
            if not s.extnav_ready or fresh_age > 3.0 * self.vins_fresh:
                self._in_loiter = False
                ctx.log.warn("    LOITER: VINS/extnav протух — откат {}".format(
                    "на ярус ниже (лесенка)" if self.sf_master
                    else "в ALT_HOLD (стики = наклоны)"))
            elif (s.mode != "LOITER" and not self._latch_warned
                    and s.now_sim - self._loiter_since > 5.0):
                # честность к пилоту: гейт открыт, но FCU отказывает («requires
                # position» — EKF без позиции). В легаси центр = пустой стек,
                # т.е. борт ФАКТИЧЕСКИ в голом ALT_HOLD и дрейфует с ветром
                # (прогон 2026-08-20); в лесенке стек не бросается до латча —
                # держит текущий ярус. Ре-ассерт продолжается — EKF может дозреть.
                self._latch_warned = True
                ctx.log.warn("    LOITER: FCU не латчит режим (requires "
                             "position?) — {}".format(
                                 "держу текущий ярус лесенки в ALT_HOLD"
                                 if self.sf_master else
                                 "ФАКТИЧЕСКИ чистый ALT_HOLD, стики = наклоны; "
                                 "тумблер вверх вернёт наш стек"))
        elif (s.extnav_ready and fresh_age < self.vins_fresh
                and (s.rel_alt or 0.0) > self.loiter_alt):
            self._in_loiter = True
            self._loiter_warned = False
            self._loiter_since = s.now_sim
            self._latch_warned = False
            ctx.log.info("    селектор: ШТАТНЫЙ LOITER — позицию держит FCU "
                         "(extnav), стики = уставки скорости")
        elif not self._loiter_warned:
            self._loiter_warned = True
            ctx.log.info("    селектор: LOITER не готов (extnav_ready="
                         f"{s.extnav_ready}, odom={s.vins_odom_count}) — "
                         + ("летим на лесенке (лучший доступный ярус)"
                            if self.sf_master else "чистый ALT_HOLD"))
        return "LOITER" if self._in_loiter else self.keep

    # --- Лесенка SF-мастера (sf_master): SC задаёт ПОТОЛОК, летим на лучшей
    # ДОСТУПНОЙ ступени: 0 демпфер → 1 VinsHold (готовность VINS) → 2 штатный
    # LOITER (вход/выход ведёт _mode_target — тот же гейт и гистерезис, что у
    # легаси-центра). Стек яруса LOITER пустеет ТОЛЬКО когда FCU фактически
    # залатчил режим (урок LoiterHold: бросать свои законы ДО смены режима
    # нельзя — центр-стики в ALT_HOLD никого не держат); пока идёт ре-ассерт,
    # продолжаем VinsHold/демпфер в семантике ALT_HOLD. ---

    def _ladder_select(self, ctx, s) -> None:
        """Прочитать пульт: потолок SC + пересев опор на выходе из MANUAL.
        Пересев обязателен: в MANUAL пилот перегоняет борт руками, и позиц-
        холдеры со старой опорой тянули бы его обратно в точку до перехвата."""
        manual = (s.pilot_switch == 1)
        if self._was_manual and not manual:
            ctx.reset_keyframe()
            self.stack.enter(s)
            ctx.log.info("    SF: возврат из MANUAL — держим от текущей точки")
        self._was_manual = manual
        lvl = s.pilot_level if s.pilot_level in (0, 1, 2) else self._level
        if lvl is not None and lvl != self._level:
            self._level = lvl
            ctx.log.info("    SC-потолок: {}".format(
                {0: "ДЕМПФЕР (без свапа на VinsHold)",
                 1: "демпфер → VINSHOLD по готовности",
                 2: "демпфер → VinsHold → LOITER по зрелости"}[lvl]))

    def _ladder_tier(self, s) -> int:
        """Активный ярус = min(потолок SC, лучший ГОТОВЫЙ). Вниз с VinsHold —
        с гистерезисом 3×fresh (как выход из LOITER): мигание свежести у порога
        не должно дёргать стек под пилотом. Протух дольше — честный демпфер."""
        lvl = self._level if self._level is not None else 0
        ho = self.handover
        if lvl < 1 or ho is None:
            tier = 0
        elif self._tier >= 1:
            tier = 1 if (s.now_sim - s.vins_last_sim) <= 3.0 * ho.fresh_sec else 0
        else:
            tier = 1 if ho.vins_ready(s) else 0
        if lvl >= 2 and self._in_loiter and s.mode == "LOITER":
            tier = 2
        return tier

    def _ladder_apply(self, ctx, s) -> None:
        """Применить ярус: стек по ярусу, пересев опор от текущей точки."""
        tier = self._ladder_tier(s)
        if tier == self._tier:
            return
        self._tier = tier
        stabs = (self._pilot_stabs if tier == 0
                 else self.handover.vins_stabs(self._pilot_stabs, s) if tier == 1
                 else [])                # LOITER: позицию держит FCU, стики =
        self.stack.switch_stabilization(stabs)    # уставки скорости (passthrough)
        self.stack.enter(s)              # пересев опор: держим ОТ ТЕКУЩЕЙ точки
        ctx.log.info("    ЛЕСЕНКА: ярус {} (потолок SC={})".format(
            {0: "ДЕМПФЕР", 1: "VINSHOLD",
             2: "LOITER — стики = уставки скорости"}[tier], self._level))

    def tick(self, ctx, s) -> StepResult:
        rc = RcCommand(throttle=s.pilot_throttle)
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
            # СТРАХОВКА «АРМ НЕ СЛУЧИЛСЯ» (урок lv2_replay_20260824_034433):
            # FCU может отвергать руддер-арм бессрочно (там — PreArm mag field
            # из-за origin, не совпавшего с точкой старта), freefly ждёт
            # пилота вечно → 59 ГБ земли в bag. Живому пилоту 300 sim-с на
            # арм хватает с запасом (реплеи армят за ~45 с, пилот 182409 —
            # за 43); вышло — завершаем миссию, запись ограничена.
            if ctx.elapsed() > 300.0:
                ctx.log.error("    freefly: арм не случился за 300 с — "
                              "завершаю миссию (FCU отвергает арм? см. PreArm "
                              "в mavros.log)")
                return _finish(rc, "FREEFLY_NOARM")
            ctx.keep_mode(s, self.keep)
            rc.roll, rc.pitch, rc.yaw = s.pilot_roll, s.pilot_pitch, s.pilot_yaw
            return _run(rc)
        if self._pilot_stabs is not None and self.sf_master:
            self._ladder_select(ctx, s)          # потолок SC + выход из MANUAL
        elif self._pilot_stabs is not None:
            pos = s.pilot_switch if s.pilot_switch in (-1, 0) else self._stab_pos
            if pos is not None and pos != self._stab_pos:
                self._stab_pos = pos
                self.stack.switch_stabilization(self._pilot_stabs if pos == -1 else [])
                self.stack.enter(s)      # пересев опор: держим ОТ ТЕКУЩЕЙ точки
                ctx.log.info("    тумблер: {}".format(
                    "НАШ СТАБИЛИЗАТОР" if pos == -1 else "чистый ALT_HOLD (стики=наклоны)"))
        # режим — ПОСЛЕ селектора (loiter_center читает свежий _stab_pos/_level)
        ctx.keep_mode(s, self._mode_target(ctx, s))
        if self._pilot_stabs is not None and self.sf_master:
            # ярус — ПОСЛЕ _mode_target: тир LOITER читает свежие _in_loiter+mode
            self._ladder_apply(ctx, s)
        elif (self.handover is not None and self._stab_pos in (None, -1)
                and self.handover.maybe_switch(self.stack, s)):
            ctx.log.info("    ✅ VINS сошёлся → Flow→Vins (hot-swap); стики двигают "
                         "точку, отпустил — держит")
        ctrl = self.stack.update(s)
        rc.roll, rc.pitch, rc.yaw = ctrl.roll, ctrl.pitch, ctrl.yaw
        if not s.armed:
            ctx.log.info("    пилот дизармил — freefly завершён")
            return _finish(rc, "FREEFLY_DONE")
        # страховка дизарма (см. docstring): жест на земле дольше порога →
        # дизармим за FCU сами (сервис → force). Пороги PWM — как жесты
        # joy_timeline (GESTURE_LVL 0.85 → центр−340). «На земле» — баро ИЛИ
        # gt (как детект касания у Land): прогон 2 серии 2026-08-23 вскрыл
        # механизм отказов — баро/EKF-высота после касания ЗАСТРЕВАЕТ на
        # ~1.4-1.5 м (gt=0.0), детектор посадки FCU не взводится, и гейт
        # только по rel_alt молчал бы вместе с ним (27 мин до дизарма).
        # На борту gt_valid=False — там остаётся баро (кампания alt_src=baro).
        landed = ((s.rel_alt is not None and s.rel_alt <= 0.3)
                  or (s.gt_valid and s.gt_z <= 0.3))
        gesture = (s.pilot_throttle <= 1160 and s.pilot_yaw <= 1160 and landed)
        # Таймер переживает КОРОТКИЕ отпускания жеста (<3 с): реплей давит
        # руддер ИМПУЛЬСАМИ 4с/2с (joy_replay, фикс закрутки на земле из
        # реплея №1) — сброс на каждом отпускании держал таймер <8 с вечно,
        # и страховка молчала все 60 с дизарм-якоря (lv2_replay_20260824_
        # 040722: FCU отказал, борт простоял 9 мин, bag 40 ГБ). Для живого
        # пилота семантика не меняется: «отпустил» = пауза >3 с.
        if not gesture:
            # короткий отпуск (<3 с) таймер держит; действия — только под жестом
            if (self._disarm_since is not None
                    and s.now_sim - self._gesture_last > 3.0):
                self._disarm_since = None
        elif self._disarm_since is None:
            self._gesture_last = s.now_sim
            self._disarm_since = s.now_sim
        else:
            self._gesture_last = s.now_sim
            held = s.now_sim - self._disarm_since
            if held > 8.0 and not self._disarm_warned:
                self._disarm_warned = True
                ctx.log.warn("    freefly: FCU не дизармится руддером — "
                             "дизармлю сервисом (страховка)")
            if held > 30.0:
                ctx.log.error("    freefly: дизарм не прошёл даже force — "
                              "завершаю миссию, БОРТ ОСТАЛСЯ ЗААРМЛЕННЫМ")
                return _finish(rc, "FREEFLY_STUCK")
            if held > 12.0 and hasattr(ctx.mode, "force_disarm"):
                ctx.try_cmd(ctx.mode.force_disarm)
            elif held > 8.0:
                ctx.try_cmd(lambda: ctx.mode.arm(False))
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
