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
from mission_pkg.plan.step import FINISH, NEXT, RUN, Land, LoiterHold  # noqa: E402
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
    # набор и сброс теперь стабилизированы: у Climb/Land есть hold-стек со стабилизаторами
    climb_st = next(st for st in steps if st.name.startswith("climb"))
    land_st = next(st for st in steps if st.name == "land")
    checks.append(("compile: Climb несёт hold-стек (набор стабилизирован)",
                   climb_st.stack is not None and len(climb_st.stack.stabs) >= 1))
    checks.append(("compile: Land несёт hold-стек (сброс стабилизирован)",
                   land_st.stack is not None and len(land_st.stack.stabs) >= 1))
    checks.append(("compile: hold-стек держит на месте (StaticSetpoint, c_*=0)",
                   climb_st.stack.traj.intent(None, 0.0).c_fwd == 0.0))

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

    # 8. Land: касание ловится по ФАКТУ (gt_z), даже когда баро rel_alt врёт ---------
    class _Ctx:
        class _M:
            def set_mode(self, m): pass
        class _L:
            def info(self, m): pass
            def warn(self, m): pass
        mode = _M(); log = _L()
        def try_cmd(self, fn): fn()
        def elapsed(self): return 0.0
    land = Land("land", 1500, 0.3, 45.0)   # ground_z=0.3, budget=45
    land.enter(_Ctx(), None)
    r_air = land.tick(_Ctx(), DroneState(rel_alt=5.0, gt_valid=True, gt_z=5.0, now_sim=1.0))
    r_touch = land.tick(_Ctx(), DroneState(rel_alt=5.0, gt_valid=True, gt_z=0.2, now_sim=1.1))
    checks.append(("land: в воздухе (баро+gt высоко) → RUN", r_air.status == RUN))
    checks.append(("land: gt_z<порог при завышенном баро → FINISH (касание по факту)",
                   r_touch.status == FINISH))

    # 9. StationKeep: профиль ускорения +τ/−2τ/+τ возвращает скорость И позицию к нулю ---
    from control_pkg.domain.control.trajectory import StationKeep     # noqa: E402
    sk = StationKeep(level=0.5, tau=2.0, axis="fwd")
    dt, v, x, t = 0.005, 0.0, 0.0, 0.0
    peak = 0.0
    while t < sk.total():
        a = sk.intent(None, t).c_fwd      # стик = ускорение (открытый контур)
        v += a * dt; x += v * dt; t += dt
        peak = max(peak, abs(x))
    checks.append(("station-keep: скорость вернулась к нулю (|v|<0.01)", abs(v) < 0.01))
    checks.append(("station-keep: позиция вернулась к старту (|x|<0.05)", abs(x) < 0.05))
    checks.append(("station-keep: экскурсия ограничена (peak>0, вернулся)", peak > 0.1))
    # компиляция sk-токена → Control-сегмент со StationKeep
    sk_steps = compile_mission(cfg, "climb3,sk_fwd3,land", "DpRollHold+DpYawHold")
    sk_seg = next(st for st in sk_steps if st.name.startswith("sk_fwd"))
    checks.append(("compile: sk_fwd → Control со StationKeep",
                   type(sk_seg.stack.traj).__name__ == "StationKeep"))

    # --- токен pilot<t>: живые стики + селектор стабилизации тумблером ---
    pl = compile_mission(cfg, "climb3,pilot20,land", "DpRollHold+DpYawHold",
                         live_pilot=True)
    pseg = next(st for st in pl if st.name.startswith("pilot_"))
    checks.append(("compile: pilot → Control с RcTransmitter",
                   type(pseg.stack.traj).__name__ == "RcTransmitter"))
    checks.append(("compile: pilot ограничен max_sec", pseg.max_sec == 20.0))
    checks.append(("compile: pilot(live) получает селектор стабилизации",
                   pseg._pilot_stabs is not None and len(pseg._pilot_stabs) == 2))
    # со scripted-пилотом селектора НЕТ (стек BS_STAB фиксирован, токен = hover)
    pl2 = compile_mission(cfg, "climb3,pilot20,land", "DpRollHold+DpYawHold")
    pseg2 = next(st for st in pl2 if st.name.startswith("pilot_"))
    checks.append(("compile: pilot(scripted) — без селектора", pseg2._pilot_stabs is None))
    # `pilot` БЕЗ числа: живому пилоту — бессрочно (max_sec=0, конец по pilot_done)
    pl3 = compile_mission(cfg, "climb3,pilot,land", "DpRollHold+DpYawHold",
                          live_pilot=True)
    pseg3 = next(st for st in pl3 if st.name.startswith("pilot_"))
    checks.append(("compile: pilot без числа (live) → бессрочно (max_sec=0)",
                   pseg3.max_sec == 0.0))
    # scripted'у бессрочная форма запрещена (центр-стики → никогда не кончится)
    try:
        compile_mission(cfg, "climb3,pilot,land", "DpRollHold+DpYawHold")
        endless_bad = False
    except ValueError:
        endless_bad = True
    checks.append(("compile: pilot без числа (scripted) → ValueError", endless_bad))

    # --- миссия freefly: весь цикл руками пилота, миссия-одиночка ---
    ff = compile_mission(cfg, "freefly", "DpRollHold+DpYawHold", live_pilot=True)
    ff_names = [st.name for st in ff]
    checks.append(("freefly: план = prearm + freefly, БЕЗ Arm и Land",
                   ff_names == ["prearm", "freefly"]))
    ffs = ff[-1]
    checks.append(("freefly: стек с RcTransmitter и селектором",
                   type(ffs.stack.traj).__name__ == "RcTransmitter"
                   and ffs._pilot_stabs is not None))
    checks.append(("freefly: геозабора нет вовсе (нет атрибута fence-проверки)",
                   getattr(ffs, "fence", 0) == 0))
    # одиночество по определению: с другими токенами — ошибка
    try:
        compile_mission(cfg, "climb3,freefly", "DpRollHold+DpYawHold", live_pilot=True)
        ff_bad = False
    except ValueError:
        ff_bad = True
    checks.append(("freefly: с другими токенами → ValueError", ff_bad))
    # scripted-пилоту запрещена: армить/сажать некому
    try:
        compile_mission(cfg, "freefly", "DpRollHold+DpYawHold")
        ff_scripted = False
    except ValueError:
        ff_scripted = True
    checks.append(("freefly: scripted → ValueError", ff_scripted))
    # прогон шага: до арма — сырые стики (руддер-арм возможен), дизарм завершает
    class _FCtx:
        class _M:
            def set_mode(self, m): pass
        mode = _M()
        log = FakeLog()
        def try_cmd(self, fn): fn()
        def elapsed(self): return 0.0
        def keep_mode(self, s, m): pass
        def reset_keyframe(self): pass
    fctx = _FCtx()
    ffs.enter(fctx, None)
    s_pre = DroneState(armed=False, pilot_roll=1600, pilot_pitch=1400,
                       pilot_yaw=1900, pilot_throttle=1100, now_sim=1.0)
    r_pre = ffs.tick(fctx, s_pre)
    checks.append(("freefly: до арма сырые оси + сырой газ (руддер-арм возможен)",
                   r_pre.status == RUN and r_pre.rc.yaw == 1900
                   and r_pre.rc.throttle == 1100 and r_pre.rc.roll == 1600))
    s_fly = DroneState(armed=True, pilot_throttle=1650, now_sim=2.0, flow_seq=1)
    r_fly = ffs.tick(fctx, s_fly)
    checks.append(("freefly: после арма газ сырой, полёт продолжается",
                   r_fly.status == RUN and r_fly.rc.throttle == 1650))
    s_dis = DroneState(armed=False, pilot_throttle=1100, now_sim=99.0, flow_seq=2)
    r_dis = ffs.tick(fctx, s_dis)
    checks.append(("freefly: дизарм после арма → FINISH FREEFLY_DONE",
                   r_dis.status == FINISH and r_dis.result == "FREEFLY_DONE"))

    # --- токен loiter<t>: ШТАТНЫЙ LOITER-на-VINS (позицию держит сам FCU) ---
    lt = compile_mission(cfg, "climb3,hover5,loiter40,land", "DpRollHold+DpYawHold")
    lseg = next(st for st in lt if st.name.startswith("loiter_"))
    checks.append(("compile: loiter40 → LoiterHold(sec=40)",
                   isinstance(lseg, LoiterHold) and lseg.sec == 40.0))
    checks.append(("compile: loiter несёт hold-стек (ожидание гейта стабилизировано)",
                   lseg.stack is not None and len(lseg.stack.stabs) >= 1))
    try:
        compile_mission(cfg, "climb3,loiter,land", "DpRollHold+DpYawHold")
        lt_bad = False
    except ValueError:
        lt_bad = True
    checks.append(("compile: loiter без длительности → ValueError", lt_bad))

    # прогон шага на фейках: WAIT (гейт закрыт) → LATCH (шлём LOITER) → HOLD → DONE
    class _LCtx:
        def __init__(self):
            self.t = 0.0
            self.modes = []
            self.kept = None
            self.log = FakeLog()
            _c = self
            class _M:                                        # noqa: E306
                def set_mode(self, m): _c.modes.append(m)
            self.mode = _M()
        def try_cmd(self, fn): fn()
        def elapsed(self): return self.t
        def keep_mode(self, s, m): self.kept = m
        def reset_keyframe(self): pass
    lctx = _LCtx()
    lh = LoiterHold("loiter", 40.0, 1500, 60.0, fresh_sec=2.0, mode_budget=20.0)
    lh.enter(lctx, None)
    s_wait = DroneState(mode="ALT_HOLD", armed=True, rel_alt=3.0, extnav_ready=False,
                        vins_odom_count=80, vins_last_sim=9.9, now_sim=10.0)
    r_w = lh.tick(lctx, s_wait)
    checks.append(("loiter: гейт закрыт (нет extnav) → RUN в ALT_HOLD, LOITER не шлётся",
                   r_w.status == RUN and "LOITER" not in lctx.modes
                   and lctx.kept == "ALT_HOLD"))
    s_gate = DroneState(mode="ALT_HOLD", armed=True, rel_alt=3.0, extnav_ready=True,
                        vins_odom_count=80, vins_last_sim=10.0, now_sim=10.1)
    lh.tick(lctx, s_gate)                      # WAIT: гейт открылся (_gated)
    r_l = lh.tick(lctx, s_gate)                # LATCH: команда LOITER пошла
    checks.append(("loiter: гейт открыт → шлётся set_mode LOITER (FCU ещё ALT_HOLD)",
                   r_l.status == RUN and "LOITER" in lctx.modes))
    s_hold = DroneState(mode="LOITER", armed=True, rel_alt=3.0, extnav_ready=True,
                        vins_odom_count=90, vins_last_sim=10.2, now_sim=10.2)
    r_h = lh.tick(lctx, s_hold)
    checks.append(("loiter: LOITER залатчен → все стики центр (держит FCU)",
                   r_h.status == RUN and r_h.rc.roll == 1500 and r_h.rc.pitch == 1500
                   and r_h.rc.yaw == 1500 and r_h.rc.throttle == 1500))
    s_done = DroneState(mode="LOITER", armed=True, rel_alt=3.0, extnav_ready=True,
                        vins_odom_count=900, vins_last_sim=50.1, now_sim=50.3)
    r_d = lh.tick(lctx, s_done)
    checks.append(("loiter: отлетали sec → NEXT LOITER_DONE",
                   r_d.status == NEXT and r_d.result == "LOITER_DONE"))
    # гейт не открылся за бюджет → пропуск (безопасная деградация, миссия идёт дальше)
    lctx2 = _LCtx(); lctx2.t = 61.0
    lh2 = LoiterHold("loiter", 40.0, 1500, 60.0)
    lh2.enter(lctx2, None)
    r_s = lh2.tick(lctx2, s_wait)
    checks.append(("loiter: гейт не открылся за бюджет → NEXT LOITER_SKIP",
                   r_s.status == NEXT and r_s.result == "LOITER_SKIP"))
    # FCU сам выпал из LOITER (failsafe) → уважаем, выходим
    lctx3 = _LCtx()
    lh3 = LoiterHold("loiter", 40.0, 1500, 60.0)
    lh3.enter(lctx3, None)
    lh3.tick(lctx3, s_gate)
    lh3.tick(lctx3, s_hold)                    # вошли в HOLD
    s_eject = DroneState(mode="LAND", armed=True, rel_alt=3.0, extnav_ready=True,
                         vins_odom_count=95, vins_last_sim=11.0, now_sim=11.0)
    r_e = lh3.tick(lctx3, s_eject)
    checks.append(("loiter: FCU выпал из LOITER → NEXT LOITER_EJECT (не ре-ассертим)",
                   r_e.status == NEXT and r_e.result == "LOITER_EJECT"))

    # --- freefly + BS_FF_LOITER: центр CH6 = штатный LOITER (гейт + гистерезис) ---
    checks.append(("freefly: без BS_FF_LOITER центр = чистый ALT_HOLD (как раньше)",
                   ffs.loiter_center is False))
    cfg_lo = BootstrapConfig(mv_level=0.3, ff_loiter=1.0)
    ffl = compile_mission(cfg_lo, "freefly", "DpRollHold+DpYawHold", live_pilot=True)
    ffls = ffl[-1]
    checks.append(("freefly+ff_loiter: loiter_center включён", ffls.loiter_center))
    fctx2 = _LCtx()
    ffls.enter(fctx2, None)
    s_arm2 = DroneState(armed=True, now_sim=1.0)            # гейт закрыт (нет extnav)
    ffls.tick(fctx2, s_arm2)
    checks.append(("freefly+ff_loiter: центр при закрытом гейте → ALT_HOLD",
                   fctx2.kept == "ALT_HOLD"))
    s_ctr = DroneState(armed=True, rel_alt=3.0, pilot_switch=0, extnav_ready=True,
                       vins_odom_count=80, vins_last_sim=2.0, now_sim=2.0, flow_seq=1)
    ffls.tick(fctx2, s_ctr)
    checks.append(("freefly+ff_loiter: центр при готовом extnav+VINS → LOITER",
                   fctx2.kept == "LOITER"))
    s_up = DroneState(armed=True, rel_alt=3.0, pilot_switch=-1, extnav_ready=True,
                      vins_odom_count=90, vins_last_sim=3.0, now_sim=3.0, flow_seq=2)
    ffls.tick(fctx2, s_up)
    checks.append(("freefly+ff_loiter: селектор вверх → возврат ALT_HOLD (наш стек)",
                   fctx2.kept == "ALT_HOLD"))
    s_man = DroneState(armed=True, rel_alt=3.0, pilot_switch=1, extnav_ready=True,
                       vins_odom_count=95, vins_last_sim=4.0, now_sim=4.0, flow_seq=3)
    ffls._stab_pos = 0                          # пилот был в центре (LOITER)…
    ffls._in_loiter = True
    ffls.tick(fctx2, s_man)
    checks.append(("freefly+ff_loiter: MANUAL-seize (+1) всегда возвращает ALT_HOLD",
                   fctx2.kept == "ALT_HOLD"))

    # --- токен в ГРАДУСАХ (yaw_l/yaw_r) ---
    # Смысл токена: угол — величина, за которую отвечает контур. Держится на двух вещах:
    # (1) длительность считается из общего темпа, (2) команда и добор в ОДНОМ сегменте,
    # иначе enter() следующего обнулит уставку вместе с недоехавшим долгом (замер Y1s:
    # недобор 11.7±1.7°, одинаковый на 30° и 60°).
    yw = compile_mission(cfg, "climb3,yaw_l30,yaw_r60,land", "DpRollHold+DpYawHold")
    l30 = next(st for st in yw if st.name.startswith("yaw_l")).stack.traj
    r60 = next(st for st in yw if st.name.startswith("yaw_r")).stack.traj
    checks.append(("compile: yaw_l → Control с YawTurn", type(l30).__name__ == "YawTurn"))
    # заказанный угол = уровень · темп · длительность команды
    deg_l = l30.level * cfg.yaw_rate_full * l30.turn
    deg_r = r60.level * cfg.yaw_rate_full * r60.turn
    checks.append(("yaw_l30 = 30° ВЛЕВО (c_yaw<0)", abs(deg_l + 30.0) < 1e-6))
    checks.append(("yaw_r60 = 60° ВПРАВО (c_yaw>0)", abs(deg_r - 60.0) < 1e-6))
    # добор внутри сегмента: после команды стик в ноль, но сегмент ещё идёт
    checks.append(("yaw_l30: команда кончилась, сегмент жив (добор)",
                   l30.intent(None, l30.turn + 0.1).c_yaw == 0.0
                   and not l30.done(l30.turn + 0.1)))
    checks.append(("yaw_l30: сегмент = команда + добор",
                   abs(l30.total() - (l30.turn + cfg.yaw_settle)) < 1e-9))
    # ЕДИНЫЙ ТЕМП: гейн команды у обоих холдеров выведен из одного yaw_rate_full,
    # иначе один токен дал бы разный угол под Gz-оракулом и под флоу-демпфером
    import math as _m

    from mission_pkg.recipes import yaw_cmd_gain
    gz = build_stabilizers(cfg, "GzYawHold")[0]
    checks.append(("единый темп: Gz = yaw_rate_full",
                   abs(_m.degrees(gz.yaw_cmd_gain) - cfg.yaw_rate_full) < 1e-6))
    checks.append(("единый темп: Dp = S·yaw_rate_full",
                   abs(yaw_cmd_gain(cfg) / cfg.yaw_flow_scale - cfg.yaw_rate_full) < 1e-6))

    print("Оффлайн-гейт stab×mission:")
    ok_all = True
    for name, ok in checks:
        print(f"  [{'OK ' if ok else 'FAIL'}] {name}")
        ok_all = ok_all and ok
    print("ИТОГ:", "✅ STAB×MISSION OK" if ok_all else "❌ СБОЙ")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
