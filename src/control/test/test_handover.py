#!/usr/bin/env python3
"""Юнит-тест рантайм switch Flow→Vins (срез: handover). Чистый python, без ROS.

Проверяет: пока VINS не готов — стек держит флоу-стабилизаторы; при «VINS ready»
(N odom + свежесть) ОДНОКРАТНО заменяет их на VinsHold (с захватом vins-опоры);
повторно не срабатывает; после switch стек регулирует по VINS; несвежий поток ≠ ready.

Запуск:  python3 src/control/test/test_handover.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.control_stack import ControlStack       # noqa: E402
from control_pkg.application.handover import VinsHandover            # noqa: E402
from control_pkg.domain.control.excitation import NoExcitation       # noqa: E402
from control_pkg.domain.control.stabilization import (               # noqa: E402
    DpRollHold, VinsHold, DpYawHold)
from control_pkg.domain.control.trajectory import StaticSetpoint     # noqa: E402
from control_pkg.domain.rc import RC_CENTER                          # noqa: E402
from control_pkg.domain.state import DroneState                      # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def s(odom, last_sim, now, vx=0.0, vins_x=0.0):
    return DroneState(gt_valid=True, vins_valid=True, vins_odom_count=odom,
                      vins_last_sim=last_sim, now_sim=now, vins_x=vins_x)


stack = ControlStack([DpRollHold(), DpYawHold()], StaticSetpoint(), NoExcitation())
vins = VinsHold()
ho = VinsHandover(vins, min_count=5, fresh_sec=2.0)

st0 = s(odom=0, last_sim=10.0, now=10.0)
stack.enter(st0)

# 1. VINS не готов (0 odom) → не переключаемся, стек держит флоу
sw = ho.maybe_switch(stack, st0)
check("0 odom → switch НЕ сработал", not sw and not ho.switched)
check("стек держит флоу (2 стабилизатора)", len(stack.stabs) == 2)

# 2. Мало odom (<min) → не готов
check("3 odom (<5) → не готов", not ho.maybe_switch(stack, s(3, 10.5, 10.5)))

# 3. Достаточно odom + свежо → switch РОВНО раз, опора захвачена (vins_x=0).
# Свап меняет ТОЛЬКО оси VinsHold (roll/pitch): yaw-холд переживает свап (4a8fd39) —
# иначе после переключения рыскание замирало в центр и живой пилот терял yaw-стик.
st_ready = s(odom=5, last_sim=11.0, now=11.0, vins_x=0.0)
sw = ho.maybe_switch(stack, st_ready)
check("5 odom + свежо → switch сработал", sw and ho.switched)
check("стек → VinsHold + yaw-холд ПЕРЕЖИЛ свап (Flow-roll снят)",
      any(isinstance(st, VinsHold) for st in stack.stabs)
      and any(isinstance(st, DpYawHold) for st in stack.stabs)
      and not any(isinstance(st, DpRollHold) for st in stack.stabs))
check("VinsHold ПОСЛЕДНИЙ в стеке (в per-axis композиции его оси побеждают)",
      isinstance(stack.stabs[-1], VinsHold))

# 4. Повторно не срабатывает
check("повторный switch не срабатывает", not ho.maybe_switch(stack, s(6, 11.05, 11.05)))

# 5. После switch: продольный дрейф vins-позы от опоры (yaw=0 → pitch) → VinsHold командует
rc = stack.update(s(odom=6, last_sim=11.1, now=11.1, vins_x=1.5))
check("после switch VinsHold регулирует (pitch≠1500 на форвард-дрейфе)", rc.pitch != 1500)

# 6. Свежесть: много odom, но поток протух → НЕ ready
ho2 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0)
stack2 = ControlStack([DpRollHold()], StaticSetpoint(), NoExcitation())
stack2.enter(s(0, 10.0, 10.0))
check("odom есть, но stale (Δ>fresh) → не ready",
      not ho2.maybe_switch(stack2, s(odom=50, last_sim=10.0, now=15.0)))

# 7. КОМПОЗИТ (DpHold/DpHoldM: ОДИН стаб с осями roll+pitch+yaw). Ловушка LV1/LV3:
# «есть yaw» сохранял композит ЦЕЛИКОМ, тот стоял ПОСЛЕ VinsHold и перезаписывал
# roll/pitch — VinsHold обезврежен, борт дрейфовал 1.3 м/с до fence при здоровом
# VINS. Порядок keep+[vins]: композит пишет все оси, VinsHold поверх — roll/pitch.
from control_pkg.domain.control.stabilization import DpHold                # noqa: E402
comp = DpHold()
stack3 = ControlStack([comp], StaticSetpoint(), NoExcitation())
stack3.enter(s(0, 10.0, 10.0))
ho3 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0)
sw3 = ho3.maybe_switch(stack3, s(5, 11.0, 11.0))
check("композит: switch сработал, композит сохранён (yaw жив)",
      sw3 and comp in stack3.stabs)
check("композит: VinsHold ПОСЛЕДНИЙ → его roll/pitch перезаписывают композит",
      isinstance(stack3.stabs[-1], VinsHold))
rc3 = stack3.update(s(odom=6, last_sim=11.1, now=11.1, vins_x=1.5))
check("композит: на форвард-дрейфе vins команда тангажа — от VinsHold (≠1500)",
      rc3.pitch != 1500)

# ============ ГЕЙТ ЗДОРОВЬЯ VINS (авто-демоут яруса 1 при разносе) ============
def sh(now, vx=0.0, vy=0.0, ipm_ok=False, ipm_vfwd=0.0, ipm_vlat=0.0,
       roll=RC_CENTER, pitch=RC_CENTER):
    return DroneState(vins_valid=True, vins_odom_count=100,
                      vins_last_sim=now, now_sim=now, vins_vx=vx, vins_vy=vy,
                      ipm_ok=ipm_ok, ipm_vfwd=ipm_vfwd, ipm_vlat=ipm_vlat,
                      pilot_roll=roll, pilot_pitch=pitch)


hg = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0,
                  v_max=12.0, ipm_tol=4.0, sane_n=3)

# 1. норма: скорость мала, IPM согласен → sane, ready
check("норма (v=1, ipm=1): sane и ready", hg.vins_sane(sh(50.0, vx=1.0,
      ipm_ok=True, ipm_vfwd=1.0)) and hg.vins_ready(sh(50.05, vx=1.0,
      ipm_ok=True, ipm_vfwd=1.0)))

# 2. физ. потолок: |v|=20 > 12 → мусор СРАЗУ (без счётчика)
check("разнос |v|=20 > потолок 12: НЕ sane (сразу)",
      not hg.vins_sane(sh(51.0, vx=20.0, vy=0.0)))
check("разнос: vins_ready = False → авто-демоут",
      not hg.vins_ready(sh(51.05, vx=20.0)))

# 3. IPM-расхождение: |vins_v−ipm_v| = 8 > tol 4, но нужно sane_n=3 подряд
hg2 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0,
                   v_max=12.0, ipm_tol=4.0, sane_n=3)
r1 = hg2.vins_sane(sh(60.0, vx=8.0, ipm_ok=True, ipm_vfwd=0.0))   # bad 1
r2 = hg2.vins_sane(sh(60.1, vx=8.0, ipm_ok=True, ipm_vfwd=0.0))   # bad 2
check("IPM-расхождение 1-2 кадра: ещё sane (защита от шума)", r1 and r2)
r3 = hg2.vins_sane(sh(60.2, vx=8.0, ipm_ok=True, ipm_vfwd=0.0))   # bad 3
check("IPM-расхождение 3 кадра подряд: НЕ sane (демоут)", not r3)

# 4. IPM ослеп в разносе (ipm_ok=False, но v<потолка): физ.потолок не судит,
#    IPM тоже — счётчик не копится (не ложный демоут при слепом канале)
hg3 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0,
                   v_max=12.0, ipm_tol=4.0, sane_n=3)
for i in range(5):
    ok_slep = hg3.vins_sane(sh(70.0 + i * 0.1, vx=5.0, ipm_ok=False))
check("IPM слеп, v под потолком: остаётся sane (чек не судит)", ok_slep)

# 5. счётчик двигается раз на тик (метод зовут оба пути лесенки за один тик)
hg4 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0,
                   v_max=12.0, ipm_tol=4.0, sane_n=3)
st_bad = sh(80.0, vx=8.0, ipm_ok=True, ipm_vfwd=0.0)
hg4.vins_sane(st_bad); hg4.vins_sane(st_bad); hg4.vins_sane(st_bad)  # тот же тик ×3
check("тот же тик ×3: счётчик = 1 (не 3) → ещё sane", hg4._bad == 1)

# 6. выкл (v_max=0, ipm_tol=0): всегда sane (старое поведение)
hg5 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0)
check("гейт выкл: разнос |v|=20 всё равно sane (обратная совместимость)",
      hg5.vins_sane(sh(90.0, vx=20.0)))

# 6б. ФИЗИКА ВИСЕНИЯ: центральные стики дольше hover_sec + |vins_v|>hover_v = разнос
# (ловит МЕДЛЕННЫЙ разнос до потолка 12). off=выкл; DZ стика = 40 PWM.
hgh = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0,
                   hover_v=3.0, hover_sec=2.0, sane_n=3)
# висим (стики центр), |vins_v|=5 > hover_v 3, но ещё < hover_sec → sane
check("висение <hover_sec, v=5: ещё sane (транзиент стопа)",
      hgh.vins_sane(sh(200.0, vx=5.0)))
hgh.vins_sane(sh(202.5, vx=5.0))                     # >2с центра → чек активен, bad 1
hgh.vins_sane(sh(202.6, vx=5.0))                     # bad 2
check("висение >hover_sec, v=5, 3 кадра: НЕ sane (медленный разнос)",
      not hgh.vins_sane(sh(202.7, vx=5.0)))
# ВАЖНО: на быстрой прямой (стик отклонён) чек ВЫКЛ — v=5 при активном стике sane
hgh2 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0,
                    hover_v=3.0, hover_sec=2.0, sane_n=3)
for i in range(60):                                  # 3с полного стика вперёд, v=5
    ok_fly = hgh2.vins_sane(sh(300.0 + i * 0.05, vx=5.0, pitch=RC_CENTER - 400))
check("стик активен (прямая), v=5: sane (чек висения выключен)", ok_fly)
# ветровой снос на висении < hover_v: sane
hgh3 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0,
                    hover_v=3.0, hover_sec=2.0, sane_n=3)
for i in range(60):                                  # 3с висения, снос 1.2 м/с (ветер)
    ok_wind = hgh3.vins_sane(sh(400.0 + i * 0.05, vx=1.2))
check("висение, ветровой снос 1.2 < hover_v 3: sane", ok_wind)

# 7. ЗАПРОС /restart на фронте sane→insane (восстановление после разноса)
hg6 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0, v_max=12.0)
hg6.vins_sane(sh(100.0, vx=1.0))                     # sane
check("пока sane: рестарт не заказан", not hg6.pop_restart_request())
hg6.vins_sane(sh(100.1, vx=20.0))                    # sane→insane фронт
check("фронт sane→insane: /restart заказан", hg6.pop_restart_request())
check("pop одноразовый: второй раз пусто", not hg6.pop_restart_request())
hg6.vins_sane(sh(100.2, vx=20.0))                    # всё ещё insane (не фронт)
check("insane продолжается (не фронт): рестарт НЕ повторяется",
      not hg6.pop_restart_request())

# ============ ЧЕК ЗАНИЖЕНИЯ |vins_v| против IPM (коллапс масштаба) ============
# lv2_joy_20260905_114248: реборн-VINS с масштабом 0.14 видел 0.4–0.9 при истинных
# 3–5.5, IPM годен и видел 5.0 — потолок и физика висения такое не ловят.
def ssc(now, vx=0.0, ipm_v=0.0, ipm_ok=True, alt=1.5, roll=RC_CENTER, pitch=RC_CENTER):
    return DroneState(vins_valid=True, vins_odom_count=300, vins_last_sim=now,
                      now_sim=now, vins_vx=vx, vins_vy=0.0, ipm_ok=ipm_ok,
                      ipm_vfwd=0.0, ipm_vlat=ipm_v, perc_alt=alt, rel_alt=alt,
                      pilot_roll=roll, pilot_pitch=pitch)


def mk_scale(**kw):
    args = dict(min_count=5, fresh_sec=2.0, hover_v=3.0, hover_sec=2.0, sane_n=10,
                scale_ratio=0.5, scale_ipm_min=2.0, scale_sec=3.0, scale_alt_max=4.0,
                scale_hold=30.0)
    args.update(kw)
    return VinsHandover(VinsHold(), **args)


def run(h, t0, dur, dt=0.05, **kw):
    """Кормит гейт dur секунд снапшотами ssc(**kw); возвращает (последний sane, t)."""
    t, ok = t0, True
    for _ in range(int(round(dur / dt))):
        t += dt
        ok = h.vins_sane(ssc(t, **kw))
    return ok, t


# 1. картина 114248: висение низко, IPM 5 м/с стойко, VINS 0.7 → не sane, латч.
# Таймеры с первого тика: висение 2 с, стойкость IPM 3 с (опорник должен быть годен
# непрерывно), затем условие занижения 3 с подряд → фронт на ~6 с.
hs = mk_scale()
ok, t = run(hs, 500.0, 2.5, vx=0.7, ipm_v=5.0)          # висение взведено, IPM ещё не стоек
check("занижение: IPM годен <3 с — ещё sane (опорник не стоек)", ok)
ok, t = run(hs, t, 3.3, vx=0.7, ipm_v=5.0)               # IPM стоек с 3 с, условие 2.8 с
check("занижение: условие <3 с — ещё sane", ok and hs.scale_trips == 0)
ok, t = run(hs, t, 0.4, vx=0.7, ipm_v=5.0)               # условие >3 с
check("занижение 3 с подряд: НЕ sane (VINS 0.7 при IPM 5)", not ok and hs.scale_trips == 1)
ok, t = run(hs, t, 5.0, vx=0.0, ipm_v=0.0)               # демпфер остановил борт
check("латч: борт встал, IPM 0 — всё ещё НЕ sane (масштаб сам не починится)", not ok)
ok, t = run(hs, t, 26.0, vx=0.0, ipm_v=0.0)              # > scale_hold 30 с
check("латч истёк (30 с): снова sane, срабатывание одно", ok and hs.scale_trips == 1)

# 2. на высоте IPM не опорник: те же скорости на 15 м → sane
hs2 = mk_scale()
ok, _ = run(hs2, 600.0, 6.0, vx=0.7, ipm_v=5.0, alt=15.0)
check("высота 15 м (> alt_max 4): чек молчит", ok and hs2.scale_trips == 0)

# 3. живой стик: чек выключен (быстрая прямая — IPM сам мусорит)
hs3 = mk_scale()
ok, _ = run(hs3, 700.0, 6.0, vx=0.7, ipm_v=5.0, pitch=RC_CENTER - 300)
check("стик активен: чек молчит", ok and hs3.scale_trips == 0)

# 4. IPM мигает (ok<3 с подряд): не опорник
hs4 = mk_scale()
t = 800.0
ok = True
for i in range(120):                                      # 6 с, каждые 2 с брак кадра
    t += 0.05
    ok = hs4.vins_sane(ssc(t, vx=0.7, ipm_v=5.0, ipm_ok=(i % 40 != 0)))
check("IPM годен не дольше 2 с подряд: чек молчит", ok and hs4.scale_trips == 0)

# 5. согласие датчиков (IPM 1.5, VINS 1.2 — ветровой снос): sane
hs5 = mk_scale()
ok, _ = run(hs5, 900.0, 6.0, vx=1.2, ipm_v=1.5)
check("IPM 1.5 (< ipm_min 2): чек молчит", ok and hs5.scale_trips == 0)
hs5b = mk_scale(hover_v=0.0)
ok, _ = run(hs5b, 950.0, 6.0, vx=4.5, ipm_v=5.0)
check("IPM 5, VINS 4.5 (согласны): чек занижения молчит", ok and hs5b.scale_trips == 0)

# 6. перерождение VINS снимает латч (новая рама = новый масштаб)
hs6 = mk_scale()
ok, t = run(hs6, 1000.0, 6.5, vx=0.7, ipm_v=5.0)
check("подготовка: латч взведён", not ok and hs6.scale_trips == 1)
hs6.note_vins_restart()
ok, _ = run(hs6, t, 0.5, vx=0.0, ipm_v=0.0)
check("note_vins_restart: латч снят, sane", ok)

# 7. выкл (ratio 0): картина 114248 остаётся sane (старое поведение)
hs7 = mk_scale(scale_ratio=0.0)
ok, _ = run(hs7, 1100.0, 6.0, vx=0.7, ipm_v=5.0)
check("scale_ratio=0: чек выключен", ok and hs7.scale_trips == 0)

# ============ ПОСЕВ ТРИМА от демпфера на входе в ярус 1 (vins_stabs) ============
# Установившийся И-член станции (валюта PWM каналов, DpHold.trim_pwm) сеется в
# DpVins.seed_trim — ветер, который демпфер уже держит, не учится заново.
from control_pkg.domain.control.vins_axes import DpVins                    # noqa: E402

comp8 = DpHold()
for x in comp8._subs:                      # рукотворный установившийся трим осей
    if getattr(x, "_axis", None) == "pitch":
        x._i = -40.0
    elif getattr(x, "_axis", None) == "roll":
        x._i = 10.0
check("DpHold.trim_pwm: каналы (osign·И-член) = (−40, 10)",
      comp8.trim_pwm() == (-40.0, 10.0))

dpv = DpVins(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0, imax=120.0)
ho8 = VinsHandover(dpv, min_count=5, fresh_sec=2.0)      # trim_seed дефолт ВКЛ
ho8.vins_stabs([comp8], s(5, 11.0, 11.0))
check("vins_stabs: трим DpVins посеян из демпфера (мир, yaw=0)",
      abs(dpv._itx + 40.0) < 1e-9 and abs(dpv._ity - 10.0) < 1e-9)
check("посев ВЗВОДИТ «ветер выучен» → дальше рабочий ki, не ki_trim (вход на ходу: "
      "ki_trim 60 за секунду переписывал посев скоростью возврата, cmd_3/wind_right)",
      dpv._trim_armed)

# начатое обучение не перетирается (дребезг гейта: трим уже нажит, trim_keep)
dpv2 = DpVins(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0, imax=120.0)
dpv2._itx = 5.0
VinsHandover(dpv2, min_count=5, fresh_sec=2.0).vins_stabs([comp8], s(5, 11.0, 11.0))
check("трим не девственный → посев отказал (своё свежее)", dpv2._itx == 5.0)

# ручка: trim_seed=False — посева нет (учить с нуля, старое поведение)
dpv3 = DpVins(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0, imax=120.0)
VinsHandover(dpv3, min_count=5, fresh_sec=2.0,
             trim_seed=False).vins_stabs([comp8], s(5, 11.0, 11.0))
check("trim_seed=False: трим остался нулевым", dpv3._itx == 0.0 and dpv3._ity == 0.0)

# /restart: сброс старой рамы, затем посев СВЕЖЕЙ — в одном входе в ярус
dpv4 = DpVins(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0, imax=120.0)
dpv4._itx, dpv4._trim_armed = 50.0, True
ho11 = VinsHandover(dpv4, min_count=5, fresh_sec=2.0)
ho11.note_vins_restart()
ho11.vins_stabs([comp8], s(5, 11.0, 11.0))
check("после /restart: сброс старого трима, посев в свежую раму (armed от посева)",
      abs(dpv4._itx + 40.0) < 1e-9 and dpv4._trim_armed)

# VinsHold посева не имеет — vins_stabs не падает
VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0).vins_stabs(
    [comp8], s(5, 11.0, 11.0))
check("VinsHold (без seed_trim): vins_stabs не падает", True)

# seed_vins напрямую (вход в ярус 2 LOITER МИНУЯ vins_stabs): трим DpVins
# сеется, DpVins в стек НЕ идёт — на нём стоит стрелка ветра HUD в LOITER.
# Это и есть «прыжок 0→2»: без явного посева трим был бы девственным.
dpv12 = DpVins(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0, imax=120.0)
ho12 = VinsHandover(dpv12, min_count=5, fresh_sec=2.0)
ho12.seed_vins([comp8], s(5, 11.0, 11.0))
check("seed_vins (вход 0→2 в LOITER): трим DpVins посеян как в vins_stabs",
      abs(dpv12._itx + 40.0) < 1e-9 and abs(dpv12._ity - 10.0) < 1e-9)
check("seed_vins идемпотентен: повторный вызов не перетирает (уже нажит)",
      ho12.seed_vins([comp8], s(5, 11.0, 11.0)) is None
      and abs(dpv12._itx + 40.0) < 1e-9)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ HANDOVER Flow→Vins OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
