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
def sh(now, vx=0.0, vy=0.0, ipm_ok=False, ipm_vfwd=0.0, ipm_vlat=0.0):
    return DroneState(vins_valid=True, vins_odom_count=100,
                      vins_last_sim=now, now_sim=now, vins_vx=vx, vins_vy=vy,
                      ipm_ok=ipm_ok, ipm_vfwd=ipm_vfwd, ipm_vlat=ipm_vlat)


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
check("тот же тик ×3: счётчик = 1 (не 3) → ещё sane", hg4._ipm_bad == 1)

# 6. выкл (v_max=0, ipm_tol=0): всегда sane (старое поведение)
hg5 = VinsHandover(VinsHold(), min_count=5, fresh_sec=2.0)
check("гейт выкл: разнос |v|=20 всё равно sane (обратная совместимость)",
      hg5.vins_sane(sh(90.0, vx=20.0)))

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

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ HANDOVER Flow→Vins OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
