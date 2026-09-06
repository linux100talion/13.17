#!/usr/bin/env python3
"""Юнит-тест гейта здоровья моста VINS→EKF (nav_pkg/nn1/bridge_gate.py). Чистый python.

Полёт lv2_joy_20260906_142811: разнос VINS после init (|v| → 49 м/с, скачки позы) →
687 подтяжек якоря → отравленная ориентация EKF → DpHold унесло. Проверяет: здоровый
поток — открыт; потолок скорости закрывает с латчем hold; скачок позы / дыра штампов =
перерождение → закрыт + флаг «якорь заново»; шторм подтяжек закрывает; внешний вердикт
ноды закрывает; после hold и здоровом потоке — открыт; reset() по /restart.

Запуск:  python3 src/nav/test/test_bridge_gate.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from nav_pkg.nn1.bridge_gate import BridgeGate      # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def stream(g, t0, n, v=0.5, dt=0.1, x0=0.0, ext=None):
    """n одометрий здорового потока: ход v м/с по x."""
    t, x = t0, x0
    op = None
    for i in range(n):
        op = g.on_odom(t, x, 0.0, v, ext)
        t += dt; x += v * dt
    return op, t, x


# 1. здоровый поток — открыт, латч заново не просится
g = BridgeGate()
op, t, x = stream(g, 100.0, 50)
check("здоровый поток 5 с: открыт, closes 0, relatch_pending нет",
      op and g.closes == 0 and not g.relatch_pending)
# 2. потолок скорости → закрыт, держится hold 5 с, потом открыт
op = g.on_odom(t, x, 0.0, 15.0); t += 0.1
check("|twist| 15 > 12: закрыт, причина vNN", (not op) and g.reason.startswith('v') and g.closes == 1)
op, t, x = stream(g, t, 30, x0=x)                   # 3 с здорового — ещё закрыт
check("через 3 с здорового потока — ещё закрыт (латч hold 5 с)", not g.is_open(t - 0.1))
op, t, x = stream(g, t, 30, x0=x)                   # ещё 3 с — открыт
check("через 6 с — открыт, closes по-прежнему 1", op and g.closes == 1)
check("после потолка скорости якорь заново НЕ просится (рама та же)", not g.relatch_pending)
# 3. скачок позы (перерождение) → закрыт + якорь заново
op = g.on_odom(t, x + 5.0, 0.0, 0.5); t += 0.1     # 5 м за 0.1 с = 50 м/с
check("скачок позы 50 м/с: закрыт, reborn, rebirths 1, якорь заново",
      (not op) and g.reason == 'reborn' and g.rebirths == 1 and g.take_relatch())
check("take_relatch снимает флаг", not g.take_relatch())
# 4. дыра штампов > 1 с — тоже перерождение
g2 = BridgeGate(); stream(g2, 10.0, 10)
op = g2.on_odom(12.5, 0.5, 0.0, 0.3)
check("дыра штампов 1.5 с: перерождение", (not op) and g2.rebirths == 1 and g2.relatch_pending)
# 5. шторм подтяжек: 3 за 5 с → закрыт, якорь заново; 2 за 5 с — нет
g3 = BridgeGate(relatch_n=3, relatch_win=5.0); _, t3, x3 = stream(g3, 20.0, 10)
a = g3.on_relatch(t3); b = g3.on_relatch(t3 + 1.0)
check("2 подтяжки за 5 с: открыт", (not a) and (not b) and g3.is_open(t3 + 1.0))
c = g3.on_relatch(t3 + 2.0)
check("3-я подтяжка за 5 с: шторм — закрыт, причина relatch, якорь заново",
      c and (not g3.is_open(t3 + 2.0)) and g3.reason == 'relatch' and g3.relatch_pending)
g4 = BridgeGate(relatch_n=3, relatch_win=5.0); _, t4, _ = stream(g4, 30.0, 10)
g4.on_relatch(t4); g4.on_relatch(t4 + 3.0); d = g4.on_relatch(t4 + 6.0)
check("3 подтяжки, но растянуты на 6 с (окно 5): не шторм", (not d) and g4.is_open(t4 + 6.0))
# 6. внешний вердикт ноды
g5 = BridgeGate(); _, t5, x5 = stream(g5, 40.0, 10)
op = g5.on_odom(t5, x5, 0.0, 0.5, ext_sane=False)
check("/vins/sane=False: закрыт, причина ext", (not op) and g5.reason == 'ext')
op, t5, x5 = stream(g5, t5 + 0.1, 60, x0=x5, ext=True)
check("после hold с sane=True: открыт", op)
# 7. reset() по /restart: следующая одометрия не сравнивается с прошлой (нет ложного
#    перерождения при скачке рамы), якорь заново
g6 = BridgeGate(); _, t6, x6 = stream(g6, 50.0, 10)
g6.reset()
op = g6.on_odom(t6, x6 + 100.0, 0.0, 0.3)
check("reset(): скачок рамы после /restart — не перерождение, открыт, якорь заново",
      op and g6.rebirths == 0 and g6.take_relatch())
# 8. state_line формат
g7 = BridgeGate(); _, t7, _ = stream(g7, 60.0, 5)
w = g7.state_line(t7).split()
check("state_line: 'open - 0 0 0'", w == ['open', '-', '0', '0', '0'])

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ BRIDGE GATE OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
