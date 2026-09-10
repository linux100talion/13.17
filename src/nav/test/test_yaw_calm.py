#!/usr/bin/env python3
"""Юнит-тест «курс спокоен» (nav_pkg/nn1/yaw_calm.py). Чистый python.

Угол рамы латчится по одной мгновенной паре курсов, у которых разная задержка: на
вираже их разность = ω·лаг, а не поворот кадра. Реплей yawab_check5 схватил раму в
размахе рыскания и заморозил 29.4° лишних — EKF ушёл на 1959 м. Полёт пилота 203257
латчился на КУРСЕ 90° и дал 5.6° и дрейф 0.08 м: мешает вращение, а не развёрнутость.

Запуск:  python3 src/nav/test/test_yaw_calm.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from nav_pkg.nn1.yaw_calm import YawCalm          # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def feed(c, t0, dur, wz_deg, dt=0.05, yaw0=0.0):
    """Крутим со скоростью wz_deg °/с; возвращает (последний вердикт, t, yaw)."""
    t, y, out = t0, yaw0, False
    n = int(round(dur / dt))
    for _ in range(n):
        t += dt
        y += math.radians(wz_deg) * dt
        out = c.update(t, y)
    return out, t, y


# 1. первый отсчёт судить не по чему
c = YawCalm(wz_max=15.0, still=0.5)
check("первый отсчёт — не спокоен", c.update(100.0, 0.0) is False)

# 2. стоим на месте: через still становится спокойно, раньше — нет
c = YawCalm(wz_max=15.0, still=0.5)
out, t, y = feed(c, 100.0, 0.4, 0.0)
check("0.4 с покоя из 0.5 — ещё не спокоен", out is False)
out, t, y = feed(c, t, 0.3, 0.0, yaw0=y)
check("прошло больше still — спокоен", out is True)

# 3. РАЗВЁРНУТОСТЬ НЕ МЕШАЕТ: развернулись на 90° и встали (полёт 203257)
c = YawCalm(wz_max=15.0, still=0.5)
out, t, y = feed(c, 200.0, 3.0, 30.0)                  # разворот 30 °/с
check("в развороте 30 °/с — НЕ спокоен", out is False)
check("накрутили ~90°", abs(math.degrees(y) - 90.0) < 2.0)
out, t, y = feed(c, t, 1.0, 0.0, yaw0=y)
check("встали на курсе 90° — спокоен (мешает вращение, а не разворот)", out is True)

# 4. мелкое рыскание реплея: непрерывные взмахи не дают спокойного окна
c = YawCalm(wz_max=15.0, still=0.5)
t, y, out = 300.0, 0.0, None
for i in range(200):                                    # 10 с качания ±, 40 °/с
    t += 0.05
    y += math.radians(40.0 * (1 if (i // 4) % 2 else -1)) * 0.05
    out = c.update(t, y)
check("непрерывное рыскание 40 °/с — спокойного окна нет", out is False)

# 5. порог: 10 °/с ниже потолка 15 — спокойно, 20 выше — нет
c = YawCalm(wz_max=15.0, still=0.5)
out, _, _ = feed(c, 400.0, 2.0, 10.0)
check("10 °/с при потолке 15 — спокоен", out is True)
c = YawCalm(wz_max=15.0, still=0.5)
out, _, _ = feed(c, 500.0, 2.0, 20.0)
check("20 °/с при потолке 15 — не спокоен", out is False)

# 6. обёртка через ±180: переход не считается гигантской скоростью
c = YawCalm(wz_max=15.0, still=0.2)
c.update(600.0, math.radians(179.9))
c.update(600.05, math.radians(-179.9))
check("через ±180: |ω| мала, а не 7000 °/с", c.wz < 10.0)

# 7. reset()
c = YawCalm(wz_max=15.0, still=0.5)
feed(c, 700.0, 2.0, 0.0)
c.reset()
check("после reset первый отсчёт снова не спокоен", c.update(800.0, 0.0) is False)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ YAW CALM OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
