#!/usr/bin/env python3
"""Юнит-тест AttitudeBuffer — ориентация на штамп кадра интерполяцией между отсчётами.

Зачем. Шум пути канала вида сверху растёт с высотой только из-за тайминга углов:
реплей ab_soft с истинными углами — 11 мм/кадр на 17.5 м, с углами «последнее
пришедшее ATTITUDE» (ступенька ~15–25 Гц) — 200–460 мм; интерполяция между двумя
штампованными отсчётами — уровень задержки 20 мс (~70 мм). Здесь — сама арифметика
и правила готовности/удержания; очередь кадров в RosPerception — тонкая обёртка.
Запуск:  python3 src/control/test/test_att_interp.py
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control_pkg.perception.attitude_buffer import AttitudeBuffer    # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


b = AttitudeBuffer(keep_sec=6.0)
check("пустой буфер: at → None, ready False", b.at(1.0) is None and not b.ready(1.0))
# линейный рост тангажа 10°/с при 12.5 Гц отсчётах — интерполяция точна
for k in range(13):
    t = k * 0.08
    b.push(t, 0.1745 * t, -0.05 * t)
p, r = b.at(0.5)
check(f"линейный тангаж: at(0.5) = {p:.4f} ≈ 0.0873 (точно), крен {r:.4f} ≈ −0.025",
      abs(p - 0.1745 * 0.5) < 1e-9 and abs(r + 0.025) < 1e-9)
check("ready: штамп ≤ последнего отсчёта (0.96) → True, позже → False",
      b.ready(0.96) and not b.ready(0.97))
p, r = b.at(2.0)
check("за концом буфера — удержание последнего (как было)", abs(p - 0.1745 * 0.96) < 1e-9)
p, r = b.at(-1.0)
check("до начала — первый отсчёт", p == 0.0 and r == 0.0)
# ступенька против интерполяции: ошибка «последнего пришедшего» посреди интервала
held = b.buf[6][1]                       # отсчёт в 0.48
true = 0.1745 * 0.52
inter, _ = b.at(0.52)
check(f"посреди интервала: ступенька ошибается на {abs(held - true) * 1000:.1f} мрад, "
      f"интерполяция — на {abs(inter - true) * 1e6:.0f} мкрад", abs(inter - true) < 1e-9
      and abs(held - true) > 5e-3)
# окно хранения: старое выкидывается
b2 = AttitudeBuffer(keep_sec=1.0)
for k in range(50):
    b2.push(k * 0.1, 0.0, 0.0)
check(f"окно 1 с: в буфере {len(b2.buf)} отсчётов (≤ 12)", len(b2.buf) <= 12
      and b2.buf[0][0] >= 4.9 - 1.0 - 1e-9)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ATTITUDE INTERP OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
