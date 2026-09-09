#!/usr/bin/env python3
"""Юнит-тест буфера поз EKF (nav_pkg/nn1/pose_buffer.py). Чистый python.

Якорь спаривал позы ПО ПРИХОДУ: свежая поза EKF против одометрии VINS, которая
приходит на 0.14–0.22 с позже (замер 181233/181936 после снятия расхождения темпа
sim/wall). На 3–5 м/с это 0.45–0.75 м мнимого расхода — он ел порог жёсткой подтяжки
и систематически тянул якорь назад по курсу, а в 173415 (подвис VINS 1.96 с на
развороте) в раму защёлкнулись 20°. Буфер отдаёт позу НА МОМЕНТ ШТАМПА кадра.

Запуск:  python3 src/nav/test/test_pose_buffer.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from nav_pkg.nn1.pose_buffer import PoseBuffer          # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


# 1. пустой буфер честно молчит
b = PoseBuffer()
check("пустой: at() → None, ready() → False", b.at(1.0) is None and not b.ready(1.0))

# 2. борт летит 5 м/с по x, отсчёты 30 Гц — выборка ровно между отсчётами
b = PoseBuffer(keep_sec=3.0)
for i in range(90):
    t = 100.0 + i / 30.0
    b.push(t, 5.0 * (t - 100.0), 0.0, 2.0, 0.0)
x, y, z, yaw = b.at(100.5)
check("интерполяция между отсчётами: x(0.5 с) = 2.5 м", abs(x - 2.5) < 1e-9)
x2, *_ = b.at(100.5 + 1.0 / 60.0)          # ровно посередине шага
check("середина шага 1/60 с: x = 2.5833 м", abs(x2 - (2.5 + 5.0 / 60.0)) < 1e-9)

# 3. ЦЕНА СТАРОГО ПОВЕДЕНИЯ: «последняя пришедшая» против «на момент штампа».
# Кадр VINS штампован 0.15 с назад — старый код брал бы свежую позу EKF
th = b.newest() - 0.15
xs, *_ = b.at(th)
xl, *_ = b.at(b.newest())
check("лаг 0.15 с при 5 м/с = 0.75 м разъезда пары", abs((xl - xs) - 0.75) < 1e-6)

# 4. глубина кольца: старое выбрасывается, запрос за краем — удержание
check("кольцо держит keep_sec", b.oldest() >= b.newest() - 3.0 - 1e-9)
xo, *_ = b.at(b.oldest() - 10.0)
xo2, *_ = b.at(b.oldest())
check("запрос до начала — удержание первого отсчёта", abs(xo - xo2) < 1e-12)
check("ready(): момент внутри кольца — да, до начала — нет",
      b.ready(b.newest() - 1.0) and not b.ready(b.oldest() - 0.1))

# 5. курс интерполируется ЧЕРЕЗ ОБЁРТКУ ±180 (между +179 и −179 один градус)
b2 = PoseBuffer()
b2.push(10.0, 0.0, 0.0, 0.0, math.radians(179.5))
b2.push(10.1, 0.0, 0.0, 0.0, math.radians(-179.5))
*_, yaw = b2.at(10.05)
check("yaw через ±180: середина = ±180°, а не 0°", abs(abs(math.degrees(yaw)) - 180.0) < 1e-6)

# 6. штамп прыгнул назад (перезапуск часов) — кольцо начинается заново, поиск не врёт
b3 = PoseBuffer()
for i in range(10):
    b3.push(500.0 + i * 0.1, i, 0.0, 0.0, 0.0)
b3.push(20.0, 42.0, 0.0, 0.0, 0.0)
check("штамп назад: кольцо сброшено", b3.oldest() == 20.0 and len(b3.buf) == 1)
x3, *_ = b3.at(20.0)
check("после сброса отдаётся новый отсчёт", abs(x3 - 42.0) < 1e-12)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ POSE BUFFER OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
