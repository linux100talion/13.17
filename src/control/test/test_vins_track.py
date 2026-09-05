#!/usr/bin/env python3
"""Оффлайн-тест VinsTrack — скорость VINS по ШТАМПАМ + детект перерождения потока.

Сценарии из bag lv2_joy_20260905_114248 (разбор control.md «Гейт здоровья»):
догоняющая пачка одометрии после стопора эстиматора (старый расчёт по времени
прихода раздувал |v| вдвое → ложный демоут + /restart), перерождение по дыре
штампов и по скачку позы, сброс потока нодой. Чистый python, без ROS.

Запуск:  python3 src/control/test/test_vins_track.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from control_pkg.application.vins_track import VinsTrack   # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


# --- 1. равномерное движение 1 м/с по x, штампы 10 Гц: скорость сходится к 1 ---
tr = VinsTrack()
reb = False
for i in range(30):
    reb |= tr.on_odom(10.0 + 0.1 * i, 1.0 * 0.1 * i, 0.0)
check("равномерное 1 м/с: |v| → 1.0 (EMA сошлась)", abs(tr.vx - 1.0) < 1e-3 and abs(tr.vy) < 1e-9)
check("равномерное движение: перерождения нет", not reb)

# --- 2. ЗАПИСЬ BAG 20260905_114248, стопор 92.8→94.4: штампы ровно через 0.10,
# приход — пачкой (14 сообщений за 0.7 с). Позы/штампы/приход из odometry.csv.
stamps = [92.50, 92.60, 92.70, 92.80, 92.90, 93.00, 93.06, 93.16, 93.26, 93.36, 93.46,
          93.56, 93.66, 93.76, 93.86, 93.92, 94.02, 94.12, 94.22]
arriv = [0.00, 0.08, 0.22, 0.32, 1.84, 2.03, 2.06, 2.08, 2.12, 2.17, 2.21,
         2.25, 2.29, 2.33, 2.37, 2.42, 2.47, 2.51, 2.55]
pos = [(-0.54, -3.53), (-0.55, -3.45), (-0.57, -3.36), (-0.58, -3.27), (-0.60, -3.17),
       (-0.61, -3.07), (-0.63, -3.00), (-0.65, -2.90), (-0.67, -2.79), (-0.69, -2.68),
       (-0.70, -2.56), (-0.72, -2.44), (-0.74, -2.31), (-0.77, -2.18), (-0.79, -2.05),
       (-0.80, -1.97), (-0.82, -1.83), (-0.84, -1.69), (-0.87, -1.55)]
tr = VinsTrack()
vmax_stamp, reb = 0.0, False
for t, (x, y) in zip(stamps, pos):
    reb |= tr.on_odom(t, x, y)
    vmax_stamp = max(vmax_stamp, math.hypot(tr.vx, tr.vy))
# старый расчёт: dt по времени ПРИХОДА (как было в RosTelemetry до правки)
vx = vy = 0.0
vmax_arrival = 0.0
for i in range(1, len(pos)):
    dt = arriv[i] - arriv[i - 1]
    if dt <= 0:
        continue
    vx = 0.6 * vx + 0.4 * (pos[i][0] - pos[i - 1][0]) / dt
    vy = 0.6 * vy + 0.4 * (pos[i][1] - pos[i - 1][1]) / dt
    vmax_arrival = max(vmax_arrival, math.hypot(vx, vy))
print(f"      пачка из bag: |v|max по штампам {vmax_stamp:.2f}, по приходу {vmax_arrival:.2f} "
      f"(twist bag ≤1.47, истина ≤1.7, порог гейта 3.0)")
check("пачка из bag: по штампам |v|max < 1.6 (честная скорость)", vmax_stamp < 1.6)
check("пачка из bag: по времени прихода |v|max > 3.0 (артефакт, ронявший ярус)",
      vmax_arrival > 3.0)
check("стопор эстиматора — НЕ перерождение (штампы регулярны)", not reb)

# --- 3. перерождение по дыре штампов: молчание 2.1 с, новая рама из нуля ---
tr = VinsTrack()
for i in range(10):
    tr.on_odom(90.0 + 0.1 * i, -0.8 + 0.02 * i, -1.5 + 0.1 * i)
v_before = math.hypot(tr.vx, tr.vy)
reb = tr.on_odom(90.9 + 2.11, -0.01, 0.09)          # как в bag: 94.22 → 96.33
check("дыра штампов 2.11 с: перерождение", reb)
check("перерождение: скорость с нуля (скачок позы не попал в EMA)",
      tr.vx == 0.0 and tr.vy == 0.0 and v_before > 0.5)
reb2 = tr.on_odom(93.11, -0.02, 0.12)
check("следующий отсчёт новой рамы: обычный, не перерождение", not reb2)

# --- 4. перерождение по скачку позы без дыры (быстрая переинициализация) ---
tr = VinsTrack()
for i in range(10):
    tr.on_odom(50.0 + 0.1 * i, 3.0, 4.0 + 0.05 * i)
reb = tr.on_odom(51.0, 0.0, 0.0)                    # 5 м за 0.1 с = 50 м/с
check("скачок 5 м за 0.1 с (>12 м/с): перерождение", reb)
check("скачок: скорость с нуля", tr.vx == 0.0 and tr.vy == 0.0)

# --- 5. честный быстрый полёт 4 м/с — не перерождение ---
tr = VinsTrack()
reb = False
for i in range(30):
    reb |= tr.on_odom(60.0 + 0.1 * i, 4.0 * 0.1 * i, 0.0)
check("полёт 4 м/с: перерождения нет, |v| ≈ 4", not reb and abs(tr.vx - 4.0) < 1e-2)

# --- 6. reset (нода послала /restart): следующий отсчёт — первый, без флага ---
tr = VinsTrack()
for i in range(5):
    tr.on_odom(70.0 + 0.1 * i, 1.0 * i, 0.0)
tr.reset()
check("reset: скорость обнулена", tr.vx == 0.0 and tr.vy == 0.0)
check("после reset первый отсчёт новой рамы — не перерождение (учтено нодой)",
      not tr.on_odom(72.5, 0.0, 0.0))
check("после reset второй отсчёт: скорость новой рамы",
      not tr.on_odom(72.6, 0.1, 0.0) and abs(tr.vx - 0.4) < 1e-9)

# --- 7. дубликат/беспорядок штампов — пропуск без деления на ноль ---
tr = VinsTrack()
tr.on_odom(80.0, 0.0, 0.0)
tr.on_odom(80.1, 0.1, 0.0)
v = tr.vx
check("дубликат штампа: пропуск, скорость не тронута",
      not tr.on_odom(80.1, 5.0, 5.0) and tr.vx == v)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ VINS TRACK OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
