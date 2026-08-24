#!/usr/bin/env python3
"""Оффлайн-тест FrameAnchor (якорь кадра VINS→EKF: рысканье + трансляция).

Урок прогонов lv1_joy_20260824_212409/213830: спавн с курсом −169° повернул мир
VINS на ~170° к ENU, трансляционный якорь кормил EKF почти перевёрнутыми
смещениями → положительная ОС LOITER → разнос до 15 м/с при идеальном масштабе
VINS. Проверяем: латч Δyaw выпрямляет кадр; при нулевом Δyaw поведение побитово
прежнее (трансляция); жёсткая подтяжка перелатчивает И поворот; мягкий дожим
тянет только трансляцию; засечка NN1 правит только трансляцию; поворот
ориентации и world-скорости согласован с позицией. Чистый python, без ROS.

Запуск:  python3 src/nav/test/test_frame_anchor.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from nav_pkg.nn1.frame_anchor import FrameAnchor, quat_yaw   # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def rz(a, p):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]])


def yaw_quat(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


# --- 0. quat_yaw: чистый Rz и обёртка углов -------------------------------
for a in (-2.95, -0.5, 0.0, 1.2, 3.0):
    q = yaw_quat(a)
    if abs(quat_yaw(*q) - a) > 1e-9:
        check(f"quat_yaw({a}) точен", False)
        break
else:
    check("quat_yaw: чистый Rz восстанавливается точно", True)

# --- 1. СЦЕНАРИЙ ПОЛЁТА: мир VINS повёрнут на 170°, якорь выпрямляет ------
# истинный трек (кадр EKF/ENU); VINS видит его в кадре, повёрнутом на −170°
# (т.е. трек VINS = Rz(+170°)·истина, как в замере по bag), плюс свой сдвиг
THETA = math.radians(170.0)
true_track = [np.array([0.0, 0.0, 3.0]), np.array([2.0, 1.0, 3.2]),
              np.array([5.0, -2.0, 3.1]), np.array([-3.0, 4.0, 2.9])]
v0 = np.array([0.7, -0.3, 0.1])            # произвольное начало мира VINS
vins_track = [rz(THETA, p - true_track[0]) + v0 for p in true_track]
true_yaw = -0.4                            # курс борта в ENU (любой)
vins_yaw = true_yaw + THETA                # тот же борт в кадре VINS

an = FrameAnchor(relatch_m=1.0, tau_sec=5.0)
ev = an.update(vins_track[0], vins_yaw, true_track[0], true_yaw, now=100.0)
check("латч на первой паре поз", ev == 'latch' and an.latched)
check("Δyaw = −170° (выпрямление кадра)",
      abs(math.degrees(an.yaw_off) + 170.0) < 1e-6)
errs = [np.linalg.norm(an.map(v) - t) for v, t in zip(vins_track, true_track)]
check("map() воспроизводит ИСТИННЫЙ трек по всем точкам (< 1 мм)",
      max(errs) < 1e-3)
# без поворота тот же трек развалился бы на метры — сам смысл фикса
bad = max(np.linalg.norm((v - vins_track[0] + true_track[0]) - t)
          for v, t in zip(vins_track, true_track))
check("контроль: трансляционный якорь на этом треке врёт на метры", bad > 5.0)

# ориентация: повёрнутый кватернион возвращает курс ENU
cq = an.rotate_quat(*yaw_quat(vins_yaw))
check("rotate_quat: курс скорректированной ориентации = курс ENU",
      abs(quat_yaw(*cq) - true_yaw) < 1e-9)
# world-скорость поворачивается тем же Rz
v_true = np.array([1.0, -2.0, 0.3])
v_vins = rz(THETA, v_true)
check("rotate(): world-скорость VINS → кадр EKF",
      np.linalg.norm(an.rotate(v_vins) - v_true) < 1e-9)

# --- 2. нулевой Δyaw: поведение тождественно прежней трансляции -----------
an = FrameAnchor(relatch_m=1.0, tau_sec=5.0)
an.update(np.array([1.0, 2.0, 3.0]), 0.3, np.array([4.0, 6.0, 3.0]), 0.3, 100.0)
check("Δyaw=0: map == vins + (EKF−VINS) (легаси-поведение)",
      np.linalg.norm(an.map(np.array([2.0, 2.0, 3.0]))
                     - np.array([5.0, 6.0, 3.0])) < 1e-9)

# --- 3. мягкий дожим: тянет ТОЛЬКО t, τ работает --------------------------
an = FrameAnchor(relatch_m=10.0, tau_sec=5.0)
an.update(np.zeros(3), 0.0, np.zeros(3), 0.0, 100.0)
# EKF уполз на 0.5 м (< relatch_m): дожим по экспоненте, yaw не трогается
for i in range(1, 11):
    ev = an.update(np.zeros(3), 0.0, np.array([0.5, 0.0, 0.0]), 0.1,
                   100.0 + 0.5 * i)
    if ev is not None:
        check("дожим не считается подтяжкой", False)
        break
else:
    check("дожим не считается подтяжкой", True)
# 10 шагов по 0.5 с при τ=5: 1−exp(−1) ≈ 0.63 пути
check("дожим тянет t к расходу (~63% за τ)", 0.55 < an.t[0] / 0.5 < 0.72)
check("дожим НЕ трогает Δyaw", an.yaw_off == 0.0)

# --- 4. жёсткая подтяжка перелатчивает И поворот --------------------------
an = FrameAnchor(relatch_m=1.0, tau_sec=5.0)
an.update(np.zeros(3), 0.0, np.zeros(3), 0.0, 100.0)
ev = an.update(np.array([10.0, 0.0, 0.0]), 0.5,
               np.array([0.0, 10.0, 0.0]), 0.5 + math.pi / 2, 100.5)
check("расход > relatch_m → 'relatch'", ev == 'relatch' and an.relatch_n == 1)
check("подтяжка перелатчивает Δyaw (90°)",
      abs(math.degrees(an.yaw_off) - 90.0) < 1e-6)
check("после подтяжки map(vins) == EKF",
      np.linalg.norm(an.map(np.array([10.0, 0.0, 0.0]))
                     - np.array([0.0, 10.0, 0.0])) < 1e-9)

# --- 5. засечка NN1: правит только трансляцию -----------------------------
an = FrameAnchor()
an.update(np.zeros(3), 0.0, np.zeros(3), math.pi / 2, 100.0)   # Δyaw=90°
an.fix_translation(np.array([7.0, 7.0, 3.0]), np.array([1.0, 0.0, 3.0]))
check("засечка: map(vins) == позиция засечки",
      np.linalg.norm(an.map(np.array([1.0, 0.0, 3.0]))
                     - np.array([7.0, 7.0, 3.0])) < 1e-9)
check("засечка НЕ трогает Δyaw", abs(an.yaw_off - math.pi / 2) < 1e-9)
# alpha-сглаживание: половина пути
t_before = an.t.copy()
an.fix_translation(np.array([9.0, 7.0, 3.0]), np.array([1.0, 0.0, 3.0]), 0.5)
check("засечка с alpha=0.5 — полпути",
      abs(an.map(np.array([1.0, 0.0, 3.0]))[0] - 8.0) < 1e-9)

# --- 6. обёртка углов: EKF −179° против VINS +175° → Δyaw = +6° -----------
an = FrameAnchor()
an.update(np.zeros(3), math.radians(175.0), np.zeros(3),
          math.radians(-179.0), 100.0)
check("обёртка углов на ±180°: Δyaw = +6°",
      abs(math.degrees(an.yaw_off) - 6.0) < 1e-9)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ FRAME ANCHOR OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
