#!/usr/bin/env python3
"""Юнит-тест TiltFilter — крен/тангаж по сырому IMU в обход EKF (perception/tilt_filter.py).

- начальная ориентация по акселерометру точна (в т.ч. при ненулевом курсе);
- интеграл гироскопа: 0.5 рад/с по оси x 1 с → крен 0.5, по оси y → тангаж 0.5;
- поправка по акселерометру сходится из неверного старта (τ);
- ноль гироскопа (seed_bias) снимает дрейф смещения;
- длительный «разгон» (акселерометр видит тягу вдоль тела): с τ 15 наклон уплывает медленно.
Запуск:  python3 src/control/test/test_tilt_filter.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control_pkg.perception.tilt_filter import G, TiltFilter   # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def q_of(roll, pitch, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def up_body(q):
    w, x, y, z = q
    return (2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z)


ok = True
for r, p, yw in [(0.1, 0.2, 0.0), (-0.3, 0.05, 1.0), (0.2, -0.25, -2.0)]:
    f = TiltFilter()
    f.update(0.0, 0, 0, 0, *[G * c for c in up_body(q_of(r, p, yw))])
    pp, rr = f.pitch_roll()
    ok &= abs(pp - p) < 1e-9 and abs(rr - r) < 1e-9
check("начальная ориентация по акселерометру точна (3 позы, курс любой)", ok)

f = TiltFilter(tau=0)
f.update(0, 0, 0, 0, 0, 0, G)
for k in range(1, 101):
    f.update(k * 0.01, 0.5, 0, 0, 0, 0, G)
check("гироскоп x 0.5 рад/с × 1 с → крен 0.5", abs(f.pitch_roll()[1] - 0.5) < 1e-6)
f = TiltFilter(tau=0)
f.update(0, 0, 0, 0, 0, 0, G)
for k in range(1, 101):
    f.update(k * 0.01, 0, 0.5, 0, 0, 0, G)
check("гироскоп y 0.5 рад/с × 1 с → тангаж 0.5 (нос вниз +, как у MAVROS)",
      abs(f.pitch_roll()[0] - 0.5) < 1e-6)

r, p = 0.15, -0.1
a = [G * c for c in up_body(q_of(r, p, 0.7))]
f = TiltFilter(tau=2.0, bias_tau=0)
f.update(0.0, 0, 0, 0, 0, 0, G)
for k in range(1, 2001):
    f.update(k * 0.01, 0, 0, 0, *a)
pp, rr = f.pitch_roll()
check(f"поправка сходится из «ровного» старта к r {r} p {p} (τ 2, 20 с): r {rr:+.4f} p {pp:+.4f}",
      abs(rr - r) < 2e-3 and abs(pp - p) < 2e-3)

bias = 0.01                                  # 0.57 °/с по крену
f1, f2 = TiltFilter(tau=15.0), TiltFilter(tau=15.0)
f2.seed_bias(bias, 0, 0)
for f in (f1, f2):
    f.update(0, bias, 0, 0, 0, 0, G)
for k in range(1, 3001):
    for f in (f1, f2):
        f.update(k * 0.01, bias, 0, 0, 0, 0, G)
e1, e2 = abs(math.degrees(f1.pitch_roll()[1])), abs(math.degrees(f2.pitch_roll()[1]))
check(f"смещение гироскопа 0.57 °/с, 30 с висения: без нуля {e1:.2f}°, с нулём {e2:.3f}°",
      e2 < 0.01 and e1 < 9.0)

# длительный разгон: истинный тангаж 0.1 рад, акселерометр видит тягу вдоль тела (0, 0, T)
f = TiltFilter(tau=15.0, bias_tau=0)
f.update(0, 0, 0, 0, *[G * c for c in up_body(q_of(0, 0.1))])
for k in range(1, 501):
    f.update(k * 0.01, 0, 0, 0, 0, 0, G / math.cos(0.1))
drift = math.degrees(0.1 - f.pitch_roll()[0])
check(f"5 с разгона с наклоном 5.7° (акселерометр — тяга): уплыло {drift:.2f}° (τ 15 → ≲ 2°)",
      0.0 < drift < 2.0)

ok_all = all(o for _, o in results)
print("ИТОГ:", "✅ TILT FILTER OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
