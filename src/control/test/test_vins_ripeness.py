#!/usr/bin/env python3
"""Оффлайн-тест VinsRipeness — детектора зрелости VINS (2-я ступень гейта).

Сценарии из замеров bag'ов lv2_replay 041803/050600: транзиент residual на
init, климб с верным/кратно-битым масштабом, перекраивание окна солвером,
полёт без вертикали, разрыв потока. Чистый python, без ROS.

Запуск:  python3 src/control/test/test_vins_ripeness.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from control_pkg.application.ripeness import VinsRipeness   # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def climb(r, t0, dur, vz, scale=1.0, res=0.0, dt=0.1, alt0=0.0, z0=0.0):
    """Кормит r одометрией климба: истинная vz, VINS масштаб scale, residual
    res имитируется рассогласованием twist. Вернёт (t, alt, z) конца."""
    t, alt, z = t0, alt0, z0
    n = int(dur / dt)
    for _ in range(n):
        t += dt
        alt += vz * dt
        z += vz * dt * scale
        # twist врёт на res по горизонтали → residual ровно res
        r.on_odom(t, (0.0, 0.0, z), (res, 0.0, vz * scale), alt)
    return t, alt, z


# --- 1. здоровый init: климб 6 м с верным масштабом, residual тихий ---
r = VinsRipeness()
t, alt, z = climb(r, 0.0, 6.0, 1.0, scale=1.0, res=0.05)
check("здоровый климб 6 с: ratio защёлкнут", r.ratio_ok)
check("здоровый климб 6 с: детектор READY (тихо > 4 с)", r.ready)
check("ratio ≈ 1", r.ratio is not None and 0.95 < r.ratio < 1.05)

# --- 2. транзиент init: residual 0.6 первые 2 с → тишина стартует после ---
r = VinsRipeness()
t, alt, z = climb(r, 0.0, 2.0, 1.0, res=0.6)
check("транзиент (res=0.6): не ready", not r.ready)
t, alt, z = climb(r, t, 3.0, 1.0, res=0.02, alt0=alt, z0=z)
check("3 с тишины после транзиента: ещё не ready (EMA+quiet 4 с)", not r.ready)
t, alt, z = climb(r, t, 3.0, 1.0, res=0.02, alt0=alt, z0=z)
check("6 с тишины: ready", r.ready)

# --- 3. масштаб ×10 (полёт №3 эпохи сломанного солвера): ratio вне полосы ---
r = VinsRipeness()
t, alt, z = climb(r, 0.0, 10.0, 1.0, scale=10.0, res=0.02)
check("масштаб ×10: ratio вне полосы, НЕ ready", not r.ready and not r.ratio_ok)
check("масштаб ×10: ratio намерен и виден (~10)",
      r.ratio is not None and r.ratio > 5.0)

# --- 3b. масштаб 0.1 (сжатие): тоже мимо полосы ---
r = VinsRipeness()
climb(r, 0.0, 10.0, 1.0, scale=0.1, res=0.02)
check("масштаб ×0.1: НЕ ready", not r.ready)

# --- 4. солвер перекраивает окно: периодические всплески residual ---
r = VinsRipeness()
t, alt, z = climb(r, 0.0, 5.0, 1.0, res=0.02)   # ratio защёлкнут, тихо
assert r.ready
t, alt, z = climb(r, t, 1.0, 0.0, res=0.8, alt0=alt, z0=z)   # всплеск
check("всплеск residual: ready снялся", not r.ready)
t, alt, z = climb(r, t, 5.0, 0.0, res=0.02, alt0=alt, z0=z)
check("тишина вернулась: ready снова", r.ready)

# --- 5. нет вертикали (ховер с init): ratio не намерен → не ready ---
r = VinsRipeness()
climb(r, 0.0, 20.0, 0.0, res=0.02)
check("без вертикали: ratio нет, НЕ ready (страховка — время)",
      not r.ready and r.ratio is None)

# --- 6. разрыв потока (>1 с) сбрасывает тишину ---
r = VinsRipeness()
t, alt, z = climb(r, 0.0, 6.0, 1.0, res=0.02)
assert r.ready
r.on_odom(t + 2.0, (0.0, 0.0, z), (0.0, 0.0, 0.0), alt)   # дыра 2 с
check("разрыв потока: тишина заново, не ready", not r.ready)

# --- 7. rel_alt=None (нет высоты): не крэш, не ready ---
r = VinsRipeness()
tt = 0.0
for _ in range(60):
    tt += 0.1
    r.on_odom(tt, (0.0, 0.0, tt), (0.0, 0.0, 1.0), None)
check("без rel_alt: не крэш, не ready", not r.ready)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ VINS RIPENESS OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
