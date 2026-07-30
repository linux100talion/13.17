#!/usr/bin/env python3
"""Юнит-тест выборки ω для кадра (RosPerception._omega_for) — без ROS.

Зачем это проверять. Оценщик вычитает вращательный поток как ω·dt, то есть ему нужен
угол, повёрнутый МЕЖДУ кадрами, а не мгновенная скорость в момент прихода сообщения.
Телеметрия реже кадров (замер живьём: /mavros/imu/data 12.5 Гц, data_raw 20.8 Гц,
камера 19-30 Гц), поэтому «последняя пришедшая» ω запаздывает до 80 мс; при 5°/с это
0.4° неснятого вращения ≈ 2 px ложного потока на fx=480 — прямо в оценку подобия
опоры. Отсюда усреднение по интервалу (прошлый кадр, этот кадр].

Адаптер создаём через object.__new__ — ROS-подписки в __init__ нам тут не нужны,
проверяется чистая арифметика выборки.

Запуск (нужен numpy → внутри контейнера nav):
  docker exec p1317_nav bash -lc 'cd /root/sim_ws/src/control && python3 test/test_omega_window.py'
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.infrastructure.ros_perception import RosPerception    # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def mk(buf, prev=None, last=(9.0, 9.0, 9.0)):
    p = object.__new__(RosPerception)
    p._gyro_buf = list(buf)
    p._prev_img_stamp = prev
    p._omega = np.array(last)
    p._gyro_own = True
    return p


# пустой буфер → отдаём то, что было (деградация, а не падение)
p = mk([])
check("пустой буфер → прежняя ω", tuple(p._omega_for(1.0)) == (9.0, 9.0, 9.0))

# первый кадр (prev нет) → ближайший по времени сэмпл, а НЕ последний пришедший
buf = [(1.00, 1.0, 0.0, 0.0), (1.05, 2.0, 0.0, 0.0), (1.10, 3.0, 0.0, 0.0)]
p = mk(buf)
check("первый кадр → ближайший сэмпл (t=1.06 → 1.05)", p._omega_for(1.06)[0] == 2.0)
check("первый кадр → ближайший сэмпл (t=1.09 → 1.10)", p._omega_for(1.09)[0] == 3.0)

# интервал (prev, stamp]: среднее ПОПАВШИХ, границы — левая открыта, правая закрыта
p = mk(buf, prev=1.00)
check("интервал (1.00, 1.10] → среднее 2 и 3 = 2.5", p._omega_for(1.10)[0] == 2.5)
p = mk(buf, prev=1.04)
check("левая граница ОТКРЫТА: (1.04, 1.05] → только 2.0", p._omega_for(1.05)[0] == 2.0)

# в интервал не попало ничего → ближайший (не последний пришедший)
p = mk(buf, prev=1.11)
check("пустой интервал → ближайший (t=1.12 → 1.10)", p._omega_for(1.12)[0] == 3.0)

# усредняются все три оси независимо
buf3 = [(2.0, 1.0, 10.0, 100.0), (2.1, 3.0, 30.0, 300.0)]
p = mk(buf3, prev=1.9)
check("средняя считается по всем трём осям",
      tuple(p._omega_for(2.1)) == (2.0, 20.0, 200.0))

# буфер не растёт без предела (иначе на длинном прогоне съест память)
p = mk([(i * 0.05, i, 0.0, 0.0) for i in range(200)], prev=0.0)
n0 = len(p._gyro_buf)

class _M:                      # минимальная заглушка сообщения Imu
    class _S:
        sec, nanosec = 20, 0
    class _W:
        x = y = z = 1.0
    header = type('h', (), {'stamp': _S})
    angular_velocity = _W

p._on_gyro(_M(), own=True)      # own=True при уже взятом источнике буфер НЕ чистит
check(f"буфер подрезается (было {n0} → стало {len(p._gyro_buf)} ≤ 120)",
      len(p._gyro_buf) <= 120)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ВЫБОРКА ω OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
