#!/usr/bin/env python3
"""Юнит-тест AltHold — внешнего контура высоты (чистый python, без ROS).

Что проверяем и ПОЧЕМУ именно это (замер J1b, «висение на 3 м» шло на 5.2):
- нет уставки/нет баро → центр (контур не выдумывает команду);
- у цели (|err| < tol) → РОВНО центр: в ALT_HOLD центр = «держи высоту», а любая
  команда меньше мёртвой зоны THR_DZ всё равно ничего не делает, зато дрожит;
- ниже цели → команда ВЫШЕ центра и сразу ЗА мёртвой зоной (перескок зоны);
- команда монотонна по ошибке и упирается в потолок rate_max, а не растёт вечно;
- пересчёт PWM→vz совпадает с замером: +300 PWM ≈ +1.58 м/с.

Запуск:  python3 src/control/test/test_alt_hold.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.altitude import AltHold      # noqa: E402
from control_pkg.domain.rc import RC_CENTER                  # noqa: E402
from control_pkg.domain.state import DroneState              # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


a = AltHold()
check("нет уставки → центр", a.throttle(DroneState(rel_alt=5.0)) == RC_CENTER)
a.set_target(3.0)
check("нет баро → центр", a.throttle(DroneState(rel_alt=None)) == RC_CENTER)
check("на цели → центр", a.throttle(DroneState(rel_alt=3.0)) == RC_CENTER)
check("в допуске (3.05) → центр", a.throttle(DroneState(rel_alt=3.05)) == RC_CENTER)

lo = a.throttle(DroneState(rel_alt=2.0))     # ниже цели → набирать
hi = a.throttle(DroneState(rel_alt=4.0))     # выше цели → снижаться
check("ниже цели → команда выше центра", lo > RC_CENTER)
check("выше цели → команда ниже центра", hi < RC_CENTER)
check("команда перескакивает мёртвую зону (|off| ≥ dz)",
      lo - RC_CENTER >= a.dz and RC_CENTER - hi >= a.dz)
check("симметрия вверх/вниз", (lo - RC_CENTER) == (RC_CENTER - hi))

# монотонность и потолок: 1 м → 0.6 м/с, 2 м → 1.2 (потолок), 5 м → тот же потолок
c1 = a.throttle(DroneState(rel_alt=2.0))
c2 = a.throttle(DroneState(rel_alt=1.0))
c3 = a.throttle(DroneState(rel_alt=-2.0))
check("больше ошибка → больше команда", c2 > c1)
check("потолок rate_max держит команду (5 м и 2 м ошибки равны)", c3 == c2)

# калибровка PWM→vz: полный размах span за rate_full м/с, зона сверху
full = a.dz + a.span                        # +500 PWM = rate_full м/с
mid = a.dz + a.span * (1.58 / a.rate_full)  # замер: +300 PWM ≈ +1.58 м/с
check(f"пересчёт совпадает с замером (+1.58 м/с → +{mid:.0f} PWM ≈ 300)",
      abs(mid - 300) < 5)
check("полное отклонение = dz+span (500)", abs(full - 500) < 1e-9)

# Потолок out_max не даёт вылезти за физический ход стика
b = AltHold(kp=10.0, rate_max=100.0, out_max=350.0)
b.set_target(50.0)
check("out_max ограничивает выход", abs(b.throttle(DroneState(rel_alt=0.0)) - RC_CENTER) == 350)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ КОНТУР ВЫСОТЫ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
