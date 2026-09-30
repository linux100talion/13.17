#!/usr/bin/env python3
"""Юнит-тест AltHold (внешний контур высоты) и ThrottleMap (карта газа ALT_HOLD полётника).
Чистый python, без ROS.

AltHold (замер J1b, «висение на 3 м» шло на 5.2 — отсюда контур): выход — скорость набора,
м/с (носитель команды в СИ, фаза B 2026-09-30):
- нет уставки/нет баро → None (контур молчит), climb_cmd → 0 (держать);
- у цели (|err| < tol) → РОВНО 0: в ALT_HOLD центр = «держи высоту»;
- ниже цели → вверх, выше → вниз, симметрично; монотонно по ошибке и до потолка rate_max.
ThrottleMap — Copter::get_pilot_desired_climb_rate_ms по параметрам полётника (сим:
RC3 1100–1900, RC3_DZ 30, THR_DZ 100, PILOT_SPD_UP 2.5, DN 0 = как вверх):
- центр 1500 внутри зоны THR_DZ → 0; прямой счёт по формуле ArduPilot;
- обратная pwm(climb): полётник, прочитав газ, исполнит ту же скорость (до µs);
- «газ в пол» CLIMB_FLOOR → RC3_MIN, потолок → RC3_MAX; параметры — живой словарь.

Запуск:  python3 src/control/test/test_alt_hold.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.altitude import (CLIMB_FLOOR, SIM_DEFAULTS,  # noqa: E402
                                                   AltHold, ThrottleMap)
from control_pkg.domain.state import DroneState              # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


a = AltHold()
check("нет уставки → молчит (None), команда 0", a.climb(DroneState(rel_alt=5.0)) is None
      and a.climb_cmd(DroneState(rel_alt=5.0)) == 0.0)
a.set_target(3.0)
check("нет баро → молчит", a.climb(DroneState(rel_alt=None)) is None)
check("на цели → 0", a.climb(DroneState(rel_alt=3.0)) == 0.0)
check("в допуске (3.05) → 0", a.climb(DroneState(rel_alt=3.05)) == 0.0)
up = a.climb(DroneState(rel_alt=2.0))
dn = a.climb(DroneState(rel_alt=4.0))
check(f"ниже цели на 1 м → вверх kp·1 = 0.6 м/с ({up:+.2f})", abs(up - 0.6) < 1e-9)
check("симметрия вверх/вниз", abs(up + dn) < 1e-12)
check("потолок rate_max (ошибка 5 м → 1.2)", a.climb(DroneState(rel_alt=-2.0)) == 1.2)

m = ThrottleMap()                                  # параметры SITL (SIM_DEFAULTS)
check("карта без прочитанных параметров не готова", not m.ready())
check("центр 1500 → 0 (зона THR_DZ)", m.climb(1500) == 0.0)
# прямой счёт: ctrl = (1800−1130)/770·1000 = 870.1; mid 480, top 580 → 2.5·290.1/420
exp = 2.5 * (1000.0 * (1800 - 1130) / 770 - 580) / 420
check(f"1800 µs → +{exp:.3f} м/с (формула ArduPilot)", abs(m.climb(1800) - exp) < 1e-12)
check("ниже RC3_MIN+RC3_DZ → −PILOT_SPD_DN (DN 0 = как вверх, −2.5)", m.climb(1100) == -2.5)
# выше мёртвой зоны низа RC3_DZ (1100…1130 полётник читает как «газ в пол» — туда же и
# вернётся: RC3_MIN) и вне зоны THR_DZ круг точен до µs
rt = [p for p in range(1131, 1901) if m.climb(p) != 0.0 and m.pwm(m.climb(p)) != p]
check(f"обратная: pwm(climb(p)) = p вне зон (расхождений {len(rt)})", not rt)
check("зона RC3_DZ у низа (1101…1130) → газ в пол RC3_MIN",
      all(m.pwm(m.climb(p)) == 1100 for p in range(1100, 1131)))
check("0 → центр 1500", m.pwm(0.0) == 1500)
check("газ в пол CLIMB_FLOOR → RC3_MIN 1100", m.pwm(CLIMB_FLOOR) == 1100)
check("выше PILOT_SPD_UP → RC3_MAX 1900", m.pwm(10.0) == 1900)
for v in (0.15, 0.6, 1.2, -0.15, -0.6, -1.2):
    got = m.climb(m.pwm(v))
    if abs(got - v) > 0.01:
        check(f"круг {v:+.2f} м/с → {got:+.3f}", False)
check("круг м/с → µs → м/с точен до 0.01 м/с (0.15…1.2 в обе стороны)", True)
live = {}
mb = ThrottleMap(live)
live.update({**SIM_DEFAULTS, 'THR_DZ': 200.0, 'RC3_MIN': 988.0, 'RC3_MAX': 2011.0})
check("живой словарь: параметры дописаны → готова, борт (THR_DZ 200) — 1650 ещё в зоне",
      mb.ready() and mb.climb(1650) == 0.0 and m.climb(1650) > 0.0)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ КОНТУР ВЫСОТЫ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
