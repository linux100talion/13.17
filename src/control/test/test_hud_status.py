#!/usr/bin/env python3
"""Юнит-тест hud_status (гейт LOITER-на-VINS → /mission/status). Чистый python.

Проверяет соответствие статуса РЕАЛЬНОМУ гейту Freefly/LoiterHold: READY ровно
при extnav_ready + свежий VINS + в воздухе; градации WAIT/DEAD и причины (why);
формат строки k=v разбирается обратно (контракт для стримера и joy_timeline).

Запуск:  python3 src/control/test/test_hud_status.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.hud import hud_status                   # noqa: E402
from control_pkg.domain.state import DroneState                      # noqa: E402

FRESH = 2.0
results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def kv(line):
    return dict(p.split('=', 1) for p in line.split() if '=' in p)


def s(odom=0, last=0.0, now=0.0, extnav=False, alt=None):
    return DroneState(vins_odom_count=odom, vins_last_sim=last, now_sim=now,
                      extnav_ready=extnav, rel_alt=alt)


# 1. Старт: одометрии не было вовсе (vins_last_sim=-1e9 из датакласса)
d = kv(hud_status(DroneState(now_sim=5.0), FRESH))
check("0 odom → DEAD/no_odom", d['st'] == 'DEAD' and d['why'] == 'no_odom')
check("age зажат (не -1e9 мусор)", float(d['age']) <= 999.0)

# 2. VINS был, но молчит дольше 3×fresh (гистерезис выхода Freefly) → DEAD/stale
d = kv(hud_status(s(odom=700, last=10.0, now=17.0, extnav=True, alt=3.0), FRESH))
check("протух >3×fresh → DEAD/stale", d['st'] == 'DEAD' and d['why'] == 'stale')

# 3. VINS свежий, но очередь EK3_SRC1_* не пройдена → WAIT/extnav
d = kv(hud_status(s(odom=100, last=10.0, now=10.1, extnav=False, alt=3.0), FRESH))
check("без extnav → WAIT/extnav", d['st'] == 'WAIT' and d['why'] == 'extnav')

# 4. Пограничная свежесть (fresh ≤ age ≤ 3×fresh) при extnav → WAIT/stale
d = kv(hud_status(s(odom=700, last=10.0, now=13.0, extnav=True, alt=3.0), FRESH))
check("age между fresh и 3×fresh → WAIT/stale",
      d['st'] == 'WAIT' and d['why'] == 'stale')

# 5. Всё готово, но на земле (≤1.5 м — LOITER невозможен по построению)
d = kv(hud_status(s(odom=700, last=10.0, now=10.1, extnav=True, alt=0.4), FRESH))
check("на земле → WAIT/ground", d['st'] == 'WAIT' and d['why'] == 'ground')
d = kv(hud_status(s(odom=700, last=10.0, now=10.1, extnav=True, alt=None), FRESH))
check("rel_alt=None (нет баро) → WAIT/ground, не крэш",
      d['st'] == 'WAIT' and d['why'] == 'ground')

# 6. Гейт открыт: extnav + свежий + в воздухе → READY (= условие Freefly)
d = kv(hud_status(s(odom=700, last=10.0, now=10.1, extnav=True, alt=3.2), FRESH))
check("extnav+свежий+в воздухе → READY", d['st'] == 'READY' and d['why'] == '-')
check("поля для analyze: extnav/odom/age/alt",
      d['extnav'] == '1' and d['odom'] == '700'
      and float(d['age']) < FRESH and float(d['alt']) == 3.2)
check("поля детектора зрелости res/rat (дефолт -1 = нет данных)",
      float(d['res']) == -1.0 and float(d['rat']) == -1.0)

# 7. Прогрев EKF (баннер «можно взлетать»): ekf=1 ровно при свежем
# local_position — зеркало критерия WaitEkfPos (fresh_sec=2.0)
d = kv(hud_status(DroneState(now_sim=5.0), FRESH))
check("ekf: позиции не было (дефолт -1e9) → ekf=0", d['ekf'] == '0')
d = kv(hud_status(DroneState(now_sim=5.0, ekf_pos_last_sim=4.0), FRESH))
check("ekf: свежий local_position (age<2) → ekf=1", d['ekf'] == '1')
d = kv(hud_status(DroneState(now_sim=7.0, ekf_pos_last_sim=4.0), FRESH))
check("ekf: протух (age≥2) → ekf=0", d['ekf'] == '0')

# 8. Высота глазами EKF3 (zekf=, строка ALT в HUD): значение только при
# СВЕЖЕМ local_position — протухший z (после GPS-kill) честно «--»
d = kv(hud_status(DroneState(now_sim=5.0, ekf_pos_last_sim=4.0, ekf_z=2.34),
                  FRESH))
check("zekf: свежий local_position → z с округлением", d['zekf'] == '2.3')
d = kv(hud_status(DroneState(now_sim=7.0, ekf_pos_last_sim=4.0, ekf_z=2.34),
                  FRESH))
check("zekf: local_position протух → '--', не последнее значение",
      d['zekf'] == '--')
d = kv(hud_status(DroneState(now_sim=5.0, ekf_pos_last_sim=4.0), FRESH))
check("zekf: свежесть есть, z ещё не пришёл (None) → '--'", d['zekf'] == '--')

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ HUD_STATUS OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
