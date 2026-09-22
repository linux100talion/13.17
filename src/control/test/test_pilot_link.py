#!/usr/bin/env python3
"""Юнит-тест СТОРОЖА СВЕЖЕСТИ ПУЛЬТА (PilotLink, 2026-09-22). Чистый python.

Зачем сторож: пока нода пишет /mavros/rc/override, RC-failsafe полётника не
сработает (radio_in жив нашей же рукой), а адаптер пульта при пропаже источника
держит ПОСЛЕДНИЕ значения — борт летел бы с замороженными стиками
(docker/sim/laptop_move.md §5.2). Сторож ловит тишину источника и велит ноде
ОТПУСТИТЬ каналы (RC_RELEASE = 0), а не публиковать команду.

Проверяет: порог и его края, выключенное состояние (0 = как было до правки),
отсутствие внешнего канала (ScriptedPilot → age None), фронты для лога (один
раз на переход), восстановление, поле pw= в /mission/status и константу
RC_RELEASE (0, не 65535 — их путаница стоила бы «не трогать» вместо «отпустить»).

Запуск:  python3 src/control/test/test_pilot_link.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.hud import hud_status                    # noqa: E402
from control_pkg.domain.pilot_link import PilotLink                   # noqa: E402
from control_pkg.domain.rc import RC_NOCHANGE, RC_RELEASE             # noqa: E402
from control_pkg.domain.state import DroneState                       # noqa: E402
from control_pkg.infrastructure.ros_pilot import ScriptedPilot        # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


# 1. Порог: живой канал молчит меньше порога — вердикта нет
lk = PilotLink(0.5)
check("enabled при stale_sec>0", lk.enabled)
check("age 0.0 — жив", lk.update(0.0) is False and lk.alive)
check("age 0.49 — ещё жив", lk.update(0.49) is False)
check("age 0.5 — ПРОТУХ (край включительно)", lk.update(0.5) is True and not lk.alive)
check("age 3.0 — протух", lk.update(3.0) is True)

# 2. Фронты: лог не должен сыпаться каждый тик
lk = PilotLink(0.5)
lk.update(0.1)
first = lk.update(1.0)
check("фронт на переходе жив→протух", first and lk.changed)
lk.update(1.5)
check("повтор протухшего — фронта нет", lk.stale and not lk.changed)
lk.update(0.02)
check("восстановление: фронт протух→жив", not lk.stale and lk.changed)
lk.update(0.02)
check("повтор живого — фронта нет", not lk.stale and not lk.changed)

# 3. Выключенный сторож = поведение до правки (ни при каком возрасте не срабатывает)
off = PilotLink(0.0)
check("stale_sec=0 — сторож выключен", not off.enabled)
check("выключенный не срабатывает даже на 100 с", off.update(100.0) is False and off.alive)

# 4. Источника нет (ScriptedPilot): сторожить нечего даже при включённом стороже
lk = PilotLink(0.5)
check("age None — вердикта нет", lk.update(None) is False and lk.alive)
check("ScriptedPilot.link_age() = None", ScriptedPilot(None, []).link_age() is None)

# 5. Константы отпускания: 0 ≠ 65535 (RC_RELEASE «отпустить» против «не трогать»)
check("RC_RELEASE = 0", RC_RELEASE == 0)
check("RC_RELEASE не равен RC_NOCHANGE", RC_RELEASE != RC_NOCHANGE)

# 6. Поле pw= в /mission/status
s = DroneState(now_sim=5.0)
check("link=None — поля pw= нет", " pw=" not in hud_status(s, 2.0))
d = dict(p.split('=', 1) for p in hud_status(s, 2.0, link=True).split() if '=' in p)
check("link=True → pw=1", d.get("pw") == "1")
d = dict(p.split('=', 1) for p in hud_status(s, 2.0, link=False).split() if '=' in p)
check("link=False → pw=0", d.get("pw") == "0")

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ PILOT LINK OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
