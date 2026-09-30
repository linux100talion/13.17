#!/usr/bin/env python3
"""att_mode — маршрутизация режима FCU при выходе в углах (без ROS).

Запуск:  python3 src/control/test/test_att_mode.py
"""
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control_pkg.application.att_mode import (GUIDED_NOGPS, AttModeProxy,  # noqa: E402
                                              effective_mode)

ok = True


def check(name, cond):
    global ok
    print(f"  [{'OK ' if cond else 'FAIL'}] {name}")
    ok &= bool(cond)


def st(armed=True, mode="ALT_HOLD", perc=None, rel=None):
    return NS(armed=armed, mode=mode, perc_alt=perc, rel_alt=rel)


print("effective_mode")
check("SF вверх, на земле, не заармлен → GUIDED_NOGPS (арм нодой, взлёт в нём)",
      effective_mode("ALT_HOLD", st(armed=False, perc=0.0), False) == GUIDED_NOGPS)
check("SF вверх, в воздухе → GUIDED_NOGPS",
      effective_mode("ALT_HOLD", st(perc=3.0), False) == GUIDED_NOGPS)
check("SF не вверх (MANUAL) → ALT_HOLD, пилот по радио",
      effective_mode("ALT_HOLD", st(mode=GUIDED_NOGPS, perc=3.0), True) == "ALT_HOLD")
for m in ("LOITER", "LAND", "RTL", "GUIDED", "21"):
    check(f"заявлен {m} → проходит как есть", effective_mode(m, st(perc=3.0), False) == m)

print("AttModeProxy")
calls = []
inner = NS(set_mode=lambda m: calls.append(("mode", m)), arm=lambda v=True: calls.append(("arm", v)),
           ready=lambda: True, force_disarm=lambda: calls.append(("force", None)))
px = AttModeProxy(inner)
px.set_mode("ALT_HOLD")
check("до первого update — как заявлено", calls[-1] == ("mode", "ALT_HOLD"))
px.update(st(perc=2.0), False)
px.set_mode("ALT_HOLD")
check("SF вверх: set_mode(ALT_HOLD) → GUIDED_NOGPS", calls[-1] == ("mode", GUIDED_NOGPS))
check("effective() — то же для keep_mode", px.effective("ALT_HOLD") == GUIDED_NOGPS)
px.set_mode("LOITER")
check("LOITER проходит", calls[-1] == ("mode", "LOITER"))
px.arm(False)
px.force_disarm()
check("arm и force_disarm — к адаптеру", calls[-2:] == [("arm", False), ("force", None)])
check("ready — к адаптеру", px.ready() is True)

print("ИТОГ:", "✅ ATT MODE OK" if ok else "❌ ПРОВАЛ")
sys.exit(0 if ok else 1)
