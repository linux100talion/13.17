#!/usr/bin/env python3
"""node_arm — жест арма/дизарма и готовность ноды (без ROS).
Запуск:  python3 src/control/test/test_node_arm.py
"""
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control_pkg.application.node_arm import ArmGesture, ready_reasons  # noqa: E402

ok = True


def check(name, cond):
    global ok
    print(f"  [{'OK ' if cond else 'FAIL'}] {name}")
    ok &= bool(cond)


def run(g, seq):
    """seq: [(t, thr, yaw, armed, landed)] → события."""
    return [(t, e) for t, *a in seq for e in [g.update(t, *a)] if e]


print("ArmGesture")
g = ArmGesture()
ev = run(g, [(t / 10, 1100, 1900, False, True) for t in range(0, 35)])
check("газ в пол + курс вправо 2 с → 'arm' на 2 с", [e for _, e in ev] == ['arm'] and abs(ev[0][0] - 2.0) < 0.11)
g = ArmGesture()
ev = run(g, [(t / 10, 1100, 1900, False, True) for t in range(0, 65)])
check("держит дальше, не заармлен (нода не была готова) → повтор каждые 2 с",
      [round(t) for t, _ in ev] == [2, 4, 6])
g = ArmGesture()
check("1.5 с — рано", run(g, [(t / 10, 1100, 1900, False, True) for t in range(0, 16)]) == [])
g = ArmGesture()
seq = ([(t / 10, 1100, 1900, False, True) for t in range(0, 15)] + [(1.5, 1500, 1500, False, True)]
       + [(1.6 + t / 10, 1100, 1900, False, True) for t in range(0, 15)])
check("отпустил до 2 с — таймер заново", run(g, seq) == [])
g = ArmGesture()
check("газ не в полу — не жест", run(g, [(t / 10, 1300, 1900, False, True) for t in range(0, 30)]) == [])
g = ArmGesture()
check("заармлен — жест арма игнорируется", run(g, [(t / 10, 1100, 1900, True, True) for t in range(0, 30)]) == [])
g = ArmGesture()
check("курс влево на земле заармленным → 'disarm'",
      [e for _, e in run(g, [(t / 10, 1100, 1100, True, True) for t in range(0, 25)])] == ['disarm'])
g = ArmGesture()
check("в воздухе дизарм жестом не срабатывает",
      run(g, [(t / 10, 1100, 1100, True, False) for t in range(0, 30)]) == [])
g = ArmGesture()
# реплей давит дизарм импульсами 4 с / 2 с — 4 с хватает
check("импульс реплея 4 с → 'disarm'",
      [e for _, e in run(g, [(t / 10, 1100, 1100, True, True) for t in range(0, 40)])] == ['disarm'])

print("ready_reasons")
s = NS(now_sim=100.0, tel_last_sim=99.9, ekf_pos_last_sim=99.8)
check("всё свежо → готов", ready_reasons(s, True, True) == [])
check("нет параметров углов и кадров → две причины", len(ready_reasons(s, False, False)) == 2)
s2 = NS(now_sim=100.0, tel_last_sim=90.0, ekf_pos_last_sim=-1e9)
check("нет телеметрии и EKF → две причины", len(ready_reasons(s2, True, True)) == 2)

print("ИТОГ:", "✅ NODE ARM OK" if ok else "❌ ПРОВАЛ")
sys.exit(0 if ok else 1)
