#!/usr/bin/env python3
"""Юнит-тесты флоу-стабилизаторов среза 3 (чистый python, без ROS):
- DpRollHold (roll): демпф к цели, velocity-assist (стик=цель → 0 коррекции),
  ПОКАДРОВАЯ интеграция (тот же flow_seq не двигает PID), stale-fade, conf-blend.
- DpYawHold (yaw): чистый rate-демпф (ki=0), velocity-assist.

Запуск:  python3 src/control/test/test_flow_strategies.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.stabilization import DpRollHold, DpYawHold  # noqa: E402
from control_pkg.domain.setpoint import Setpoint                          # noqa: E402
from control_pkg.domain.state import DroneState                           # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def st(seq, lat=0.0, yaw=0.0, conf=0.5, dt=0.05, now=0.05):
    return DroneState(flow_seq=seq, flow_lateral=lat, flow_yaw=yaw,
                      flow_conf=conf, flow_dt=dt, now_sim=now)


# --- DpRollHold: активный демпф ---
fd = DpRollHold()  # kp8 ki2 kd0 imax120 max150 conf[.05,.2] osign1 cmd_gain0
fd.enter(st(-1))
rc = fd.update(st(1, lat=5.0, now=0.05), Setpoint(), 0.05)
# blend=1, err=5, i=2*5*0.05=0.5, u=8*5+0.5=40.5 → roll=1540
check("DpRollHold демпфит боковой поток (roll=1540)", rc.roll == 1540)

# ПОКАДРОВО: тот же flow_seq → PID НЕ двигается (выход держится)
rc2 = fd.update(st(1, lat=5.0, now=0.06), Setpoint(), 0.05)
check("DpRollHold покадрово: тот же seq → выход держится", rc2.roll == 1540)

# velocity-assist: стик задаёт цель = поток → ошибка 0 → нет коррекции
fd2 = DpRollHold(cmd_gain=1.0)
fd2.enter(st(-1))
rc = fd2.update(st(1, lat=5.0, now=0.05), Setpoint(c_right=5.0), 0.05)
check("DpRollHold velocity-assist: стик=поток → roll=центр", rc.roll == 1500)

# stale: кадр протух (>0.5с без нового seq) → fade в центр
fd3 = DpRollHold()
fd3.enter(st(-1))
fd3.update(st(1, lat=5.0, now=0.05), Setpoint(), 0.05)
rc = fd3.update(st(1, lat=5.0, now=1.0), Setpoint(), 0.05)   # тот же seq, время ушло
check("DpRollHold stale → fade в центр", rc.roll == 1500)

# confidence ниже порога → авторитет 0
fd4 = DpRollHold()
fd4.enter(st(-1))
rc = fd4.update(st(1, lat=5.0, conf=0.0, now=0.05), Setpoint(), 0.05)
check("DpRollHold conf<min → авторитет 0 (центр)", rc.roll == 1500)

# --- DpYawHold: чистый rate-демпф (ki=0) ---
yh = DpYawHold()  # kp6 ki0 → yu=6*err
yh.enter(st(-1))
rc = yh.update(st(1, yaw=3.0, now=0.05), Setpoint(), 0.05)
check("DpYawHold rate-демпф (yaw=1518)", rc.yaw == 1518)
# velocity-assist: стик=цель → ошибка 0
yh2 = DpYawHold(cmd_gain=1.0)
yh2.enter(st(-1))
rc = yh2.update(st(1, yaw=3.0, now=0.05), Setpoint(c_yaw=3.0), 0.05)
check("DpYawHold velocity-assist: стик=поток → yaw=центр", rc.yaw == 1500)
# DpYawHold трогает ТОЛЬКО yaw (roll/pitch = центр в его выходе)
check("DpYawHold владеет только yaw", rc.roll == 1500 and rc.pitch == 1500)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ФЛОУ-СТАБИЛИЗАТОРЫ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
