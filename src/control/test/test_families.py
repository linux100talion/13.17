#!/usr/bin/env python3
"""Юнит-тест новых членов семейств Gz*/Dp* (чистый python, без ROS):
- axes алиасов (GzPosHold/GzRollHold/GzPitchHold/GzYawHold, Dp*);
- GzHold yaw-курс-холд (ошибка курса → yaw≠центр);
- per-axis: GzRollHold в стеке держит roll, pitch/yaw = пилот;
- DpPitchHold демпфит flow_longitudinal → pitch;
- DpHold (композит) командует всеми тремя осями.

Запуск:  python3 src/control/test/test_families.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.control_stack import ControlStack       # noqa: E402
from control_pkg.domain.control.excitation import NoExcitation       # noqa: E402
from control_pkg.domain.control.stabilization import (               # noqa: E402
    DpHold, DpPitchHold, GzPitchHold, GzPosHold, GzRollHold, GzYawHold)
from control_pkg.domain.control.trajectory import ConstProfile, StaticSetpoint     # noqa: E402
from control_pkg.domain.setpoint import Setpoint                     # noqa: E402
from control_pkg.domain.state import DroneState                      # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


# --- axes алиасов ---
check("GzPosHold владеет roll+pitch+yaw", GzPosHold().axes == frozenset({"roll", "pitch", "yaw"}))
check("GzRollHold владеет только roll", GzRollHold().axes == frozenset({"roll"}))
check("GzPitchHold владеет только pitch", GzPitchHold().axes == frozenset({"pitch"}))
check("GzYawHold владеет только yaw", GzYawHold().axes == frozenset({"yaw"}))

# --- GzHold yaw-курс-холд: ошибка курса → yaw≠центр ---
gy = GzYawHold()
gy.enter(DroneState(gt_valid=True, gt_yaw=0.0, now_sim=0.05))
rc = gy.update(DroneState(gt_valid=True, gt_yaw=0.2, now_sim=0.10), Setpoint(), 0.05)
check("GzYawHold: увод курса → yaw корректирует (≠1500)", rc.yaw != 1500)

# --- per-axis: GzRollHold в стеке держит roll, pitch/yaw = профиль-оператор ---
# оператор ConstProfile: c_fwd=0.2→pitch 1420 (вперёд = ниже центра), c_yaw=-0.125→yaw 1450
stack = ControlStack([GzRollHold()], ConstProfile(10, c_fwd=0.2, c_yaw=-0.125), NoExcitation())
s0 = DroneState(gt_valid=True, gt_x=0.0, gt_y=0.0, gt_yaw=0.0, now_sim=0.05)
stack.enter(s0)
rc = stack.update(DroneState(gt_valid=True, gt_x=0.0, gt_y=1.0, gt_yaw=0.0, now_sim=0.10))
check("GzRollHold в стеке: roll держит позицию (≠1500)", rc.roll != 1500)
check("GzRollHold в стеке: pitch НЕЗАНЯТ → профиль-оператор (1420)", rc.pitch == 1420)
check("GzRollHold в стеке: yaw НЕЗАНЯТ → профиль-оператор (1450)", rc.yaw == 1450)

# --- DpPitchHold: УДЕРЖАНИЕ продольного положения по опорному кадру → pitch ---
# Сигнал теперь kf_logs (log масштаба от опоры, −0.0121 на метр), а не поток в px.
# Два тика с ОДНИМ значением: на втором производная нулевая, остаётся чистое kp·err —
# иначе kd на ступеньке загоняет выход в потолок и проверять нечего.
dp = DpPitchHold(kp=2000.0, ki=0.0, kd=1000.0)
dp.enter(DroneState(flow_seq=-1))
back = DroneState(flow_seq=1, kf_logs=-0.02, flow_conf=0.5, flow_dt=0.05, now_sim=0.05)
dp.update(back, Setpoint(), 0.05)
rc = dp.update(DroneState(flow_seq=2, kf_logs=-0.02, flow_conf=0.5,
                          flow_dt=0.05, now_sim=0.10), Setpoint(), 0.05)
# уехали назад (масштаб меньше) → нос ВНИЗ → летим к опоре: 1500 + 2000·(−0.02) = 1460
check("DpPitchHold: ушли назад → нос вниз (pitch=1460)", rc.pitch == 1460)
check("DpPitchHold владеет только pitch (roll/yaw центр)", rc.roll == 1500 and rc.yaw == 1500)
fwd = DroneState(flow_seq=3, kf_logs=+0.02, flow_conf=0.5, flow_dt=0.05, now_sim=0.15)
dp.update(fwd, Setpoint(), 0.05)
rc = dp.update(DroneState(flow_seq=4, kf_logs=+0.02, flow_conf=0.5,
                          flow_dt=0.05, now_sim=0.20), Setpoint(), 0.05)
check("DpPitchHold: ушли вперёд → нос вверх (pitch=1540)", rc.pitch == 1540)

# --- DpHold: композит всех трёх осей (roll/yaw по потоку, pitch по опоре) ---
dh = DpHold()
dh.enter(DroneState(flow_seq=-1))
rc = dh.update(DroneState(flow_seq=1, flow_lateral=5.0, kf_logs=-0.02,
                          flow_yaw=3.0, flow_conf=0.5, flow_dt=0.05, now_sim=0.05),
               Setpoint(), 0.05)
check("DpHold командует roll+pitch+yaw", rc.roll != 1500 and rc.pitch != 1500 and rc.yaw != 1500)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ СЕМЕЙСТВА Gz*/Dp* OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
