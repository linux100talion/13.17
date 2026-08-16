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
from control_pkg.domain.rc import RC_CENTER                           # noqa: E402
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

# --- GzYawHold: ЗНАК курс-холда (K1_slope: при +1 борт раскрутило на 439°) ---
gy = GzYawHold()
gy.enter(DroneState(gt_valid=True, gt_x=0.0, gt_y=0.0, gt_yaw=0.0, now_sim=0.0))
rc = gy.update(DroneState(gt_valid=True, gt_x=0.0, gt_y=0.0, gt_yaw=0.2, now_sim=0.1),
               Setpoint(), 0.1)
# борт довернулся на +0.2 рад → гасить надо командой ВЫШЕ центра
check("GzYawHold: уход в +yaw → команда выше центра", rc.yaw > RC_CENTER)
rc = gy.update(DroneState(gt_valid=True, gt_x=0.0, gt_y=0.0, gt_yaw=-0.2, now_sim=0.2),
               Setpoint(), 0.1)
check("GzYawHold: уход в −yaw → команда ниже центра", rc.yaw < RC_CENTER)

# --- GzYawHold: ЗНАК yaw-КОМАНДЫ (mv_cw/mv_ccw обязаны значить одно и то же
# с холдером и без него). Открытый контур: c_yaw>0 → PWM выше центра (ControlStack),
# а «выше центра» = разворот в −yaw ENU = ВПРАВО (замер K1_slope). Значит холдер на
# c_yaw>0 обязан вести _yawsp в −yaw и выдавать команду ТОЖЕ выше центра.
open_loop = ControlStack([], ConstProfile(10, c_yaw=0.3), NoExcitation())
open_loop.enter(DroneState(gt_valid=True, gt_yaw=0.0, now_sim=0.0))
rc_open = open_loop.update(DroneState(gt_valid=True, gt_yaw=0.0, now_sim=0.1))
gy = GzYawHold()
gy.enter(DroneState(gt_valid=True, gt_yaw=0.0, now_sim=0.0))
rc_hold = gy.update(DroneState(gt_valid=True, gt_yaw=0.0, now_sim=0.1),
                    Setpoint(c_yaw=0.3), 0.1)
check("yaw-команда: открытый контур на c_yaw>0 → выше центра", rc_open.yaw > RC_CENTER)
check("yaw-команда: GzYawHold на c_yaw>0 → ТУДА ЖЕ (выше центра)", rc_hold.yaw > RC_CENTER)
check("yaw-команда: GzYawHold ведёт уставку в −yaw (вправо)", gy._yawsp < 0.0)

# --- DpPitchHold: УДЕРЖАНИЕ продольного положения по опорному кадру → pitch ---
# Сигнал теперь kf_logs (log масштаба от опоры, −0.0121 на метр), а не поток в px.
# Два тика с ОДНИМ значением: на втором производная нулевая, остаётся чистое kp·err —
# иначе kd на ступеньке загоняет выход в потолок и проверять нечего.
dp = DpPitchHold(kp=2000.0, ki=0.0, kd=1000.0)
dp.enter(DroneState(flow_seq=-1))
back = DroneState(flow_seq=1, kf_logs=-0.02, kf_valid=True, flow_conf=0.5, flow_dt=0.05, now_sim=0.05)
dp.update(back, Setpoint(), 0.05)
rc = dp.update(DroneState(flow_seq=2, kf_logs=-0.02, kf_valid=True, flow_conf=0.5,
                          flow_dt=0.05, now_sim=0.10), Setpoint(), 0.05)
# уехали назад (масштаб меньше) → нос ВНИЗ → летим к опоре: 1500 + 2000·(−0.02) = 1460
check("DpPitchHold: ушли назад → нос вниз (pitch=1460)", rc.pitch == 1460)
check("DpPitchHold владеет только pitch (roll/yaw центр)", rc.roll == 1500 and rc.yaw == 1500)
fwd = DroneState(flow_seq=3, kf_logs=+0.02, kf_valid=True, flow_conf=0.5, flow_dt=0.05, now_sim=0.15)
dp.update(fwd, Setpoint(), 0.05)
rc = dp.update(DroneState(flow_seq=4, kf_logs=+0.02, kf_valid=True, flow_conf=0.5,
                          flow_dt=0.05, now_sim=0.20), Setpoint(), 0.05)
check("DpPitchHold: ушли вперёд → нос вверх (pitch=1540)", rc.pitch == 1540)

# Опора протухла (ушла высота — оценщик снял kf_valid): НЕ командуем. Иначе на наборе
# kd берёт производную дребезга и кладёт раму на 7° (замер H6_kd: вся скорость на входе
# в висение, 1.5 м/с, оказалась самодельной).
rc = dp.update(DroneState(flow_seq=5, kf_logs=+0.02, kf_valid=False, flow_conf=0.5,
                          flow_dt=0.05, now_sim=0.25), Setpoint(), 0.05)
check("DpPitchHold: опора протухла → команда в центр", rc.pitch == RC_CENTER)

# D-член берётся из ОКОННОЙ скорости опоры (s.kf_vel), а не разностью кадров: та
# коррелирует с истинной скоростью на +0.27 против +0.80 у окна (замер J1b, где
# мёртвый kd дал автоколебание ±20 м). Проверяем оба свойства: (1) при нулевом
# положении едущий вперёд борт тормозится, (2) разность кадров больше НЕ участвует.
dv = DpPitchHold(kp=1500.0, ki=0.0, kd=5000.0)
dv.enter(DroneState(flow_seq=-1))
rc = dv.update(DroneState(flow_seq=1, kf_logs=0.0, kf_vel=+0.011, kf_valid=True,
                          flow_conf=0.5, flow_dt=0.05, now_sim=0.05), Setpoint(), 0.05)
# едем вперёд 1 м/с (kf_vel=+0.011) при нулевой ошибке → 1500 + 5000·0.011 = 1555
check("DpPitchHold: ход вперёд при нулевой ошибке → торможение (pitch=1555)", rc.pitch == 1555)
rc = dv.update(DroneState(flow_seq=2, kf_logs=+0.02, kf_vel=0.0, kf_valid=True,
                          flow_conf=0.5, flow_dt=0.05, now_sim=0.10), Setpoint(), 0.05)
# положение прыгнуло на +0.02 за кадр, но kf_vel=0 → чистое kp: 1500 + 1500·0.02 = 1530
check("DpPitchHold: скачок положения БЕЗ скорости → kd молчит (pitch=1530)", rc.pitch == 1530)

# --- DpHold: композит всех трёх осей (roll/yaw по потоку, pitch по опоре) ---
# ⚠️ Курс молчит первые arm_frames хороших кадров (гейт достоверности, лечение YW1s1),
# поэтому композит проверяется НЕ на первом кадре: иначе тест про владение осями
# провалился бы на взведении, к которому он отношения не имеет.
dh = DpHold()
dh.enter(DroneState(flow_seq=-1))
for k in range(1, 8):
    rc = dh.update(DroneState(flow_seq=k, flow_lateral=5.0, kf_logs=-0.02, kf_valid=True,
                              flow_yaw=3.0, flow_conf=0.5, flow_dt=0.05, now_sim=0.05 * k),
                   Setpoint(), 0.05)
check("DpHold командует roll+pitch+yaw", rc.roll != 1500 and rc.pitch != 1500 and rc.yaw != 1500)

# --- проекция стик-команды по ТЕКУЩЕМУ курсу: «вперёд» = куда смотрит нос СЕЙЧАС ---
# Живой полёт 2026-08-16: проекция шла по курсу ВХОДА в фазу → после разворота на 180°
# «стик вперёд» продолжал везти по старому курсу. Уставка обязана ехать по носу.
import math                                                          # noqa: E402

g1 = GzPosHold()
g1.enter(DroneState(gt_valid=True, gt_x=0.0, gt_y=0.0, gt_yaw=0.0, now_sim=0.0))
g1.update(DroneState(gt_valid=True, gt_x=0.0, gt_y=0.0, gt_yaw=0.0, now_sim=0.05),
          Setpoint(c_fwd=1.0), 0.05)
fwd0 = g1._spx
check("курс 0: стик вперёд двигает уставку в +x", fwd0 > 0)
g2 = GzPosHold()
g2.enter(DroneState(gt_valid=True, gt_x=0.0, gt_y=0.0, gt_yaw=0.0, now_sim=0.0))
g2.update(DroneState(gt_valid=True, gt_x=0.0, gt_y=0.0, gt_yaw=math.pi, now_sim=0.05),
          Setpoint(c_fwd=1.0), 0.05)
check("после разворота на 180° стик вперёд двигает уставку в −x (по носу)",
      g2._spx < 0 and abs(g2._spx + fwd0) < 1e-9)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ СЕМЕЙСТВА Gz*/Dp* OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
