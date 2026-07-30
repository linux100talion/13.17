#!/usr/bin/env python3
"""Юнит-тест ВПРЫСКА КОМАНДЫ в холдер положения (_FlowDamper1D, режим `pos`).

Зачем. Команда пилота/скрипта/профиля приходит осям одинаково — нормированным `c_*`.
Но ось, чей сигнал СКОРОСТЬ (крен: flow_lateral), обязана читать её как ЦЕЛЬ скорости,
а ось, чей сигнал ПОЛОЖЕНИЕ (тангаж: kf_logs), — как скорость УСТАВКИ. Если холдеру
положения вычесть c_*·cmd_gain из сигнала (как делалось для скорости), стик задаст не
движение, а постоянное смещение точки удержания: пилот жмёт «вперёд» — борт уезжает на
N метров и встаёт, отпускает — возвращается. Поэтому pitch_cmd_gain и держали в нуле.

Что проверяем:
1. rate-ось (крен) не изменилась: команда вычитается из сигнала;
2. pos-ось (тангаж) при нулевой команде ведёт себя ровно как раньше (уставка = 0);
3. команда ДВИЖЕТ уставку, и ошибка считается до неё (борт, летящий с командной
   скоростью, не получает возражений);
4. D-член вычитает скорость уставки — иначе при kd=5000 команда 2 м/с рождает 145 PWM
   сопротивления при потолке 150 и контур душит собственную команду насыщением;
5. пока сигнал негоден (kf_valid=False на наборе высоты), уставка НЕ едет;
6. отпустили стик — уставка ОСТАЛАСЬ смещённой (новая точка удержания, не возврат).

Запуск:  python3 src/control/test/test_setpoint_integrator.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.stabilization import (               # noqa: E402
    DpPitchHold, DpRollHold)
from control_pkg.domain.rc import RC_CENTER                          # noqa: E402
from control_pkg.domain.setpoint import Setpoint                     # noqa: E402
from control_pkg.domain.state import DroneState                      # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def frame(seq, t, **kw):
    kw.setdefault('kf_valid', True)
    return DroneState(flow_seq=seq, now_sim=t, flow_dt=0.05, flow_conf=0.5, **kw)


# --- 1. режимы разведены по природе сигнала ---
check("крен читает команду как ЦЕЛЬ СКОРОСТИ (rate)", DpRollHold._cmd_mode == "rate")
check("тангаж читает команду как СКОРОСТЬ УСТАВКИ (pos)", DpPitchHold._cmd_mode == "pos")

# крен: сигнал 5 px/кадр, команда c_right=0.5 при cmd_gain=10 → цель 5 → ошибка 0
dr = DpRollHold(kp=8.0, ki=0.0, kd=0.0, cmd_gain=10.0, osign=1.0)
dr.enter(DroneState(flow_seq=-1))
rc = dr.update(frame(1, 0.05, flow_lateral=5.0), Setpoint(c_right=0.5), 0.05)
check("крен: сигнал = заданной скорости → команды нет (1500)", rc.roll == RC_CENTER)

# --- 2. тангаж без команды = прежнее поведение (уставка 0) ---
dp = DpPitchHold(kp=1500.0, ki=0.0, kd=0.0, cmd_gain=0.029)
dp.enter(DroneState(flow_seq=-1))
rc = dp.update(frame(1, 0.05, kf_logs=-0.02), Setpoint(), 0.05)
check("тангаж, стик в центре: чистое удержание (1500 + 1500·(−0.02) = 1470)",
      rc.pitch == 1470)
check("тангаж, стик в центре: уставка стоит на нуле", dp.hold_dbg()[0] == 0.0)

# --- 3. команда ДВИЖЕТ уставку ---
# cmd_gain=0.029 log/с = 2 м/с при крутизне 0.0145 log/м. Полный стик 10 кадров по 50 мс
# = 0.5 с → уставка уезжает на 0.0145 log ≈ 1 м.
dp = DpPitchHold(kp=1500.0, ki=0.0, kd=0.0, cmd_gain=0.029)
dp.enter(DroneState(flow_seq=-1))
for i in range(10):
    dp.update(frame(i + 1, 0.05 * (i + 1), kf_logs=0.0), Setpoint(c_fwd=1.0), 0.05)
sp = dp.hold_dbg()[0]
check(f"полный стик 0.5 с → уставка уехала на {sp:.4f} log ≈ 1 м (0.0145)",
      abs(sp - 0.0145) < 1e-6)
# борт на месте, уставка впереди → err<0 → нос ВНИЗ (летим вперёд, за уставкой)
rc = dp.update(frame(11, 0.55, kf_logs=0.0), Setpoint(c_fwd=1.0), 0.05)
check("уставка впереди борта → нос вниз (команда ниже центра)", rc.pitch < RC_CENTER)

# борт, ИДУЩИЙ ровно с командной скоростью, догоняет уставку → возражений нет
dp2 = DpPitchHold(kp=1500.0, ki=0.0, kd=0.0, cmd_gain=0.029)
dp2.enter(DroneState(flow_seq=-1))
pos = 0.0
for i in range(10):
    pos += 0.029 * 0.05                       # борт едет с заданной скоростью
    rc = dp2.update(frame(i + 1, 0.05 * (i + 1), kf_logs=pos), Setpoint(c_fwd=1.0), 0.05)
check("борт идёт с командной скоростью → kp-член ≈ 0 (|off| ≤ 1)",
      abs(rc.pitch - RC_CENTER) <= 1)

# --- 4. D-член вычитает скорость уставки ---
# kd=5000, команда 2 м/с (0.029 log/с). Борт стоит: kf_vel=0. Без вычитания уставки
# D дал бы −5000·0.029 = −145 PWM... нет: dot=0 → D=0, а с вычитанием D=+145 (тянет
# ВПЕРЁД, помогая команде). Прямая проверка — борт ИДЁТ с командной скоростью:
dv = DpPitchHold(kp=0.0, ki=0.0, kd=5000.0, cmd_gain=0.029)
dv.enter(DroneState(flow_seq=-1))
rc = dv.update(frame(1, 0.05, kf_logs=0.0, kf_vel=0.029), Setpoint(c_fwd=1.0), 0.05)
check("борт идёт с командной скоростью → D молчит (1500)", rc.pitch == RC_CENTER)
# а лишняя скорость сверх командной — гасится
rc = dv.update(frame(2, 0.10, kf_logs=0.0, kf_vel=0.029 + 0.0145), Setpoint(c_fwd=1.0), 0.05)
check("перебор скорости на 1 м/с сверх команды → торможение (1500+72)",
      rc.pitch == 1500 + int(5000 * 0.0145))
# и БЕЗ вычитания та же команда упёрлась бы в потолок: 5000·0.029 = 145 при max 150
check("масштаб проблемы: kd·командная скорость = 145 PWM при потолке 150",
      round(5000 * 0.029) == 145)

# --- 5. негодный сигнал: уставка НЕ едет ---
dn = DpPitchHold(kp=1500.0, ki=0.0, kd=0.0, cmd_gain=0.029)
dn.enter(DroneState(flow_seq=-1))
for i in range(10):
    dn.update(frame(i + 1, 0.05 * (i + 1), kf_logs=0.0, kf_valid=False), Setpoint(c_fwd=1.0), 0.05)
check("опора негодна (набор высоты) → уставка стоит", dn.hold_dbg()[0] == 0.0)

# --- 6. отпустили стик → уставка ОСТАЛАСЬ (новая точка удержания) ---
dh = DpPitchHold(kp=1500.0, ki=0.0, kd=0.0, cmd_gain=0.029)
dh.enter(DroneState(flow_seq=-1))
for i in range(10):
    dh.update(frame(i + 1, 0.05 * (i + 1), kf_logs=0.0), Setpoint(c_fwd=1.0), 0.05)
moved = dh.hold_dbg()[0]
for i in range(10):
    dh.update(frame(11 + i, 0.05 * (11 + i), kf_logs=0.0), Setpoint(), 0.05)
check("стик в центр → уставка замерла на новом месте (не поехала назад)",
      dh.hold_dbg()[0] == moved and moved > 0)
check("уставка в /flow_dbg5 отдаётся только pos-осью (у крена None)",
      dr.hold_dbg() is None and dh.hold_dbg() is not None)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ИНТЕГРАТОР УСТАВКИ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
