#!/usr/bin/env python3
"""Оффлайн-тест TrackHold (стики LOITER в осях мира, чистый python).

Проверяет геометрию контр-вращения и политику латча:
- Δψ=0 → выход бит-в-бит равен базе ControlStack (сырой passthrough): включение
  ручки без разворота ничего не меняет;
- нос повернулся на +90° (влево, ENU) при живом pitch-вперёд → команда «вправо»
  (мировой вектор на месте); 180° → «назад»;
- латч по фронту отклонения стика, сброс в центре: разворот С ЦЕНТРАЛЬНЫМИ
  стиками рамы не копит («вперёд = куда сейчас смотрю» на каждом нажатии);
- оба стика в упоре: кламп ±span, PWM в границах провода;
- att_yaw молчит (0.0 констант) → деградация в сырой passthrough;
- в ControlStack композиция per-axis: track владеет roll/pitch, yaw — у соседа.

Запуск:  python3 src/control/test/test_track_hold.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.control_stack import ControlStack           # noqa: E402
from control_pkg.domain.control.excitation import NoExcitation           # noqa: E402
from control_pkg.domain.control.track_hold import TrackHold              # noqa: E402
from control_pkg.domain.control.trajectory import ConstProfile           # noqa: E402
from control_pkg.domain.rc import RC_CENTER                              # noqa: E402
from control_pkg.domain.setpoint import Setpoint                         # noqa: E402
from control_pkg.domain.state import DroneState                          # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def state(yaw=0.0, t=0.05):
    return DroneState(att_yaw=yaw, now_sim=t)


DT = 0.05

# --- 1. Δψ=0: бит-в-бит с базой ControlStack (сырой passthrough) ---
# База: roll = int(1500 + c_right·400), pitch = int(1500 − c_fwd·400).
th = TrackHold()
th.enter(state())
for f, r in ((0.5, -0.25), (1.0, 0.0), (-0.7, 0.33)):
    rc = th.update(state(yaw=0.7), Setpoint(f, r, 0.0), DT)   # yaw≠0, но Δψ=0
    base_roll = int(RC_CENTER + r * 400)
    base_pitch = int(RC_CENTER - f * 400)
    check(f"Δψ=0 (f={f}, r={r}): roll/pitch = база passthrough",
          (rc.roll, rc.pitch) == (base_roll, base_pitch))
    th.enter(state())          # сброс латча между случаями

# --- 2. нос +90° (влево, ENU) при живом pitch-вперёд → команда «вправо» ---
# Мир: летели на восток носом на восток; нос стал на север → тот же восточный
# вектор = «вправо» от носа. pitch возвращается в центр, roll — в +400.
th = TrackHold()
th.enter(state())
th.update(state(yaw=0.0), Setpoint(1.0, 0.0, 0.0), DT)        # латч ψ0=0
rc = th.update(state(yaw=math.pi / 2), Setpoint(1.0, 0.0, 0.0), DT)
check("нос +90°: полный вперёд → полный вправо (roll 1900)", rc.roll == 1900)
check("нос +90°: продольная компонента ушла (pitch 1500)", rc.pitch == 1500)

# --- 3. нос 180° → «назад» (мировой вектор всё ещё тот же) ---
rc = th.update(state(yaw=math.pi), Setpoint(1.0, 0.0, 0.0), DT)
check("нос 180°: вперёд стал назад (pitch 1900)", rc.pitch == 1900)
check("нос 180°: боковой centre (roll 1500)", rc.roll == 1500)

# --- 4. промежуточный угол: модуль вектора сохраняется ---
rc = th.update(state(yaw=math.pi / 4), Setpoint(1.0, 0.0, 0.0), DT)
mag = math.hypot(rc.roll - RC_CENTER, rc.pitch - RC_CENTER)
check("нос +45°: |вектор| = 400 (пифагор по осям)", abs(mag - 400) < 1.5)

# --- 5. латч: разворот с ЦЕНТРАЛЬНЫМИ стиками рамы не копит ---
th = TrackHold()
th.enter(state())
th.update(state(yaw=0.0), Setpoint(0.0, 0.0, 0.0), DT)        # центр: рамы нет
rc = th.update(state(yaw=1.2), Setpoint(1.0, 0.0, 0.0), DT)   # нажали ПОСЛЕ разворота
check("латч по нажатию: «вперёд = куда сейчас смотрю» (pitch 1100)",
      (rc.roll, rc.pitch) == (1500, 1100))
# отпустили → рама сброшена → новое нажатие с нового курса снова чистое
th.update(state(yaw=1.2), Setpoint(0.0, 0.0, 0.0), DT)
rc = th.update(state(yaw=-2.0), Setpoint(1.0, 0.0, 0.0), DT)
check("сброс в центре: повторное нажатие чистое с нового курса",
      (rc.roll, rc.pitch) == (1500, 1100))

# --- 6. центр = «стоять»: выход центр (держит FCU) ---
rc = th.update(state(yaw=-2.0), Setpoint(0.0, 0.0, 0.0), DT)
check("центр стиков → центр PWM (позицию держит FCU)",
      (rc.roll, rc.pitch) == (1500, 1500))

# --- 7. оба стика в упоре + разворот: кламп держит провод в границах ---
th = TrackHold()
th.enter(state())
th.update(state(yaw=0.0), Setpoint(1.0, 1.0, 0.0), DT)
ok = True
for yaw in (0.3, 0.79, 1.5, 2.5):
    rc = th.update(state(yaw=yaw), Setpoint(1.0, 1.0, 0.0), DT)
    ok = ok and 1100 <= rc.roll <= 1900 and 1100 <= rc.pitch <= 1900
check("диагональ в упоре: PWM в границах ±400 при любом Δψ", ok)

# --- 8. wrap ±π: непрерывный разворот через границу не рвёт команду ---
th = TrackHold()
th.enter(state())
th.update(state(yaw=3.0), Setpoint(1.0, 0.0, 0.0), DT)        # латч у границы
rc_a = th.update(state(yaw=3.14), Setpoint(1.0, 0.0, 0.0), DT)
rc_b = th.update(state(yaw=-3.14), Setpoint(1.0, 0.0, 0.0), DT)  # перескок ±π
check("wrap ±π: команда непрерывна (скачок PWM < 8)",
      abs(rc_a.roll - rc_b.roll) < 8 and abs(rc_a.pitch - rc_b.pitch) < 8)

# --- 9. att_yaw молчит (всегда 0.0) → чистый passthrough, как без TrackHold ---
th = TrackHold()
th.enter(state())
rc = th.update(state(yaw=0.0), Setpoint(0.6, -0.4, 0.0), DT)
check("нет IMU (att_yaw=0): passthrough (roll 1340, pitch 1260)",
      (rc.roll, rc.pitch) == (1340, 1260))

# --- 10. композиция в ControlStack: track владеет roll/pitch, yaw открыт профилю ---
stack = ControlStack([TrackHold()], ConstProfile(10, c_fwd=1.0, c_yaw=0.25),
                     NoExcitation())
s = state(yaw=0.0)
stack.enter(s)
stack.update(s)                                    # латч ψ0=0 (стик жив)
s2 = state(yaw=math.pi / 2, t=0.10)
rc = stack.update(s2)
check("стек: roll/pitch у TrackHold (повёрнуты: roll 1900, pitch 1500)",
      (rc.roll, rc.pitch) == (1900, 1500))
check("стек: yaw не занят → открытый контур профиля (1600)", rc.yaw == 1600)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ TRACK HOLD OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
