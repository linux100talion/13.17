#!/usr/bin/env python3
"""Офлайн-тест чистого ядра joy_rc_bridge (без ROS-спина): раскладка каналов TX12,
совпадение CH1..4 с тем, что нода получала бы из joy_sticks, failsafe приёмника.
  python3 src/sim/test_joy_rc_bridge.py      (нужен rclpy/sensor_msgs для импорта модуля)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'control'))
from control_pkg.infrastructure.ros_pilot import JOY_SIGNS_DEFAULT, joy_sticks  # noqa: E402
import joy_rc_bridge as b  # noqa: E402

ok = True


def check(name, cond):
    global ok
    print(f"  [{'OK ' if cond else 'FAIL'}] {name}")
    ok &= bool(cond)


S = JOY_SIGNS_DEFAULT
# стик «вперёд» у TX12 по HID: pitch-ось минус (знак +1) → PWM ниже центра (нос вниз)
ax = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ch = b.joy_to_channels(ax, [], S)
check("16 каналов", len(ch) == 16)
check("CH1..4 = joy_sticks ноды", ch[:4] == list(joy_sticks(ax, S)[:4]))
check("тангаж вперёд = 1100 (ниже центра, конвенция ArduPilot)", ch[1] == 1100)
# газ: HID-ось газа зеркальна (знак −1): axes[2]=−1 → газ вверх
ch = b.joy_to_channels([0.0, 0.0, -1.0, 0.0], [], S)
check("газ в упор вверх = 1900", ch[2] == 1900)
# тумблеры CH5..7 и кнопки CH9+
ch = b.joy_to_channels([0, 0, 0, 0, -1.0, 0.0, 1.0], [0, 1, 0], S)
check("CH5 −1 → 1000, CH6 0 → 1500, CH7 +1 → 2000", ch[4:7] == [1000, 1500, 2000])
check("CH8 не используется = 1500", ch[7] == 1500)
check("кнопка b1 → CH10 = 2000, остальные 1000", ch[9] == 2000 and ch[8] == 1000 and ch[10] == 1000)
# короткий /joy (старые записи): отсутствующие оси — центр
ch = b.joy_to_channels([0.0, 0.0, 0.0, 0.0], [], S)
check("нет осей 4..6 → 1500", ch[4:7] == [1500, 1500, 1500])
fs = b.failsafe_channels()
check("failsafe: стики центр, газ 950, кнопки 1000",
      fs[0] == fs[1] == fs[3] == 1500 and fs[2] == 950 and all(v == 1000 for v in fs[8:]))
print("ИТОГ:", "✅ JOY_RC_BRIDGE OK" if ok else "❌ ПРОВАЛ")
sys.exit(0 if ok else 1)
