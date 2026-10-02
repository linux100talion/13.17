#!/usr/bin/env python3
"""joy_board_check.py — сверка пульта НА БОРТУ глазами лётной ноды (полётник не трогает).

Пишет /joy (от crsf_joy) N секунд, потом прогоняет запись ТЕМ ЖЕ ядром, что у JoyPilot
(control_pkg.infrastructure.ros_pilot: joy_sticks / joy_master / parse_land_src) со знаками и
кнопками профиля борта (mission/board.txt) и печатает, что нода увидела на каждое движение.

Запуск (с хоста борта; crsf_joy должен уже публиковать /joy):
    docker exec -i vins_project_13_7 bash -c 'source /root/vins_ws/install/setup.bash && \
        python3 - 120' < ~/vins_ws/joy_board_check.py

Порядок движений пилота (по одному органу, с паузой ~2 с, газ НЕ в ноль при курсе вправо —
ARMING_RUDDER 2 армит полётник жестом «газ вниз + курс вправо»):
  1 крен: правый стик вправо до упора → центр → влево → центр
  2 тангаж: правый стик ОТ СЕБЯ → центр → НА СЕБЯ → центр
  3 газ: вверх → центр (НЕ вниз до упора)
  4 курс (газ в центре!): вправо → центр → влево → центр
  5 SC: вверх → центр → вниз;  6 SF: вверх → центр → вниз
  7 SA нажать-отпустить;  8 SD нажать-отпустить
Ожидание (конвенция ноды): крен вправо → 1900; тангаж от себя → 1100 (ArduPilot: вперёд =
ниже центра); газ вверх → 1900; курс вправо → 1900; SF вверх → мастер ВКЛ; SC вверх → L0
(демпфер), центр → L1 (VINS), вниз → L2 (LOITER); SA/SD — фронт > 0.5.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

from control_pkg.infrastructure.ros_pilot import joy_master, joy_sticks, parse_land_src

SIGNS = (1.0, -1.0, 1.0, 1.0)      # BS_JOY_SIGNS профиля mission/board.txt
LAND, RTH = 'a9', 'a10'            # BS_LAND_JOY / BS_RTH_JOY
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0


def main():
    rclpy.init()
    node = Node('joy_board_check')
    rec = []
    node.create_subscription(Joy, '/joy', lambda m: rec.append((time.time(), list(m.axes))),
                             qos_profile_sensor_data)
    print(f"пишу /joy {DUR:.0f} с — двигай органы по порядку (шапка скрипта)", flush=True)
    t_end = time.time() + DUR
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not rec:
        print("!! /joy не пришёл ни разу — crsf_joy жив? пульт связан?")
        return
    t0 = rec[0][0]
    print(f"кадров {len(rec)} за {rec[-1][0] - t0:.1f} с ({len(rec) / max(rec[-1][0] - t0, 1e-3):.0f} Гц)")

    # стики: моменты, когда ось уходила за ±150 PWM от центра — с направлением и экстремумом
    names = ('крен', 'тангаж', 'газ', 'курс')
    print("\n== СТИКИ глазами ноды (PWM после знаков борта; события — уход за ±150 от 1500) ==")
    for i, nm in enumerate(names):
        evs, cur = [], None
        for t, ax in rec:
            v = joy_sticks(ax, SIGNS)[i]
            side = 'ВВЕРХ(>1500)' if v > 1650 else ('ВНИЗ(<1500)' if v < 1350 else None)
            if side != cur:
                if side:
                    evs.append([t - t0, side, v])
                cur = side
            elif side and abs(v - 1500) > abs(evs[-1][2] - 1500):
                evs[-1][2] = v
        vals = [joy_sticks(ax, SIGNS)[i] for _, ax in rec]
        print(f"  {nm:6s} мин {min(vals)} макс {max(vals)} | "
              + (", ".join(f"{t:5.1f}с {s} пик {v}" for t, s, v in evs) or "не двигался"))

    print("\n== ТУМБЛЕРЫ: SF (мастер) и SC (потолок лесенки) — смена состояния во времени ==")
    prev = None
    for t, ax in rec:
        sw, lvl = joy_master(ax)
        st = (sw, lvl)
        if st != prev:
            sf = 'SF ВВЕРХ (мастер ВКЛ)' if sw == -1 else 'SF не вверх (MANUAL)'
            lad = {0: 'L0 демпфер', 1: 'L1 VINS', 2: 'L2 LOITER'}.get(lvl, lvl)
            ch = f"CH6={ax[5]:+.2f} CH7={ax[6]:+.2f}" if len(ax) > 6 else ''
            print(f"  {t - t0:5.1f}с  {sf:22s} SC→ {lad:11s} ({ch})")
            prev = st

    print("\n== КНОПКИ (оси профиля борта, порог 0.5) ==")
    for spec, nm in ((LAND, 'SA посадка'), (RTH, 'SD возврат')):
        kind, idx = parse_land_src(spec)
        presses, on = [], False
        for t, ax in rec:
            v = ax[idx] if idx < len(ax) else -9
            if v > 0.5 and not on:
                presses.append(t - t0)
            on = v > 0.5
        print(f"  {nm} ({spec}): нажатий {len(presses)}"
              + (" на " + ", ".join(f"{t:.1f}с" for t in presses) if presses else ""))


if __name__ == '__main__':
    main()
