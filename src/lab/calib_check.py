#!/usr/bin/env python3
"""calib_check — разбор КАЛИБРОВОЧНОГО прогона (план src/control/ToDo.md, блоки A/B).

Отвечает на один вопрос: что команда делает с бортом. Для каждого сегмента, где
контур держал ненулевой стик, считает крутизну канала k [°/PWM] и ускорение
a₁ [м/с² на PWM] — те самые константы, из которых гейны демпфера выводятся
формулой, а не подбираются.

Сегменты НЕ задаются руками: скрипт режет прогон по самой команде (`/flow_dbg.x`
= roll_off, `/flow_dbg2.x` = pitch_off, оба sim-штампованы) — участок, где |команда|
держится выше порога, и есть сегмент миссии `mv_*`. Так разбор не зависит от того,
как долго стек поднимался и сколько sim-времени съел набор.

Истина по углам и позиции — `/model/iris_cam/odometry` (gz). Скорость считается ИЗ
ПОЗИЦИИ (конечная разность), а не из `twist`: у twist неоднозначен фрейм (world vs
body), у позиции — нет. Оценка углов у борта — `/mavros/imu/data`; расхождение с
истиной печатается отдельно, это факт 2 из ToDo (борт держит СВОЙ горизонт).

Запуск ВНУТРИ nav (нужен rosbag2_py):
    docker exec -e CC_BAG=/root/sim_ws/output/A1_pitch_sign_bag p1317_nav bash -lc \
      'source /opt/ros/humble/setup.bash; python3 /lab/calib_check.py'

Env: CC_BAG (bag), CC_MIN_PWM (порог сегмента, 20), CC_MIN_SEC (мин. длина, 1.0 sim-сек).
"""
import math
import os

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu

BAG = os.environ.get('CC_BAG', '/root/sim_ws/output/scene_bag')
MIN_PWM = float(os.environ.get('CC_MIN_PWM', 20))
MIN_SEC = float(os.environ.get('CC_MIN_SEC', 1.0))
G = 9.80665


def euler(q):
    """кватернион → (крен, тангаж, курс) в РАДИАНАХ.

    ⚠️ Знак тангажа в одометрии gz этой сцены: **>0 = НОС ВНИЗ**. Установлено прогоном
    A1: команда +150 PWM дала угол −12.31° и движение НАЗАД, а назад летят носом вверх.
    Не менять «на глаз» — конвенция проверена физикой, а не документацией.
    """
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read(bag):
    """→ odom(t,x,y,z,roll,pitch,yaw), cmd(t,roll_off,pitch_off), imu(t,roll,pitch).
    Время ВЕЗДЕ sim (header.stamp): bag-receive-время это wall и на RTF≈0.055 растянуто."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, roll_c, pitch_c, imu = [], [], [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            r_, pt, y_ = euler(m.pose.pose.orientation)
            od.append((stamp(m), p.x, p.y, p.z, r_, pt, y_))
        elif topic == '/flow_dbg':
            m = deserialize_message(raw, Vector3Stamped)
            roll_c.append((stamp(m), m.vector.x))
        elif topic == '/flow_dbg2':
            m = deserialize_message(raw, Vector3Stamped)
            pitch_c.append((stamp(m), m.vector.x))
        elif topic == '/mavros/imu/data':
            m = deserialize_message(raw, Imu)
            r_, pt, _ = euler(m.orientation)
            imu.append((stamp(m), r_, pt))
    return (np.array(od), np.array(roll_c), np.array(pitch_c),
            np.array(imu) if imu else np.zeros((0, 3)))


def segments(cmd):
    """Участки, где |команда| ≥ MIN_PWM и знак не менялся → [(t0,t1,средняя команда)]."""
    if len(cmd) == 0:
        return []
    out, i = [], 0
    while i < len(cmd):
        if abs(cmd[i, 1]) < MIN_PWM:
            i += 1
            continue
        sgn, j = np.sign(cmd[i, 1]), i
        while j < len(cmd) and abs(cmd[j, 1]) >= MIN_PWM and np.sign(cmd[j, 1]) == sgn:
            j += 1
        if cmd[j - 1, 0] - cmd[i, 0] >= MIN_SEC:
            out.append((cmd[i, 0], cmd[j - 1, 0], float(np.mean(cmd[i:j, 1]))))
        i = j
    return out


def body_vel(od, t0, t1):
    """Скорость в теле ИЗ ПОЗИЦИИ (фрейм twist неоднозначен, фрейм позиции — нет)."""
    m = (od[:, 0] >= t0) & (od[:, 0] <= t1)
    t, x, y, yaw = od[m, 0], od[m, 1], od[m, 2], od[m, 6]
    if len(t) < 5:
        return None
    vx, vy = np.gradient(x, t), np.gradient(y, t)
    return t, vx * np.cos(yaw) + vy * np.sin(yaw), -vx * np.sin(yaw) + vy * np.cos(yaw)


def report_axis(name, cmd, od, ang_col, vel_idx):
    segs = segments(cmd)
    if not segs:
        print(f"\n{name}: сегментов с |командой| ≥ {MIN_PWM:.0f} PWM нет")
        return
    print(f"\n{name}: {len(segs)} сегмент(ов)")
    print(f"  {'t0..t1 sim':>18} {'команда':>9} {'угол':>9} {'k':>11} "
          f"{'a':>10} {'a₁':>13} {'путь':>9}")
    for t0, t1, c in segs:
        m = (od[:, 0] >= t0) & (od[:, 0] <= t1)
        if m.sum() < 5:
            continue
        # хвост сегмента: угол успел установиться (переход занимает 200–400 мс)
        tail = (od[:, 0] >= t0 + 0.5 * (t1 - t0)) & (od[:, 0] <= t1)
        ang = math.degrees(np.mean(od[tail, ang_col]))
        bv = body_vel(od, t0, t1)
        a = np.polyfit(bv[0], bv[vel_idx], 1)[0] if bv else float('nan')
        v0, v1 = (bv[vel_idx][0], bv[vel_idx][-1]) if bv else (float('nan'),) * 2
        dx = od[m][-1, 1] - od[m][0, 1]
        dy = od[m][-1, 2] - od[m][0, 2]
        yaw0 = od[m][0, 6]
        path = dx * math.cos(yaw0) + dy * math.sin(yaw0) if vel_idx == 1 \
            else -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        print(f"  {t0:9.1f}..{t1:7.1f} {c:+8.0f}  {ang:+8.2f}° "
              f"{ang / c:+8.4f}°/PWM {a:+8.3f} м/с² {a / c:+9.5f} м/с²/PWM {path:+8.1f} м")
        # v0 — НЕ ноль: к началу сегмента борт уже разогнан собственным смещением
        # (факт 2). Без него путь сегмента не сходится с a·t²/2 и выглядит «недобором».
        print(f"  {'':18} v тела: {v0:+.2f} → {v1:+.2f} м/с   "
              f"проверка пути v₀t+at²/2 = {v0 * (t1 - t0) + 0.5 * a * (t1 - t0) ** 2:+.1f} м")
    print(f"  справка: статика {30.0 / 400:.3f} °/PWM (ATC_ANGLE_MAX=30°, полный стик 400 PWM), "
          f"a₁ = g·k·π/180 = {G * (30.0 / 400) * math.pi / 180:.4f} м/с²/PWM")


def report_bias(od, imu, cmd_r, cmd_p, segs):
    """Смещение объекта (PWM₀ из A4): углы и разгон при стиках в центре.

    Окно — ТОЛЬКО ДО первого командного сегмента. После сегмента борт летит с
    набранной скоростью, а посадка добавляет свой наклон: усреднять по всему
    «стики в центре» — значит намерить не смещение, а последствия команды.
    """
    if len(cmd_p) == 0:
        return
    t_cut = segs[0][0] if segs else od[-1, 0]
    tp = np.interp(od[:, 0], cmd_p[:, 0], cmd_p[:, 1])
    tr = np.interp(od[:, 0], cmd_r[:, 0], cmd_r[:, 1]) if len(cmd_r) else np.zeros(len(od))
    still = ((np.abs(tp) < MIN_PWM) & (np.abs(tr) < MIN_PWM)
             & (od[:, 3] > 1.0) & (od[:, 0] < t_cut))       # в воздухе, до первой команды
    if still.sum() < 20:
        print("\nсмещение объекта: нет участка со стиками в центре выше 1 м до первой команды")
        return
    s = od[still]
    bv = body_vel(od, s[0, 0], s[-1, 0])
    a_bias = np.polyfit(bv[0], bv[1], 1)[0] if bv else float('nan')
    print(f"\nсмещение объекта (стики в центре, до первой команды t<{t_cut:.1f}, "
          f"{still.sum()} проб, {s[-1, 0] - s[0, 0]:.1f} sim-сек):")
    print(f"  разгон вперёд {a_bias:+.3f} м/с², к концу окна v={bv[1][-1]:+.2f} м/с")
    print(f"  истина gz:  крен {math.degrees(np.mean(s[:, 4])):+.2f}°  "
          f"тангаж {math.degrees(np.mean(s[:, 5])):+.2f}°  "
          f"курс {math.degrees(s[-1, 6] - s[0, 6]) / max(1e-6, s[-1, 0] - s[0, 0]):+.2f}°/с")
    if len(imu):
        mi = (imu[:, 0] >= s[0, 0]) & (imu[:, 0] <= s[-1, 0])
        if mi.sum() > 20:
            print(f"  FCU считает: крен {math.degrees(np.mean(imu[mi, 1])):+.2f}°  "
                  f"тангаж {math.degrees(np.mean(imu[mi, 2])):+.2f}°   "
                  f"← расхождение с истиной = причина смещения (ToDo факт 2)")


def main():
    od, cmd_r, cmd_p, imu = read(BAG)
    print(f"bag: {BAG}")
    print(f"  одометрия {len(od)} | команда roll {len(cmd_r)} / pitch {len(cmd_p)} | "
          f"IMU FCU {len(imu)}")
    if len(od) == 0 or len(cmd_p) == 0:
        print("  ОШИБКА: нет одометрии или /flow_dbg2 — нечего считать")
        return
    print(f"  окно sim: {od[0, 0]:.1f}..{od[-1, 0]:.1f} ({od[-1, 0] - od[0, 0]:.1f} сек), "
          f"высота max {od[:, 3].max():.2f} м")
    # ⚠️ Знак тангажа в ЭТОЙ одометрии: >0 = НОС ВНИЗ (подтверждено прогоном A1 —
    # команда +150 PWM дала угол −12.3° и движение НАЗАД, а назад летят носом вверх).
    report_axis("ПРОДОЛЬНЫЙ канал (pitch)", cmd_p, od, 5, 1)
    report_axis("БОКОВОЙ канал (roll)", cmd_r, od, 4, 2)
    report_bias(od, imu, cmd_r, cmd_p, segments(cmd_p))


if __name__ == '__main__':
    main()
