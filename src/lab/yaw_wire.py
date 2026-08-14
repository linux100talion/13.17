#!/usr/bin/env python3
"""ПРОВОДКА оси КУРСА: истинный разворот → визуальный курс → ошибка → PWM.

Зачем. В серии `E2` размах курса за висение вышел 23…360° (в одном прогоне борт
провернулся кругом), и corr(размах курса, |уход|) = +0.69 — теснее, чем у крутизны
продольного канала. Но «ось не держит» — это два РАЗНЫХ диагноза, и лечатся они
по-разному:

  • контур НЕ ВИДИТ разворота (ошибка ≈ 0, пока борт крутится) — врёт сигнал либо его
    съедает утечка накопителя (`DpYawHold.leak_sec`, 8 с: на временах ≫8 с ось по
    построению перестаёт быть курс-холдом и работает демпфером скорости);
  • контур ВИДИТ и не может (ошибка растёт, PWM в потолке) — не хватает власти.

Скрипт кладёт в одну таблицу истинный курс (gz), визуальную ошибку и PWM. Дополнительно
считает, какую долю истинного разворота вообще увидел сигнал: наклон регрессии ошибки
контура на истинный курс. Ноль = ось слепа, единица = видит один в один.

Запуск:
  REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
  docker run --rm -v $REPO/src/lab:/lab:ro \
    -v $REPO/docker/sim/output:/out:ro ros:humble-ros-base bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/yaw_wire.py /out/E2s2_bag'
"""
import math
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
HOVER_Z = 2.0
ROWS = 10
S_PX = 0.324          # px/кадр на °/с — цена единицы сигнала (замер Y4)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, d6 = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            od.append((st(m), m.pose.pose.position.z, yaw_of(m.pose.pose.orientation)))
        elif topic == '/flow_dbg6':
            m = deserialize_message(raw, Vector3Stamped)
            d6.append((st(m), m.vector.x, m.vector.y, m.vector.z))
    f = lambda a: np.array(a) if a else None
    return f(od), f(d6)


def at(arr, ts, col):
    if arr is None or len(arr) == 0:
        return np.full(len(ts), np.nan)
    return arr[np.clip(np.searchsorted(arr[:, 0], ts), 0, len(arr) - 1), col]


def main(bags):
    print(f"{'прогон':8s} | {'размах°':>7s} | {'нетто°':>7s} | {'видит':>6s} | "
          f"{'corr':>5s} | {'σ PWM':>6s} | {'в потолке':>9s}")
    for bag in bags:
        name = bag.rstrip('/').split('/')[-1].replace('_bag', '')
        od, d6 = load(bag)
        if od is None or d6 is None:
            print(f"{name:8s} | данных нет")
            continue
        h = od[od[:, 1] > HOVER_Z]
        if len(h) < 20:
            print(f"{name:8s} | висения нет")
            continue
        yaw = np.degrees(np.unwrap(h[:, 2]))
        yaw -= yaw[0]
        err = at(d6, h[:, 0], 2)          # ошибка контура, единицы сигнала
        pwm = at(d6, h[:, 0], 3)
        ok = np.isfinite(err)
        # Ошибка живёт в единицах сигнала (1 ед. = 1/S градусов), истина — в градусах.
        # Наклон регрессии, приведённый через S, и есть «какую долю разворота видно».
        seen, corr = float('nan'), float('nan')
        if ok.sum() > 20 and np.ptp(yaw[ok]) > 2.0:
            k, _ = np.polyfit(yaw[ok], err[ok] / S_PX, 1)
            seen, corr = -k, np.corrcoef(yaw[ok], err[ok])[0, 1]
        sel = np.linspace(0, len(h) - 1, ROWS).astype(int)
        print(f"{name:8s} | {np.ptp(yaw):7.0f} | {yaw[-1]:+7.0f} | {seen:6.2f} | "
              f"{corr:+5.2f} | {np.nanstd(pwm):6.1f} | "
              f"{100 * np.nanmean(np.abs(pwm) >= 149):8.0f}%")
        print("           курс°: " + ' '.join(f'{yaw[i]:+.0f}' for i in sel))
        print("           ошибка:" + ' '.join(f'{err[i]:+.2f}' for i in sel))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
