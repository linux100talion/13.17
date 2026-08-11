#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""СДВИГ КАНАЛА ВИДА СВЕРХУ В ПОЛЁТЕ — измеренная скорость против истины по одометрии.

ЗАЧЕМ ОТДЕЛЬНО ОТ СТЕНДА. `att_extrap_test.py` меряет сдвиг ОФЛАЙН, переигрывая кадры, и
для этого нужен тяжёлый бэг (STRIP=0, ~2 ГБ) — такой снимается редко. Здесь то же число
берётся из ЛЁГКОГО бэга любого прогона: канал сам кладёт своё измерение в телеметрию.
    /flow_dbg8 = (путь вперёд, ПРОДОЛЬНАЯ скорость, достоверность) — измерение напрямую;
    /flow_dbg9 = (БОКОВАЯ скорость, продольная, достоверность) — тоже напрямую, из
                 снапшота. Появился в коммите 1891720;
    /flow_dbg7 = (цель по скорости, ошибка до неё, PWM) у КРЕН-контура — боковая скорость
                 восстанавливается как цель − ошибка. ЛЕГАСИ-путь для бэгов до 1891720:
                 идёт ЧЕРЕЗ контур, поэтому в прогоне без демпфера крена его нет вовсе.

ЧТО СМОТРЕТЬ. `сдвиг` = среднее (измеренная − истинная). Для демпфера СКОРОСТИ это самое
опасное число: демпфер зануляет то, что ВИДИТ, поэтому постоянный сдвиг измерения = ровно
такая же постоянная РЕАЛЬНАЯ скорость борта. Ни СКО, ни корреляция его не показывают.
Цена: 0.1 м/с × 20 с висения = 2 м ухода, причём одностороннего.

⚠️ Знак сдвига и знак ухода противоположны: борт едет ТУДА, ОТКУДА канал показывает
движение (демпфер гасит мнимую скорость реальной).

Запуск (контейнер одноразовый, монтировать АБСОЛЮТНЫМИ путями):
  docker run --rm -v /root/13.17/src/lab:/lab:ro \
    -v /root/13.17/docker/sim/output:/out:ro ros:humble-ros-base bash -lc \
    'python3 /lab/ipm_bias_check.py J2s1 L2s1 M2s1'
"""
import math
import os
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


print('%-7s| %-10s | %8s | %6s | %8s | %8s | %8s | %8s | %6s'
      % ('прогон', 'ось', 'сдвиг', 'масшт', 'размах ист', 'изм.ср', 'ист.ср', 'СКО ош', 'corr'))
for b in sys.argv[1:]:
    u = '/out/%s_bag' % b
    if not os.path.isdir(u):
        continue
    r = SequentialReader()
    r.open(StorageOptions(uri=u, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, d7, d8, d9 = [], [], [], []
    while r.has_next():
        t, raw, _ = r.read_next()
        if t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((st(m), p.x, p.y, p.z, yaw(m.pose.pose.orientation)))
        elif t == '/flow_dbg7':
            m = deserialize_message(raw, Vector3Stamped)
            d7.append((st(m), m.vector.x, m.vector.y))
        elif t == '/flow_dbg8':
            m = deserialize_message(raw, Vector3Stamped)
            d8.append((st(m), m.vector.y, m.vector.z))
        elif t == '/flow_dbg9':
            m = deserialize_message(raw, Vector3Stamped)
            d9.append((st(m), m.vector.x, m.vector.z))
    od = np.array(od)
    if not len(od):
        continue
    # окно висения: выше 2.5 м, с отступом от набора и от начала снижения
    g = np.arange(od[0, 0], od[-1, 0], 0.05)
    z = np.interp(g, od[:, 0], od[:, 3])
    x = np.interp(g, od[:, 0], od[:, 1])
    y = np.interp(g, od[:, 0], od[:, 2])
    hd = np.interp(g, od[:, 0], np.unwrap(od[:, 4]))
    idx = np.nonzero(z > 2.5)[0]
    if len(idx) < 100:
        continue
    S = slice(idx[0] + 40, idx[-1] - 40)
    vx, vy = np.gradient(x, 0.05)[S], np.gradient(y, 0.05)[S]
    tf = vx * np.cos(hd[S]) + vy * np.sin(hd[S])      # истина, связанная система
    tl = -vx * np.sin(hd[S]) + vy * np.cos(hd[S])
    for name, src, meas in (('продольная', np.array(d8) if d8 else None,
                             lambda a: a[:, 1]),
                            # ПРЯМОЙ слот приоритетнее восстановленного из контура: dbg9
                            # пишется из снапшота и живёт даже без демпфера крена.
                            ('боковая', np.array(d9) if d9 else (np.array(d7) if d7 else None),
                             # ⚠️ ошибка в стабилизаторе считается как СИГНАЛ − цель
                             # (stabilization.py: `err = self._signal(s) - self._target`),
                             # значит измерение = цель + ошибка. Обратный порядок даёт
                             # зеркальный сигнал и corr ≈ −0.8 с истиной — проверка на месте.
                             (lambda a: a[:, 1]) if d9 else (lambda a: a[:, 1] + a[:, 2]))):
        if src is None or len(src) < 50:
            print('%-7s| %-10s | нет телеметрии' % (b, name))
            continue
        v = np.interp(g[S], src[:, 0], meas(src))
        t_ = tf if name == 'продольная' else tl
        ok = np.isfinite(v) & np.isfinite(t_)
        c = np.corrcoef(v[ok], t_[ok])[0, 1] if ok.sum() > 10 else float('nan')
        # МАСШТАБ — наклон прямой `изм = a·ист + b`. Отдельно от сдвига, потому что это
        # РАЗНЫЕ болезни: сдвиг гонит борт при нулевой скорости, масштаб недожимает
        # демпфер пропорционально самой скорости. На висении без ветра масштаб не
        # проявляется вовсе (истинная скорость болтается вокруг нуля — 13% от нуля есть
        # ноль), поэтому мерить его надо на прогоне с постоянным сносом.
        # ⚠️ Разброс истины должен быть достаточным: при |ист| ≲ СКО ошибки наклон
        # оценивается по шуму. Колонка `размах ист` для этого и напечатана.
        rng = t_[ok].max() - t_[ok].min() if ok.sum() > 10 else float('nan')
        a = (np.polyfit(t_[ok], v[ok], 1)[0] if ok.sum() > 10 else float('nan'))
        print('%-7s| %-10s | %+8.3f | %6.2f | %8.2f | %+8.3f | %+8.3f | %8.3f | %+6.2f'
              % (b, name, np.mean(v[ok] - t_[ok]), a, rng, v[ok].mean(), t_[ok].mean(),
                 np.std(v[ok] - t_[ok]), c))
