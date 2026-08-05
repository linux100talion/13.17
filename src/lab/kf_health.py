"""Здоровье ОПОРЫ продольного контура: почему один и тот же `kd` даёт разный исход.

Читает /flow_dbg3 (kf_logs, kf_vel, kf_n) и /flow_dbg4 (segs, reseeds, rejects) на
том же 40-секундном окне висения, что и hover_stats.py. Отвечает на вопрос «контур
терял опору или честно не справлялся»: пересевы и провалы числа точек видны только
здесь — по одному kf_vel их не отличить от настоящего разбега.
"""
import os, sys
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from geometry_msgs.msg import Vector3Stamped

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, d3, d4 = [], [], []
    while r.has_next():
        t, raw, ts = r.read_next()
        if t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            od.append((st(m), m.pose.pose.position.z))
        elif t == '/flow_dbg3':
            m = deserialize_message(raw, Vector3Stamped)
            d3.append((st(m), m.vector.x, m.vector.y, m.vector.z))
        elif t == '/flow_dbg4':
            m = deserialize_message(raw, Vector3Stamped)
            d4.append((st(m), m.vector.x, m.vector.y, m.vector.z))
    return np.array(od), np.array(d3), np.array(d4)


print(f'{"прогон":8} | {"kf_n мед":>8} | {"kf_n мин":>8} | {"n<30":>5} | {"сегм":>5} | '
      f'{"пересев":>7} | {"выброс":>6} | {"скачки vel":>10} | {"|kf_vel|":>8}')
for bag in sys.argv[1:]:
    od, d3, d4 = load(bag)
    if not len(od) or not len(d3):
        print(f'{os.path.basename(bag)}: пусто'); continue
    z = od[:, 1]
    t0 = od[int(np.argmax(z > 0.9 * np.percentile(z, 90))), 0]
    sel = (d3[:, 0] >= t0) & (d3[:, 0] <= t0 + 40)
    n, vel = d3[sel, 3], d3[sel, 2]
    # ⚠️ `kf_segs` ОБНУЛЯЕТСЯ дважды за прогон (перезапуск оценщика на смене режима —
    # замер по N3s1/N3s2/N3s3/N0s3, сбросы на t+8…13 с). Поэтому «конец минус начало»
    # для него врёт: по N3s3 так вышло 5 сегментов при 50 в максимуме. Считаем СУММОЙ
    # ПРИРОСТОВ. `kf_reseeds` не сбрасывается ни разу, но считаем так же — единообразно.
    if len(d4):
        w = d4[(d4[:, 0] >= t0) & (d4[:, 0] <= t0 + 40)]
        if len(w) > 1:
            d = np.diff(w[:, 1:], axis=0)
            segs, res, rej = np.sum(np.where(d > 0, d, 0), axis=0)
        else:
            segs = res = rej = 0
    else:
        segs = res = rej = float('nan')
    # «скачок» = кадровое изменение скорости больше 3 СКО: подпись пересева опоры
    dv = np.abs(np.diff(vel))
    jumps = int(np.sum(dv > 3 * np.std(vel))) if len(dv) else 0
    print(f'{os.path.basename(bag).replace("_bag",""):8} | {np.median(n):8.0f} | {n.min():8.0f} | '
          f'{100*np.mean(n < 30):4.0f}% | {segs:5.0f} | {res:7.0f} | {rej:6.0f} | '
          f'{jumps:10d} | {np.mean(np.abs(vel)):8.4f}')
