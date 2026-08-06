"""ЧИСТЫЙ СНОС боковой оси — метрика для свипа `roll_ki`.

Зачем отдельно от `roll_osc_check` и `roll_brake_check`. У интеграла на петле по
СКОРОСТИ своя работа: петля по скорости позицию не держит в принципе, а `ki`
подтягивает ОСТАТОЧНУЮ скорость к нулю, то есть борется со смещением канала. Ни
доля потолка, ни средняя |v|, ни выбег после команды этого не видят: |v| считает
модуль (снос и болтанка неразличимы), а выбег меряет только 6 с после команды.

Считает по СВОБОДНЫМ участкам (цель `/flow_dbg7.x` = 0, то есть команды нет):
  чистый снос  = смещение вбок от начала участка к концу, м (со знаком);
  снос/с       = он же, делённый на длительность — сопоставимо между участками;
  |путь|       = сумма модулей приращений: сколько борт наездил ВСЕГО.
Отношение |чистый снос| / |путь| = насколько движение направленное: у чистой
болтанки ≈0, у сноса →1. Это и есть разделение «болтается» против «уезжает».
"""
import math, os, sys
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
MIN_SEC = float(os.environ.get('MIN_SEC', '4.0'))   # короче — не участок, а зазор


def yw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


print(f'{"прогон":8} | {"ki":>4} | {"чист.снос":>10} | {"снос/с":>9} | {"|путь|":>8} | '
      f'{"направл":>7} | {"участков":>8}')
for arg in sys.argv[1:]:
    bag, ki = arg.split(':')
    r = SequentialReader()
    r.open(StorageOptions(uri='/out/' + bag + '_bag', storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))
    od, d7 = [], []
    while r.has_next():
        t, raw, ts = r.read_next()
        if t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((st(m), p.x, p.y, p.z, yw(m.pose.pose.orientation)))
        elif t == '/flow_dbg7':
            m = deserialize_message(raw, Vector3Stamped)
            d7.append((st(m), m.vector.x))
    od, d7 = np.array(od), np.array(d7)
    if not len(d7):
        print(f'{bag:8} | нет /flow_dbg7'); continue
    g = np.arange(od[0, 0], od[-1, 0], 0.05)
    x, y = np.interp(g, od[:, 0], od[:, 1]), np.interp(g, od[:, 0], od[:, 2])
    z = np.interp(g, od[:, 0], od[:, 3])
    hd = np.interp(g, od[:, 0], np.unwrap(od[:, 4]))
    tgt = np.interp(g, d7[:, 0], d7[:, 1])
    # свободно = команды нет И держим высоту (на наборе/посадке крен не судим)
    free = (np.abs(tgt) <= 1e-6) & (z > 0.9 * np.percentile(z, 90))
    e = np.nonzero(np.diff(free.astype(int)))[0]
    segs, a = [], (0 if free[0] else None)
    for k in e:
        if free[k + 1] and a is None:
            a = k + 1
        elif not free[k + 1] and a is not None:
            segs.append((a, k)); a = None
    if a is not None:
        segs.append((a, len(g) - 1))
    segs = [(i, j) for i, j in segs if g[j] - g[i] >= MIN_SEC]
    net, path, dur = 0.0, 0.0, 0.0
    for i, j in segs:
        h = hd[i]
        lat = -(x[i:j] - x[i]) * math.sin(h) + (y[i:j] - y[i]) * math.cos(h)
        net += lat[-1]
        path += float(np.sum(np.abs(np.diff(lat))))
        dur += g[j] - g[i]
    if not segs:
        print(f'{bag:8} | {ki:>4} | нет свободных участков'); continue
    print(f'{bag:8} | {ki:>4} | {net:+9.2f}м | {net/dur:+8.3f} | {path:7.1f}м | '
          f'{abs(net)/max(path,1e-9):7.2f} | {len(segs):8d}')
