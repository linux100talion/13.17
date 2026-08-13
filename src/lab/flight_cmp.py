#!/usr/bin/env python3
"""flight_cmp.py — ПУТЬ БОРТА по бэгам: вход, висение, посадка + что делали рули.

Зачем именно так. Кампания гейтов вида сверху (IG1s/IG2s/BW, разбор в tune.md)
показала две вещи, ради которых эта таблица и написана:

1. СМОТРЕТЬ НАДО НА ПУТЬ БОРТА, А НЕ НА ПОВЕДЕНИЕ КОМАНДЫ. По «насыщение тангажа
   исчезло» обе версии гейта выглядели победой (0% против 22-43%), а по факту одна
   унесла борт на 39 м, вторая удвоила выброс на входе: насыщения не было потому,
   что не было и команды.
2. УХОД НАДО СЧИТАТЬ ОТ ТОЧКИ ОТРЫВА, А НЕ ОТ НАЧАЛА ОКНА ВИСЕНИЯ. Борт успевает
   уехать ДО окна и там зависнуть — «уход 4 м» при выбросе 14 м читался как норма.
   Поэтому выброс разнесён на ВХОД (первые 10 с) и ПОСАДКУ (после 22 с): на снижении
   высота едет, и если у канала есть гейт по высоте, он там закрыт по построению —
   мешать эти два куска нельзя.

Окна: взлёт [отрыв, +3с], висение [отрыв+5с, +22с], вход [отрыв, +10с],
посадка [отрыв+22с, +45с]. Отрыв = первый кадр с истинной высотой > 0.5 м.

Запуск (в контейнере nav или в одноразовом sim-nav с -v output:/out):
  python3 /lab/flight_cmp.py YW3s1 YW3s2 IG2s1      # имена прогонов
  python3 /lab/flight_cmp.py /out/YW3s1_bag         # или готовые пути
"""
import math
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

T = {'/flow_dbg': 'd1', '/flow_dbg2': 'd2', '/flow_dbg8': 'd8', '/flow_dbg9': 'd9',
     '/flow_dbg6': 'd6'}
st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    out = {v: [] for v in T.values()}
    out['od'] = []
    while r.has_next():
        t, raw, _ = r.read_next()
        if t in T:
            m = deserialize_message(raw, Vector3Stamped)
            out[T[t]].append((st(m), m.vector.x, m.vector.y, m.vector.z))
        elif t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p, v = m.pose.pose.position, m.twist.twist.linear
            out['od'].append((st(m), p.x, p.y, p.z, yw(m.pose.pose.orientation),
                              math.hypot(v.x, v.y), v.z))
    return {k: np.array(v) for k, v in out.items()}


def win(a, t0, t1):
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)] if len(a) else a


hdr = (f'{"прогон":8} | {"тангаж сат":>10} | {"max|тангаж|":>11} | {"max|крен|":>9} | '
       f'{"ipm вперёд":>10} | {"ipm вбок":>8} | {"истинная":>8} | {"Δкурс":>7}')
print('ВЗЛЁТ (3 с от отрыва)\n' + hdr)
rows = {}
def bag(name):
    """Имя прогона или готовый путь — чтобы звать и из контейнера, и по абсолютному пути."""
    return name if '/' in name else f'/out/{name}_bag'


def short(name):
    """Короткое имя прогона для колонки (из пути тоже)."""
    return name.rstrip('/').split('/')[-1].replace('_bag', '')


for name in sys.argv[1:]:
    d = load(bag(name))
    od = d['od']
    z = od[:, 3]
    i0 = int(np.argmax(z > 0.5))
    t0 = od[i0, 0]
    tk = lambda k: win(d[k], t0, t0 + 3.0)
    p, r8, r9 = tk('d2'), tk('d8'), tk('d9')
    o = win(od, t0, t0 + 3.0)
    sat = 100.0 * np.mean(np.abs(p[:, 1]) >= 149) if len(p) else float('nan')
    yaw0 = np.unwrap(od[:, 4])
    y = np.interp([t0, t0 + 3.0], od[:, 0], yaw0)
    rows[name] = dict(sat=sat, pmax=np.abs(p[:, 1]).max(), rmax=np.abs(win(d['d1'], t0, t0 + 3)[:, 1]).max(),
                      vf=np.abs(r8[:, 2]).max(), vl=np.abs(r9[:, 1]).max(),
                      vt=o[:, 5].max(), dy=math.degrees(y[1] - y[0]))
    q = rows[name]
    print(f'{short(name):8} | {q["sat"]:9.0f}% | {q["pmax"]:11.0f} | {q["rmax"]:9.0f} | '
          f'{q["vf"]:9.1f}м/с | {q["vl"]:7.1f} | {q["vt"]:7.1f} | {q["dy"]:+6.1f}°')

print('\nВИСЕНИЕ (17 с) + ВЫБРОС от точки отрыва за весь полёт')
print(f'{"прогон":8} | {"уход":>6} | {"вход":>5} | {"посадка":>7} | {"размах прод":>11} | {"тангаж сат":>10} | '
      f'{"|тангаж|":>8} | {"|крен|":>6} | {"H СКО":>6} | {"курс СКО":>8}')
for name in sys.argv[1:]:
    d = load(bag(name))
    od = d['od']
    i0 = int(np.argmax(od[:, 3] > 0.5))
    t0, t1 = od[i0, 0] + 5.0, od[i0, 0] + 22.0
    o = win(od, t0, t1)
    p = win(d['d2'], t0, t1)
    r = win(d['d1'], t0, t1)
    x, y = o[:, 1] - o[0, 1], o[:, 2] - o[0, 2]
    yy = np.unwrap(o[:, 4])
    f = x * math.cos(yy[0]) + y * math.sin(yy[0])
    dist = lambda t1_, t2_: (lambda a: np.hypot(a[:, 1] - od[i0, 1],
                                                a[:, 2] - od[i0, 2]).max()
                             if len(a) else float('nan'))(
        win(od, od[i0, 0] + t1_, od[i0, 0] + t2_))
    print(f'{short(name):8} | {np.hypot(x, y).max():5.1f}м | {dist(0, 10):4.1f}м | '
          f'{dist(22, 45):6.1f}м | {f.max() - f.min():10.1f}м | '
          f'{100.0 * np.mean(np.abs(p[:, 1]) >= 149):9.0f}% | {np.abs(p[:, 1]).mean():8.0f} | '
          f'{np.abs(r[:, 1]).mean():6.0f} | {np.std(o[:, 3]):5.2f}м | '
          f'{math.degrees(np.std(yy)):7.1f}°')
