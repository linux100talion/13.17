#!/usr/bin/env python3
"""ipm_vz_phantom.py — ФАНТОМ НАБОРА канала вида сверху: сколько ложной продольной скорости
даёт вертикальный ход борта (2026-10-02, после перехода сима на горизонтальную камеру).

Полоса земли лежит впереди, поэтому ошибка высоты двигает её вдоль X, и набор/снижение
читаются каналом как ход вперёд/назад. Прежний замер (наклонённая на 0.26 рад камера,
gates.md): наклон ошибки +0.67 м/с на каждый м/с набора. Здесь — то же для текущих bag.

Мерка: ошибка продольной скорости канала e = /flow_dbg8.y − истина (twist тела x) против
вертикальной скорости истины vz (twist тела z), МНК e = k·vz + b, только годные кадры
(/flow_dbg8.z > 0.5), борт в окне высоты VZ_HMIN..VZ_HMAX (умолч. > 0.3 м), стики крена/тангажа/курса в центре (газ —
любой: набор и снижение идут им). Плюс медиана ошибки в окнах набора/снижения |vz| > 0.2.
⚠️ В /flow_dbg8 скорость — .y; .x — накопленный путь.

  source /opt/ros/jazzy/setup.bash; python3 src/lab/ipm_vz_phantom.py docker/sim/output/joystick/<RUN> …
"""
import bisect
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gust_hold_compare import resolve, stamp                     # noqa: E402
from geometry_msgs.msg import Vector3Stamped                     # noqa: E402
from nav_msgs.msg import Odometry                                # noqa: E402
from rclpy.serialization import deserialize_message              # noqa: E402
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions  # noqa: E402
from std_msgs.msg import String                                  # noqa: E402

STICK = 20
H_MIN = float(os.environ.get('VZ_HMIN', '0.3'))   # окно высоты истины, м: взлёт/посадку
H_MAX = float(os.environ.get('VZ_HMAX', '1e9'))   # отрезать (пол 0.5 м, земля вплотную)


def read(bag):
    rd = SequentialReader()
    rd.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    rd.set_filter(StorageFilter(topics=['/mission/status', '/model/iris_cam/odometry', '/flow_dbg8']))
    S, T, F = [], [], []
    while rd.has_next():
        topic, raw, _ = rd.read_next()
        if topic == '/mission/status':
            d = dict(kv.partition('=')[::2] for kv in deserialize_message(raw, String).data.split())
            if 't' in d and 'rcp' in d:
                ok = all(abs(int(d[k])) < STICK for k in ('rcp', 'rcr', 'rcy'))
                S.append((float(d['t']), ok))
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            v = m.twist.twist.linear
            T.append((stamp(m), m.pose.pose.position.z, v.x, v.z))
        else:
            m = deserialize_message(raw, Vector3Stamped)
            F.append((stamp(m), m.vector.y, m.vector.z))
    S.sort()
    T.sort()
    F.sort()
    return S, np.array(T), F


def main(runs):
    allx, ally = [], []
    print('  прогон                               кадров  наклон e/vz   сдвиг   corr | окна |vz|>0.2: '
          'набор (медиана e, vz)   снижение')
    for arg in runs:
        label, bag, _ = resolve(arg)
        S, T, F = read(bag)
        g0 = float(np.median(T[:60, 1]))
        st_t = [s[0] for s in S]
        x, y = [], []
        for t, vf, ok in F:
            if ok <= 0.5:
                continue
            j = bisect.bisect_right(st_t, t) - 1
            if j < 0 or not S[j][1]:
                continue
            i = min(np.searchsorted(T[:, 0], t), len(T) - 1)
            if not H_MIN <= T[i, 1] - g0 <= H_MAX:
                continue
            x.append(T[i, 3])
            y.append(vf - T[i, 2])
        x, y = np.array(x), np.array(y)
        if len(x) < 50:
            print(f'  {os.path.basename(label):36s} мало кадров ({len(x)})')
            continue
        A = np.column_stack([x, np.ones(len(x))])
        (k, b), *_ = np.linalg.lstsq(A, y, rcond=None)
        c = float(np.corrcoef(x, y)[0, 1])
        up, dn = x > 0.2, x < -0.2
        for lo, hi in ((0.2, 0.6), (0.6, 2.0)):
            for sg, nm in ((1, 'набор'), (-1, 'сниж')):
                m = (sg * x > lo) & (sg * x <= hi)
                if m.sum() > 10:
                    print(f'      {nm} |vz| {lo}-{hi}: n={m.sum():4d} vz {np.median(x[m]):+.2f}  '
                          f'e {np.median(y[m]):+.2f}  e/vz {np.median(y[m]) / np.median(x[m]):+.2f}')
        f = (lambda m: f'{np.median(y[m]):+.2f} ({np.median(x[m]):+.2f}, n={m.sum()})' if m.sum() > 10 else '—')
        print(f'  {os.path.basename(label):36s} {len(x):5d}   {k:+.3f}     {b:+.3f}  {c:+.2f} | {f(up):24s} {f(dn)}')
        allx.append(x)
        ally.append(y)
    if len(allx) > 1:
        x, y = np.concatenate(allx), np.concatenate(ally)
        A = np.column_stack([x, np.ones(len(x))])
        (k, b), *_ = np.linalg.lstsq(A, y, rcond=None)
        print(f'  ВСЕ ВМЕСТЕ                            {len(x):5d}   {k:+.3f}     {b:+.3f}  '
              f'{float(np.corrcoef(x, y)[0, 1]):+.2f}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1:])
