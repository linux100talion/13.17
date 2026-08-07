#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ДОМАШНЯЯ ОПОРА: сколько дома ещё видно — измеритель удаления БЕЗ масштаба.

Вопрос, из которого вырос скрипт: почему человек по видео сразу видит, что борт унесло
за край сцены, а алгоритм этого не видит.

Человек не измеряет — он УЗНАЁТ: картинка стала другой, значит я далеко. Наш же канал
спрашивает «на сколько изменился масштаб сцены с прошлой опоры», причём опору пересевает
каждые пару секунд, так что память о доме живёт только в накопителе — сумме приращений,
которые врут знаком в 10-30% случаев (замер kf_sign_check). Сумма таких шагов — случайное
блуждание, а не расстояние.

Вдобавок масштаб отвечает на вопрос «далеко ли я» слабее всего: он меряет движение вдоль
луча, а при уходе на десятки метров главное, что происходит с картинкой, — меняется НАБОР
видимых объектов. Эту, самую сильную, компоненту мы сейчас выбрасываем: доля выживших
точек опоры используется лишь как порог «мало точек → пересеять», то есть как помеха.

Здесь она проверяется как ИЗМЕРИТЕЛЬ. Опора сеется один раз на входе в висение и НЕ
пересевается; точки ведутся КЛТ с обратной проверкой (как в боевом оценщике). Печатается
доля выживших против истинного удаления от точки старта висения: корреляция, и на каком
расстоянии доля падает до 1/2 и 1/4. Масштаб, знак и интегрирование для этого не нужны.

⚠️ Нужен бэг С КАДРАМИ (`/image_color`).

Запуск (в контейнере nav):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash;
    HO_BAG=/root/sim_ws/output/E5f1_bag python3 /lab/home_overlap.py'
"""
import math
import os
import sys

import numpy as np

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image

BAG = os.environ.get('HO_BAG', '/root/sim_ws/output/E5f1_bag')
HOVER_Z = 2.0
MAX_FEATS = 200
# параметры КЛТ и детектора — как в боевом FlowEstimator
LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
FEAT = dict(maxCorners=MAX_FEATS, qualityLevel=0.01, minDistance=8, blockSize=7)
BACK_TOL = 1.0          # px — допуск обратной проверки (отсев переприлипших)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read(bag):
    br = CvBridge()
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    frames, od = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/image_color':
            m = deserialize_message(raw, Image)
            frames.append((stamp(m), cv2.cvtColor(br.imgmsg_to_cv2(m, 'bgr8'),
                                                 cv2.COLOR_BGR2GRAY)))
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((stamp(m), p.x, p.y, p.z))
    frames.sort(key=lambda f: f[0])
    return frames, np.array(od)


def track(prev, cur, pts):
    """КЛТ вперёд + обратно; возвращает маску выживших и новые координаты."""
    p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, pts.reshape(-1, 1, 2), None, **LK)
    p0b, st2, _ = cv2.calcOpticalFlowPyrLK(cur, prev, p1, None, **LK)
    ok = (st.reshape(-1) == 1) & (st2.reshape(-1) == 1)
    back = np.linalg.norm(p0b.reshape(-1, 2) - pts, axis=1)
    ok &= back < BACK_TOL
    return ok, p1.reshape(-1, 2)


def main():
    frames, od = read(BAG)
    print(f'бэг {BAG}: кадров {len(frames)}, одометрии {len(od)}')
    if not len(frames) or not len(od):
        sys.exit('⚠️ нет кадров или одометрии')
    h = od[od[:, 3] > HOVER_Z]
    if len(h) < 20:
        sys.exit('⚠️ висения нет')
    t0, t1 = h[0, 0], h[-1, 0]
    frames = [f for f in frames if t0 <= f[0] <= t1]
    if len(frames) < 20:
        sys.exit('⚠️ кадров в висении мало')

    prev = frames[0][1]
    pts = cv2.goodFeaturesToTrack(prev, mask=None, **FEAT)
    if pts is None or len(pts) < 20:
        sys.exit('⚠️ на старте висения не нашлось точек')
    pts = pts.reshape(-1, 2)
    n0 = len(pts)
    print(f'домашняя опора посеяна: {n0} точек, висение {t1 - t0:.0f} с\n')

    ts, frac, dist = [], [], []
    for t, gray in frames[1:]:
        ok, nxt = track(prev, gray, pts)
        pts = nxt[ok]
        prev = gray
        d = math.hypot(np.interp(t, h[:, 0], h[:, 1]) - h[0, 1],
                       np.interp(t, h[:, 0], h[:, 2]) - h[0, 2])
        ts.append(t - t0)
        frac.append(len(pts) / n0)
        dist.append(d)
        if len(pts) < 4:
            print(f'все точки дома потеряны на {t - t0:.1f} с, удаление {d:.1f} м')
            break
    ts, frac, dist = np.array(ts), np.array(frac), np.array(dist)

    print(f"{'t,с':>5s} | {'удаление,м':>10s} | {'доля дома':>9s}")
    for i in np.linspace(0, len(ts) - 1, min(14, len(ts))).astype(int):
        print(f'{ts[i]:5.1f} | {dist[i]:10.1f} | {frac[i]:9.2f}')

    print(f'\ncorr(доля дома, удаление) = {np.corrcoef(frac, dist)[0, 1]:+.2f}')
    for thr in (0.5, 0.25, 0.1):
        below = np.where(frac <= thr)[0]
        if len(below):
            i = below[0]
            print(f'доля дома упала до {thr:.2f} на {ts[i]:5.1f} с, '
                  f'удаление {dist[i]:5.1f} м')
        else:
            print(f'доля дома до {thr:.2f} не падала (минимум {frac.min():.2f} '
                  f'при удалении {dist[np.argmin(frac)]:.1f} м)')
    # монотонность: доля обязана падать с удалением, иначе как измеритель не годится
    order = np.argsort(dist)
    print(f'корреляция рангов (Спирмен) = '
          f'{np.corrcoef(np.argsort(np.argsort(frac)), np.argsort(np.argsort(dist)))[0, 1]:+.2f}')
    print(f'доля дома в конце {frac[-1]:.2f} при удалении {dist[-1]:.1f} м')


if __name__ == '__main__':
    main()
