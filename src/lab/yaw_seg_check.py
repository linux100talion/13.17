#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yaw_seg_check — разбор СЕГМЕНТОВ разворота: сколько ЗАКАЗАЛИ и сколько ОТДАЛИ.

Отличие от yaw_check.py (тот про удержание курса в висении): здесь предмет —
токен `yaw_l30`/`yaw_r60`, то есть КОМАНДА на угол. Проверяем три вещи разом:

1. ЗНАК. Куда борт поехал относительно токена. Знаки: истинный курс — ENU
   (влево = +yaw), стик — c_yaw>0 = ВПРАВО. Значит `yaw_l30` обязан дать
   Δyaw ≈ +30°, `yaw_r60` — Δyaw ≈ −60°. Противоположный знак = инверсия в
   тракте команды, и никакие гейны разбирать смысла не имеет.
2. МАСШТАБ. |Δyaw| против заказанного — довернул / перевернул.
3. ОТКУДА ошибка масштаба: из ДАТЧИКА или из УТЕЧКИ накопителя. Считаем
   визуальный курс `head = ∫flow_yaw·dt` (как его считает DpYawHold) двумя
   способами — с утечкой leak и без — и переводим в градусы через S. Если
   без утечки head/S совпал с истиной, а с утечкой нет — виновата утечка,
   а не датчик.

Сегменты ищем по ИСТИННОЙ скорости рыскания (порог WZ_MIN), потому что
yaw-команды в телеметрии пока нет (/flow_dbg несёт roll/pitch — ToDo §9).

Запускать В КОНТЕЙНЕРЕ (или в одноразовом из образа sim-nav):
  docker run --rm -v .../output:/out -v .../src/lab:/lab:ro sim-nav:latest \
    bash -lc 'source /opt/ros/humble/setup.bash; BAG=/out/Y2_kp10_bag python3 /lab/yaw_seg_check.py'

Env: BAG, ODOM_TOPIC (/model/iris_cam/odometry), FLOW_TOPIC (/flow_dbg2),
     S (0.253 px/кадр на °/с), LEAK (8.0 с), WZ_MIN (2.0 °/с), MIN_SEC (1.0).
"""
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

BAG = os.environ.get('BAG', '/out/scene_bag')
ODOM = os.environ.get('ODOM_TOPIC', '/model/iris_cam/odometry')
FLOW = os.environ.get('FLOW_TOPIC', '/flow_dbg2')
S = float(os.environ.get('S', '0.253'))          # px/кадр на °/с
LEAK = float(os.environ.get('LEAK', '8.0'))      # с; 0 = без утечки
WZ_MIN = float(os.environ.get('WZ_MIN', '2.0'))  # °/с — порог «идёт разворот»
MIN_SEC = float(os.environ.get('MIN_SEC', '1.0'))


def read(bag, topics):
    """{топик: [(t_сек, msg)]} — t по штампу заголовка (sim-время), не по записи."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {'nav_msgs/msg/Odometry': Odometry,
           'geometry_msgs/msg/Vector3Stamped': Vector3Stamped}
    out = {t: [] for t in topics}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in out:
            continue
        m = deserialize_message(data, cls[types[topic]])
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        out[topic].append((t, m))
    return out


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def segments(t, wz):
    """Интервалы, где |wz| держится выше порога дольше MIN_SEC."""
    hot = np.abs(wz) > WZ_MIN
    segs, i = [], 0
    while i < len(hot):
        if not hot[i]:
            i += 1
            continue
        j = i
        while j < len(hot) and hot[j]:
            j += 1
        if t[j - 1] - t[i] >= MIN_SEC:
            segs.append((i, j - 1))
        i = j
    return segs


def main():
    d = read(BAG, [ODOM, FLOW])
    od, fl = d[ODOM], d[FLOW]
    if not od:
        print(f'нет {ODOM} в {BAG}')
        return
    t = np.array([x[0] for x in od])
    t -= t[0]
    yaw = np.degrees(np.unwrap([yaw_of(m.pose.pose.orientation) for _, m in od]))
    wz = np.degrees([m.twist.twist.angular.z for _, m in od])
    # сглаживание ω_z: сырой сигнал дрожит, порог по нему рвёт сегмент на клочья
    k = 15
    wzs = np.convolve(wz, np.ones(k) / k, mode='same')

    ft = np.array([x[0] for x in fl]) - od[0][0]
    fy = np.array([m.vector.z for _, m in fl])
    # /flow_dbg2 шлётся каждый ТИК ноды, а flow_yaw обновляется по КАДРУ: берём
    # только точки, где значение сменилось — это и есть покадровая частота
    if len(fy) > 1:
        keep = np.r_[True, np.diff(fy) != 0.0]
        ft, fy = ft[keep], fy[keep]

    print(f'=== {os.path.basename(BAG)} ===')
    print(f'кадров flow: {len(fy)}, длительность {t[-1]:.0f} c, '
          f'курс {yaw[0]:+.1f}° → {yaw[-1]:+.1f}°')

    segs = segments(t, wzs)
    print(f'найдено ротаций: {len(segs)}')
    for n, (i, j) in enumerate(segs, 1):
        d_yaw = yaw[j] - yaw[i]
        dur = t[j] - t[i]
        sel = (ft >= t[i]) & (ft <= t[j])
        head_raw = head_leak = 0.0
        prev = None
        s_pairs = []
        for tk, v in zip(ft[sel], fy[sel]):
            fdt = 0.05 if prev is None else max(1e-3, tk - prev)
            prev = tk
            head_raw += v * fdt
            head_leak += v * fdt
            if LEAK > 0:
                head_leak -= head_leak * min(1.0, fdt / LEAK)
            s_pairs.append((tk, v))
        wz_at = np.interp([p[0] for p in s_pairs], t, wz)
        vv = np.array([p[1] for p in s_pairs])
        corr = np.corrcoef(vv, wz_at)[0, 1] if len(vv) > 2 else float('nan')
        scale = float(np.dot(vv, wz_at) / np.dot(wz_at, wz_at)) if len(vv) > 2 else float('nan')
        print(f'  ротация {n}: {t[i]:6.1f}..{t[j]:6.1f} c ({dur:4.1f} c)  '
              f'Δyaw_истина {d_yaw:+7.1f}°')
        print(f'      визуальный курс: без утечки {head_raw / S:+7.1f}°, '
              f'с утечкой {LEAK:g}c {head_leak / S:+7.1f}°')
        print(f'      датчик в сегменте: S={scale:+.4f} px/кадр на °/с, corr={corr:+.3f}')

    # висения между ротациями: дрейф курса
    if segs:
        bounds = [(segs[a][1], segs[a + 1][0]) for a in range(len(segs) - 1)]
        bounds.append((segs[-1][1], len(t) - 1))
        for n, (i, j) in enumerate(bounds, 1):
            if t[j] - t[i] < 2.0:
                continue
            seg = yaw[i:j + 1]
            print(f'  висение {n}: {t[i]:6.1f}..{t[j]:6.1f} c  '
                  f'дрейф {seg[-1] - seg[0]:+6.1f}°, СКО {seg.std():5.2f}°, '
                  f'размах {seg.max() - seg.min():5.1f}°')
    print(f'  дрожь ω_z за весь полёт: СКО {wz.std():.2f} °/с')


if __name__ == '__main__':
    main()
