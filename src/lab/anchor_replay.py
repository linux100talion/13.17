#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""anchor_replay — A/B СПАРИВАНИЯ ПОЗ ЯКОРЯ ПО BAG: «по приходу» против «по штампу».

Якорь кадра (`nav_pkg/nn1/frame_anchor.py`) считает раму по ПАРЕ поз — своей (VINS) и
полётника (EKF). До 2026-09-09 пара бралась ПО ПРИХОДУ: последняя поза EKF против
только что пришедшей одометрии. Но одометрия считается на кадре и опаздывает, причём
неравномерно (замер 181233/181936: медиана 0.14–0.15 с, 90 % 0.20–0.22 с, хвост 0.33 с;
⚠️ мерить это по bag можно только СНЯВ расхождение ТЕМПА sim/wall — 1 % за полёт даёт
секунду мнимой задержки и выглядит как катастрофа). Лечение — `PoseBuffer`: поза EKF
берётся НА МОМЕНТ ШТАМПА кадра интерполяцией.

Стенд гоняет ДВА якоря по одному bag'у — со старым и новым спариванием — и печатает:

  РАЗЪЕЗД ПАРЫ   |поза EKF последняя − поза EKF на момент штампа|, м. Это и есть
                 цена лага в метрах: столько мнимого расхода видел старый якорь;
  РАСХОД         |EKF − map(VINS)| на каждом обновлении (то, за чем якорь гонится).
                 Меньше — лучше: якорь чинит дрейф, а не собственный лаг;
  ПОДТЯЖЕК       сколько раз расход перевалил anchor_relatch_m (жёсткая подтяжка).
                 Каждая — рывок трансляции в полётнике, лишние не нужны.

Запуск (нужен nav_pkg в PYTHONPATH — проще в контейнере nav):
    docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
      source /root/sim_ws/install/setup.bash;
      python3 /lab/anchor_replay.py /root/sim_ws/output/joystick/<RUN>'
"""
import argparse
import math
import os
import sys

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from std_msgs.msg import String

from nav_pkg.nn1.frame_anchor import FrameAnchor, quat_yaw
from nav_pkg.nn1.pose_buffer import PoseBuffer


def bag_dir(path):
    inner = os.path.join(path, 'bag')
    return inner if os.path.isdir(inner) else path


def main():
    ap = argparse.ArgumentParser(description='A/B спаривания поз якоря по bag')
    ap.add_argument('bag', help='каталог прогона или сам bag')
    ap.add_argument('--relatch-m', type=float, default=1.0)
    ap.add_argument('--tau-sec', type=float, default=5.0)
    ap.add_argument('--grace-sec', type=float, default=10.0)
    ap.add_argument('--buf-sec', type=float, default=3.0)
    a = ap.parse_args()

    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir(a.bag), storage_id='sqlite3'), ConverterOptions('', ''))
    have = {t.name for t in r.get_all_topics_and_types()}
    want = [t for t in ('/odometry', '/mavros/local_position/pose', '/nn1/bridge') if t in have]
    if '/odometry' not in want or '/mavros/local_position/pose' not in want:
        print('  в bag нет одометрии VINS или позы EKF — сравнивать нечего')
        return 1
    r.set_filter(StorageFilter(topics=want))

    anc_old = FrameAnchor(relatch_m=a.relatch_m, tau_sec=a.tau_sec, grace_sec=a.grace_sec)
    anc_new = FrameAnchor(relatch_m=a.relatch_m, tau_sec=a.tau_sec, grace_sec=a.grace_sec)
    buf = PoseBuffer(a.buf_sec)
    last = None                     # последняя ПРИШЕДШАЯ поза EKF: (x, y, z, yaw)
    open_now, seen_closed, started = False, False, False
    gap, dn_old, dn_new = [], [], []

    def dev(anc, vp, vy, ep, ey, now):
        """Расход ДО обновления (за чем гонится якорь) + сам шаг якоря."""
        d = (float(np.linalg.norm((ep - anc.rotate(vp))[:2] - anc.t[:2]))
             if anc.latched else 0.0)
        anc.update(vp, vy, ep, ey, now)
        return d

    while r.has_next():
        top, raw, t = r.read_next()
        ts = t * 1e-9
        if top == '/mavros/local_position/pose':
            m = deserialize_message(raw, PoseStamped)
            y = quat_yaw(*(getattr(m.pose.orientation, k) for k in 'xyzw'))
            last = (m.pose.position.x, m.pose.position.y, m.pose.position.z, y)
            buf.push(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, *last)
        elif top == '/nn1/bridge':
            head = deserialize_message(raw, String).data.split()[:1]
            if head == ['closed']:
                seen_closed, open_now = True, False
            elif head == ['open']:
                open_now = True
                started = started or seen_closed     # «рабочее» открытие
        elif top == '/odometry':
            if not (started and open_now) or last is None:
                continue
            m = deserialize_message(raw, Odometry)
            th = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            vp = np.array([m.pose.pose.position.x, m.pose.pose.position.y,
                           m.pose.pose.position.z])
            vy = quat_yaw(*(getattr(m.pose.pose.orientation, k) for k in 'xyzw'))
            at = buf.at(th) if buf.ready(th) else last
            gap.append(math.hypot(last[0] - at[0], last[1] - at[1]))
            dn_old.append(dev(anc_old, vp, vy, np.array(last[:3]), last[3], ts))
            dn_new.append(dev(anc_new, vp, vy, np.array(at[:3]), at[3], ts))

    if len(dn_old) < 50:
        print(f'  слишком мало обновлений якоря ({len(dn_old)}) — мост почти не открывался')
        return 1

    def q(v, name, unit='м'):
        v = sorted(v)
        n = len(v)
        print(f"  {name}: медиана {v[n//2]:6.3f} {unit}   90% {v[int(0.9*n)]:6.3f} {unit}"
              f"   макс {v[-1]:6.3f} {unit}")

    print(f"  {os.path.basename(a.bag.rstrip('/'))}: обновлений якоря {len(dn_old)}, "
          f"порог подтяжки {a.relatch_m:g} м\n")
    q(gap, 'РАЗЪЕЗД ПАРЫ (цена лага)  ')
    q(dn_old, 'РАСХОД, пара ПО ПРИХОДУ   ')
    q(dn_new, 'РАСХОД, пара ПО ШТАМПУ    ')
    print(f"\n  ПОДТЯЖЕК: по приходу {anc_old.relatch_n}, по штампу {anc_new.relatch_n}")
    print(f"  Δyaw якоря: по приходу {math.degrees(anc_old.yaw_off):+.1f}°, "
          f"по штампу {math.degrees(anc_new.yaw_off):+.1f}° "
          "(угол задаётся при рождении рамы, подтяжка его не трогает)")
    mo = sorted(dn_old)[len(dn_old) // 2]
    mn = sorted(dn_new)[len(dn_new) // 2]
    if mo > 0:
        print(f"  ИТОГ: медианный расход {mo:.3f} → {mn:.3f} м "
              f"({100.0 * (mo - mn) / mo:+.0f} %)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
