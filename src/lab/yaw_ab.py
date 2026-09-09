#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yaw_ab — СУДЬЯ A/B «ЧЕЙ КУРС ДЕРЖИТ EKF»: компас против VINS (cmd/yaw_ab).

Одна таблица по нескольким прогонам. Вопрос кампании один: даёт ли `EK3_SRC1_YAW=6`
(курс EKF от нашей позы) выигрыш против компаса — и не платим ли мы за него.

Что считает по каждому bag:

  yawsrc          чей курс реально стоял (поле статуса; у стороны vins обязан
                  переключиться после латча возврата);
  Δyaw якоря      значение после латча и РАЗМАХ за полёт. Обязан СТОЯТЬ: угол
                  задаётся при рождении рамы, подтяжка его больше не трогает
                  (frame_anchor, разбор 173415). Поехал — читать лог ray_tracer;
  ОШИБКА КУРСА    |курс EKF − курс истины Gazebo|, медиана и 90 %. Считается ТОЛЬКО
                  на спокойных участках (|ω| < --wz, по умолчанию 5 °/с): на вираже
                  сравнение меряет разницу ЗАДЕРЖЕК трактов, а не качество источника
                  (замер 171337: размах Δyaw 5.1° при 41.6 °/с против 2.4° при 9.5);
  ДРЕЙФ EKF       |(EKF − EKF₀) − (истина − истина₀)| на конец полёта — цена в метрах.
                  Кадры EKF и истины оба ENU, привязка по моменту отрыва;
  ПРОМАХ          |точка касания − точка отрыва| по истине.

⚠️ Числа сравнимы, только если полёты СХОЖИ по схеме (см. cmd/yaw_ab/README.txt):
длительность, удаление и количество разворотов входят в ошибку курса напрямую.

Запуск С ХОСТА (ROS jazzy) или в контейнере nav:
    python3 src/lab/yaw_ab.py docker/sim/output/joystick/<RUN> [<RUN> ...]
    docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
      python3 /lab/yaw_ab.py /root/sim_ws/output/joystick/<RUN> ...'
"""
import argparse
import bisect
import math
import os
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from std_msgs.msg import String

TOPICS = ['/mission/status', '/model/iris_cam/odometry', '/mavros/local_position/pose',
          '/mavros/state', '/nn1/bridge']


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def bag_dir(path):
    """Каталог прогона или сам bag — принимаем и то, и другое."""
    inner = os.path.join(path, 'bag')
    return inner if os.path.isdir(inner) else path


def load(path):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir(path), storage_id='sqlite3'),
           ConverterOptions('', ''))
    have = {t.name for t in r.get_all_topics_and_types()}
    r.set_filter(StorageFilter(topics=[t for t in TOPICS if t in have]))
    d = {'st': [], 'tr': [], 'ek': [], 'arm': [], 'br': []}
    while r.has_next():
        top, raw, t = r.read_next()
        ts = t * 1e-9
        if top == '/mission/status':
            d['st'].append((ts, deserialize_message(raw, String).data))
        elif top == '/model/iris_cam/odometry':
            p = deserialize_message(raw, Odometry).pose.pose
            d['tr'].append((ts, p.position.x, p.position.y, quat_yaw(p.orientation),
                            p.position.z))
        elif top == '/mavros/local_position/pose':
            p = deserialize_message(raw, PoseStamped).pose
            d['ek'].append((ts, p.position.x, p.position.y, quat_yaw(p.orientation)))
        elif top == '/mavros/state':
            d['arm'].append((ts, deserialize_message(raw, State).armed))
        elif top == '/nn1/bridge':
            d['br'].append((ts, deserialize_message(raw, String).data))
    return d


def field(line, key):
    for w in line.split():
        if w.startswith(key + '='):
            return w[len(key) + 1:]
    return None


def stats(path, wz_max):
    d = load(path)
    if not d['tr'] or not d['ek']:
        return None
    trt = [p[0] for p in d['tr']]

    def truth_at(ts):
        """Интерполяция истины + скорость разворота на этом участке."""
        i = min(max(bisect.bisect_left(trt, ts), 1), len(d['tr']) - 1)
        a, b = d['tr'][i - 1], d['tr'][i]
        dt = b[0] - a[0]
        if dt <= 0:
            return a[1], a[2], a[3], 0.0
        k = (ts - a[0]) / dt
        w = math.degrees(wrap(b[3] - a[3])) / dt
        return (a[1] + k * (b[1] - a[1]), a[2] + k * (b[2] - a[2]),
                a[3] + k * wrap(b[3] - a[3]), w)

    # --- отрыв и касание: по armed (первый/последний) ---
    # ОТРЫВ И КАСАНИЕ — по ВЫСОТЕ ИСТИНЫ, как в rth_check.py (порог 0.3 м над
    # уровнем стоянки). По armed считать нельзя: между армом и отрывом борт стоит
    # и ползёт, а после касания запись идёт ещё десятки секунд — и то и другое
    # натекает в дрейф (192430: 3.65 м «по armed» против 1.09 м по высоте).
    z0 = sorted(x[4] for x in d['tr'][:20])[10] if len(d['tr']) >= 20 else d['tr'][0][4]
    air = [x[0] for x in d['tr'] if x[4] - z0 > 0.3]
    if air:
        up, down = air[0], air[-1]
    else:
        up = next((ts for ts, a in d['arm'] if a), d['ek'][0][0])
        down = next((ts for ts, a in reversed(d['arm']) if a), d['ek'][-1][0])
    down = max(down, up + 1.0)

    base_e = next(p for p in d['ek'] if p[0] >= up)
    bx, by = truth_at(base_e[0])[:2]

    err, drift, miss = [], 0.0, 0.0
    for ts, x, y, ya in d['ek']:
        if not (up <= ts <= down):
            continue
        tx, ty, tya, w = truth_at(ts)
        if abs(w) <= wz_max:
            err.append(abs(math.degrees(wrap(tya - ya))))
        drift = math.hypot((x - base_e[1]) - (tx - bx), (y - base_e[2]) - (ty - by))
    tx, ty, _, _ = truth_at(down)
    miss = math.hypot(tx - bx, ty - by)

    # --- чей курс стоял и что делал угол якоря ---
    src = 'compass'
    for ts, s in d['st']:
        v = field(s, 'yawsrc')
        if v == 'vins':
            src = 'vins'
            break
    # Δyaw смотрим ТОЛЬКО после первого открытия моста (+1 с): до него якорь может
    # быть залатчен ещё на земле, а на открытии latch_yaw задаёт РАБОЧИЙ угол — смена
    # там законна и размахом не считается. После — угол обязан стоять; всё, что
    # шевелится дальше, это перерождение рамы, и его видно именно здесь.
    # «рабочее» открытие — первое open ПОСЛЕ закрытия: именно там срабатывает
    # take_open_reset/latch_yaw. Самая первая строка моста бывает open просто
    # потому, что вердикт зрелости ещё не пришёл.
    seen_closed, t_open = False, None
    for ts, w in d['br']:
        head = w.split()[:1]
        if head == ['closed']:
            seen_closed = True
        elif head == ['open'] and seen_closed:
            t_open = ts
            break
    if t_open is None:
        t_open = next((ts for ts, w in d['br'] if w.split()[:1] == ['open']), None)
    dyaw = [float(w.split()[5]) for ts, w in d['br']
            if len(w.split()) >= 6 and w.split()[5] not in ('-',)
            and (t_open is None or ts >= t_open + 1.0)]
    return dict(src=src, err=sorted(err), drift=drift, miss=miss,
                dyaw0=dyaw[0] if dyaw else None,
                dspan=(max(dyaw) - min(dyaw)) if dyaw else None,
                secs=down - up)


def main():
    ap = argparse.ArgumentParser(description='A/B «чей курс держит EKF»: таблица по прогонам')
    ap.add_argument('runs', nargs='+', help='каталоги прогонов (или сами bag)')
    ap.add_argument('--wz', type=float, default=5.0,
                    help='потолок |ω| для «спокойного» участка, °/с (default 5)')
    a = ap.parse_args()

    print(f"  ошибка курса считается на |ω| < {a.wz:g} °/с (на вираже сравнение меряет "
          f"лаг трактов)\n")
    print(f"  {'прогон':<26} {'курс':<8} {'Δyaw якоря':>12} {'размах':>7} "
          f"{'ошибка курса':>14} {'дрейф':>7} {'промах':>7} {'в возд.':>8}")
    print("  " + "-" * 96)
    rows = []
    for run in a.runs:
        try:
            s = stats(run, a.wz)
        except Exception as e:                      # noqa: BLE001 — стенд, не библиотека
            print(f"  {os.path.basename(run.rstrip('/')):<26} ОШИБКА: {e}")
            continue
        if s is None:
            print(f"  {os.path.basename(run.rstrip('/')):<26} нет истины/позы EKF в bag")
            continue
        rows.append((run, s))
        e = s['err']
        emed = e[len(e) // 2] if e else float('nan')
        e90 = e[int(0.9 * len(e))] if e else float('nan')
        d0 = '--' if s['dyaw0'] is None else f"{s['dyaw0']:+.1f}°"
        ds = '--' if s['dspan'] is None else f"{s['dspan']:.1f}°"
        print(f"  {os.path.basename(run.rstrip('/')):<26} {s['src']:<8} {d0:>12} {ds:>7} "
              f"{emed:>7.2f}° 90%{e90:5.2f}° {s['drift']:>6.2f}м {s['miss']:>6.2f}м "
              f"{s['secs']:>7.0f}с")

    # --- сводка по сторонам ---
    for side in ('compass', 'vins'):
        v = [r for _, r in rows if r['src'] == side]
        if not v:
            continue
        med = sorted(x['err'][len(x['err']) // 2] for x in v if x['err'])
        dr = sorted(x['drift'] for x in v)
        if med:
            print(f"\n  {side:<8}: ошибка курса медиана по прогонам {med[len(med)//2]:.2f}°"
                  f" (n={len(med)}), дрейф {dr[len(dr)//2]:.2f} м")
    print("\n  ⚠️ числа сравнимы только при СХОЖЕЙ схеме полёта — см. cmd/yaw_ab/README.txt")


if __name__ == '__main__':
    sys.exit(main())
