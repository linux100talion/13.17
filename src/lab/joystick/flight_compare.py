#!/usr/bin/env python3
"""flight_compare.py — сравнение freefly-прогонов joystick-серии бок о бок.

В отличие от /lab/flight_cmp.py (кампания гейтов: фикс-окна от отрыва), тут
фазы выравниваются ПО СОБЫТИЯМ цикла freefly: арм → отрыв → CH6-центр →
латч LOITER (если в bag есть /mavros/state) → касание → дизарм. На каждый
прогон — тайминги событий, метрики пути (макс. удаление/высота, длина пути),
«рулёжка» (RMS стиков в воздухе — сколько пилот/реплей работал руками) и
жизнь VINS (/odometry). Плюс экспорт траекторий в CSV для графиков.

Запуск ВНУТРИ p1317_nav (bag'и — имена прогонов из output/joystick/ или пути):
  python3 /lab/joystick/flight_compare.py lv1_joy_20260822_182409 lv1_replay_20260822_201352
Env: CMP_OUT (default /root/sim_ws/output/joystick/compare) — куда CSV.
"""
import math
import os
import sys

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Joy

sys.path.insert(0, os.path.dirname(__file__))
from joy_timeline import default_signs, zoh  # noqa: E402  (те же конвенции)

GESTURE_LVL, GESTURE_MIN = 0.85, 0.5
LIFT, TOUCH = 0.3, 0.15


def read(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))
    joy, gt, st, vins = [], [], [], []
    while r.has_next():
        topic, raw, t_ns = r.read_next()
        if topic == '/joy':
            m = deserialize_message(raw, Joy)
            joy.append((t_ns, list(m.axes) + [0.0] * 6))
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            gt.append((t_ns, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                       p.x, p.y, p.z))
        elif topic == '/mavros/state':
            from mavros_msgs.msg import State
            m = deserialize_message(raw, State)
            st.append((t_ns, bool(m.armed), m.mode))
        elif topic == '/odometry':
            vins.append(t_ns)
    return joy, np.array(gt), st, vins


def analyze(bag, signs):
    joy, gt, st, vins = read(bag)
    if len(gt) < 10 or not joy:
        sys.exit(f"ОШИБКА: в {bag} мало данных (gt={len(gt)}, joy={len(joy)})")
    ns2sim = lambda ns: np.interp(ns, gt[:, 0], gt[:, 1])
    t0 = float(gt[0, 1])
    # gt-метрики
    x0 = float(np.median(gt[:20, 2])); y0 = float(np.median(gt[:20, 3]))
    z0 = float(np.median(gt[:20, 4]))
    tt = gt[:, 1] - t0
    alt = gt[:, 4] - z0
    dist = np.hypot(gt[:, 2] - x0, gt[:, 3] - y0)
    path = float(np.sum(np.hypot(np.diff(gt[:, 2]), np.diff(gt[:, 3]))))
    # стики (семантика) на сетке 20 Гц
    jt = ns2sim(np.array([j[0] for j in joy], dtype=np.int64)) - t0
    ja = np.array([j[1][:6] for j in joy])
    sem = {k: signs[i] * ja[:, i] for i, k in
           enumerate(('roll', 'pitch', 'thr', 'yaw'))}
    grid = np.arange(jt[0], jt[-1], 0.05)
    sg = {k: zoh(jt, v, grid) for k, v in sem.items()}
    sw = zoh(jt, ja[:, 5], grid)
    ev = {}
    # события: FCU-state (если есть) → точные арм/режимы; иначе — по жестам
    if st:
        s_t = ns2sim(np.array([s[0] for s in st], dtype=np.int64)) - t0
        pa, pm = None, None
        for (ns, armed, mode), t in zip(st, s_t):
            if pa is not None and armed and not pa:
                ev.setdefault('арм', t)
            if pa and not armed:
                ev.setdefault('дизарм', t)
            if pm not in (None, mode) and mode == 'LOITER':
                ev.setdefault('латч LOITER', t)
            if pm not in (None, mode) and mode == 'LAND':
                ev.setdefault('FCU→LAND', t)
            pa, pm = armed, mode
    else:
        g = (sg['thr'] < -GESTURE_LVL) & (sg['yaw'] > GESTURE_LVL)
        i = np.argmax(g) if g.any() else None
        if i is not None:
            ev['арм'] = float(grid[i]) + GESTURE_MIN   # ≈ конец удержания
        g = (sg['thr'] < -GESTURE_LVL) & (sg['yaw'] < -GESTURE_LVL)
        if g.any():
            ev['дизарм'] = float(grid[np.argmax(g)])
    up = np.where(alt > LIFT)[0]
    if len(up):
        ev['отрыв'] = float(tt[up[0]])
        imax = int(np.argmax(alt))
        dn = np.where((tt > tt[imax]) & (alt < TOUCH))[0]
        if len(dn):
            ev['касание'] = float(tt[dn[0]])
    c = np.where((sw > -0.5) & (sw < 0.5))[0]         # CH6 в центре
    if len(c) and sw[0] <= -0.5:
        ev['CH6-центр (перв.)'] = float(grid[c[0]])
    # рулёжка в воздухе (между отрывом и касанием)
    air = ((grid >= ev.get('отрыв', 0)) &
           (grid <= ev.get('касание', grid[-1])))
    rms = {k: float(np.sqrt(np.mean(sg[k][air] ** 2))) if air.any() else 0.0
           for k in ('roll', 'pitch', 'yaw')}
    vt = (ns2sim(np.array(vins, dtype=np.int64)) - t0) if vins else np.array([])
    d_at_c = (float(np.interp(ev['CH6-центр (перв.)'], tt, dist))
              if 'CH6-центр (перв.)' in ev else float('nan'))
    return {
        'события': ev, 'длительность': float(tt[-1]),
        'макс. высота, м': float(alt.max()),
        'макс. удаление, м': float(dist.max()),
        'удаление при CH6-центр, м': d_at_c,
        'длина пути (гориз.), м': path,
        'RMS стиков в воздухе (r/p/y)':
            f"{rms['roll']:.2f}/{rms['pitch']:.2f}/{rms['yaw']:.2f}",
        'VINS /odometry': (f"{len(vins)} сообщ, {vt[0]:.0f}..{vt[-1]:.0f}с"
                           if len(vt) else 'нет в bag'),
        '_traj': np.column_stack([tt, gt[:, 2] - x0, gt[:, 3] - y0, alt, dist]),
    }


def main():
    if len(sys.argv) < 3:
        sys.exit("нужно ≥2 bag'а (имена прогонов или пути)")
    signs = default_signs()
    out = os.environ.get('CMP_OUT', '/root/sim_ws/output/joystick/compare')
    os.makedirs(out, exist_ok=True)
    runs = {}
    for a in sys.argv[1:]:
        bag = a if '/' in a else f'/root/sim_ws/output/joystick/{a}/bag'
        name = a.rstrip('/').split('/')[-2] if '/' in a else a
        runs[name] = analyze(bag, signs)
        np.savetxt(os.path.join(out, f'{name}_traj.csv'), runs[name]['_traj'],
                   delimiter=',', header='t,x,y,alt,dist', comments='')
    names = list(runs)
    w = max(len(n) for n in names) + 2
    print(f"\n{'':34s}" + ''.join(f"{n:>{w}s}" for n in names))
    all_ev = []
    for r in runs.values():
        for k in r['события']:
            if k not in all_ev:
                all_ev.append(k)
    for k in sorted(all_ev, key=lambda k: min(
            r['события'].get(k, 1e9) for r in runs.values())):
        print(f"  {k:32s}" + ''.join(
            f"{r['события'].get(k, float('nan')):>{w}.1f}" for r in runs.values()))
    for k in ('длительность', 'макс. высота, м', 'макс. удаление, м',
              'удаление при CH6-центр, м', 'длина пути (гориз.), м'):
        print(f"  {k:32s}" + ''.join(
            f"{r[k]:>{w}.1f}" for r in runs.values()))
    for k in ('RMS стиков в воздухе (r/p/y)', 'VINS /odometry'):
        print(f"  {k:32s}" + ''.join(f"{str(r[k]):>{w}s}" for r in runs.values()))
    print(f"\nтраектории → {out}/<имя>_traj.csv (t,x,y,alt,dist)")


if __name__ == '__main__':
    main()
