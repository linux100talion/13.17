#!/usr/bin/env python3
"""vins_sane_replay.py — ПРОКРУТИТЬ настоящий гейт здоровья VINS (Handover.vins_sane)
по bag и увидеть, когда и ПОЧЕМУ он объявил бы VINS больным.

Зачем: гейт живёт в лётной ноде, его счётчики/латчи в bag не пишутся — по одному
tier= в /mission/status не видно, какой из трёх каналов сработал (потолок, физика
висения, занижение против IPM) и не сработал бы ложно на здоровом полёте.
Реплей кормит РЕАЛЬНЫЙ класс тем же, что видела нода: скорость VINS по штампам
(VinsTrack, как в RosTelemetry), IPM-канал из /flow_dbg8|9, высота перцепции и
стики из /mission/status (palt=/rcr=/rcp=), sim-сетка 0.05 с. Рядом — истина
Gazebo, чтобы судить, прав ли был гейт.

Так проверялся чек занижения (config.vins_scale_*): положительный bag
lv2_joy_20260905_114248 (унос при VINS «0.5 м/с») и отрицательные — контрольные
полёты 132408/133636 и серии 2026-09-03/04 (ветер 10, руки пилота).

Запуск внутри p1317_nav (src/lab смонтирован как /lab, control_pkg — в
/root/sim_ws/src/control):
  docker exec p1317_nav bash -lc "source /opt/ros/humble/setup.bash; \\
      source /root/sim_ws/install/setup.bash; \\
      python3 /lab/vins_sane_replay.py /root/sim_ws/output/joystick/<RUN>/bag \\
          [--scale-ratio 0.5 --scale-alt-max 4 --scale-sec 3 --hover-v 3 --sane-n 10]"
Стек не трогает — только чтение bag (дисциплина прогонов не нарушается).
"""
import argparse
import bisect
import math
import sys

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from std_msgs.msg import String

sys.path.insert(0, '/root/sim_ws/src/control')
sys.path.insert(0, '/ctl')
from control_pkg.application.handover import VinsHandover      # noqa: E402
from control_pkg.application.vins_track import VinsTrack       # noqa: E402
from control_pkg.domain.rc import RC_CENTER                    # noqa: E402
from control_pkg.domain.state import DroneState                # noqa: E402

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    r.set_filter(StorageFilter(topics=['/odometry', '/flow_dbg8', '/flow_dbg9',
                                       '/mission/status', '/model/iris_cam/odometry']))
    od, d8, d9, stat, gt = [], [], [], [], []
    while r.has_next():
        t, raw, _ = r.read_next()
        if t == '/odometry':
            m = deserialize_message(raw, Odometry)
            od.append((st(m), m.pose.pose.position.x, m.pose.pose.position.y))
        elif t == '/flow_dbg8':
            m = deserialize_message(raw, Vector3Stamped)
            d8.append((st(m), m.vector.y, m.vector.z > 0.5))     # (vfwd м/с, ok)
        elif t == '/flow_dbg9':
            m = deserialize_message(raw, Vector3Stamped)
            d9.append((st(m), m.vector.x))                        # vlat м/с
        elif t == '/mission/status':
            d = dict(kv.partition('=')[::2] for kv in
                     deserialize_message(raw, String).data.split())
            if 't' in d:
                stat.append((float(d['t']), d))
        elif t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            v = m.twist.twist.linear
            gt.append((st(m), math.hypot(v.x, v.y), m.pose.pose.position.z))
    for a in (od, d8, d9, stat, gt):
        a.sort(key=lambda x: x[0])
    return od, d8, d9, stat, gt


def latest(times, arr, t):
    i = bisect.bisect_right(times, t) - 1
    return arr[i] if i >= 0 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--v-max', type=float, default=12.0)
    ap.add_argument('--sane-n', type=int, default=10)
    ap.add_argument('--hover-v', type=float, default=3.0)
    ap.add_argument('--hover-sec', type=float, default=2.0)
    ap.add_argument('--scale-ratio', type=float, default=0.5)
    ap.add_argument('--scale-ipm-min', type=float, default=2.0)
    ap.add_argument('--scale-sec', type=float, default=3.0)
    ap.add_argument('--scale-alt-max', type=float, default=4.0)
    ap.add_argument('--scale-hold', type=float, default=30.0)
    ap.add_argument('--dt', type=float, default=0.05)
    ap.add_argument('--vins-scale', type=float, default=1.0,
                    help='ЭМУЛЯЦИЯ коллапса масштаба: скорость VINS × k (0.2 = реборн с масштабом 0.2)')
    a = ap.parse_args()

    od, d8, d9, stat, gt = load(a.bag)
    if not od or not stat:
        sys.exit("в bag нет /odometry или /mission/status")
    # скорость VINS по штампам — тем же VinsTrack, что в RosTelemetry
    tr, vel = VinsTrack(), []
    for t, x, y in od:
        tr.on_odom(t, x, y)
        vel.append((t, tr.vx * a.vins_scale, tr.vy * a.vins_scale))
    T_od = [v[0] for v in vel]
    T8 = [x[0] for x in d8]
    T9 = [x[0] for x in d9]
    T_st = [x[0] for x in stat]
    T_gt = [x[0] for x in gt]

    ho = VinsHandover(None, min_count=300, fresh_sec=2.0, v_max=a.v_max,
                      sane_n=a.sane_n, hover_v=a.hover_v, hover_sec=a.hover_sec,
                      scale_ratio=a.scale_ratio, scale_ipm_min=a.scale_ipm_min,
                      scale_sec=a.scale_sec, scale_alt_max=a.scale_alt_max,
                      scale_hold=a.scale_hold)
    t0, t1 = T_st[0], T_st[-1]
    n = int((t1 - t0) / a.dt)
    was = True
    events, under_ticks, insane_ticks = [], 0, 0
    trips_prev = 0
    alt_min = alt_max = None
    t = t0
    for _ in range(n):
        t += a.dt
        v = latest(T_od, vel, t)
        if v is None:
            continue
        r8 = latest(T8, d8, t)
        r9 = latest(T9, d9, t)
        sd = latest(T_st, stat, t)[1]
        palt = sd.get('palt', '--')
        perc_alt = float(palt) if palt not in ('--', None) else None
        s = DroneState(now_sim=t, vins_valid=True, vins_odom_count=300,
                       vins_last_sim=v[0], vins_vx=v[1], vins_vy=v[2],
                       ipm_ok=bool(r8[2]) if r8 else False,
                       ipm_vfwd=r8[1] if r8 else 0.0, ipm_vlat=r9[1] if r9 else 0.0,
                       perc_alt=perc_alt, rel_alt=float(sd.get('alt', 0.0)),
                       pilot_roll=RC_CENTER + int(float(sd.get('rcr', 0))),
                       pilot_pitch=RC_CENTER + int(float(sd.get('rcp', 0))))
        if sd.get('tier') in ('1', '2') and perc_alt is not None:
            alt_min = perc_alt if alt_min is None else min(alt_min, perc_alt)
            alt_max = perc_alt if alt_max is None else max(alt_max, perc_alt)
        sane = ho.vins_sane(s)
        if ho._scale_bad_since is not None:
            under_ticks += 1
        if not sane:
            insane_ticks += 1
        cause = ('scale' if t < ho._scale_until
                 else 'cap' if math.hypot(s.vins_vx, s.vins_vy) > a.v_max > 0
                 else 'hover' if ho._bad >= a.sane_n else '-')
        if was and not sane or ho.scale_trips != trips_prev:
            g = latest(T_gt, gt, t)
            events.append((t, cause, math.hypot(s.vins_vx, s.vins_vy),
                           math.hypot(s.ipm_vfwd, s.ipm_vlat), perc_alt,
                           g[1] if g else float('nan'), sd.get('tier')))
            trips_prev = ho.scale_trips
        was = sane

    print(f"bag: {a.bag}")
    print(f"  сетка {a.dt} с, {n} тиков, {t0:.1f}–{t1:.1f} sim-с; высота перцепции на "
          f"ярусах ≥1: {alt_min if alt_min is not None else '--'}–"
          f"{alt_max if alt_max is not None else '--'} м")
    print(f"  ручки: v_max {a.v_max} sane_n {a.sane_n} hover_v {a.hover_v} "
          f"scale ratio {a.scale_ratio} ipm_min {a.scale_ipm_min} sec {a.scale_sec} "
          f"alt_max {a.scale_alt_max} hold {a.scale_hold}"
          + (f" | ЭМУЛЯЦИЯ: VINS × {a.vins_scale}" if a.vins_scale != 1.0 else ""))
    print(f"  фронты sane→insane: {len(events)}; тиков insane {insane_ticks} "
          f"({insane_ticks * a.dt:.1f} с); тиков с условием занижения "
          f"{under_ticks} ({under_ticks * a.dt:.1f} с); срабатываний чека занижения "
          f"{ho.scale_trips}")
    if events:
        print("   t(sim)  причина  |vins_v|  |ipm_v|  palt  истина|v|  tier(bag)")
        for e in events:
            print(f"  {e[0]:7.2f}  {e[1]:7s}  {e[2]:7.2f}  {e[3]:7.2f}  "
                  f"{e[4] if e[4] is not None else float('nan'):5.2f}  {e[5]:8.2f}  {e[6]}")


if __name__ == '__main__':
    main()
