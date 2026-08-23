#!/usr/bin/env python3
"""phase_stats.py — временнáя статистика фаз freefly-прогона по bag'ам.

Фазы (все длительности в sim-секундах, границы — из топиков bag):
  prearm     старт лётной ноды (первый /mission/status) → латч ALT_HOLD
  ekf_gps    ALT_HOLD → ARMED: шаг ekf_warmup (EKF держит позицию по GPS)
             + руддер-жест (~3 с проходит на FCU только в шаге freefly)
  climb      ARMED → отрыв (gt z − базлайн > 0.3 м)
  vins_init  отрыв → ПЕРВЫЙ /odometry (монокуляру нужен параллакс —
             инициализация возможна только в движении)
  ripe_swap  первый /odometry → st=READY в /mission/status: зрелость VINS
             (ripe_n=600 odom для freefly) + очередь EK3_SRC1_* + свежесть
  pilot      READY → щелчок CH6 в центр (в реплее задан сценарием!)
  loiter     CH6 центр → mode=LOITER в /mavros/state (латч FCU)
  total      старт ноды → LOITER

Bag'и ДО 2026-08-23 не содержат /mission/status → prearm/ripe_swap/pilot
недоступны («—»), остальное считается. Время: header.stamp (sim) у
стемпованных топиков; /mission/status и /joy (wall) — через кусочную карту
wall→sim по receive-времени /model/iris_cam/odometry (как в joy_timeline).

Запуск ВНУТРИ p1317_nav (см. src/lab/CLAUDE.md про SRC):
  python3 /lab/phase_stats.py /root/sim_ws/output/joystick/*/bag
"""
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import String

LIFTOFF_ALT = 0.3          # м над базлайном gt — как в joy_timeline
PHASES = ["prearm", "ekf_gps", "climb", "vins_init", "ripe_swap",
          "pilot", "loiter", "total"]


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read_run(bag):
    from mavros_msgs.msg import State
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"),
           ConverterOptions("cdr", "cdr"))
    topics = {t.name for t in r.get_all_topics_and_types()}
    want = ["/mission/status", "/mavros/state", "/joy", "/odometry",
            "/model/iris_cam/odometry"]
    r.set_filter(StorageFilter(topics=[t for t in want if t in topics]))

    ref = []                       # (recv_ns, sim) — мост wall→sim
    status = []                    # (recv_ns, st)
    states = []                    # (sim, mode, armed)
    joy = []                       # (recv_ns, sw_raw)
    gt = []                        # (sim, z)
    t_odom1 = None
    while r.has_next():
        topic, raw, t_ns = r.read_next()
        if topic == "/model/iris_cam/odometry":
            m = deserialize_message(raw, Odometry)
            ref.append((t_ns, stamp(m)))
            gt.append((stamp(m), m.pose.pose.position.z))
        elif topic == "/mission/status":
            m = deserialize_message(raw, String)
            kv = dict(p.split("=", 1) for p in m.data.split() if "=" in p)
            status.append((t_ns, kv.get("st", "?")))
        elif topic == "/mavros/state":
            m = deserialize_message(raw, State)
            states.append((stamp(m), m.mode, bool(m.armed)))
        elif topic == "/joy":
            m = deserialize_message(raw, Joy)
            a = list(m.axes) + [0.0] * 6
            joy.append((t_ns, a[5]))
        elif topic == "/odometry" and t_odom1 is None:
            t_odom1 = stamp(deserialize_message(raw, Odometry))

    ref_ns = np.array([x[0] for x in ref], dtype=np.int64)
    ref_sim = np.array([x[1] for x in ref])

    def to_sim(ns):
        if len(ref) >= 2:
            return float(np.interp(ns, ref_ns, ref_sim))
        return None

    # --- границы фаз ---
    t = {}
    if status:
        t["start"] = to_sim(status[0][0])
        ready = [s for s in status if s[1] == "READY"]
        if ready:
            t["ready"] = to_sim(ready[0][0])
    for ts, mode, _a in states:
        if mode == "ALT_HOLD":
            t["althold"] = ts
            break
    for ts, _m, armed in states:
        if armed:
            t["armed"] = ts
            break
    if gt and "armed" in t:
        z = np.array([g[1] for g in gt])
        zt = np.array([g[0] for g in gt])
        z0 = np.median(z[:max(1, min(20, len(z)))])
        up = np.where((z - z0 > LIFTOFF_ALT) & (zt > t["armed"]))[0]
        if len(up):
            t["liftoff"] = float(zt[up[0]])
    if t_odom1 is not None:
        t["odom1"] = t_odom1
    # CH6 → центр: первый переход в |raw|<0.5 после арма (реплей: щелчок один)
    if joy and "armed" in t:
        prev_c = None
        for ns, raw_v in joy:
            c = abs(raw_v) < 0.5
            ts = to_sim(ns)
            if ts is not None and ts > t["armed"] and c and prev_c is False:
                t["ch6"] = ts
                break
            prev_c = c
    for ts, mode, _a in states:
        if mode == "LOITER":
            t["loiter"] = ts
            break

    def dur(a, b):
        return t[b] - t[a] if a in t and b in t else None
    return {
        "prearm": dur("start", "althold"),
        "ekf_gps": dur("althold", "armed"),
        "climb": dur("armed", "liftoff"),
        "vins_init": dur("liftoff", "odom1"),
        "ripe_swap": dur("odom1", "ready"),
        "pilot": dur("ready", "ch6"),
        "loiter": dur("ch6", "loiter"),
        "total": dur("start", "loiter"),
    }


def main():
    bags = sys.argv[1:]
    if not bags:
        sys.exit("нужно: phase_stats.py <bag_dir> [...]")
    rows = []
    for bag in bags:
        name = bag.rstrip("/").split("/")[-2]      # .../<RUN>/bag
        try:
            rows.append((name, read_run(bag)))
        except Exception as e:                     # noqa: BLE001 — свод не падает
            print(f"⚠️ {name}: {e}")

    def fmt(v):
        return f"{v:7.1f}" if v is not None else "      —"

    w = max(len(n) for n, _ in rows)
    print(f"{'прогон':<{w}} " + " ".join(f"{p:>7}" for p in PHASES))
    for name, d in rows:
        print(f"{name:<{w}} " + " ".join(fmt(d[p]) for p in PHASES))
    print()
    print(f"{'сводка (по имеющимся)':<{w}}")
    for p in PHASES:
        vals = [d[p] for _n, d in rows if d[p] is not None]
        if vals:
            print(f"  {p:<10} n={len(vals)}  mean={np.mean(vals):6.1f}  "
                  f"min={min(vals):6.1f}  max={max(vals):6.1f}")


if __name__ == "__main__":
    main()
