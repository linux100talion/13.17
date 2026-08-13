#!/usr/bin/env python3
"""gate_replay.py — ПРОКРУТИТЬ настоящий `_IpmGated` по кадрам из бэга и посчитать,
КТО закрыл ось: правдоподобие, vz или взведение.

Ось молчит — это факт из ленты (тангаж 0 при ipm_vfwd 6 м/с). Причину по бэгу не
видно: счётчики гейта в телеметрию не выходят. Поэтому кормим реальный класс теми же
кадрами (ipm_*, rel_alt с удержанием, sim-время) и смотрим счётчики напрямую.
"""
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from std_msgs.msg import Float64

sys.path.insert(0, '/root/sim_ws/src/control')
sys.path.insert(0, '/ctl')   # одноразовый контейнер: -v src/control:/ctl:ro
from control_pkg.domain.control.stabilization import DpPitchRate   # noqa: E402
from control_pkg.domain.rc import RC_CENTER                        # noqa: E402
from control_pkg.domain.setpoint import Setpoint                   # noqa: E402
from control_pkg.domain.state import DroneState                    # noqa: E402

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    d8, d9, alt, od, d2 = [], [], [], [], []
    while r.has_next():
        t, raw, ts = r.read_next()
        if t == '/flow_dbg8':
            m = deserialize_message(raw, Vector3Stamped)
            d8.append((st(m), m.vector.y, m.vector.z, ts * 1e-9))
        elif t == '/flow_dbg9':
            m = deserialize_message(raw, Vector3Stamped)
            d9.append((st(m), m.vector.x))
        elif t == '/mavros/global_position/rel_alt':
            alt.append((ts * 1e-9, deserialize_message(raw, Float64).data))
        elif t == '/flow_dbg2':
            m = deserialize_message(raw, Vector3Stamped)
            d2.append((st(m), m.vector.x))
        elif t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            od.append((st(m), m.pose.pose.position.z))
    return (np.array(d8), np.array(d9), np.array(alt), np.array(od), np.array(d2))


for name in sys.argv[1:]:
    d8, d9, alt, od, d2 = load(name if '/' in name else f'/out/{name}_bag')
    off = float(np.median(d8[:, 0] - d8[:, 3]))
    alt[:, 0] += off
    t0 = od[int(np.argmax(od[:, 1] > 0.5)), 0]
    ax = DpPitchRate(kp=100.0, ki=100.0, kd=0.0, imax=150.0,
                     max_speed=8.0, vz_max=0.35, arm_frames=5)
    ax.enter(DroneState(flow_seq=-1))
    n = live = mism = 0
    show = []
    prev = dict(r=0, v=0)
    causes = dict(no_ok=0, rej=0, vz=0, arm=0)
    for i, (t, vf, ok, _) in enumerate(d8):
        if t < t0 or t > t0 + 25.0:
            continue
        a = alt[alt[:, 0] <= t, 1]
        s = DroneState(flow_seq=i + 1, now_sim=t, flow_dt=0.05, flow_conf=0.5,
                       rel_alt=float(a[-1]) if len(a) else None,
                       ipm_ok=bool(ok), ipm_vfwd=float(vf),
                       ipm_vlat=float(np.interp(t, d9[:, 0], d9[:, 1])))
        rc = ax.update(s, Setpoint(), 0.05)
        real = float(np.interp(t, d2[:, 0], d2[:, 1]))
        n += 1
        if rc.pitch != RC_CENTER and abs(real) < 1.0:
            mism += 1
            if len(show) < 12:
                show.append(f't={t-t0:5.1f} ipm={vf:6.2f} ok={ok:.0f} '
                            f'реплей={rc.pitch-1500:5.0f} бэг={real:5.0f}')
        if rc.pitch != RC_CENTER:
            live += 1
        elif not ok:
            causes['no_ok'] += 1
        elif ax._rejects > prev['r']:
            causes['rej'] += 1
        elif ax._vz_blocks > prev['v']:
            causes['vz'] += 1
        else:
            causes['arm'] += 1
        prev = dict(r=ax._rejects, v=ax._vz_blocks)
    print(f'  расхождение реплея с бэгом: {mism} кадров из {n} '
          f'(реплей командует, в бэге ноль)')
    for row in show[:12]:
        print('   ', row)
    print(f'{name}: кадров {n} | ось командует {100*live/n:.0f}% | молчит из-за: '
          f'канал провален {100*causes["no_ok"]/n:.0f}%, неправдоподобно '
          f'{100*causes["rej"]/n:.0f}%, набор/снижение {100*causes["vz"]/n:.0f}%, '
          f'взведение {100*causes["arm"]/n:.0f}%')
