#!/usr/bin/env python3
"""ipm_ok.py — по /mission/status прогонов: доля годных кадров канала вида сверху В ПОЛЁТЕ
(между первым и последним ipm=1, т.е. без земли до взлёта и после касания), разбивка брака по
кодам ipmf (таблица — src/nav/hud.md 3.5), и жил ли VINS (odom= растёт, brg= мост открыт).
Внутри nav-контейнера:  python3 /root/repo/cmd/cam_tilt/ipm_ok.py <run_dir> […]"""
import re
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

for run in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=run.rstrip('/') + '/bag', storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/mission/status']))
    rows = []
    while r.has_next():
        _, data, _ = r.read_next()
        kv = dict(re.findall(r'(\w+)=(\S+)', deserialize_message(data, String).data))
        rows.append(kv)
    ok = [i for i, kv in enumerate(rows) if kv.get('ipm') == '1']
    if not ok:
        print(f'# {run}: ни одного годного кадра канала'); continue
    fl = rows[ok[0]:ok[-1] + 1]
    codes = {}
    for kv in fl:
        codes[kv.get('ipmf', '?')] = codes.get(kv.get('ipmf', '?'), 0) + 1
    good = sum(kv.get('ipm') == '1' for kv in fl)
    odo = [int(kv['odom']) for kv in fl if kv.get('odom', '').isdigit()]
    brg = sum(kv.get('brg') == '1' for kv in fl)
    print(f'# {run.rstrip("/").split("/")[-1]}')
    print(f'  полёт t {fl[0].get("t")}…{fl[-1].get("t")} с: годных {100 * good / len(fl):.1f}% из {len(fl)}; '
          f'коды ipmf {sorted(codes.items(), key=lambda x: -x[1])}')
    print(f'  VINS odom {odo[0] if odo else "-"}→{odo[-1] if odo else "-"}; мост brg=1 в {100 * brg / len(fl):.0f}% кадров')
