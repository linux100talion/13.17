#!/usr/bin/env python3
"""Юнит-тест: на ПОЛУ высоты HUD vis = «--» (RosPerception.merge, без ROS).

cam (высота камеры для геометрии канала) не опускается ниже пола ipm_alt_floor; пока она
стоит на полу, это не высота камеры, а пол — cam + δ (vis) смысла не имеет, поправку в
снимок не отдаём. Над полом — отдаём как есть.

Запуск:  python3 src/control/test/test_vis_floor.py
"""
import inspect
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                                       # noqa: E402

from control_pkg.domain.state import DroneState                          # noqa: E402
from control_pkg.infrastructure.ros_perception import RosPerception      # noqa: E402
from control_pkg.perception.flow_estimator import FlowEstimator          # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def mk(alt):
    p = object.__new__(RosPerception)
    for name in set(re.findall(r'self\.(_\w+)', inspect.getsource(RosPerception.merge))):
        if not callable(getattr(RosPerception, name, None)):
            setattr(p, name, 0)                 # агрегаты потока — нулями, они тут не важны
    p._ipm_noise = (0.0, 0.0)
    p._alt, p._alt_wall, p._alt_stale = alt, time.time(), 2.0
    p._pitch = p._roll = 0.0
    p._est = FlowEstimator(640, 640, 640, 360, np.eye(3), ipm_alt_floor=0.5,
                           ipm_ground_clear=0.195, ipm_alt_est=True, ipm_scale_exact=True)
    p._est.ipm_alt_delta, p._est.ipm_alt_sigma = 0.08, 0.07
    return p


s = DroneState(now_sim=1.0)
mk(0.1).merge(s)
check(f'на полу (perc 0.1 → cam {s.cam_agl:.3f}): поправки нет → vis --',
      s.cam_agl_delta is None and s.cam_agl_sigma is None)
s = DroneState(now_sim=1.0)
mk(1.0).merge(s)
check(f'над полом (perc 1.0 → cam {s.cam_agl:.3f}): поправка отдаётся',
      s.cam_agl_delta == 0.08 and s.cam_agl_sigma == 0.07)

bad = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(bad)}/{len(results)} OK')
sys.exit(1 if bad else 0)
