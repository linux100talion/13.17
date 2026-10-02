#!/usr/bin/env python3
"""ipm_alt_est_replay.py — ПОПРАВКА ВЫСОТЫ ПО ЗУМУ ЗЕМЛИ против истины Gazebo (2026-10-02).

Лётный `_ipm_update` (конфиг канала — мета прогона, камера — CAM_CFG) по кадрам bag с
высотой перцепции как у ноды (z EKF − z₀ латча по отрыву) и включённой оценкой
(ipm_alt_est, perception/alt_est.py). Истинная поправка: δ_ист = (истинная высота камеры
над землёй) − (принятая геометрией) = [z_истина − z_земли_истина + клиренс + вынос_z] −
_ipm_geom_h(высота перцепции). Печатает ленту раз в IA_STEP с: высота, принятая/истинная
камера, оценка δ ± σ, δ_ист, и итог — ошибку оценки там, где σ < 0.10.

  source /opt/ros/jazzy/setup.bash
  CAM_IDEAL=1 python3 src/lab/ipm_alt_est_replay.py docker/sim/output/joystick/<RUN>
Env: IA_STEP (2 с), IA_T0/IA_T1 (окно, с от первого odom), IA_GROUND_CLEAR (клиренс, по
умолчанию из меты прогона), IA_TRUTH_H=1 — кормить ИСТИННОЙ высотой (проверка самой меры).
"""
import math
import os
import sqlite3
import sys

import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from ipm_yaw_err import (CameraMount, FlowEstimator, IPM_KNOBS, euler, load_meta,  # noqa: E402
                         stamp, deserialize_message)
from geometry_msgs.msg import PoseStamped                                          # noqa: E402
from nav_msgs.msg import Odometry                                                  # noqa: E402
from sensor_msgs.msg import Image                                                  # noqa: E402

STEP = float(os.environ.get('IA_STEP', '2'))
T_WIN = (float(os.environ.get('IA_T0', '-1')), float(os.environ.get('IA_T1', '1e9')))
TRUTH_H = os.environ.get('IA_TRUTH_H', '0') == '1'
GROUND_TOP_REST = 0.195     # сим iris: base_link стоя над верхом травы (world SDF)


def main(run):
    run = run.rstrip('/')
    db = sqlite3.connect(os.path.join(run, 'bag', 'scene_bag_0.db3'))
    tid = {n: i for n, i in db.execute('select name,id from topics')}

    def msgs(topic, typ):
        for (raw,) in db.execute('select data from messages where topic_id=? order by timestamp',
                                 (tid[topic],)):
            yield deserialize_message(raw, typ)

    od = []
    for m in msgs('/model/iris_cam/odometry', Odometry):
        p, w = m.pose.pose.position, m.twist.twist.angular
        od.append((stamp(m), p.z) + euler(m.pose.pose.orientation) + (w.z,))
    od = np.array(od)                                   # t z roll pitch wz
    t0, g0 = od[0, 0], float(np.median(od[:60, 1]))
    lp = np.array([(stamp(m), m.pose.position.z)
                   for m in msgs('/mavros/local_position/pose', PoseStamped)])
    air_i = int(np.argmax(od[:, 1] - g0 > 0.05))
    z0 = float(np.interp(od[max(0, air_i - 30), 0], lp[:, 0], lp[:, 1]))
    cfg_all = load_meta(run)
    cfg = {k: getattr(cfg_all, k) for k in IPM_KNOBS}
    cfg.update(ipm_scale_exact=True, ipm_alt_est=True,
               ipm_ground_clear=float(os.environ.get('IA_GROUND_CLEAR', cfg_all.ipm_ground_clear)))
    cam = CameraMount.load()
    lever = cam.t if cfg_all.ipm_lever else (0.0, 0.0, 0.0)
    print(f'# {os.path.basename(run)}: клиренс {cfg["ipm_ground_clear"]}, вынос {lever}, '
          f'высота {"ИСТИННАЯ" if TRUTH_H else "EKF − z₀"}')
    print('     t    h_ист  кам_принят кам_ист   δ_ист    оценка δ ± σ        ошибка')
    est, nxt, errs = None, None, []
    for m in msgs('/image_color', Image):
        t = stamp(m)
        if not (T_WIN[0] <= t - t0 <= T_WIN[1]):
            continue
        g = cv2.cvtColor(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3),
                         cv2.COLOR_BGR2GRAY)
        if est is None:
            fx, fy, cx, cy = cam.intrinsics_for(m.width, m.height)
            est = FlowEstimator(fx, fy, cx, cy, cam.flow_R, 1.0, cam_tilt=cam.tilt,
                                cam_lever=lever, **cfg)
            nxt = t
        roll = float(np.interp(t, od[:, 0], od[:, 2]))
        pitch = float(np.interp(t, od[:, 0], od[:, 3]))
        wz = float(np.interp(t, od[:, 0], od[:, 4]))
        h_true = float(np.interp(t, od[:, 0], od[:, 1])) - g0          # корпус от позы «стою»
        h = h_true if TRUTH_H else max(0.0, float(np.interp(t, lp[:, 0], lp[:, 1])) - z0)
        est._ipm_update(g, t, h, pitch, roll, wz)
        cam_acc = est._ipm_geom_h(h, pitch, roll)
        cam_true = h_true + GROUND_TOP_REST + est._lever_level(pitch, roll)[2]
        d_true = cam_true - cam_acc
        d, s = est.ipm_alt_delta, est.ipm_alt_sigma
        if d is not None and s is not None and s < 0.10 and h_true > 0.3:
            errs.append(d - d_true)
        if t >= nxt:
            nxt += STEP
            ds = '--' if d is None else f'{d:+.3f} ± {s:.3f}' if s is not None else f'{d:+.3f}'
            er = '' if d is None else f'{d - d_true:+.3f}'
            print(f'  {t - t0:6.1f}  {h_true:5.2f}   {cam_acc:5.2f}    {cam_true:5.2f}   '
                  f'{d_true:+.3f}   {ds:18s} {er}')
    if errs:
        e = np.array(errs)
        print(f'  ИТОГ (σ < 0.10, в воздухе): ошибка оценки δ медиана {np.median(e):+.3f} м, '
              f'|ошибка| 90 % {np.percentile(np.abs(e), 90):.3f} м, кадров {len(e)}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for r in sys.argv[1:]:
        main(r)
