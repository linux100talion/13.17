#!/usr/bin/env python3
"""ipm_yaw_err.py — откуда в канале вида сверху ОШИБКА, ЗАВИСЯЩАЯ ОТ РАЗВОРОТА (2026-10-02).

Серия cmd/ipm_lever: боковая ошибка канала (ipm_vlat − истина) линейна по ω_z с наклоном
≈ −0.04 м (без учёта выноса) вместо ожидаемых +0.14 (ход вынесенной вперёд камеры), то есть
в лётном канале сидит лишняя составляющая ≈ −0.15…−0.2 м·ω_z. Стенд переигрывает ЛЁТНЫЙ
`_ipm_update` на кадрах bag (конфиг канала — из меты прогона, камера — CAM_CFG) несколькими
оценщиками сразу, у каждого свой источник/обработка входов, и для каждого печатает тот же
наклон ошибки по ω_z, что судья ipm_lever_ab.py (кадры: ярус 0, стики крена/тангажа/газа в
центре, высота > 0.3 м, канал годен).

Варианты (IY_VARIANTS, через запятую; по умолчанию все):
  flight   — как в полёте: ω — сырой гироскоп /mavros/imu/data_raw (среднее за межкадровый
             интервал, как RosPerception), углы — истина, высота — EKF z − z₀ латча;
  truth    — ω, углы и высота — истина Gazebo (ω — среднее за интервал);
  pt       — truth, но ω — мгновенная истина на штамп кадра (не среднее);
  noderot  — truth без вычитания разворота (ipm_derot=0);
  w110     — truth, ω × 1.10 (чувствительность к масштабу ω);
  lagp/lagm— truth, ω сдвинута на +IY_LAG / −IY_LAG с (по умолчанию 0.03);
  raw      — truth без фильтра скорости (ipm_vel_tau=0: голый МНК-наклон пути);
  lever    — truth + учёт выноса камеры (cam_lever из CAM_CFG);
  rawlever — raw + lever;
  nowzp    — truth, но в ПРОГНОЗ фильтра скорости ω_z не идёт (перекрёстные члены −ω×v выкл);
  acc0     — truth без ФВЧ ускорения прогноза (ipm_acc_tau=0);
  noacc    — truth, прогноз без ускорения по наклону (только −ω×v);
  accw     — truth, ФВЧ ускорения в осях курса (ipm_acc_world=1) — ИСПРАВЛЕНИЕ;
  accwlever— accw + учёт выноса камеры;
  accwneg/accw2 — проверка: поворот оценки с обратным знаком / вдвое (должно быть хуже).

Читает bag из sqlite без rosbag2_py — бегает на хосте (`source /opt/ros/jazzy/setup.bash`):
  python3 src/lab/ipm_yaw_err.py docker/sim/output/joystick/<RUN>
Env: IY_VARIANTS, IY_LAG, IY_T0/IY_T1 (окно, с от первого odom).
"""
import math
import os
import sqlite3
import sys

import numpy as np
import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
for _rel in ('../control', '../mission'):
    sys.path.insert(0, os.path.abspath(os.path.join(_HERE, _rel)))

from geometry_msgs.msg import PoseStamped                                # noqa: E402
from nav_msgs.msg import Odometry                                        # noqa: E402
from rclpy.serialization import deserialize_message                      # noqa: E402
from sensor_msgs.msg import Image, Imu                                   # noqa: E402
from std_msgs.msg import String                                          # noqa: E402

from control_pkg.perception.camera_mount import CameraMount              # noqa: E402
from control_pkg.perception.flow_estimator import FlowEstimator          # noqa: E402
from mission_pkg.config import BootstrapConfig                           # noqa: E402

IPM_KNOBS = ('ipm_model', 'ipm_derot', 'ipm_wz_tau', 'ipm_wz_gate', 'ipm_win', 'ipm_adapt',
             'ipm_vel_tau', 'ipm_alt_floor', 'ipm_scale_ref', 'ipm_acc_tau', 'ipm_wz_bias_max',
             'ipm_acc_world')
ALL = ('flight', 'truth', 'pt', 'noderot', 'w110', 'lagp', 'lagm', 'raw', 'lever',
       'rawlever', 'nowzp', 'acc0', 'noacc', 'accw', 'accwlever', 'accwneg', 'accw2')
LAG = float(os.environ.get('IY_LAG', '0.03'))
T_WIN = (float(os.environ.get('IY_T0', '-1')), float(os.environ.get('IY_T1', '1e9')))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def euler(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))


def load_meta(run):
    m = {}
    for f in sorted(os.listdir(run)):
        if f.endswith('.env'):
            for line in open(os.path.join(run, f), encoding='utf-8', errors='replace'):
                line = line.strip()
                if line.startswith('BS_') and '=' in line:
                    k, v = line.split('=', 1)
                    m[k] = v
    m.setdefault('BS_IPM_LEVER', '0')
    m.setdefault('BS_IPM_ACC_WORLD', '0')
    m.setdefault('BS_IPM_SCALE_EXACT', '0')
    return BootstrapConfig.from_mapping(m, 'meta')


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
        p, v, w = m.pose.pose.position, m.twist.twist.linear, m.twist.twist.angular
        od.append((stamp(m), p.z) + euler(m.pose.pose.orientation) + (v.x, v.y, w.z))
    od = np.array(od)                      # t z roll pitch vx vy(влево+) wz
    t0 = od[0, 0]
    g0 = float(np.median(od[:60, 1]))
    imu = np.array([(stamp(m), m.angular_velocity.z) for m in msgs('/mavros/imu/data_raw', Imu)])
    lp = np.array([(stamp(m), m.pose.position.z)
                   for m in msgs('/mavros/local_position/pose', PoseStamped)])
    st = []
    for m in msgs('/mission/status', String):
        d = dict(kv.partition('=')[::2] for kv in m.data.split())
        if 't' in d and 'rct' in d:
            ok = (d.get('tier') == '0' and abs(int(d['rct'])) < 20 and abs(int(d['rcp'])) < 20
                  and abs(int(d['rcr'])) < 20)
            arm = d.get('ekf') is not None
            st.append((float(d['t']), ok, arm))
    st = np.array(st)
    # z₀ латча: z EKF на фронте отрыва (как нода латчит по арму — до отрыва z стоит)
    air_i = int(np.argmax(od[:, 1] - g0 > 0.1))
    z0 = float(np.interp(od[max(0, air_i - 30), 0], lp[:, 0], lp[:, 1]))

    cfg_all = load_meta(run)
    cfg = {k: getattr(cfg_all, k) for k in IPM_KNOBS}
    cam = CameraMount.load()
    names = [v for v in os.environ.get('IY_VARIANTS', ','.join(ALL)).split(',') if v]
    print(f'# {os.path.basename(run)}  BS_IPM_LEVER(полёт)={int(cfg_all.ipm_lever)}  '
          f'камера t={cam.t} наклон {math.degrees(cam.tilt):.1f}°')
    print('  конфиг: ' + ' '.join(f'{k.replace("ipm_", "")}={v}' for k, v in cfg.items()))

    def mean_w(src, ta, tb):
        """среднее ω_z за [ta, tb] (как окно ω RosPerception)."""
        if tb <= ta:
            return float(np.interp(tb, src[:, 0], src[:, 1]))
        m = (src[:, 0] > ta) & (src[:, 0] <= tb)
        if m.sum() == 0:
            return float(np.interp(tb, src[:, 0], src[:, 1]))
        return float(src[m, 1].mean())

    wtruth = od[:, [0, 6]]
    ests, rows, prev_t = {}, {n: [] for n in names}, None
    for m in msgs('/image_color', Image):
        t = stamp(m)
        if not (T_WIN[0] <= t - t0 <= T_WIN[1]):
            prev_t = t
            continue
        buf = np.frombuffer(m.data, dtype=np.uint8)
        gray = cv2.cvtColor(buf.reshape(m.height, m.width, 3), cv2.COLOR_BGR2GRAY)
        if not ests:
            fx = m.width / 2.0
            for n in names:
                c = dict(cfg)
                if n == 'noderot':
                    c['ipm_derot'] = 0.0
                if n in ('raw', 'rawlever'):
                    c['ipm_vel_tau'] = 0.0
                if n == 'acc0':
                    c['ipm_acc_tau'] = 0.0
                if n in ('accw', 'accwlever', 'accwneg', 'accw2'):
                    c['ipm_acc_world'] = True
                ests[n] = FlowEstimator(fx, fx, m.width / 2.0, m.height / 2.0, cam.flow_R, 1.0,
                                        cam_tilt=cam.tilt,
                                        cam_lever=cam.t if n in ('lever', 'rawlever', 'accwlever')
                                        else (0.0, 0.0, 0.0), **c)
                if n == 'nowzp':
                    e0 = ests[n]
                    vp = e0._vel_predict
                    e0._vel_predict = lambda st_, p_, r_, wz_, _vp=vp: _vp(st_, p_, r_, 0.0)
                if n in ('accwneg', 'accw2'):
                    e2 = ests[n]
                    e2.ipm_acc_world = True
                    ad = e2._acc_debias
                    kk = -1.0 if n == 'accwneg' else 2.0
                    e2._acc_debias = lambda st_, f_, l_, wz_=0.0, _ad=ad, _k=kk: _ad(st_, f_, l_, _k * wz_)
                if n == 'noacc':
                    e1 = ests[n]
                    vp1 = e1._vel_predict
                    e1._vel_predict = lambda st_, p_, r_, wz_, _vp=vp1: _vp(st_, 0.0, 0.0, wz_)
        ta = prev_t if prev_t is not None else t - 1 / 30
        prev_t = t
        roll = float(np.interp(t, od[:, 0], od[:, 2]))
        pitch = float(np.interp(t, od[:, 0], od[:, 3]))
        h_true = max(0.0, float(np.interp(t, od[:, 0], od[:, 1])) - g0)
        h_ekf = max(0.0, float(np.interp(t, lp[:, 0], lp[:, 1])) - z0)
        w_tr = mean_w(wtruth, ta, t)
        vy = float(np.interp(t, od[:, 0], od[:, 5]))
        j = int(np.searchsorted(st[:, 0], t)) - 1
        mask = j >= 0 and st[j, 1] > 0.5 and h_true > 0.3
        for n, e in ests.items():
            w, h = w_tr, h_true
            if n == 'flight':
                w, h = mean_w(imu, ta, t), h_ekf
            elif n == 'pt':
                w = float(np.interp(t, od[:, 0], od[:, 6]))
            elif n == 'w110':
                w = 1.10 * w_tr
            elif n == 'lagp':
                w = mean_w(wtruth, ta - LAG, t - LAG)
            elif n == 'lagm':
                w = mean_w(wtruth, ta + LAG, t + LAG)
            e._ipm_update(gray, t, h, pitch, roll, w)
            if mask and e.ipm_ok:
                rows[n].append((float(np.interp(t, od[:, 0], od[:, 6])), e.ipm_vlat - vy,
                                e.ipm_vfwd - float(np.interp(t, od[:, 0], od[:, 4]))))
    print('  вариант   n     наклон бок/ω  сдвиг   СКО  | наклон прод/ω')
    for n in names:
        r = np.array(rows[n])
        if len(r) < 50:
            print(f'  {n:8s} мало кадров ({len(r)})')
            continue
        A = np.column_stack([r[:, 0], np.ones(len(r))])
        (k, b), *_ = np.linalg.lstsq(A, r[:, 1], rcond=None)
        (kf, _bf), *_ = np.linalg.lstsq(A, r[:, 2], rcond=None)
        sd = float(np.std(r[:, 1] - A @ [k, b]))
        print(f'  {n:8s} {len(r):5d}  {k:+7.3f} м   {b:+6.3f}  {sd:5.3f} |  {kf:+7.3f} м')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for r in sys.argv[1:]:
        main(r)
