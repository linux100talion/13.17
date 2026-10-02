#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Пост-рендер КАНАЛА ВИДА СВЕРХУ на видео из rosbag → scene_ipm.mp4.

Тот же артефакт прогона, что и `scene_hud.mp4`, только про IPM: слева кадр
камеры с НАРИСОВАННОЙ полосой земли (четыре угла, спроецированные лётным
`_ipm_px`), справа — сам выпрямленный варп, по которому канал и считает LK.
Видно глазами то, что раньше читалось только кодом брака в `/flow_dbg8.z`:
где полоса легла, чем она заполнена, и на каких кадрах канал отваливается.

⚠️ ВАРП В BAG НЕ ПИШЕТСЯ — его приходится ПЕРЕСЧИТЫВАТЬ. Считает настоящий
`FlowEstimator._ipm_update` из `control_pkg` (импорт из bind-mounted исходников,
как у `hud_video.py`), с ЛЁТНЫМ конфигом прогона (`BootstrapConfig` + `BS_*`
того же окружения, которым летели) и ЛЁТНОЙ высотой перцепции. Поэтому:

  * УГЛЫ И ω (`IPM_ATT_SRC`, default auto): в симе — ИСТИНА GAZEBO
    (`/model/iris_cam/odometry`; в freefly-бэгах `/mavros/imu/data` не пишется).
    Реплей чуть ОПТИМИСТИЧНЕЕ полёта по ориентации (нет ступеньки ATTITUDE
    12.5 Гц и лага) — картинка полосы точнее, чем было в воздухе. На РЕАЛЬНОМ
    БОРТУ истины нет — `imu`: кватернион и ω_z `/mavros/imu/data` (AHRS полётника,
    ENU/FLU — та же конвенция, что у Gazebo; борт пишет его 200 Гц, auto_bag_m.sh).
    Нода при `dphold/att_own` летит СВОИМ фильтром по сырому IMU — `imu` ему близок,
    но не тождествен (строка `att src:` в кадре);
  * коды брака и скорости РЕПЛЕЯ могут разойтись с лётными. Поэтому в кадре
    рисуются ОБЕ пары: `ipm` — реплей, `rec` — что канал выдал В ПОЛЁТЕ
    (`/flow_dbg8`, `/flow_dbg9`), плюс `true` — истина Gazebo. Расхождение
    `ipm` и `rec` = мера того, насколько идеальные углы льстят каналу.

ВЫСОТА (`IPM_ALT_SRC`, default auto — восстанавливается лётная формула):
    latch  — max(0, z `/mavros/local_position/pose` − z₀), z₀ = z на арме:
             ровно то, что делает `ros_perception.latch_alt_zero` при
             `perc_alt_src=local` + `perc_alt_zero>0` (профиль LV=2);
    ekf    — max(0, z) без латча (`perc_alt_zero=0`);
    status — `palt=` из `/mission/status` (ровно лётное значение, но округлённое
             до 0.1 м — запасной путь для `perc_alt_src=global|baro`);
    true   — истинная AGL Gazebo (запасной путь сима; это уже НЕ полёт);
    baro   — барометр полётника (`/mavros/imu/static_pressure`), ноль = медиана
             первой секунды bag (борт стоит на земле): реальный борт без EKF/ноды,
             прогоны «в руках». Тоже НЕ лётная формула — подпись в кадре.
Что выбрано — написано в кадре строкой `alt src:`, чтобы архивное видео нельзя
было прочитать не так.

Запускается ВНУТРИ nav-контейнера (нужен cv_bridge/cv2 и control_pkg):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash;
    python3 /lab/ipm_video.py'
Env: SCENE_BAG (…/output/scene_bag), SCENE_IPM_MP4 (…/output/scene_img/scene_ipm.mp4),
SCENE_TOPIC (/image_color), SCENE_FPS (0 = авто по кадрам), IPM_ZOOM (3),
IPM_PAD (3 с запаса вокруг окна «в армии»), IPM_ALL (1 = писать весь bag),
IPM_ALT_SRC (auto|latch|ekf|status|true|baro), IPM_ATT_SRC (auto|truth|imu)
+ весь лётный BS_IPM_*/BS_PERC_ALT_*.
Бортовой bag (2026-10-02): истины нет → углы imu, высота — EKF/статус ноды, иначе baro;
`BS_*` — из `bs.env` рядом с bag или PROFILES (как у сима).

Пересборка по архивному прогону: `BS_*` подхватываются из его меты
`joystick/<NAME>/<NAME>.env` (рядом с bag) — иначе старое видео пересчиталось бы
СЕГОДНЯШНИМИ дефолтами `config.py`. Явный env снаружи всё равно главнее.
"""
import math
import os
import sys

import numpy as np

import cv2

# исходники, а не colcon-install: артефакт прогона обязан считаться ТЕМ кодом,
# что лежит в репе на момент прогона (colcon КОПИРУЕТ ament_python-пакеты)
sys.path.insert(0, '/root/sim_ws/src/control')
sys.path.insert(0, '/root/sim_ws/src/mission')

from geometry_msgs.msg import PoseStamped, Vector3Stamped                # noqa: E402
from nav_msgs.msg import Odometry                                        # noqa: E402
from rclpy.serialization import deserialize_message                      # noqa: E402
from rosbag2_py import (ConverterOptions, SequentialReader,              # noqa: E402
                        StorageFilter, StorageOptions)
from sensor_msgs.msg import Image                                        # noqa: E402
from std_msgs.msg import String                                          # noqa: E402

from control_pkg.perception.flow_estimator import FlowEstimator          # noqa: E402
from mission_pkg.config import BootstrapConfig                           # noqa: E402
from mission_pkg.nodes.bootstrap_node import CAMERA, FLOW_R, FLOW_ROTSIGN, CAM_TILT  # noqa: E402
# ⚠️ камера — ТЕКУЩИЙ CAM_CFG (не та, с которой летал bag): старые прогоны (до
# 2026-10-01, наклон 0.26) рисовать с CAM_CFG на yaml той камеры.

from ipm_panel import FAIL_ASCII, dbg_z_decode, warp_panel               # noqa: E402

BAG = os.environ.get('SCENE_BAG', '/root/sim_ws/output/scene_bag')
MP4 = os.environ.get('SCENE_IPM_MP4', '/root/sim_ws/output/scene_img/scene_ipm.mp4')
TOPIC = os.environ.get('SCENE_TOPIC', '/image_color')
FPS_ENV = float(os.environ.get('SCENE_FPS', '0'))
ZOOM = int(os.environ.get('IPM_ZOOM', '3'))
PAD = float(os.environ.get('IPM_PAD', '3'))
ALL_FRAMES = os.environ.get('IPM_ALL', '0') == '1'
ALT_SRC = os.environ.get('IPM_ALT_SRC', 'auto')
ATT_SRC = os.environ.get('IPM_ATT_SRC', 'auto')
TRUTH = '/model/iris_cam/odometry'
FPS_PROBE_N = 60                 # кадров на авто-оценку fps
# ручки канала, которые кладёт в оценщик bootstrap_node (те же BS_-имена)
IPM_KNOBS = ('ipm_model', 'ipm_derot', 'ipm_wz_tau', 'ipm_wz_gate', 'ipm_win', 'ipm_adapt',
             'ipm_vel_tau', 'ipm_alt_floor', 'ipm_scale_ref', 'ipm_acc_tau', 'ipm_acc_world',
             'ipm_scale_exact', 'ipm_ground_clear', 'ipm_alt_est')


def env_from_archive(bag):
    """Догрузить `BS_*` из меты архивного прогона (`joystick/<NAME>/<NAME>.env`).

    Живой прогон рисует видео СВОИМ окружением (мета пишется шагом позже, её
    рядом с bag ещё нет). А вот пересборка по старому архиву без этого молча
    взяла бы СЕГОДНЯШНИЕ дефолты `config.py` — и видео показывало бы канал,
    которым не летели. Уже установленное снаружи не трогаем."""
    d = os.path.dirname(os.path.abspath(bag.rstrip('/')))
    metas = [f for f in sorted(os.listdir(d)) if f.endswith('.env')] \
        if os.path.isdir(d) else []
    if not metas and os.path.isfile(os.path.join(bag, 'bs.env')):   # бортовой bag
        d, metas = bag, ['bs.env']
    if not metas:
        return None
    path = os.path.join(d, metas[0])
    n = 0
    for line in open(path):
        line = line.strip()
        if line.startswith('export '):          # формат load.py / bs.env борта
            line = line[7:]
        if not line.startswith('BS_') or '=' not in line:
            continue
        k, v = line.split('=', 1)
        v = v.strip("'")
        if k not in os.environ:
            os.environ[k] = v
            n += 1
    # прогоны до 2026-10-02 летели без учёта выноса камеры — ключа в их мете нет
    for k in ('BS_IPM_LEVER', 'BS_IPM_ACC_WORLD', 'BS_IPM_SCALE_EXACT', 'BS_IPM_GROUND_CLEAR',
              'BS_IPM_ALT_EST'):   # ручки 2026-10-02: старые прогоны без них
        if k not in os.environ:
            os.environ[k] = '0'
            n += 1
    return f'{path} (+{n} BS_*)'


def flight_cfg():
    """Лётный конфиг канала: дефолты `BootstrapConfig` + `BS_*` окружения.

    Ровно та же лесенка, что у `bootstrap_arch2.sh` (`BS_IPM_WIN` → `--ipm-win`),
    и тот же источник дефолтов — иначе видео рисовалось бы по конфигу, которым
    не летели."""
    # 2026-09-07: дефолтов в коде нет — полный env BS_* (мета прогона, load_meta) или
    # PROFILES (пост-рендер из freefly_lv до архива); иначе SystemExit с именами ключей.
    base = BootstrapConfig.from_run()
    cfg = {k: getattr(base, k) for k in IPM_KNOBS}
    # вынос камеры — как в bootstrap_node: значения из CAM_CFG, включает BS_IPM_LEVER
    cfg['cam_lever'] = CAMERA.t if base.ipm_lever else (0.0, 0.0, 0.0)
    return base, cfg, []


def euler(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def fit_to(img, size):
    """Привести кадр к размеру писателя (страховка от молчаливой потери кадров).

    `cv2.VideoWriter` кадр другого размера просто ВЫБРАСЫВАЕТ, без ошибки — так
    первое видео канала доехало 182 кадрами из 1924. Панель теперь фиксирована
    (`ipm_panel`), но подгонка + счётчик оставлены: молча терять кадры дороже."""
    if (img.shape[1], img.shape[0]) == size:
        return img, False
    return cv2.resize(img, size, interpolation=cv2.INTER_AREA), True


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read_refs(bag, have):
    """Первый проход: истина, EKF z, арм, лётные `/flow_dbg8|9`, `palt` статуса.

    Кадры тут НЕ грузим (их тысячи) — только опоры, по которым потом кормится
    оценщик и подписывается картинка."""
    topics = [t for t in (TRUTH, '/mavros/local_position/pose',
                          '/mavros/state', '/flow_dbg8', '/flow_dbg9',
                          '/mission/status', '/mavros/imu/data',
                          '/mavros/imu/static_pressure') if t in have]
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    r.set_filter(StorageFilter(topics=topics))
    od, lp, d8, d9, st, im, pr = [], [], [], [], [], [], []
    t_arm, t_disarm, now = None, None, 0.0
    State = None
    if '/mavros/state' in topics:
        from mavros_msgs.msg import State                       # noqa: F811
    from sensor_msgs.msg import FluidPressure, Imu
    imu_last = -1.0
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            now = stamp(m)
            p, v, w = m.pose.pose.position, m.twist.twist.linear, m.twist.twist.angular
            od.append((now, p.z) + euler(m.pose.pose.orientation) + (v.x, v.y, w.z))
        elif topic == '/mavros/imu/data':
            m = deserialize_message(raw, Imu)
            now = stamp(m)
            if now - imu_last >= 0.01:           # 200 Гц → 100 Гц: кадров 15 Гц хватит
                imu_last = now
                im.append((now,) + euler(m.orientation) + (m.angular_velocity.z,))
        elif topic == '/mavros/imu/static_pressure':
            m = deserialize_message(raw, FluidPressure)
            now = stamp(m)
            if not pr or now - pr[-1][0] >= 0.05:
                pr.append((now, m.fluid_pressure))
        elif topic == '/mavros/local_position/pose':
            m = deserialize_message(raw, PoseStamped)
            now = stamp(m)
            lp.append((now, m.pose.position.z))
        elif topic == '/mavros/state':
            m = deserialize_message(raw, State)
            now = stamp(m)
            if m.armed:
                if t_arm is None:
                    t_arm = now
                t_disarm = now
        elif topic in ('/flow_dbg8', '/flow_dbg9'):
            m = deserialize_message(raw, Vector3Stamped)
            now = stamp(m)
            (d8 if topic == '/flow_dbg8' else d9).append(
                (now, m.vector.y if topic == '/flow_dbg8' else m.vector.x, m.vector.z))
        else:                                    # /mission/status — String без header
            m = deserialize_message(raw, String)  # время = последний виденный штамп
            for kv in m.data.split():
                if kv.startswith('palt=') and kv[5:] != '--':
                    st.append((now, float(kv[5:])))
    return (np.array(od), np.array(lp), np.array(d8), np.array(d9),
            np.array(st), t_arm, t_disarm, np.array(im), np.array(pr))


def pick_alt_src(base, have, lp, st, t_arm, od, pr):
    """Какой высотой кормить оценщик — восстанавливаем лётную формулу; без неё
    (реальный борт без EKF-позиции и без ноды) — истина сима или барометр."""
    if ALT_SRC != 'auto':
        return ALT_SRC
    src = base.perc_alt_src
    zero = float(base.perc_alt_zero)
    if src == 'local' and len(lp):
        return 'latch' if zero > 0 and t_arm is not None else 'ekf'
    if len(st):
        return 'status'
    if len(od):
        return 'true'
    if len(pr):
        return 'baro'
    raise SystemExit('⚠️ высоту взять неоткуда: нет EKF-позиции, palt= статуса, истины '
                     'и /mavros/imu/static_pressure')


def baro_agl(pr):
    """Высота по барометру над стартом: ноль = медиана первой секунды (борт на земле)."""
    p0 = float(np.median(pr[pr[:, 0] <= pr[0, 0] + 1.0, 1]))
    return pr[:, 0], 44330.0 * (1.0 - (pr[:, 1] / p0) ** 0.1903)


def main():
    os.makedirs(os.path.dirname(MP4), exist_ok=True)
    r = SequentialReader()
    r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    have = {t.name for t in r.get_all_topics_and_types()}
    if TOPIC not in have:
        raise SystemExit(f'⚠️ в bag нет {TOPIC} — нечего рисовать')
    od, lp, d8, d9, st, t_arm, t_disarm, im, pr = read_refs(BAG, have)
    att = ATT_SRC if ATT_SRC != 'auto' else ('truth' if len(od) else 'imu')
    if att == 'truth' and not len(od):
        raise SystemExit(f'⚠️ IPM_ATT_SRC=truth, а {TRUTH} в bag нет/пуст')
    if att == 'imu' and not len(im):
        raise SystemExit('⚠️ углы взять неоткуда: нет ни истины Gazebo, ни /mavros/imu/data')
    # углы/ω: столбцы t, roll, pitch, wz — из истины или из AHRS полётника
    A = od[:, [0, 2, 3, 7]] if att == 'truth' else im[:, [0, 1, 2, 4]]
    meta = env_from_archive(BAG)
    base, cfg, defaulted = flight_cfg()
    t0 = A[0, 0]
    ground = float(np.median(od[:60, 1])) if len(od) else 0.0   # борт стоит на земле
    src = pick_alt_src(base, have, lp, st, t_arm, od, pr)
    if src == 'true' and not len(od):
        raise SystemExit('⚠️ IPM_ALT_SRC=true, а истины Gazebo в bag нет')
    if src == 'baro':
        if not len(pr):
            raise SystemExit('⚠️ IPM_ALT_SRC=baro, а /mavros/imu/static_pressure в bag нет')
        bt, bh = baro_agl(pr)
    z0 = 0.0
    if src == 'latch':
        # НОДА латчит последнее пришедшее z на переходе armed (не медиану) —
        # воспроизводим её, а не свою оценку смещения
        i = int(np.searchsorted(lp[:, 0], t_arm, 'right')) - 1
        z0 = float(lp[max(0, i), 1])
    if src in ('latch', 'ekf') and not len(lp):
        raise SystemExit('⚠️ нет /mavros/local_position/pose — высоту не восстановить')
    if src == 'status' and not len(st):
        raise SystemExit('⚠️ нет palt= в /mission/status — высоту не восстановить')
    # окно записи: «в армии» ± запас; вне его канал заведомо в гейте земли
    w0, w1 = -1e9, 1e9
    if not ALL_FRAMES and t_arm is not None:
        w0, w1 = t_arm - t0 - PAD, t_disarm - t0 + PAD
    print(f'bag {BAG}')
    if meta:
        print(f'  мета прогона: {meta}')
    print(f'  высота перцепции: {src}' + (f' (z₀={z0:+.3f} м на арме)' if src == 'latch' else '')
          + f'; углы и ω: {att}')
    print('  конфиг канала: ' + ' '.join(
        f'{k.replace("ipm_", "")}={v}' + ('*' if k in defaulted else '')
        for k, v in cfg.items()))
    if meta and defaulted:
        # ручка, которой в прогоне ЕЩЁ НЕ БЫЛО, приезжает сегодняшним дефолтом —
        # это единственное, чего мета восстановить не может; помечаем звёздочкой
        print('  ⚠️ звёздочкой — не задано ни env, ни метой прогона, взят дефолт '
              'config.py: ' + ' '.join(k.replace('ipm_', '') for k in defaulted))
    print(f'  окно записи: {"весь bag" if ALL_FRAMES or t_arm is None else f"{w0:.1f}…{w1:.1f} с"}')

    est = None
    writer, probe, probe_t, wsize = None, [], [], None
    n_seen = n_drawn = n_fit = 0
    fails = {}
    star = {k: ('*' if k in defaulted else '') for k in IPM_KNOBS}
    cfg_line = ('cfg win {win} adapt {adapt} floor {floor} vel_tau {vel} '
                'acc_tau {acc} derot {der}/{wz}').format(
        win=f"{cfg['ipm_win']:g}{star['ipm_win']}",
        adapt=f"{cfg['ipm_adapt']:g}{star['ipm_adapt']}",
        floor=f"{cfg['ipm_alt_floor']:g}{star['ipm_alt_floor']}",
        vel=f"{cfg['ipm_vel_tau']:g}{star['ipm_vel_tau']}",
        acc=f"{cfg['ipm_acc_tau']:g}{star['ipm_acc_tau']}",
        der=f"{cfg['ipm_derot']:g}{star['ipm_derot']}",
        wz=f"{cfg['ipm_wz_tau']:g}{star['ipm_wz_tau']}")

    r = SequentialReader()
    r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    r.set_filter(StorageFilter(topics=[TOPIC]))
    while r.has_next():
        _topic, raw, _ = r.read_next()
        msg = deserialize_message(raw, Image)
        t = stamp(msg)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == 'bgr8':
            gray = cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_BGR2GRAY)
        elif msg.encoding in ('mono8', '8UC1'):
            gray = buf.reshape(msg.height, msg.width).copy()
        else:
            continue
        if est is None:
            # интринсики — как у ноды: конфиг камеры под разрешение кадра (CAM_IDEAL
            # в контейнере сима — идеальная камера Gazebo)
            fx, fy, cx, cy = CAMERA.intrinsics_for(msg.width, msg.height)
            est = FlowEstimator(fx, fy, cx, cy,
                                FLOW_R, FLOW_ROTSIGN, cam_tilt=CAM_TILT, **cfg)
            print(f'  кадр {msg.width}×{msg.height} → fx/fy={fx:.0f}/{fy:.0f} '
                  f'cx={cx:.0f} cy={cy:.0f}')
        roll = float(np.interp(t, A[:, 0], A[:, 1]))
        pitch = float(np.interp(t, A[:, 0], A[:, 2]))
        wz = float(np.interp(t, A[:, 0], A[:, 3]))
        agl = float(np.interp(t, od[:, 0], od[:, 1])) - ground if len(od) else None
        if src == 'true':
            alt = max(0.0, agl)
        elif src == 'baro':
            alt = max(0.0, float(np.interp(t, bt, bh)))
        elif src == 'status':
            alt = float(np.interp(t, st[:, 0], st[:, 1]))
        else:
            alt = max(0.0, float(np.interp(t, lp[:, 0], lp[:, 1])) - z0)
        # оценщик кормится ВСЕМИ кадрами (фильтр скорости и оценка нуля ω_z живут
        # историей) — окно режет только ЗАПИСЬ
        est._ipm_update(gray, t, alt, pitch, roll, wz)
        n_seen += 1
        if not (w0 <= t - t0 <= w1):
            continue
        fails[est.ipm_fail] = fails.get(est.ipm_fail, 0) + 1
        rec = 'rec --'
        if len(d8):
            i = int(np.argmin(np.abs(d8[:, 0] - t)))
            rok, rf = dbg_z_decode(float(d8[i, 2]))
            rlat = float(np.interp(t, d9[:, 0], d9[:, 1])) if len(d9) else float('nan')
            rec = (f'rec {float(d8[i,1]):+.2f}/{rlat:+.2f} '
                   f'{FAIL_ASCII.get(rf, "?")}{"" if rok else "!"}')
        img = warp_panel(
            gray, est, alt, pitch, roll, t - t0, zoom=ZOOM, agl=agl,
            extra=(f'ipm {est.ipm_vfwd:+.2f}/{est.ipm_vlat:+.2f}  {rec}  '
                   + (f'true {float(np.interp(t, od[:,0], od[:,5])):+.2f}/'
                      f'{float(np.interp(t, od[:,0], od[:,6])):+.2f} m/s (fwd/lat)'
                      if len(od) else 'true -- (no truth)'),
                   f'alt src: {src}  att src: {att}   {cfg_line}'))
        n_drawn += 1
        if writer is None:
            probe.append(img); probe_t.append(t)
            if len(probe) >= FPS_PROBE_N:
                writer = open_writer(probe_t, img.shape)
                wsize = (img.shape[1], img.shape[0])
                for fr in probe:
                    fr, adj = fit_to(fr, wsize)
                    n_fit += adj
                    writer.write(fr)
                probe.clear()
        else:
            img, adj = fit_to(img, wsize)
            n_fit += adj
            writer.write(img)
    if writer is None and probe:            # короткий полёт: кадров меньше пробы
        writer = open_writer(probe_t, probe[0].shape)
        wsize = (probe[0].shape[1], probe[0].shape[0])
        for fr in probe:
            fr, adj = fit_to(fr, wsize)
            n_fit += adj
            writer.write(fr)
        probe.clear()
    if writer is None:
        raise SystemExit('⚠️ в окне записи нет кадров — видео не собрано')
    writer.release()
    print(f'Записано {n_drawn} кадров канала (из {n_seen} прокрученных) → {MP4}')
    if n_fit:
        print(f'  ⚠️ {n_fit} кадров подогнано под размер писателя '
              f'{wsize[0]}×{wsize[1]} (панель должна быть фиксированной)')
    for f, c in sorted(fails.items(), key=lambda kv: -kv[1]):
        print(f'    {FAIL_ASCII.get(f, "?"):10s} {c:5d}  {100.0*c/max(1,n_drawn):5.1f}%')


def open_writer(ts, shape):
    # fps — по МЕДИАНЕ межкадрового sim-интервала: среднее врёт из-за всплеска
    # в начале bag (первые кадры прилетают пачкой), и ролик уезжает по темпу
    dt = float(np.median(np.diff(ts))) if len(ts) > 1 else 0.0
    fps = FPS_ENV if FPS_ENV > 0 else max(1.0, min(1.0 / dt if dt > 0 else 10.0, 60.0))
    w = cv2.VideoWriter(MP4, cv2.VideoWriter_fourcc(*'mp4v'), fps,
                        (shape[1], shape[0]))
    if not w.isOpened():
        raise SystemExit(f'⚠️ VideoWriter не открылся для {MP4} '
                         f'(codec mp4v / size {shape[1]}×{shape[0]})')
    print(f'  fps {fps:.1f}, кадр {shape[1]}×{shape[0]}')
    return w


if __name__ == '__main__':
    main()
