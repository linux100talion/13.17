#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ГЕОМЕТРИЯ ПОЛОСЫ КАНАЛА ВИДА СВЕРХУ (IPM) — A/B по ОДНИМ кадрам из бэга.

Вопрос, ради которого стенд написан (2026-08-28, по картинке `scene_ipm.mp4`
прогона `joystick/yaw/swing/1`): «квадратик» полосы занимает ~половину ширины
кадра — а если взять полосу ПОШИРЕ, сигнал станет чище? И заодно числом
подтвердить давнее «далеко смотреть нельзя».

Ответ стендом (yaw/swing/1: 1184 кадра воздуха на 2.6 м, лётный конфиг, высота =
EKF z − z₀ латча; подтверждён на base/LV2/1/194912 и 034514 — полёты на 0.3 м):
- ширина `yhalf` 1.0 / 2.0 (полёт) / 3.5 м → шум приращения пути за кадр
  fwd 7.4 / 7.2 / 7.2 мм, lat 6.6 / 6.4 / 6.4; попарная разница приращений с
  базой 0.2–0.6 мм при ошибке 7 мм (corr ошибок +1.00), на низких полётах 0.0–0.1
  мм. Пиксельная часть шума ≈3%: медиана по 120–350 фичам УЖЕ СОШЛАСЬ, площадь и
  лимит фич (200→400) ничего не добавляют. Ширина полосы — НЕ рычаг.
- длиннее / дальше: 3–6 → 3–9 → 3–12 → 6–9 м: fwd 7.2 → 8.5 → 9.8 → 9.5 мм,
  vlat corr 0.71 → 0.64 → 0.57 → 0.36, чувствительность к Δpitch между кадрами
  631 → 747 → 830 → 1078 мм/рад (∝ h/sin²α); на 0.3 м полоса 6–9 даёт ×1.7 шума
  и vfwd corr 0.84 → 0.71. «Далеко нельзя» — подтверждено числом.
- что реально шумит: Δpitch (R²≈0.4 продольной ошибки на swing) — остаток ~8% от
  сырой геометрической чувствительности ≈ рассинхрон «штамп кадра ↔ угол» ~2.5 мс
  (штампы кадров квантованы 4 мс /clock; в полёте ATTITUDE 12.5 Гц — хуже);
  боковая на разворотах — Δroll и ω_z·dt (утечка X·dψ; в коэффициент ω_z входит и
  плечо камеры 0.15 м = 150 мм/рад: истина twist — у base_link, а канал видит камеру).
  Побочно: путь меряется 0.89 от истины на ВСЕХ вариантах swing при palt/AGL 0.998 —
  постоянный масштаб (кандидат — смещение наклона камеры ~1.5–2°); на низких
  полётах масштаб 0.96–1.06. Боковой gain 0.42 на swing — особенность разворотов
  (лаг фильтра при вращающемся body-frame): на 194912/034514 он 0.84/0.72.
Вывод для демпфера: ошибка скорости 0.2 м/с — лаг (win + vel_tau), шум наклона
МНК от 7 мм/кадр ≈ 0.03 м/с; окно можно резать до ~0.15 с, не упираясь в пиксели.

⚠️ СТЕНД ГОНЯЕТ БОЕВОЙ КОД: выпрямление, гейты, дерот и скорость считает настоящий
`FlowEstimator._ipm_update` из `control_pkg` (как `ipm_alt_replay.py`); варианты
отличаются ТОЛЬКО геометрией полосы (`ipm_yhalf`, `ipm_x0`, `ipm_x1`) и лимитом
фич (`max_feats`) — это ctor-аргументы класса, в `config.py`/`BS_*` ручек нет.
Остальные ручки канала — ЛЁТНЫЕ: дефолты `BootstrapConfig` + `BS_IPM_*` окружения
+ мета архивного прогона `<NAME>.env` рядом с bag (та же лесенка, что у `ipm_video.py`).

Что меряется на каждом варианте (окно «в воздухе» = истинная AGL > IB_AIR):
- коды брака кадра, доля кадров с углами полосы ЗА кадром (чёрные клинья в варпе —
  при yhalf 3.5 таких уже 53%);
- сколько фич посеяно / выжило LK (счётчик перехватом `cv2.calcOpticalFlowPyrLK`:
  наружу `_ipm_update` его не отдаёт);
- скорости канала против истины Gazebo (gain / corr / сдвиг / СКО) — то, что видит
  демпфер, с лагом МНК и фильтра;
- СЫРОЙ сигнал: приращение пути `ipm_fwd/ipm_lat` за кадр минус истина·dt (мм) —
  без окна и фильтра; его разложение МНК на Δpitch, Δroll, ω_z·dt и долю пути
  (масштаб); попарно с базовым вариантом — разница приращений (= пиксельная часть).
`IB_DETAIL=1` — для базового варианта ещё разрез «тихие/динамичные кадры»,
регрессия на Δh (высота как источник выбросов) и топ выбросов с контекстом.

⚠️ УГЛЫ И ω — ИСТИНА GAZEBO (`/model/iris_cam/odometry`; в freefly-бэгах
`/mavros/imu/data` не пишется), стенд чуть оптимистичнее полёта по ориентации.
Истина скорости — body-twist того же топика (FLU: x вперёд, y ВЛЕВО; сверка
конвенции — в `ipm_alt_replay.py`). Высота перцепции: `IB_ALT_SRC=latch` (дефолт)
= EKF local z − z₀, где z₀ — последнее z перед фронтом armed (правило ноды, нужен
`mavros_msgs`); без `mavros_msgs` (хост) — медиана z за 2 с до отрыва (на земле
EKF z стоит ±0.03, разница с латчем ноды в шуме). `true` — истинная AGL (потолок).
⚠️ НЕ брать `palt=` из `/mission/status`: там одна десятичная (шаг 0.1 м) — ступени
высоты дают ложные выбросы ±100 мм/кадр (первая версия стенда на это попалась).

Бэг читается НАПРЯМУЮ из sqlite (+ `rclpy.serialization`), без `rosbag2_py` и без
`mavros_msgs` — чтобы стенд бегал и на ХОСТЕ (venv с cv2 + `source
/opt/ros/jazzy/setup.bash`), когда контейнер nav погашен, и в контейнере.

Запуск на хосте (из корня репо):
  source /opt/ros/jazzy/setup.bash
  IB_BAG=docker/sim/output/joystick/yaw/swing/1/bag python3 src/lab/ipm_band_ab.py
В контейнере nav:
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash;
    IB_BAG=/root/sim_ws/output/joystick/<RUN>/bag python3 /lab/ipm_band_ab.py'
Пути к исходникам: `../control`, `../mission` от этого файла (репо) или
`/root/sim_ws/src/{control,mission}` (контейнер); `CTRL_SRC`/`MISSION_SRC` — явно.
⚠️ Исходники, а не colcon-install: colcon КОПИРУЕТ ament_python-пакет, иначе стенд
молча проверит версию кода на момент последнего colcon build.

Env: IB_BAG (каталог bag), IB_VARIANTS («yhalf:feats:x0:x1,…», дефолт — свип ширины
+ длины/дальности из шапки), IB_ALT_SRC (latch|true), IB_AIR (0.15 м), IB_DETAIL (0/1),
IB_CSV (дамп по кадрам и вариантам); интринсики — из разрешения кадра (pinhole 90°
hfov, как RosPerception) + весь лётный BS_IPM_* / BS_PERC_ALT_*.
"""
import math
import os
import sqlite3
import sys
from collections import Counter

import numpy as np

import cv2

_HERE = os.path.dirname(os.path.abspath(__file__))
for _env, _rel, _cont in (('CTRL_SRC', '../control', '/root/sim_ws/src/control'),
                          ('MISSION_SRC', '../mission', '/root/sim_ws/src/mission')):
    for _cand in (os.environ.get(_env, ''), os.path.join(_HERE, _rel), _cont):
        if _cand and os.path.isdir(_cand):
            sys.path.insert(0, os.path.abspath(_cand))
            break

from geometry_msgs.msg import PoseStamped                                # noqa: E402
from nav_msgs.msg import Odometry                                        # noqa: E402
from rclpy.serialization import deserialize_message                      # noqa: E402
from sensor_msgs.msg import Image                                        # noqa: E402

import control_pkg.perception.ipm as ipm_mod                             # noqa: E402
from control_pkg.perception.flow_estimator import FlowEstimator          # noqa: E402
from mission_pkg.config import BootstrapConfig                           # noqa: E402

BAG = os.environ.get('IB_BAG', '/root/sim_ws/output/scene_bag')
AIR = float(os.environ.get('IB_AIR', 0.15))
ALT_SRC = os.environ.get('IB_ALT_SRC', 'latch')
DETAIL = os.environ.get('IB_DETAIL', '0') == '1'
CSV = os.environ.get('IB_CSV', '')
# ДИАГНОСТИКА ТАЙМИНГА УГЛОВ: IB_ATT_DELAY (с) — брать истинные углы на delay раньше
# штампа кадра (эмуляция устаревшей ориентации); IB_ATT_HOLD_HZ — «ступенька»
# (sample-and-hold с таким темпом, как ATTITUDE-сообщения); IB_T0/IB_T1 — окно
# реплея по sim-времени от первого odom (ускоряет прогон одного участка).
ATT_DELAY = float(os.environ.get('IB_ATT_DELAY', '0'))
ATT_HOLD_HZ = float(os.environ.get('IB_ATT_HOLD_HZ', '0'))
# IB_ATT_EXTRAP=1 — «ступеньку» дотягивать до кадра истинной ω (эмуляция att_extrap
# ноды с хорошим гироскопом): угол = удержанный отсчёт + ω·(t − t_отсчёта).
ATT_EXTRAP = os.environ.get('IB_ATT_EXTRAP', '0') == '1'
T_WIN = (float(os.environ.get('IB_T0', '-1')), float(os.environ.get('IB_T1', '1e9')))
# ровно то, что кладёт в оценщик bootstrap_node (FLOW_R / FLOW_ROTSIGN); сам
# bootstrap_node не импортируем — он тянет rclpy и весь ROS-стек (на хосте нет mavros)
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
FLOW_ROTSIGN = 1.0
# ручки канала, которые кладёт в оценщик bootstrap_node (те же BS_-имена, что ipm_video)
IPM_KNOBS = ('ipm_model', 'ipm_derot', 'ipm_wz_tau', 'ipm_wz_gate', 'ipm_win', 'ipm_adapt',
             'ipm_vel_tau', 'ipm_alt_floor', 'ipm_scale_ref', 'ipm_acc_tau')
# базовая геометрия полосы = класс-дефолты FlowEstimator (ими и летаем)
BASE = (2.0, 200, 3.0, 6.0)
DEFAULT_VARIANTS = '1.0:200:3:6,2.0:200:3:6,2.0:400:3:6,3.5:350:3:6,' \
                   '2.0:300:3:9,2.0:400:3:12,2.0:200:6:9'
FAIL_NAME = {0: 'годен', 1: 'гейт высоты', 2: 'окно не видно', 3: 'варп за кадром',
             4: 'мало фич', 5: 'мало выживших LK', 6: 'выключен', 7: 'нет опорного'}


def parse_variants(spec):
    out = []
    for tok in spec.split(','):
        tok = tok.strip()
        if not tok:
            continue
        yh, nf, x0, x1 = tok.split(':')
        v = (float(yh), int(nf), float(x0), float(x1))
        tag = 'ПОЛЁТ' if v == BASE else (
            'ДАЛЬШЕ' if v[2] > BASE[2] else 'ДЛИННЕЕ' if v[3] > BASE[3] else '')
        name = f'y{v[0]:g} f{v[1]} x{v[2]:g}-{v[3]:g}' + (f' ({tag})' if tag else '')
        out.append((name, v))
    return out


def env_from_archive(bag):
    """`BS_*` из меты архивного прогона `joystick/<NAME>/<NAME>.env` (см. ipm_video)."""
    d = os.path.dirname(os.path.abspath(bag.rstrip('/')))
    metas = [f for f in sorted(os.listdir(d)) if f.endswith('.env')] \
        if os.path.isdir(d) else []
    if not metas:
        return None
    n = 0
    for line in open(os.path.join(d, metas[0])):
        line = line.strip()
        if not line.startswith('BS_') or '=' not in line:
            continue
        k, v = line.split('=', 1)
        if k not in os.environ:
            os.environ[k] = v
            n += 1
    return f'{metas[0]} (+{n} BS_*)'


def flight_cfg():
    """Лётный конфиг канала: дефолты `BootstrapConfig` + `BS_*` (лесенка ipm_video)."""
    base = BootstrapConfig()
    cfg, defaulted = {}, []
    for k in IPM_KNOBS:
        d = getattr(base, k)
        v = os.environ.get('BS_' + k.upper())
        if v is None or v == '':
            cfg[k] = d
            defaulted.append(k)
        else:
            cfg[k] = str(v) if isinstance(d, str) else float(v)
    return cfg, defaulted


class CV2Proxy:
    """Модуль cv2 для канала IPM (perception/ipm.py — LK живёт там) с перехватом:
    счётчик точек наружу не отдаётся."""

    def __init__(self):
        self.n_in = self.n_ok = 0
        self.first = True

    def __getattr__(self, name):
        return getattr(cv2, name)

    def calcOpticalFlowPyrLK(self, *a, **k):
        r = cv2.calcOpticalFlowPyrLK(*a, **k)
        if self.first:                 # прямой вызов пары (prev→rect); второй — обратный
            self.n_in = len(a[2])
            self.n_ok = int(r[1].sum())
        self.first = not self.first
        return r


def euler(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def messages(db, tid, topic):
    return db.execute('select data from messages where topic_id=? order by timestamp',
                      (tid[topic],))


def latch_z0(db, tid, lp, t_off):
    """z₀ латча: правило ноды (последнее z перед фронтом armed) или запасной вариант."""
    if '/mavros/state' in tid:
        try:
            from mavros_msgs.msg import State
            for (raw,) in messages(db, tid, '/mavros/state'):
                m = deserialize_message(raw, State)
                if m.armed:
                    i = int(np.searchsorted(lp[:, 0], stamp(m), 'right')) - 1
                    return float(lp[max(0, i), 1]), 'z перед фронтом armed (как нода)'
        except ImportError:
            pass
    pre = (lp[:, 0] >= t_off - 2.0) & (lp[:, 0] <= t_off)
    return float(np.median(lp[pre, 1])), 'медиана z за 2 с до отрыва (нет mavros_msgs)'


def fit_line(x, y):
    A = np.column_stack([x, np.ones(len(x))])
    (g, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    return g, b


def main():
    db = sqlite3.connect(os.path.join(BAG, 'scene_bag_0.db3'))
    tid = {n: i for n, i in db.execute('select name,id from topics')}
    for need in ('/image_color', '/model/iris_cam/odometry', '/mavros/local_position/pose'):
        if need not in tid and not (need.startswith('/mavros') and ALT_SRC == 'true'):
            sys.exit(f'⚠️ в bag нет {need}')
    od = []
    for (raw,) in messages(db, tid, '/model/iris_cam/odometry'):
        m = deserialize_message(raw, Odometry)
        p, v, w = m.pose.pose.position, m.twist.twist.linear, m.twist.twist.angular
        od.append((stamp(m), p.z) + euler(m.pose.pose.orientation)
                  + (v.x, v.y, w.x, w.y, w.z))
    od = np.array(od)          # t, z, roll, pitch, yaw, vf, vl(влево+), p, q, r
    t0 = od[0, 0]
    ground = float(np.median(od[:60, 1]))          # борт стоит на земле
    agl = od[:, 1] - ground
    air = agl > AIR
    if not air.any():
        sys.exit('⚠️ борт не отрывался — сравнивать нечего')
    w0 = od[int(np.argmax(air)), 0]
    w1 = od[len(air) - 1 - int(np.argmax(air[::-1])), 0]
    meta = env_from_archive(BAG)
    cfg, defaulted = flight_cfg()
    print(f'bag {BAG}')
    if meta:
        print(f'  мета прогона: {meta}')
    print('  конфиг канала: ' + ' '.join(
        f'{k.replace("ipm_", "")}={v}' + ('*' if k in defaulted else '')
        for k, v in cfg.items()) + ('   (* — дефолт config.py)' if defaulted else ''))
    sel_air = (od[:, 0] >= w0) & (od[:, 0] <= w1)
    print(f'  воздух {w0-t0:.1f}…{w1-t0:.1f} с, AGL сред {agl[sel_air].mean():.2f} макс '
          f'{agl.max():.2f} м; |v| сред {np.hypot(od[sel_air,5], od[sel_air,6]).mean():.2f} '
          f'м/с, |ω_z| сред {np.abs(od[sel_air,9]).mean():.2f} макс '
          f'{np.abs(od[sel_air,9]).max():.2f} рад/с')
    # высота перцепции
    if ALT_SRC == 'true':
        alt_ts, alt_v = od[:, 0], np.maximum(0.0, agl)
        print('  высота перцепции: ИСТИННАЯ AGL (потолок)')
    else:
        lp = np.array([(stamp(m), m.pose.position.z) for m in
                       (deserialize_message(raw, PoseStamped) for (raw,) in
                        messages(db, tid, '/mavros/local_position/pose'))])
        z0, how = latch_z0(db, tid, lp, w0)
        alt_ts, alt_v = lp[:, 0], np.maximum(0.0, lp[:, 1] - z0)
        print(f'  высота перцепции: EKF local z − z₀, z₀={z0:+.3f} м ({how}); '
              f'{len(lp)} сэмпл., в воздухе palt/AGL = '
              f'{np.mean(np.interp(od[sel_air,0], alt_ts, alt_v) / np.maximum(agl[sel_air], 0.3)):.3f}')
    variants = parse_variants(os.environ.get('IB_VARIANTS', DEFAULT_VARIANTS))
    base_name = next((n for n, v in variants if v == BASE), variants[0][0])

    proxy = CV2Proxy()
    ipm_mod.cv2 = proxy
    ests, rows, corners = {}, {}, {}
    n = 0
    W = H = None
    for (raw,) in messages(db, tid, '/image_color'):
        msg = deserialize_message(raw, Image)
        t = stamp(msg)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == 'bgr8':
            gray = cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_BGR2GRAY)
        elif msg.encoding in ('mono8', '8UC1'):
            gray = buf.reshape(msg.height, msg.width).copy()
        else:
            continue
        if not ests:
            # интринсики — из разрешения кадра (pinhole 90° hfov), как RosPerception
            W, H = msg.width, msg.height
            fx = W / 2.0
            for name, (yh, nf, x0, x1) in variants:
                ests[name] = FlowEstimator(fx, fx, W / 2.0, H / 2.0, FLOW_R, FLOW_ROTSIGN,
                                           max_feats=nf, ipm_yhalf=yh, ipm_x0=x0,
                                           ipm_x1=x1, **cfg)
                rows[name], corners[name] = [], Counter()
            print(f'  кадр {W}×{H} → fx=fy={fx:.0f}; варианты: '
                  + ', '.join(n_ for n_, _ in variants))
        if not (T_WIN[0] <= t - t0 <= T_WIN[1]):
            continue
        ta = t - ATT_DELAY
        if ATT_HOLD_HZ > 0:
            ta = math.floor(ta * ATT_HOLD_HZ) / ATT_HOLD_HZ
        roll = float(np.interp(ta, od[:, 0], od[:, 2]))
        pitch = float(np.interp(ta, od[:, 0], od[:, 3]))
        if ATT_EXTRAP and ta < t:
            # дотяжка гироскопом (тело: p = ω_x → крен, q = ω_y → тангаж, как attitude_at)
            roll += float(np.interp(ta, od[:, 0], od[:, 7])) * (t - ta)
            pitch += float(np.interp(ta, od[:, 0], od[:, 8])) * (t - ta)
        wz = float(np.interp(t, od[:, 0], od[:, 9]))
        alt = float(np.interp(t, alt_ts, alt_v))
        in_air = w0 <= t <= w1
        for name, est in ests.items():
            proxy.first, proxy.n_in, proxy.n_ok = True, 0, 0
            est._ipm_update(gray, t, alt, pitch, roll, wz)
            rows[name].append((t, est.ipm_fail, float(est.ipm_ok), est.ipm_vfwd,
                               est.ipm_vlat, est.ipm_fwd, est.ipm_lat, proxy.n_in,
                               proxy.n_ok, alt, est.ipm_noise_fwd, est.ipm_noise_lat))
            g = est._ipm_prev_geo
            if g is not None and in_air:
                # углы полосы, которую канал ТОЛЬКО ЧТО обработал, за кадром → чёрные
                # клинья в варпе (проецируем той же высотой, что считала варп)
                x0g, ln, yhg, _ = g
                h = max(alt, est.ipm_alt_floor) if est.ipm_alt_floor > 0 else alt
                out = 0
                for X, Y in ((x0g, -yhg), (x0g, yhg), (x0g + ln, yhg), (x0g + ln, -yhg)):
                    p = est._ipm_px(X, Y, h, pitch, roll)
                    if p is None or not (0 <= p[0] < W and 0 <= p[1] < H):
                        out += 1
                corners[name][out] += 1
        n += 1
    print(f'  кадров прокручено: {n}')

    incr = {}
    for name, _v in variants:
        a = np.array(rows[name])
        sel = (a[:, 0] >= w0) & (a[:, 0] <= w1)
        ok = sel & (a[:, 2] > 0.5)
        print(f'\n=== {name} ===')
        c = Counter(int(v) for v in a[sel, 1])
        print('  коды брака: ' + '  '.join(
            f'{f} {FAIL_NAME.get(f, "?")} {100.0*k/max(1, sel.sum()):.0f}%'
            for f, k in c.most_common()))
        cc, tot = corners[name], max(1, sum(corners[name].values()))
        print(f'  углы полосы за кадром: нет {100.0*cc[0]/tot:.0f}%  один '
              f'{100.0*cc[1]/tot:.0f}%  два+ '
              f'{100.0*sum(v for k, v in cc.items() if k >= 2)/tot:.0f}%')
        if ok.sum() < 20:
            print('  измерений мало — метрику не считаем')
            continue
        print(f'  фич посеяно сред {a[ok,7].mean():.0f}, выжило LK сред {a[ok,8].mean():.0f}; '
              f'ipm_ok {100.0*a[sel,2].mean():.0f}% кадров')
        ts = a[ok, 0]
        for lab, meas, true in (('vfwd (вперёд+)', a[ok, 3], np.interp(ts, od[:, 0], od[:, 5])),
                                ('vlat (влево+) ', a[ok, 4], np.interp(ts, od[:, 0], od[:, 6]))):
            g, _b = fit_line(true, meas)
            cor = np.corrcoef(meas, true)[0, 1] if true.std() > 1e-6 else float('nan')
            print(f'  {lab}: gain {g:+.2f}  corr {cor:+.2f}  сдвиг {np.mean(meas-true):+.3f}  '
                  f'СКО ош {np.std(meas-true):.3f} м/с  (|v_ист| сред {np.abs(true).mean():.2f})')
        # СЫРОЙ сигнал: приращение пути за кадр против истины, только соседние годные кадры
        idx = np.where(ok)[0]
        rec = []
        for i0, i1 in zip(idx[:-1], idx[1:]):
            dt = a[i1, 0] - a[i0, 0]
            if i1 != i0 + 1 or not (0.005 < dt < 0.1):
                continue
            tm = 0.5 * (a[i0, 0] + a[i1, 0])
            vf = float(np.interp(tm, od[:, 0], od[:, 5]))
            vl = float(np.interp(tm, od[:, 0], od[:, 6]))
            ang0 = [float(np.interp(a[i0, 0], od[:, 0], od[:, k])) for k in (2, 3)]
            ang1 = [float(np.interp(a[i1, 0], od[:, 0], od[:, k])) for k in (2, 3)]
            rates = [float(np.interp(tm, od[:, 0], od[:, k])) for k in (7, 8, 9)]
            rec.append((a[i1, 0], (a[i1, 5] - a[i0, 5] - vf * dt) * 1e3,
                        (a[i1, 6] - a[i0, 6] - vl * dt) * 1e3,
                        ang1[1] - ang0[1], ang1[0] - ang0[0], rates[2] * dt, vf * dt,
                        vl * dt, a[i1, 9] - a[i0, 9], 0.5 * (a[i1, 9] + a[i0, 9]),
                        vf, *rates))
        r = np.array(rec)
        # столбцы: 0 t, 1 err_f мм, 2 err_l мм, 3 Δpitch, 4 Δroll, 5 ω_z·dt, 6 путь_f,
        #          7 путь_l, 8 Δh, 9 h, 10 vf, 11 p, 12 q, 13 r
        incr[name] = r
        ef, el = r[:, 1], r[:, 2]
        print(f'  сырой сигнал (приращение за кадр − истина·dt), мм: fwd СКО {ef.std():.1f} '
              f'медиана|.| {np.median(np.abs(ef)):.1f}  lat СКО {el.std():.1f} '
              f'медиана|.| {np.median(np.abs(el)):.1f}   (n={len(r)})')
        for lab, err, path in (('fwd', ef, r[:, 6]), ('lat', el, r[:, 7])):
            A = np.column_stack([r[:, 3], r[:, 4], r[:, 5], path * 1e3, np.ones(len(r))])
            coef, *_ = np.linalg.lstsq(A, err, rcond=None)
            fit = A @ coef
            print(f'    разложение {lab}: R²={1-np.var(err-fit)/np.var(err):.2f}  '
                  f'Δpitch {coef[0]:+.0f}  Δroll {coef[1]:+.0f}  ω_z·dt {coef[2]:+.0f} мм/рад  '
                  f'масштаб пути {1+coef[3]:.3f}  остаток СКО {np.std(err-fit):.1f} мм')

    if base_name in incr:
        print(f'\n=== ПОПАРНО с базой «{base_name}»: разница приращений за кадр '
              f'(= пиксельная часть ошибки) ===')
        rb = incr[base_name]
        for name, _v in variants:
            if name == base_name or name not in incr:
                continue
            rv = incr[name]
            _c, ib, iv = np.intersect1d(rb[:, 0], rv[:, 0], return_indices=True)
            print(f'  {name:28s} n={len(ib)}  fwd СКО разницы {np.std(rb[ib,1]-rv[iv,1]):.1f} мм '
                  f'(corr ошибок {np.corrcoef(rb[ib,1], rv[iv,1])[0,1]:+.2f})  '
                  f'lat {np.std(rb[ib,2]-rv[iv,2]):.1f} мм '
                  f'(corr {np.corrcoef(rb[ib,2], rv[iv,2])[0,1]:+.2f})')

    if DETAIL and base_name in incr:
        r = incr[base_name]
        ef, el = r[:, 1], r[:, 2]
        quiet = (np.abs(r[:, 11]) < 0.1) & (np.abs(r[:, 12]) < 0.1) & (np.abs(r[:, 13]) < 0.1)
        print(f'\n=== ДЕТАЛИ базы «{base_name}» ===')
        print(f'  тихие кадры (|p|,|q|,|r| < 0.1 рад/с): {quiet.sum()} → fwd СКО '
              f'{ef[quiet].std():.1f} медиана|.| {np.median(np.abs(ef[quiet])):.1f}  '
              f'lat СКО {el[quiet].std():.1f} мм')
        print(f'  динамичные: {(~quiet).sum()} → fwd СКО {ef[~quiet].std():.1f} медиана|.| '
              f'{np.median(np.abs(ef[~quiet])):.1f}  lat СКО {el[~quiet].std():.1f} мм')
        for lab, k in (('|q| pitch-rate', 12), ('|p| roll-rate', 11), ('|r| yaw-rate', 13)):
            x = np.abs(r[:, k])
            print(f'    corr(|err_fwd|, {lab}) {np.corrcoef(np.abs(ef), x)[0,1]:+.2f}   '
                  f'corr(|err_lat|, {lab}) {np.corrcoef(np.abs(el), x)[0,1]:+.2f}')
        dh = r[:, 8]
        print(f'  Δh перцепции между кадрами: СКО {dh.std()*1e3:.1f} мм, макс '
              f'{np.abs(dh).max()*1e3:.1f} мм')
        g, b = fit_line(dh / r[:, 9], ef)
        fit = g * dh / r[:, 9] + b
        print(f'  err_fwd ≈ {g*1e-3:+.2f} м · (Δh/h)  R²={1-np.var(ef-fit)/np.var(ef):.2f} '
              f'(ступенчатая высота вылезла бы здесь)')
        big = np.abs(ef) > 40
        print(f'  кадров |err_fwd| > 40 мм: {big.sum()}, из них |Δh| > 20 мм: '
              f'{np.sum(big & (np.abs(dh) > 0.02))}')
        print('  топ-8 выбросов fwd: t, err мм, vf, q(pitch-rate), p(roll-rate), r(yaw-rate)')
        for j in np.argsort(-np.abs(ef))[:8]:
            print(f'    t={r[j,0]-t0:5.1f}  err {ef[j]:+6.1f}  vf {r[j,10]:+.2f}  '
                  f'q {r[j,12]:+.2f}  p {r[j,11]:+.2f}  r {r[j,13]:+.2f}')

    if CSV:
        with open(CSV, 'w') as f:
            f.write('t,variant,fail,ok,vfwd,vlat,ipm_fwd,ipm_lat,n_in,n_ok,alt,noise_fwd,noise_lat\n')
            for name, _v in variants:
                for row in rows[name]:
                    f.write(f'{row[0]-t0:.3f},{name},{int(row[1])},{row[2]:.0f},'
                            f'{row[3]:.4f},{row[4]:.4f},{row[5]:.4f},{row[6]:.4f},'
                            f'{row[7]},{row[8]},{row[9]:.3f},{row[10]:.4f},{row[11]:.4f}\n')
        print(f'\nдамп по кадрам → {CSV}')


if __name__ == '__main__':
    main()
