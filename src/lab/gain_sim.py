#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ГЕЙНЫ ДЕМПФЕРА: идентификация контура по бэгу + КОНТРФАКТНЫЙ свип.

Инструмент лесенки настройки: после каждого прогона отвечает на два вопроса —
«что контур сделал на самом деле» и «что было бы при других гейнах», не тратя
на второй ни одного полёта.

ЗАЧЕМ. Демпфер (`DpRollRate`/`DpPitchRate`) — ПИ по скорости вида сверху, и его
поведение целиком задают четыре числа: гейны kp/ki, запаздывание канала и ветер.
Крутить их вслепую дорого: прогон стоит 5-7 минут плюс пилота, а разница между
соседними ступенями видна только в первых 5 секундах после отрыва.

МОДЕЛЬ (идентифицируется из самого бэга, не постулируется):
    v̇ = α·PWM + β·v + γ + порыв(t)          — плант
    v_изм = апериодика(v, τ_s)               — что видит канал
    i = clamp(i + ki·e·dt, ±imax);  u = clamp(kp·e + i, ±150)   — как в
    `_FlowDamper1D.update` (e = v_изм − цель)
Смысл коэффициентов: α — цена PWM в ускорении (150 PWM = столько-то м/с²),
β — аэродемпфер (в этом симе ≈0, борт ничто не тормозит), **γ — ВЕТЕР**, и
именно из него берётся ДОЛГ ИНТЕГРАТОРА: постоянные γ/α PWM может выдать только
интегратор, и он набирает их за (γ/α)/ki метров ИЗМЕРЕННОГО пути. Это и есть
цена первых секунд после отрыва в метрах — от kp она не зависит.

⚠️ ЧЕГО СТЕНД НЕ ЗНАЕТ (и почему его числа — отношения, а не абсолют):
  * контур FCU по углу (задержка ~0.1-0.2 с) в модели ОТСУТСТВУЕТ → на высоких
    гейнах стенд ОПТИМИСТИЧЕН по устойчивости;
  * абсолютный пик скорости занижается в 1.35-1.4× (замер: 0.77 против
    лётных 1.06 и 0.55 против 0.78) — сравнивать строки свипа между собой;
  * контрфакт держит порыв r(t) НЕЗАВИСИМЫМ от манёвра: чем сильнее новые гейны
    меняют траекторию, тем больше это допущение врёт;
  * τ_s ЗАВИСИТ ОТ ОКНА: на чистом висении у земли 0.60 с, а на окне с набором
    до 4.5 м и манёврами — 1.05 с (подгонка лага съедает и ошибку масштаба).
    Поэтому окно по умолчанию — 12 с после отрыва (GS_WIN).
⚠️ Ось считается ОДНА и в предположении, что стик по ней В ЦЕНТРЕ: стенд сам
проверяет `/joy` в окне и ругается, если пилот рулил (тогда цель не ноль и
модель мерит не то).

Запуск (в контейнере nav — нужен rosbag2_py; cv2 и control_pkg НЕ нужны):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    python3 /lab/gain_sim.py /root/sim_ws/output/joystick/<RUN>/bag'
Лётные гейны берутся из меты прогона (`<RUN>/<NAME>.env`) — их же стенд
использует для СВЕРКИ модели с записью; без меты задать GS_KP/GS_KI руками.

Env: GS_WIN (окно с отрыва, с; 12), GS_AXIS (roll|pitch), GS_KP/GS_KI (лётные
гейны, если нет меты), GS_IMAX (150), GS_SWEEP (список «kp:ki:pos_kp» через
запятую), GS_VMAX (потолок станции, 1.0).
"""
import os
import sys

import numpy as np

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import (ConverterOptions, SequentialReader, StorageFilter,
                        StorageOptions)
from sensor_msgs.msg import Joy

DT = 0.05                      # шаг модели, с (тик ноды 20 Гц)
UMAX = 150.0
WIN = float(os.environ.get('GS_WIN', '12'))
AXIS = os.environ.get('GS_AXIS', 'roll')
VMAX = float(os.environ.get('GS_VMAX', '1.0'))
SWEEP = os.environ.get('GS_SWEEP', '90:30:0, 90:60:0, 90:90:0, 120:60:0,'
                                   '120:90:0, 60:60:0, 90:0:0.5, 90:60:0.3')
# ось → (истина в od, топик PWM, топик измеренной скорости, поле)
AX = {'roll':  (5, '/flow_dbg', '/flow_dbg9', 1),
      'pitch': (4, '/flow_dbg2', '/flow_dbg8', 2)}
TOPICS = ['/model/iris_cam/odometry', '/flow_dbg', '/flow_dbg2',
          '/flow_dbg8', '/flow_dbg9', '/joy']


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def load(bag):
    """Только опоры: истина Gazebo, PWM осей, скорости канала, стики."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    have = {t.name for t in r.get_all_topics_and_types()}
    r.set_filter(StorageFilter(topics=[t for t in TOPICS if t in have]))
    od, joy, dbg = [], [], {t: [] for t in TOPICS if t.startswith('/flow')}
    now = 0.0
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            now = stamp(m)
            p, v = m.pose.pose.position, m.twist.twist.linear
            od.append((now, p.x, p.y, p.z, v.x, v.y))     # 4 = вперёд, 5 = влево
        elif topic == '/joy':
            m = deserialize_message(raw, Joy)
            a = list(m.axes) + [0.0] * 8
            joy.append((now, a[0], a[1]))                 # 0 = крен, 1 = тангаж
        else:
            m = deserialize_message(raw, Vector3Stamped)
            now = stamp(m)
            dbg[topic].append((now, m.vector.x, m.vector.y, m.vector.z))
    return np.array(od), np.array(joy), {k: np.array(v) for k, v in dbg.items()}


def flight_gains(bag):
    """Лётные гейны из меты прогона (`<RUN>/<NAME>.env`) — иначе GS_KP/GS_KI."""
    d = os.path.dirname(os.path.abspath(bag.rstrip('/')))
    env = {}
    if os.path.isdir(d):
        for f in sorted(os.listdir(d)):
            if f.endswith('.env'):
                for line in open(os.path.join(d, f)):
                    if '=' in line and line.startswith('BS_'):
                        k, v = line.strip().split('=', 1)
                        env[k] = v
                break
    up = AXIS.upper()
    kp = env.get(f'BS_{up}_RATE_KP', os.environ.get('GS_KP', '30'))
    ki = env.get(f'BS_{up}_RATE_KI', os.environ.get('GS_KI', '30'))
    imax = env.get('BS_ROLL_IMAX', os.environ.get('GS_IMAX', '150'))
    return float(kp), float(ki), float(imax), bool(env)


def lagf(x, tau):
    """Апериодика первого порядка — модель запаздывания канала."""
    a = 1.0 - np.exp(-DT / max(tau, 1e-6))
    y, acc = np.empty_like(x), x[0]
    for i, v in enumerate(x):
        acc += a * (v - acc)
        y[i] = acc
    return y


def peaks(t, v, gap=2.0, floor=0.08):
    """Экстремумы знакопеременного сигнала (для периода и логдекремента)."""
    out = []
    for i in range(2, len(v) - 2):
        if (v[i] - v[i - 1]) * (v[i + 1] - v[i]) < 0 and abs(v[i]) > floor:
            if out and t[i] - out[-1][0] < gap:
                if abs(v[i]) > abs(out[-1][1]):
                    out[-1] = (t[i], v[i])
            else:
                out.append((t[i], v[i]))
    return out


def sim(kp, ki, imax, al, be, ga, r, tau_s, v0, n, pos_kp=0.0):
    """Контрфакт: тот же порыв, те же начальные условия, другие гейны."""
    v, vm, vc, i, x = np.empty(n), v0, v0, 0.0, 0.0
    a = 1.0 - np.exp(-DT / max(tau_s, 1e-6))
    for k in range(n):
        v[k] = vc
        vm += a * (vc - vm)                  # что видит канал
        x += vm * DT                         # путь по ИЗМЕРЕННОЙ скорости (ipm_lat)
        tgt = np.clip(-pos_kp * x, -VMAX, VMAX) if pos_kp > 0 else 0.0
        e = vm - tgt
        i = np.clip(i + ki * e * DT, -imax, imax)
        u = np.clip(kp * e + i, -UMAX, UMAX)
        vc += (al * u + be * vc + ga + r[k]) * DT
    return v


def main(bag):
    col, t_pwm, t_meas, f_meas = AX[AXIS]
    od, joy, dbg = load(bag)
    if not len(od):
        sys.exit('⚠️ в bag нет /model/iris_cam/odometry — истины нет, мерить нечем')
    if t_pwm not in dbg or not len(dbg[t_pwm]):
        sys.exit(f'⚠️ в bag нет {t_pwm} — команды оси нет')
    ground = float(np.median(od[:60, 3]))
    agl = od[:, 3] - ground
    lift = int(np.argmax(agl > 0.10))
    t_lift = od[lift, 0]
    t_end = od[int(np.max(np.where(agl > 0.05))), 0]
    kp0, ki0, imax, from_meta = flight_gains(bag)
    print(f'=== {bag.rstrip("/").split("/")[-2]} · ось {AXIS} ===')
    print(f'  отрыв t={t_lift-od[0,0]:.1f} с, воздух {t_end-t_lift:.0f} с, '
          f'AGL макс {agl.max():.2f} м')
    print(f'  лётные гейны: kp={kp0:.0f} ki={ki0:.0f} imax={imax:.0f} '
          f'({"из меты прогона" if from_meta else "GS_KP/GS_KI"})')

    # ── измеренное поведение контура ────────────────────────────────────────
    air = (od[:, 0] >= t_lift) & (od[:, 0] <= t_end)
    tt, vt = od[air, 0] - t_lift, od[air, col]
    pk = peaks(tt, vt)
    if len(pk) >= 3:
        per = float(np.mean(np.diff([p[0] for p in pk[:6]]))) * 2
        rat = float(np.mean([abs(pk[i + 1][1] / pk[i][1])
                             for i in range(min(4, len(pk) - 1))]))
        dec = -np.log(max(rat, 1e-6) ** 2)
        z = dec / np.sqrt(4 * np.pi ** 2 + dec ** 2)
        print('  пики скорости: ' + '  '.join(f'{p[0]:.1f}с {p[1]:+.2f}' for p in pk[:6]))
        print(f'  период ≈ {per:.1f} с (ω_n {2*np.pi/per:.2f} рад/с), ζ ≈ {z:+.2f}'
              + (' ⚠️ ОТРИЦАТЕЛЬНОЕ — колебание РАСТЁТ' if z < 0 else
                 ' (комфорт 0.7)'))
    x0, y0 = od[lift, 1], od[lift, 2]
    d = np.hypot(od[:, 1] - x0, od[:, 2] - y0)
    i5 = int(np.searchsorted(od[:, 0], t_lift + 5.0))
    print(f'  снос: за 5 с {d[min(i5,len(d)-1)]:.2f} м, пик {d[air].max():.1f} м')

    # ── окно идентификации ──────────────────────────────────────────────────
    t1 = min(t_end, t_lift + WIN) if WIN > 0 else t_end
    g = np.arange(t_lift, t1, DT)
    if len(g) < 40:
        sys.exit('⚠️ окно короче 2 с — идентифицировать нечего')
    v = np.interp(g, od[:, 0], od[:, col])
    u = np.interp(g, dbg[t_pwm][:, 0], dbg[t_pwm][:, 1])
    if len(joy):
        w = (joy[:, 0] >= t_lift) & (joy[:, 0] <= t1)
        s_max = float(np.abs(joy[w, 1 if AXIS == 'roll' else 2]).max()) if w.sum() else 0.0
        if s_max > 0.05:
            print(f'  ⚠️ ПИЛОТ РУЛИЛ этой осью в окне (стик до {s_max:.2f}) — цель НЕ ноль, '
                  f'идентификация врёт. Сузить GS_WIN или взять другой прогон.')
    # 1. запаздывание канала: подгонка апериодики к тому, что канал печатал
    tau_s = float('nan')
    if t_meas in dbg and len(dbg[t_meas]):
        m = np.interp(g, dbg[t_meas][:, 0], dbg[t_meas][:, f_meas])
        taus = np.arange(0.05, 1.65, 0.05)
        tau_s = float(taus[int(np.argmin([np.mean((lagf(v, t) - m) ** 2) for t in taus]))])
    if not np.isfinite(tau_s):
        tau_s = 0.6
        print('  ⚠️ нет скорости канала в bag — τ_s взят 0.6 с по умолчанию')
    # 2. плант; знак α должен быть ОТРИЦАТЕЛЕН (команда гасит скорость)
    vd = np.gradient(v, DT)
    A = np.column_stack([u, v, np.ones_like(v)])
    (al, be, ga), *_ = np.linalg.lstsq(A, vd, rcond=None)
    if al > 0:
        print('  ⚠️ α>0 — провод оси развёрнут (osign); PWM инвертирован для модели')
        u, al = -u, -al
    r = vd - np.column_stack([u, v, np.ones_like(v)]) @ np.array([al, be, ga])
    n = len(g)
    print(f'\n  ОКНО ИДЕНТИФИКАЦИИ {WIN:.0f} с после отрыва')
    print(f'  датчик: τ_s ≈ {tau_s:.2f} с (канал против истины)')
    print(f'  плант:  v̇ = {al:+.5f}·PWM {be:+.3f}·v {ga:+.3f} + порыв '
          f'(СКО порыва {r.std():.2f} м/с²)')
    print(f'          150 PWM = {abs(al)*150:.2f} м/с²; аэродемпфер '
          f'{("нет (β≥0)" if be >= 0 else f"{-1/be:.1f} с")}')
    wind = abs(ga / al)
    print(f'  ДОЛГ ИНТЕГРАТОРА: ветер = {wind:.0f} PWM; P выдал бы столько лишь при '
          f'постоянной ошибке {wind/max(kp0,1e-6):.2f} м/с, значит платит ki:')
    print(f'          при ki={ki0:.0f} — {wind/max(ki0,1e-6):.2f} м ИЗМЕРЕННОГО пути '
          f'(≈{wind/max(ki0,1e-6)/0.55:.1f} м истинного, канал на разгоне видит ~0.55)')
    base = sim(kp0, ki0, imax, al, be, ga, r, tau_s, v[0], n)
    print(f'  СВЕРКА с записью: пик |v| модель {np.abs(base).max():.2f} против '
          f'{np.abs(v).max():.2f} м/с — модель занижает в '
          f'{np.abs(v).max()/max(np.abs(base).max(),1e-6):.2f}×')

    # ── контрфактный свип ───────────────────────────────────────────────────
    n5 = min(n, int(5.0 / DT))
    print('\n    kp    ki  pos_kp | пик |v| | путь за 5 с | путь макс | путь в конце')
    for item in SWEEP.split(','):
        try:
            kp, ki, pk_ = (float(x) for x in item.strip().split(':'))
        except ValueError:
            continue
        s = sim(kp, ki, imax, al, be, ga, r, tau_s, v[0], n, pos_kp=pk_)
        x = np.cumsum(s) * DT
        mark = ' ← лётные' if (kp, ki, pk_) == (kp0, ki0, 0.0) else ''
        print(f'  {kp:5.0f} {ki:5.0f} {pk_:6.1f} | {np.abs(s).max():7.2f} | '
              f'{abs(x[n5-1]):11.1f} | {np.abs(x).max():9.1f} | '
              f'{abs(x[-1]):12.1f}{mark}')
    print('  ⚠️ числа модели — ОТНОШЕНИЯ строк, не абсолют (см. докстринг): контур '
          'FCU по углу не смоделирован, на высоких гейнах стенд оптимистичен.')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit('usage: gain_sim.py <bag> [<bag> ...]')
    for b in sys.argv[1:]:
        main(b)
