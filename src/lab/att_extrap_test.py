#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ЗАПАЗДЫВАНИЕ ОРИЕНТАЦИИ в канале вида сверху — стенд A/B по ОДНИМ кадрам.

⚠️ СТЕНД ГОНЯЕТ БОЕВОЙ КОД, А НЕ СВОЮ КОПИЮ. Выпрямление и скорость считает настоящий
`FlowEstimator._ipm_update` из `control_pkg`, углы на кадр — настоящая `attitude_at` из
`control_pkg.infrastructure.ros_perception`. Стенд отвечает только за чтение бэга и
за арифметику сверки. Своя копия геометрии (как в `ipm_flow_test.py`) для A/B не годится:
она может «подтвердить» правку, которой в полёте нет.

ЧТО ЛЕЧИМ. `_ipm_rectify` строит полосу земли по текущим крену/тангажу, то есть вращение
оно снимает САМО. Течёт не вращение, а РАССИНХРОН: ATTITUDE идёт 12.5 Гц, камера 20-30 Гц,
а `RosPerception` до правки брал углы как «последнее пришедшее», без привязки к штампу
кадра. Внутри одного интервала ATTITUDE угол держится СТУПЕНЬКОЙ, истинный уходит со
скоростью ω — ошибка растёт линейно, полоса земли уезжает, и в скорость подмешивается
член, ПРОПОРЦИОНАЛЬНЫЙ УГЛОВОЙ СКОРОСТИ.

ЧИСЛО, КОТОРОЕ ЖДЁМ. Точка земли 4.5 м впереди при h=3 и наклоне 0.26 рад лежит на 163 px
ниже центра (fy≈480 на 960×540); ошибка крена δφ двигает её вбок на δφ·163 px, метр на
пиксель там 4.5/480 → 1.53 м на радиан. Регрессия лётного сигнала на истину (серия G3)
дала 0.85 / 1.04 / 1.32 / 1.01 на спокойных прогонах и 2.13 на дёрганом — сходится.

ЧТО СРАВНИВАЕТСЯ (три способа взять угол на кадр, кадры и код одни и те же):
    hold   — последнее сообщение с t ≤ штампа кадра. ЭТО ПРЕЖНЕЕ ПОВЕДЕНИЕ (extrap=False).
    extrap — hold + гироскоп·Δt (правка: `att_extrap` в config.py).
    near   — ближайшее по времени сообщение. НЕ реализуемо в полёте (заглядывает вперёд),
             нужно как ПОТОЛОК: показывает, сколько вообще можно выиграть синхронизацией.

МЕРКА — не «стало красивее», а РЕГРЕССИЯ измеренной скорости на истину:
    v_изм = a·v_ист + b·ωx + c·ωy + d
`a` — масштаб канала (цель 1.0), `b` — та самая утечка крена в М НА РАД/С (цель 0),
R² — сколько дисперсии объяснено. Отдельной колонкой — СДВИГ (среднее v_изм − v_ист):
для демпфера СКОРОСТИ он опаснее всего остального, потому что зануление смещённого
измерения = постоянная реальная скорость борта, а R² и СКО ошибки его не видят.
Правка засчитана, если |b| упал, `a` не уехал И сдвиг не вырос.

⚠️ Нужен бэг С КАДРАМИ (`STRIP=0` в hover_series.sh).
⚠️ Высота — ИЗ ОДОМЕТРИИ, а не из баро: у `rel_alt` в бэге wall-штампы против sim-штампов
кадров, при RTF≈0.07 интерполяция упирается в край (правило 11 в tune.md).

Запуск (в контейнере nav — нужен cv2 и control_pkg):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /root/sim_ws/install/setup.bash;
    PYTHONPATH=/root/sim_ws/src/control:$PYTHONPATH \
    AE_BAG=/root/sim_ws/output/I1s1_bag python3 /lab/att_extrap_test.py'
⚠️ `PYTHONPATH` на ИСХОДНИКИ, а не на install: colcon КОПИРУЕТ ament_python-пакет при
сборке, и без этого стенд молча возьмёт версию кода на момент последнего colcon build —
то есть проверит не то, что лежит в дереве. Ровно та ошибка, ради которой стенд вообще
переписан на боевой код.
Env: AE_BAG, AE_MAXF, AE_EXTRAP_MAX (потолок дотяжки, 0.2).
"""
import math
import os
import sys

import numpy as np

import cv2
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image, Imu

from control_pkg.infrastructure.ros_perception import attitude_at
from control_pkg.perception.flow_estimator import FlowEstimator

BAG = os.environ.get('AE_BAG', '/root/sim_ws/output/I1s1_bag')
MAXF = int(os.environ.get('AE_MAXF', 0))
EXTRAP_MAX = float(os.environ.get('AE_EXTRAP_MAX', 0.2))
# Что перебираем. МОДЕЛЬ проекции земли в кадр (`ipm_model` в FlowEstimator) — это
# ГЕОМЕТРИЯ, СПОСОБ взять угол на кадр — это ВРЕМЯ. Первый прогон стенда показал, что
# время ни при чём (даже недостижимый `near` не убрал утечку), поэтому по умолчанию
# свипаем модели при одном способе `hold`, а полный крест зовётся через env.
MODELS = os.environ.get('AE_MODELS', 'legacy,signs,exact').split(',')
MODES = os.environ.get('AE_MODES', 'hold').split(',')
# Вычитание разворота по гироскопу: 0 = выкл, ±1 = знак. Знак НЕ выводится, а выбирается
# этим стендом — см. `ipm_derot` в FlowEstimator._ipm_update.
DEROTS = [float(x) for x in os.environ.get('AE_DEROT', '0').split(',')]
# ФВЧ по ω_z: постоянная времени в секундах, 0 = не снимать смещение нуля (см. `ipm_wz_tau`
# в FlowEstimator). Смысл колонки — СДВИГ: вычитание разворота по СЫРОМУ гироскопу вливает
# в боковую скорость постоянную составляющую ω_z × плечо полосы 4.5 м.
WZTAUS = [float(x) for x in os.environ.get('AE_WZTAU', '0').split(',')]
# ОТКУДА БРАТЬ ω_z ДЛЯ ВЫЧИТАНИЯ РАЗВОРОТА. У гироскопа есть смещение нуля (своё у каждого
# прогона), и вычитание умножает его на плечо полосы 4.5 м. Три источника:
#   gyro — как есть (плюс ФВЧ ipm_wz_tau внутри оценщика, если он включён);
#   att  — скорость по КВАТЕРНИОНУ MAVROS (dψ/dt): нуля не имеет вовсе, но идёт 12.5 Гц
#          против 30 Гц кадров, то есть ступенчатая и запаздывает;
#   comp — гироскоп МИНУС оценка нуля, а ноль оценивается по разнице (гиро − кватернион):
#          в разности настоящего разворота нет, остаётся только смещение датчика.
#          Быстрая часть берётся у гироскопа, медленная — у кватерниона.
# ⚠️ `comp` считается ЗДЕСЬ и подаётся готовым (ipm_wz_tau=0), чтобы сравнить источники,
# не трогая лётный код посреди серии.
WZSRC = os.environ.get('AE_WZSRC', 'gyro').split(',')
HOVER_Z = 2.0
# ровно то, что кладёт в оценщик bootstrap_node (FLOW_R) и RosPerception (интринсики)
CAM_W, CAM_H = 960, 540
FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]


def euler(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    frames, od, imu = [], [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        # Кадры: живьём оценщик ест `/image_mono`, но в бэг пишется `/image_color`
        # (mono8 не записывается — см. capture_scene.sh). Обе картинки выходят из одного
        # дебайера, серый из bgr8 берём сами: cv_bridge тут не нужен и не зовётся,
        # чтобы не тащить в стенд его ABI-привязку к системному OpenCV.
        if topic in ('/image_mono', '/image_color'):
            m = deserialize_message(raw, Image)
            buf = np.frombuffer(m.data, dtype=np.uint8)
            if m.encoding in ('mono8', '8UC1'):
                frames.append((stamp(m), buf.reshape(m.height, m.width).copy()))
            elif m.encoding == 'bgr8':
                frames.append((stamp(m), cv2.cvtColor(buf.reshape(m.height, m.width, 3),
                                                      cv2.COLOR_BGR2GRAY)))
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((stamp(m), p.x, p.y, p.z) + euler(m.pose.pose.orientation))
        elif topic == '/mavros/imu/data':
            # ровно тот источник, что читает нода: и углы, и ω из ОДНОГО сообщения
            m = deserialize_message(raw, Imu)
            w = m.angular_velocity
            imu.append((stamp(m),) + euler(m.orientation) + (w.x, w.y, w.z))
    frames.sort(key=lambda f: f[0])
    return frames, np.array(od), np.array(imu)


def att_for(imu, t, mode):
    """Углы (тангаж, крен) на момент кадра. `hold`/`extrap` — БОЕВАЯ `attitude_at`."""
    if mode == 'near':
        i = int(np.argmin(np.abs(imu[:, 0] - t)))
        return imu[i, 2], imu[i, 1]
    j = np.searchsorted(imu[:, 0], t, side='right') - 1
    if j < 0:
        return imu[0, 2], imu[0, 1]
    # буфер гироскопа ноды — тот же поток; отдаём хвост, как он лежит в `_gyro_buf`
    lo = max(0, j - 119)
    buf = [(row[0], row[4], row[5], row[6]) for row in imu[lo:j + 1]]
    return attitude_at(imu[j, 2], imu[j, 1], imu[j, 0], buf, t,
                       extrap=(mode == 'extrap'), extrap_max=EXTRAP_MAX)


def omega_for(imu, t, t_prev):
    """ω как её берёт нода: СРЕДНЯЯ за межкадровый интервал, иначе ближайшая."""
    if t_prev is not None:
        sel = (imu[:, 0] > t_prev) & (imu[:, 0] <= t)
        if sel.sum():
            return float(imu[sel, 6].mean())
    return float(imu[int(np.argmin(np.abs(imu[:, 0] - t))), 6])


def wz_series(frames, imu, src, tau):
    """ω_z на каждый кадр по выбранному источнику (см. WZSRC)."""
    wa = np.gradient(np.unwrap(imu[:, 3]), imu[:, 0])       # разворот по кватерниону
    raw, t_prev = [], None
    for t, _ in frames:
        if t_prev is not None:
            sel = (imu[:, 0] > t_prev) & (imu[:, 0] <= t)
            g = float(imu[sel, 6].mean()) if sel.sum() else float(
                imu[int(np.argmin(np.abs(imu[:, 0] - t))), 6])
            a = float(wa[sel].mean()) if sel.sum() else float(
                wa[int(np.argmin(np.abs(imu[:, 0] - t)))])
        else:
            i = int(np.argmin(np.abs(imu[:, 0] - t)))
            g, a = float(imu[i, 6]), float(wa[i])
        raw.append((t, g, a))
        t_prev = t
    raw = np.array(raw)
    if src == 'att':
        return raw[:, 2]
    if src != 'comp':
        return raw[:, 1]
    # оценка нуля по разнице, тем же весом max(1−e^(−dt/τ), 1/n), СТАРТ С ОКНА:
    # до висения разность мусорная (полётник доводит курс по компасу — см. замер
    # «до вис: гиро−ATT» +0.044 против −0.006 в висении).
    out = np.empty(len(raw)); bias = None; n = 0; tp = None
    for i in range(len(raw)):
        d = raw[i, 1] - raw[i, 2]
        if bias is None:
            bias, tp, n = d, raw[i, 0], 1
        else:
            dt = raw[i, 0] - tp; tp = raw[i, 0]; n += 1
            if dt > 0:
                bias += max(1.0 - math.exp(-dt / max(tau, 1e-6)), 1.0 / n) * (d - bias)
        out[i] = raw[i, 1] - bias
    return out


def replay(frames, od, imu, mode, model='legacy', derot=0.0, wztau=0.0, wzsrc='gyro'):
    """Канал вида сверху БОЕВЫМ кодом. Возвращает (t, v_вперёд, v_вбок) по кадрам."""
    est = FlowEstimator(CAM_W / 2.0, CAM_W / 2.0, CAM_W / 2.0, CAM_H / 2.0, FLOW_R,
                        ipm_model=model, ipm_derot=derot,
                        ipm_wz_tau=(wztau if wzsrc == 'gyro' else 0.0))
    wzs = wz_series(frames, imu, wzsrc, wztau)
    # ⚠️ ПРОГРЕВ ОЦЕНКИ СМЕЩЕНИЯ ω_z. Стенд гоняет только окно висения, а в полёте нода
    # живёт с момента старта и к висению её ФВЧ уже сошёлся. Без прогрева стенд мерил бы
    # не поправку, а её переходный процесс (при τ=30 и окне 24 с — почти только его).
    # Кормим гироскопом всё, что в бэге ДО первого кадра окна — ровно как живьём.
    if wzsrc == 'gyro' and wztau > 0.0 and len(frames):
        for row in imu[imu[:, 0] < frames[0][0]]:
            est._wz_debias(float(row[0]), float(row[6]))
    ts, vf, vl, miss = [], [], [], 0
    for i, (t, gray) in enumerate(frames):
        pitch, roll = att_for(imu, t, mode)
        h = float(np.interp(t, od[:, 0], od[:, 3]))
        est._ipm_update(gray, t, max(h, 0.5), pitch, roll, float(wzs[i]))
        if est.ipm_ok:
            ts.append(t)
            vf.append(est.ipm_vfwd)
            vl.append(est.ipm_vlat)
        else:
            miss += 1
    return np.array(ts), np.array(vf), np.array(vl), miss


def main():
    frames, od, imu = read(BAG)
    print(f'бэг {BAG}: кадров {len(frames)}, одометрии {len(od)}, imu {len(imu)}')
    if not len(frames) or not len(od) or not len(imu):
        sys.exit('⚠️ нет кадров/одометрии/imu — нужен бэг со STRIP=0')
    hov = od[od[:, 3] > HOVER_Z]
    if len(hov) < 20:
        sys.exit('⚠️ висения в бэге нет')
    t0, t1 = hov[0, 0], hov[-1, 0]
    frames = [f for f in frames if t0 <= f[0] <= t1]
    if MAXF:
        frames = frames[:MAXF]
    if len(frames) < 40:
        sys.exit(f'⚠️ кадров в висении мало ({len(frames)})')
    dtm = float(np.median(np.diff([f[0] for f in frames])))
    dta = float(np.median(np.diff(imu[:, 0])))
    print(f'висение {t1 - t0:.1f} с, кадров {len(frames)} ({1 / dtm:.1f} Гц), '
          f'ATTITUDE {1 / dta:.1f} Гц → ступенька до {dta * 1000:.0f} мс')
    # ⚠️ КОЛЛИНЕАРНОСТЬ РЕГРЕССОРОВ. Утечка `b` (по ω_x) и колонка ω_z — коэффициенты ОДНОЙ
    # регрессии. Если угловые скорости в прогоне связаны, МНК растащит общий эффект между
    # ними произвольно, и «утечка ушла с крена на курс» окажется не физикой, а перекладкой.
    # Печатаем связь явно: |corr| выше ~0.5 — колонки читать порознь нельзя.
    warm = frames[0][0] - imu[0, 0]
    print(f'прогрев ФВЧ до окна: {warm:.1f} с; corr(ω_x, ω_z) = '
          f'{np.corrcoef(imu[:, 4], imu[:, 6])[0, 1]:+.2f}, '
          f'ω_z: среднее {imu[:, 6].mean():+.4f} рад/с, СКО {imu[:, 6].std():.4f}')

    # истина: скорость в СВЯЗАННОЙ системе
    dx, dy = np.diff(hov[:, 1]), np.diff(hov[:, 2])
    ym = hov[:-1, 6]
    fwd = np.concatenate([[0.0], np.cumsum(dx * np.cos(ym) + dy * np.sin(ym))])
    lat = np.concatenate([[0.0], np.cumsum(-dx * np.sin(ym) + dy * np.cos(ym))])
    vt_fwd_all = np.gradient(fwd, hov[:, 0])
    vt_lat_all = np.gradient(lat, hov[:, 0])

    # ⚠️ СМЕЩЕНИЕ (`сдвиг`) — ОТДЕЛЬНАЯ КОЛОНКА, и она важнее R². Демпфер СКОРОСТИ
    # зануляет то, что видит, поэтому постоянный сдвиг измерения = постоянная РЕАЛЬНАЯ
    # скорость борта: он уползает ровно, односторонне, с низкой дисперсией. При этом ни
    # R², ни СКО ошибки сдвига НЕ ВИДЯТ — они меряют корреляцию и разброс, а константа
    # уходит в свободный член регрессии. Именно так серия L2s ушла вбок −4.78 ± 4.48 м
    # (все пять прогонов в одну сторону) при отличном R²=0.90 на стенде.
    print(f"\n{'модель':7s} | {'derot':>5s} | {'τ ФВЧ':>5s} | {'ист':4s} | {'способ':7s} | {'ось':10s} | "
          f"{'ω_z':4s} | {'масштаб a':>9s} | {'утечка b, м':>11s} | {'ωz, м':>6s} | {'сдвиг':>6s} | "
          f"{'R²':>5s} | {'СКО ош':>6s}")
    for model in MODELS:
      for dr in DEROTS:
       for tau in WZTAUS:
        for src in WZSRC:
         for mode in MODES:
          head = f'{model:7s} | {dr:+5.0f} | {tau:5.1f} | {src:4s} | {mode:7s}'
          ts, vf, vl, miss = replay(frames, od, imu, mode, model, dr, tau, src)
          if len(ts) < 20:
              print(f'{head} | измерений мало ({len(ts)})')
              continue
          # ⚠️ СЛЕПАЯ ЗОНА СТЕНДА. Здесь печатается доля кадров, на которых канал НЕ дал
          # измерения (не выпрямилось или сорвался шаг потока). На спокойном бэге она около
          # нуля — и тогда стенд НЕ проверяет поведение при срывах. Ровно там пряталась
          # рассинхронизация опорного кадра и его времени (серия K2s): офлайн 0 срывов,
          # в полёте на трясущемся борту — вразнос. Доля заметно выше нуля = числа ниже
          # описывают лишь спокойный режим.
          print(f'{head} | без измерения {miss} кадров из '
                f'{len(frames)} ({100.0 * miss / max(1, len(frames)):.1f}%)')
          wx = np.interp(ts, imu[:, 0], imu[:, 4])
          wy = np.interp(ts, imu[:, 0], imu[:, 6])   # ωz: разворот — его вычитает ipm_derot
          for name, v, tr in (('продольная', vf, vt_fwd_all), ('боковая', vl, vt_lat_all)):
              vt = np.interp(ts, hov[:, 0], tr)
              ok = np.isfinite(v) & (np.abs(v) > 0)
              if ok.sum() < 20:
                  continue
              A = np.column_stack([vt[ok], wx[ok], wy[ok], np.ones(int(ok.sum()))])
              c, *_ = np.linalg.lstsq(A, v[ok], rcond=None)
              r2 = 1 - (v[ok] - A @ c).var() / v[ok].var()
              print(f'{head} | {name:10s} | {c[0]:+9.2f} | '
                    f'{c[1]:+11.2f} | {c[2]:+6.2f} | {np.mean(v[ok] - vt[ok]):+6.2f} | '
                    f'{r2:5.2f} | {np.std(v[ok] - vt[ok]):6.2f}')
    print('\nПравка засчитана, если |b| у extrap заметно меньше, чем у hold, а масштаб a '
          'остался около 1.0.\nnear — недостижимый потолок (заглядывает вперёд), мерка '
          'того, сколько вообще даёт синхронизация.')


if __name__ == '__main__':
    main()
