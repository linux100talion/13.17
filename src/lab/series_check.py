#!/usr/bin/env python3
"""series_check — сводка по СЕРИИ одинаковых прогонов (src/lab/calib_series.sh).

Одиночный прогон говорит, что дрейф есть. Серия отвечает на другой вопрос:
ПОВТОРЯЕМ ли он. Это требование п.3 очереди (src/control/ToDo.md): разброс между
прогонами должен быть < 30%, иначе помеху не отличить от собственных коррекций
демпфера, и любую метрику придётся усреднять по нескольким прогонам.

По каждому прогону считает:
  рампа   — скорость ухода оценки горизонта (истина gz − AHRS), °/с. Главная метрика.
  err_ahrs / err_raw — DC ошибки гироскопа в воздухе: AHRS против сырого INS. Разделяет
            «врёт датчик» и «врёт оценщик» (см. факт 2d).
  v_кон   — модуль горизонтальной скорости в конце полёта, м/с, и её курс.
  тангаж₀ — ИСТИННЫЙ тангаж перед отрывом. Меняется от прогона к прогону из-за оседания
            на стойках при спавне; если снос коррелирует с ним, определяет старт, а не полёт.

Запуск ВНУТРИ nav:
    docker exec -e BAGS="lift_stat1_bag lift_stat2_bag" p1317_nav bash -lc \
      'source /opt/ros/humble/setup.bash; python3 /lab/series_check.py'
"""
import math
import os

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

OUT = os.environ.get("SERIES_DIR", "/root/sim_ws/output")
TFIX = float(os.environ.get("TFIX", 20))   # отметка сравнения, сек от отрыва
D = np.degrees(1)


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def roll_of(q):
    return math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))


def pitch_of(q):
    return math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=f"{OUT}/{bag}", storage_id="sqlite3"),
           ConverterOptions("cdr", "cdr"))
    od, ah, rw = [], [], []
    while r.has_next():
        t, raw, _ = r.read_next()
        if t == "/model/iris_cam/odometry":
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((st(m), p.x, p.y, p.z, pitch_of(m.pose.pose.orientation),
                       yaw_of(m.pose.pose.orientation), m.twist.twist.angular.y,
                       roll_of(m.pose.pose.orientation)))
        elif t == "/mavros/imu/data":
            m = deserialize_message(raw, Imu)
            ah.append((st(m), m.angular_velocity.y, pitch_of(m.orientation),
                       roll_of(m.orientation)))
        elif t == "/mavros/imu/data_raw":
            m = deserialize_message(raw, Imu)
            rw.append((st(m), m.angular_velocity.y))
    return (np.array(od), np.array(ah),
            np.array(rw) if rw else np.zeros((0, 2)))


def metrics(bag):
    od, ah, rw = read(bag)
    if len(od) < 200 or len(ah) < 50:
        return None
    air = od[:, 3] > 1.0
    if air.sum() < 100:
        return None
    t_a = od[air, 0]
    # рампа: истина − оценка AHRS, наклон по времени. ⚠️ ПО ОБЕИМ ОСЯМ: помеха свободно
    # перетекает между креном и тангажом, и судить по одной оси — значит объявлять победу
    # там, где просто перетекло (так вышло с A4d: тангаж −9×, крен без изменений, путь тот же).
    tt = t_a - t_a[0]
    dp_series = (od[air, 4] - np.interp(t_a, ah[:, 0], ah[:, 2])) * D
    dr_series = (od[air, 7] - np.interp(t_a, ah[:, 0], ah[:, 3])) * D
    ramp_p = float(np.polyfit(tt, dp_series, 1)[0])
    ramp_r = float(np.polyfit(tt, dr_series, 1)[0])
    ramp = ramp_p
    t_lift = t_a[0]
    # DC ошибки гироскопа: отдельно НА ЗЕМЛЕ ДО ОТРЫВА и в воздухе.
    # Земля критична: там аппарат неподвижен, значит смещение НАБЛЮДАЕМО и EKF обязан
    # его выучить верно. Если ошибка велика уже на земле — фильтр не успел сойтись
    # до взлёта, и малый GBIAS_P_NSE заморозил неверное значение.
    def dc(arr, phase):
        if len(arr) < 50:
            return float("nan")
        ti = arr[:, 0]
        z = np.interp(ti, od[:, 0], od[:, 3])
        tr = np.interp(ti, od[:, 0], od[:, 6])
        m = (z > 1.0) if phase == "air" else ((z < 0.30) & (ti < t_lift))
        return float(((arr[m, 1] - tr[m]) * D).mean()) if m.sum() > 30 else float("nan")
    # ЧАСТОТА ТЕЛЕМЕТРИИ IMU, sim-Гц. ⚠️ Это НЕ вход оценщика: EKF ест IMU напрямую от
    # Gazebo по JSON в lockstep (физика 250 Гц), а сюда приходит то, что FCU ОТДАЁТ.
    # Держим в сводке потому, что величина ухода с ней коррелирует (факт 2e, corr −0.956) —
    # как ИНДИКАТОР загрузки бокса, а не как причина. Без неё нельзя понять, сравнимы ли
    # два прогона вообще.
    def hz(arr):
        if len(arr) < 50:
            return float("nan")
        m = (arr[:, 0] >= t_a[0]) & (arr[:, 0] <= t_a[-1])
        return float((m.sum() - 1) / (arr[m, 0][-1] - arr[m, 0][0])) if m.sum() > 20 else float("nan")
    # Δугол на ФИКСИРОВАННОЙ отметке TFIX от отрыва. Главный способ сравнивать прогоны:
    # метрика «рампа» (наклон фита по всему полёту) негодна, потому что кривая ухода
    # насыщается, а прогоны длятся по-разному — длинное окно даёт заниженный наклон
    # (факт 2e: разброс 46% по рампе против 28% по Δуглу на общей отметке).
    dpf = drf = pathf = float("nan")
    if tt[-1] >= TFIX:
        k = int(np.searchsorted(tt, TFIX))
        dpf = float(dp_series[k] - dp_series[0])
        drf = float(dr_series[k] - dr_series[0])
        pathf = float(np.hypot(od[air, 1][k] - od[air, 1][0], od[air, 2][k] - od[air, 2][0]))
    # скорость и курс сноса в конце
    x, y = od[air, 1], od[air, 2]
    vx, vy = np.gradient(x, t_a), np.gradient(y, t_a)
    v_end = float(np.hypot(vx[-3:].mean(), vy[-3:].mean()))
    course = math.degrees(math.atan2(vy[-3:].mean(), vx[-3:].mean()))
    path = float(np.hypot(x[-1] - x[0], y[-1] - y[0]))
    # истинный тангаж перед отрывом (последняя треть стоянки, без оседания)
    # ⚠️ только ДО отрыва: иначе окно захватывает посадку, а после жёсткой посадки
    #    борт лежит под большим углом и метрика превращается в мусор.
    gi = np.nonzero((od[:, 3] < 0.30) & (od[:, 0] < t_lift))[0]
    p0 = float(od[gi[len(gi) * 2 // 3:], 4].mean() * D) if len(gi) > 30 else float("nan")
    return dict(bag=bag, sec=float(t_a[-1] - t_a[0]), ramp=ramp,
                ramp_r=ramp_r, ramp_p=ramp_p, hz_ah=hz(ah), hz_rw=hz(rw),
                dp_f=dpf, dr_f=drf, path_f=pathf,
                ahrs=dc(ah, "air"), gnd=dc(ah, "gnd"), raw=dc(rw, "air"),
                v=v_end, course=course, path=path, p0=p0)


rows = [m for m in (metrics(b) for b in os.environ["BAGS"].split()) if m]
if not rows:
    print("нет пригодных bag'ов")
    raise SystemExit(1)

print(f"НА ОБЩЕЙ ОТМЕТКЕ t = {TFIX:.0f} с от отрыва (главное сравнение):")
print(f"{'прогон':<16}{'полёт':>7}{'Гц AHRS':>9}{'Гц сыр':>8}"
      f"{'Δкрен':>9}{'Δтангаж':>10}{'путь':>9}")
for r in rows:
    print(f"{r['bag'].replace('_bag', ''):<16}{r['sec']:>6.1f}с{r['hz_ah']:>9.1f}{r['hz_rw']:>8.1f}"
          f"{r['dr_f']:>8.2f}°{r['dp_f']:>9.2f}°{r['path_f']:>8.1f}м")
print(f"\nпо всему полёту (окна РАЗНЫЕ, для сравнения прогонов не годится):")
print(f"{'прогон':<16}{'рампа КРЕН':>12}{'рампа ТАНГАЖ':>14}{'ВОЗДУХ':>9}{'ПУТЬ':>9}{'v_кон':>9}")
for r in rows:
    print(f"{r['bag'].replace('_bag', ''):<16}{r['ramp_r']:>10.3f}°/с{r['ramp_p']:>12.3f}°/с"
          f"{r['ahrs']:>7.3f}°/с{r['path']:>8.1f}м{r['v']:>7.2f}м/с")
print("  ПУТЬ — главная метрика: одна ось может «улучшиться», пока уход утёк в другую.")
print("  Гц — телеметрия, НЕ вход EKF (тот ест IMU от Gazebo в lockstep). Индикатор загрузки.")
print("  ЗЕМЛЯ/ВОЗДУХ = ошибка гироскопа у AHRS до отрыва и в полёте (по тангажу).")

print(f"\n{'метрика':<14}{'среднее':>10}{'СКО':>9}{'разброс':>10}{'вердикт':>28}")
for key, lbl, unit in (("dp_f", f"Δтангаж@{TFIX:.0f}с", "°"), ("dr_f", f"Δкрен@{TFIX:.0f}с", "°"),
                       ("path_f", f"путь@{TFIX:.0f}с", "м"), ("path", "путь весь", "м"),
                       ("v", "v конечная", "м/с"),
                       ("ramp_r", "рампа крен", "°/с"), ("ramp_p", "рампа тангаж", "°/с"),
                       ("hz_ah", "частота IMU", "Гц")):
    a = np.array([r[key] for r in rows], dtype=float)
    a = a[~np.isnan(a)]
    if len(a) < 2:
        continue
    rel = a.std() / abs(a.mean()) if abs(a.mean()) > 1e-9 else float("inf")
    verd = "повторяемо (<30%)" if rel < 0.30 else "НЕ повторяемо (>30%)"
    print(f"{lbl:<14}{a.mean():>9.3f}{a.std():>9.3f}{rel * 100:>9.0f}%{verd:>28}")

# Корреляция «тангаж на земле ↔ снос» УБРАНА: спавн детерминированный, истинный тангаж
# перед отрывом по всем прогонам 0.01–0.08°, то есть одно значение в пределах шума
# измерения. Корреляция на таком диапазоне строится на шуме и вводит в заблуждение.
