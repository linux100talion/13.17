#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ВИД СВЕРХУ (IPM) → поток → МЕТРИЧЕСКАЯ СКОРОСТЬ. Проверка замены масштабному каналу.

Зачем. Нынешний продольный канал меряет log(масштаб) созвездия, и цена метра у него
плавает в 14 раз (замер по сериям E2-E6), потому что ГЛУБИНА точек не контролируется:
точка на земле в 5 м даёт при ходе 0.25 м целых 0.05 log, точка у горизонта в 50 м —
0.005. Смесь меняется от кадра к кадру, вместе с ней меняется крутизна, а иногда и знак.

Выпрямление снимает это целиком:
  • в виде сверху продольный ход — РАВНОМЕРНЫЙ СДВИГ, а не изменение масштаба;
  • масштаб берётся из БАРО: пиксель выпрямленной плоскости = известное число метров,
    то есть на выходе сразу МЕТРЫ, а не безразмерные единицы;
  • обе горизонтальные оси меряются одинаково;
  • болтанка высоты превращается из ЛОЖНОГО СМЕЩЕНИЯ в ошибку масштаба
    (±0.2 м на 3 м = 7%), что несравнимо безобиднее.

Геометрия у нас рабочая: камера наклонена вниз на 15°, вертикальный обзор 58.7°,
нижний край кадра смотрит под 44°, строка 360 (граница маски детектора) — под 26°.
При высоте 3 м это земля с 3.1 до 6.3 м перед бортом — небольшая, но настоящая полоса,
и ровно там, где детектор и так ищет точки.

Чего ждать плохого (проверяется этим же стендом):
  • сцена НЕ плоская: в mili_fortress есть стены и здания, всё, что выше земли, даёт
    параллакс. Лечится медианой по полосе — но если стен много, крутизна уплывёт;
  • скользящий угол: дальняя часть полосы растягивается и шумит, поэтому берём ближнюю;
  • высота из баро входит множителем: её ошибка идёт прямо в метры.

Как считаем. Четыре угла ЗЕМНОЙ полосы (X вперёд, Y вбок, от надира борта) проецируются
в кадр через интринсики и текущие углы (наклон камеры + тангаж/крен борта). Гомография
кадр→сетка выпрямляет полосу в метрический вид сверху; LK между соседними выпрямленными
кадрами даёт сдвиг в пикселях сетки, а он умножается на цену пикселя — и это метры.

ГЛАВНОЕ — МЕРИМ СКОРОСТЬ, А НЕ ПУТЬ. Задача пре-VINS демпфера не «вернуть борт в точку»,
а ОСТАНОВИТЬ его. Для остановки нужна скорость, а у скорости нет накопления — значит
физически не могут существовать отказы, которые съели всю кампанию E2-E7: неверно
засчитанный сегмент, стёртая память, квантование накопителя в ±0.03.
Подтверждение уже лежит в лётных данных: крен у нас ось ПО СКОРОСТИ, тангаж — ПО
ПОЛОЖЕНИЮ, и уход по осям (та же рама, те же кадры) расходится втрое-всемеро:
    серия   вперёд (положение)   вбок (скорость)
    E2      +20.2 ± 13.7 м       +5.8 ± 7.5 м
    E3       −3.0 ± 27.3         −0.9 ± 3.2
    E6      +17.0 ± 23.8         −2.5 ± 5.9
Скорость отвергли когда-то потому, что продольный поток мерился плохо (corr −0.22 с
истинной скоростью) — но мерился он там, где 93% точек сидят на линии горизонта и почти
не движутся. Выпрямление чинит ровно это.

Критерий: крутизна «намеренная скорость / истинная» около 1.0 и ОДИНАКОВАЯ от прогона к
прогону — в отличие от нынешних 14× у масштабного канала. Путь печатается вторым, как
контроль накопления.

⚠️ Нужен бэг С КАДРАМИ (`/image_color`).

Запуск (в контейнере nav — нужен cv_bridge):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash;
    IPM_BAG=/root/sim_ws/output/E7f1_bag python3 /lab/ipm_flow_test.py'
Env: IPM_BAG, IPM_XMIN/IPM_XMAX (полоса по ходу, м), IPM_YHALF (полуширина, м),
     IPM_RES (м на пиксель сетки), IPM_MAXF.
"""
import math
import os
import sys

import numpy as np

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float64

BAG = os.environ.get('IPM_BAG', '/root/sim_ws/output/E7f1_bag')
MAXF = int(os.environ.get('IPM_MAXF', 0))
X_MIN = float(os.environ.get('IPM_XMIN', 3.0))    # ближний край полосы, м перед бортом
X_MAX = float(os.environ.get('IPM_XMAX', 6.0))    # дальний край (дальше — растяжение и шум)
Y_HALF = float(os.environ.get('IPM_YHALF', 2.0))  # полуширина полосы, м
RES = float(os.environ.get('IPM_RES', 0.02))      # м на пиксель сетки
PSIGN = float(os.environ.get('IPM_PSIGN', 1.0))   # знак тангажа в наклоне (проверка соглашения)
CAM_W, CAM_H = 960, 540
FX = FY = CAM_W / 2.0
CX, CY = CAM_W / 2.0, CAM_H / 2.0
TILT = 0.26            # наклон камеры вниз, рад (как в боевом конфиге)
HOVER_Z = 2.0
LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
FEAT = dict(maxCorners=300, qualityLevel=0.01, minDistance=8, blockSize=7)
BACK_TOL = 1.0


def euler(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read(bag):
    br = CvBridge()
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    frames, od, imu, alt = [], [], [], []
    while r.has_next():
        topic, raw, ts = r.read_next()
        if topic == '/image_color':
            m = deserialize_message(raw, Image)
            frames.append((stamp(m), cv2.cvtColor(br.imgmsg_to_cv2(m, 'bgr8'),
                                                 cv2.COLOR_BGR2GRAY)))
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((stamp(m), p.x, p.y, p.z) + euler(m.pose.pose.orientation))
        elif topic == '/mavros/imu/data':
            m = deserialize_message(raw, Imu)
            imu.append((stamp(m),) + euler(m.orientation))
        elif topic == '/mavros/global_position/rel_alt':
            m = deserialize_message(raw, Float64)
            alt.append((ts * 1e-9, m.data))
    frames.sort(key=lambda f: f[0])
    return frames, np.array(od), np.array(imu), np.array(alt)


def ground_to_px(X, Y, h, a_down, roll):
    """Точка земли (X вперёд, Y вправо от надира, глубина h вниз) → пиксель кадра.

    Оси камеры: x вправо, y вниз, z вперёд по оптической оси. Камера смотрит вниз на
    угол a_down от горизонта, плюс крен. Точка в «горизонтной» системе (z вперёд
    горизонтально, y вниз): P = (Y, h, X). Поворачиваем её В систему камеры: наклон
    камеры вниз на a — это поворот системы вокруг оси x, значит точка поворачивается на
    −a; крен — поворот вокруг оси z.
    """
    ca, sa = math.cos(a_down), math.sin(a_down)
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    cr, sr = math.cos(roll), math.sin(roll)
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    P = Rz @ (Rx @ np.array([Y, h, X], dtype=np.float64))
    if P[2] <= 0.05:
        return None
    return np.array([CX + FX * P[0] / P[2], CY + FY * P[1] / P[2]])


def rectify(gray, h, pitch, roll):
    """Полоса земли → метрический вид сверху. None, если полоса не проецируется в кадр."""
    a = TILT + PSIGN * pitch              # суммарный наклон вниз камеры
    corners = [(X_MIN, -Y_HALF), (X_MIN, +Y_HALF), (X_MAX, +Y_HALF), (X_MAX, -Y_HALF)]
    src = []
    for X, Y in corners:
        p = ground_to_px(X, Y, h, a, roll)
        if p is None:
            return None, 0.0
        src.append(p)
    src = np.array(src, dtype=np.float32)
    if src[:, 0].min() < -CAM_W or src[:, 0].max() > 2 * CAM_W:
        return None, 0.0                  # полоса ушла далеко за кадр — доверять нечему
    w = int(2 * Y_HALF / RES)
    hh = int((X_MAX - X_MIN) / RES)
    dst = np.array([[0, hh - 1], [w - 1, hh - 1], [w - 1, 0], [0, 0]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(gray, M, (w, hh)), RES


def flow_step(prev, cur):
    """Медианный сдвиг между выпрямленными кадрами, в пикселях сетки. (dX, dY) или None."""
    pts = cv2.goodFeaturesToTrack(prev, mask=None, **FEAT)
    if pts is None or len(pts) < 20:
        return None
    p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, pts, None, **LK)
    p0b, st2, _ = cv2.calcOpticalFlowPyrLK(cur, prev, p1, None, **LK)
    ok = (st.reshape(-1) == 1) & (st2.reshape(-1) == 1)
    ok &= np.linalg.norm(p0b.reshape(-1, 2) - pts.reshape(-1, 2), axis=1) < BACK_TOL
    if ok.sum() < 15:
        return None
    d = (p1.reshape(-1, 2) - pts.reshape(-1, 2))[ok]
    return float(np.median(d[:, 0])), float(np.median(d[:, 1]))


def lsq_vel(ts, xs, win):
    """Скорость наклоном МНК в скользящем окне win секунд (как kf_vel в оценщике)."""
    out = np.full(len(ts), np.nan)
    for i in range(len(ts)):
        j = np.searchsorted(ts, ts[i] - win)
        if i - j + 1 < 4 or ts[i] - ts[j] < 0.5 * win:
            continue
        tc = ts[j:i + 1] - ts[j:i + 1].mean()
        xc = xs[j:i + 1] - xs[j:i + 1].mean()
        d = float(np.dot(tc, tc))
        if d > 0:
            out[i] = float(np.dot(tc, xc) / d)
    return out


def main():
    frames, od, imu, alt = read(BAG)
    print(f'бэг {BAG}: кадров {len(frames)}, одометрии {len(od)}, imu {len(imu)}, баро {len(alt)}')
    if not len(frames) or not len(od) or not len(imu):
        sys.exit('⚠️ нет кадров/одометрии/imu')
    h = od[od[:, 3] > HOVER_Z]
    t0, t1 = (h[0, 0], h[-1, 0]) if len(h) > 20 else (od[0, 0], od[-1, 0])
    frames = [f for f in frames if t0 <= f[0] <= t1]
    if MAXF:
        frames = frames[:MAXF]
    print(f'полоса земли {X_MIN}..{X_MAX} м, ±{Y_HALF} м вбок, {RES} м/пиксель, '
          f'кадров в висении {len(frames)}')

    # истина: продольный и боковой путь в СВЯЗАННОЙ системе, нарастающим итогом
    dx, dy = np.diff(h[:, 1]), np.diff(h[:, 2])
    ym = h[:-1, 6]
    fwd = np.concatenate([[0.0], np.cumsum(dx * np.cos(ym) + dy * np.sin(ym))])
    lat = np.concatenate([[0.0], np.cumsum(-dx * np.sin(ym) + dy * np.cos(ym))])

    prev_rect, prev_t = None, None
    ts, mx, my, skipped = [], [], [], 0
    sx = sy = 0.0
    for t, gray in frames:
        i = int(np.argmin(np.abs(imu[:, 0] - t)))
        # ⚠️ ВЫСОТУ БЕРЁМ ИЗ ОДОМЕТРИИ, А НЕ ИЗ БАРО. У `/mavros/global_position/rel_alt`
        # в бэге нет header.stamp, и время записи — WALL, тогда как кадры штампованы
        # SIM-часами. При RTF≈0.07 шкалы расходятся в 14 раз, интерполяция упирается в
        # край массива и подставляет высоту от начала прогона. Масштаб выпрямления
        # пропорционален высоте, поэтому ошибка идёт прямо в метры (занижение в 3-5 раз).
        # В ПОЛЁТЕ этой беды нет: нода читает живой топик. Это дефект офлайн-разбора.
        a_h = float(np.interp(t, od[:, 0], od[:, 3]))
        rect, res = rectify(gray, max(a_h, 0.5), imu[i, 2], imu[i, 1])
        if rect is None:
            skipped += 1
            prev_rect = None
            continue
        if prev_rect is not None and prev_rect.shape == rect.shape:
            f = flow_step(prev_rect, rect)
            if f is not None:
                # сетка: столбец = Y вправо, строка = X вперёд (строка 0 — дальний край).
                # Картинка едет ПРОТИВ движения борта, поэтому знак минус.
                # Знаки проверены замером по трём бэгам: продольная крутизна вышла
                # +1.00/+1.03/+0.97, боковая −0.91/−1.06/−1.09 — величина верная, знак
                # боковой был перевёрнут, поэтому здесь плюс, а не минус.
                sy += +f[0] * res
                sx += +f[1] * res
                ts.append(t)
                mx.append(sx)
                my.append(sy)
        prev_rect, prev_t = rect, t

    if len(ts) < 20:
        sys.exit(f'⚠️ измерений мало ({len(ts)}), пропущено кадров {skipped}')
    ts = np.array(ts)
    mx, my = np.array(mx), np.array(my)
    tf = np.interp(ts, h[:, 0], fwd)
    tl = np.interp(ts, h[:, 0], lat)
    print(f'измерений {len(ts)}, кадров пропущено {skipped}\n')
    # --- ГЛАВНОЕ: СКОРОСТЬ (её и будет отрабатывать демпфер) ---
    # Покадровая производная непригодна по арифметике: пиксель при 0.02 м/пкс и 30 Гц
    # даёт 0.6 м/с, а межкадровый сдвиг — единицы пикселей. Поэтому скорость берётся
    # НАКЛОНОМ МНК в скользящем окне — так же, как kf_vel в боевом оценщике.
    vt_x = np.interp(ts, h[:, 0], np.gradient(fwd, h[:, 0]))
    vt_y = np.interp(ts, h[:, 0], np.gradient(lat, h[:, 0]))
    print(f"{'окно, с':>7s} | {'ось':10s} | {'крутизна v':>10s} | {'corr v':>6s} | "
          f"{'СКО ошибки':>10s} | {'v_ист СКО':>9s}")
    for win in (float(w) for w in os.environ.get('IPM_WINS', '0.0,0.3,0.5,1.0,2.0').split(',')):
        for name, m, tr in (('продольная', mx, vt_x), ('боковая', my, vt_y)):
            v = np.gradient(m, ts) if win <= 0 else lsq_vel(ts, m, win)
            ok = np.isfinite(v)
            if ok.sum() < 20 or np.std(tr[ok]) < 0.05:
                continue
            k = np.polyfit(tr[ok], v[ok], 1)[0]
            print(f'{win:7.1f} | {name:10s} | {k:+10.2f} | '
                  f'{np.corrcoef(v[ok], tr[ok])[0, 1]:+6.2f} | '
                  f'{np.std(v[ok] - tr[ok]):10.2f} | {np.std(tr[ok]):9.2f}')
    print('крутизна 1.00 = скорость намерена верно; одинаковая по прогонам = масштаб стабилен\n')
    # --- контроль: тот же поток, проинтегрированный в путь ---
    print(f"{'ось':10s} | {'путь намерен':>12s} | {'путь истинный':>13s} | {'крутизна':>8s} | {'corr':>5s}")
    for name, m, tr in (('продольная', mx, tf), ('боковая', my, tl)):
        k = np.polyfit(tr, m, 1)[0] if np.ptp(tr) > 1.0 else float('nan')
        print(f'{name:10s} | {m[-1]:+12.1f} | {tr[-1]:+13.1f} | {k:+8.2f} | '
              f'{np.corrcoef(m, tr)[0, 1]:+5.2f}')


if __name__ == '__main__':
    main()
