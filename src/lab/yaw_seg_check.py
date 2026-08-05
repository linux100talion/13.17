#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yaw_seg_check — разбор СЕГМЕНТОВ разворота: сколько ЗАКАЗАЛИ и сколько ОТДАЛИ.

Отличие от yaw_check.py (тот про удержание курса в висении): здесь предмет —
токен `yaw_l30`/`yaw_r60`, то есть КОМАНДА на угол. Проверяем три вещи разом:

1. ЗНАК. Куда борт поехал относительно токена. Знаки: истинный курс — ENU
   (влево = +yaw), стик — c_yaw>0 = ВПРАВО. Значит `yaw_l30` обязан дать
   Δyaw ≈ +30°, `yaw_r60` — Δyaw ≈ −60°. Противоположный знак = инверсия в
   тракте команды, и никакие гейны разбирать смысла не имеет.
2. МАСШТАБ. |Δyaw| против заказанного — довернул / перевернул.
3. ОТКУДА ошибка масштаба: из ДАТЧИКА или из УТЕЧКИ накопителя. Считаем
   визуальный курс `head = ∫flow_yaw·dt` (как его считает DpYawHold) двумя
   способами — с утечкой leak и без — и переводим в градусы через S. Если
   без утечки head/S совпал с истиной, а с утечкой нет — виновата утечка,
   а не датчик.

Если в бэге есть **`/flow_dbg6`** (уставка курса, ошибка, PWM рыскания), счёт
идёт по НЕЙ, а не по офлайн-реконструкции: там настоящий покадровый `fdt`,
которого в `/flow_dbg2` не восстановить. Тогда же считается и КАЛИБРОВКА
`yaw_flow_scale`: контур гонит `head` к уставке, уставка в единицах сигнала
равна `S_конфиг · заказанные градусы`, значит
    S_истинный = S_конфиг · (заказано° / отдано°),
а заказ виден как размах уставки. Плюс проверка насыщения по третьему слоту
(|PWM| у потолка `yaw_max` = контур упёрся, гейны разбирать рано).

Сегменты ищем по движению уставки `/flow_dbg6`, а без неё — по ИСТИННОЙ
скорости рыскания (порог WZ_MIN).

Запускать В КОНТЕЙНЕРЕ (или в одноразовом из образа sim-nav):
  docker run --rm -v .../output:/out -v .../src/lab:/lab:ro sim-nav:latest \
    bash -lc 'source /opt/ros/humble/setup.bash; BAG=/out/Y2_kp10_bag python3 /lab/yaw_seg_check.py'

Env: BAG, ODOM_TOPIC (/model/iris_cam/odometry), FLOW_TOPIC (/flow_dbg2),
     HOLD_TOPIC (/flow_dbg6), S (0.253 px/кадр на °/с), LEAK (8.0 с),
     WZ_MIN (2.0 °/с), MIN_SEC (1.0), SETTLE (6.0 с добора после команды),
     YAW_MAX (150 PWM — потолок контура, для проверки насыщения).
"""
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

BAG = os.environ.get('BAG', '/out/scene_bag')
ODOM = os.environ.get('ODOM_TOPIC', '/model/iris_cam/odometry')
FLOW = os.environ.get('FLOW_TOPIC', '/flow_dbg2')
HOLD = os.environ.get('HOLD_TOPIC', '/flow_dbg6')
SETTLE = float(os.environ.get('SETTLE', '6.0'))
YAW_MAX = float(os.environ.get('YAW_MAX', '150'))
S = float(os.environ.get('S', '0.253'))          # px/кадр на °/с
LEAK = float(os.environ.get('LEAK', '8.0'))      # с; 0 = без утечки
WZ_MIN = float(os.environ.get('WZ_MIN', '2.0'))  # °/с — порог «идёт разворот»
MIN_SEC = float(os.environ.get('MIN_SEC', '1.0'))


def read(bag, topics):
    """{топик: [(t_сек, msg)]} — t по штампу заголовка (sim-время), не по записи."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {'nav_msgs/msg/Odometry': Odometry,
           'geometry_msgs/msg/Vector3Stamped': Vector3Stamped}
    out = {t: [] for t in topics}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in out:
            continue
        m = deserialize_message(data, cls[types[topic]])
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        out[topic].append((t, m))
    return out


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def segments(t, wz):
    """Интервалы, где |wz| держится выше порога дольше MIN_SEC."""
    hot = np.abs(wz) > WZ_MIN
    segs, i = [], 0
    while i < len(hot):
        if not hot[i]:
            i += 1
            continue
        j = i
        while j < len(hot) and hot[j]:
            j += 1
        if t[j - 1] - t[i] >= MIN_SEC:
            segs.append((i, j - 1))
        i = j
    return segs


def by_setpoint(ht, hsp):
    """Интервалы, где УСТАВКА едет, — это и есть командные сегменты токена.
    Уставка меняется только пока стик отклонён, поэтому границы точные (в отличие
    от порога по ω_z, который рвёт медленный доворот на клочья)."""
    mov = np.nonzero(np.abs(np.diff(hsp)) > 1e-9)[0]
    if not len(mov):
        return []
    # СКЛЕЙКА с выдержкой: уставка меняется только на НОВОМ КАДРЕ, а /flow_dbg6 шлётся
    # каждый тик ноды — между кадрами она стоит. Без выдержки один токен рассыпается на
    # десяток «сегментов» по числу кадров.
    segs, a = [], mov[0]
    for u, v in zip(mov, mov[1:]):
        if ht[v] - ht[u] > 0.5:
            segs.append((a, u + 1))
            a = v
    segs.append((a, mov[-1] + 1))
    return [(i, j) for i, j in segs if ht[j] - ht[i] >= 0.5]


def report_hold(t, yaw, ht, hsp, herr, hpwm):
    """Разбор по /flow_dbg6: заказ (размах уставки) против отдачи (истинный курс)."""
    segs = by_setpoint(ht, hsp)
    print(f'--- по уставке /flow_dbg6: командных сегментов {len(segs)} ---')
    if not segs:
        return
    rows = []
    for n, (i, j) in enumerate(segs, 1):
        order = (hsp[j] - hsp[i]) / S                    # заказано, °
        t0, t1 = ht[i], ht[j]
        t_end = min(t1 + SETTLE, ht[-1])
        y0 = float(np.interp(t0, t, yaw))
        y1 = float(np.interp(t1, t, yaw))
        y2 = float(np.interp(t_end, t, yaw))
        sel = (ht >= t0) & (ht <= t_end)
        pwm = np.abs(hpwm[sel]).max() if sel.sum() else float('nan')
        sat = 100.0 * float((np.abs(hpwm[sel]) >= YAW_MAX - 1).mean()) if sel.sum() else 0.0
        head_end = hsp[j] + herr[j]                      # накопленный визуальный курс
        rows.append((order, y2 - y0))
        print(f'  сегмент {n}: {t0:6.1f}..{t1:5.1f} c (+добор {SETTLE:g}c)')
        print(f'      ЗАКАЗАНО {order:+7.1f}°   ОТДАНО {y2 - y0:+7.1f}°  '
              f'(к концу команды {y1 - y0:+.1f}°, за добор {y2 - y1:+.1f}°)')
        print(f'      визуальный курс на конце команды {head_end / S:+7.1f}°, '
              f'ошибка контура {herr[j] / S:+6.1f}°')
        print(f'      PWM рыскания: пик {pwm:.0f} из {YAW_MAX:.0f}, '
              f'в насыщении {sat:.0f}% времени')
        # ТЕМП УСТАВКИ: заказ/(время команды) против номинала level·yaw_rate_full.
        # Проседает — значит интегратор уставки видит НЕ ВСЁ время сегмента (кадры,
        # пропущенные доменным тиком, в него не попали).
        print(f'      темп уставки {order / max(1e-3, t1 - t0):+6.2f} °/с '
              f'за {t1 - t0:.1f} c команды')
    good = [(o, g) for o, g in rows if abs(o) > 5.0 and abs(g) > 1.0]
    if good:
        print('  --- калибровка ---')
        for o, g in good:
            print(f'      заказ {o:+.1f}° → отдано {g:+.1f}°: '
                  f'{"ЗНАК ВЕРЕН" if o * g > 0 else "⚠ ЗНАК ЗЕРКАЛЕН"}, '
                  f'S_истинный = {S * o / g:+.4f}')
        sc = [S * o / g for o, g in good if o * g > 0]
        if sc:
            print(f'      S: конфиг {S:.4f} → замер {np.mean(sc):.4f} '
                  f'(±{np.std(sc):.4f} по {len(sc)} сегм.)')


def main():
    d = read(BAG, [ODOM, FLOW, HOLD])
    od, fl = d[ODOM], d[FLOW]
    if not od:
        print(f'нет {ODOM} в {BAG}')
        return
    t = np.array([x[0] for x in od])
    t -= t[0]
    yaw = np.degrees(np.unwrap([yaw_of(m.pose.pose.orientation) for _, m in od]))
    wz = np.degrees([m.twist.twist.angular.z for _, m in od])
    # сглаживание ω_z: сырой сигнал дрожит, порог по нему рвёт сегмент на клочья
    k = 15
    wzs = np.convolve(wz, np.ones(k) / k, mode='same')

    ft = np.array([x[0] for x in fl]) - od[0][0]
    fy = np.array([m.vector.z for _, m in fl])
    # /flow_dbg2 шлётся каждый ТИК ноды, а flow_yaw обновляется по КАДРУ: берём
    # только точки, где значение сменилось — это и есть покадровая частота
    if len(fy) > 1:
        keep = np.r_[True, np.diff(fy) != 0.0]
        ft, fy = ft[keep], fy[keep]

    print(f'=== {os.path.basename(BAG)} ===')
    print(f'кадров flow: {len(fy)}, длительность {t[-1]:.0f} c, '
          f'курс {yaw[0]:+.1f}° → {yaw[-1]:+.1f}°')

    hold = d.get(HOLD) or []
    if hold:
        ht = np.array([x[0] for x in hold]) - od[0][0]
        report_hold(t, yaw, ht,
                    np.array([m.vector.x for _, m in hold]),
                    np.array([m.vector.y for _, m in hold]),
                    np.array([m.vector.z for _, m in hold]))
    else:
        print(f'--- {HOLD} в бэге НЕТ: заказ восстановить нечем, только истина ---')

    segs = segments(t, wzs)
    print(f'найдено ротаций (по ω_z): {len(segs)}')
    for n, (i, j) in enumerate(segs, 1):
        d_yaw = yaw[j] - yaw[i]
        dur = t[j] - t[i]
        sel = (ft >= t[i]) & (ft <= t[j])
        head_raw = head_leak = 0.0
        prev = None
        s_pairs = []
        for tk, v in zip(ft[sel], fy[sel]):
            fdt = 0.05 if prev is None else max(1e-3, tk - prev)
            prev = tk
            head_raw += v * fdt
            head_leak += v * fdt
            if LEAK > 0:
                head_leak -= head_leak * min(1.0, fdt / LEAK)
            s_pairs.append((tk, v))
        wz_at = np.interp([p[0] for p in s_pairs], t, wz)
        vv = np.array([p[1] for p in s_pairs])
        corr = np.corrcoef(vv, wz_at)[0, 1] if len(vv) > 2 else float('nan')
        scale = float(np.dot(vv, wz_at) / np.dot(wz_at, wz_at)) if len(vv) > 2 else float('nan')
        print(f'  ротация {n}: {t[i]:6.1f}..{t[j]:6.1f} c ({dur:4.1f} c)  '
              f'Δyaw_истина {d_yaw:+7.1f}°')
        print(f'      визуальный курс: без утечки {head_raw / S:+7.1f}°, '
              f'с утечкой {LEAK:g}c {head_leak / S:+7.1f}°')
        print(f'      датчик в сегменте: S={scale:+.4f} px/кадр на °/с, corr={corr:+.3f}')

    # висения между ротациями: дрейф курса
    if segs:
        bounds = [(segs[a][1], segs[a + 1][0]) for a in range(len(segs) - 1)]
        bounds.append((segs[-1][1], len(t) - 1))
        for n, (i, j) in enumerate(bounds, 1):
            if t[j] - t[i] < 2.0:
                continue
            seg = yaw[i:j + 1]
            print(f'  висение {n}: {t[i]:6.1f}..{t[j]:6.1f} c  '
                  f'дрейф {seg[-1] - seg[0]:+6.1f}°, СКО {seg.std():5.2f}°, '
                  f'размах {seg.max() - seg.min():5.1f}°')
    print(f'  дрожь ω_z за весь полёт: СКО {wz.std():.2f} °/с')


if __name__ == '__main__':
    main()
