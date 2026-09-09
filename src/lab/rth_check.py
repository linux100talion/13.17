#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rth_check — ПРОМАХ ВОЗВРАТА ДОМОЙ: где взлетели, где сели и сколько соврал EKF.

Судит прогоны возврата: cmd/rth (RTL полётника), cmd/smart_rth (SMART_RTL) и
cmd/rth_track (НАШ возврат по треку в GUIDED). Три числа, которые
и есть ответ на вопрос «вернулся ли борт в точку взлёта»:

  ПРОМАХ ПО ИСТИНЕ      |точка касания − точка отрыва| в мире Gazebo — то, что
                        увидел бы человек на поле;
  ПРОМАХ ГЛАЗАМИ EKF    то же в координатах /mavros/local_position/pose. У RTL он
                        обязан быть ~0: полётник летит ровно к своей цифре home,
                        а не к физической точке. ⚠️ У возврата ПО ТРЕКУ (GUIDED)
                        сравнивать надо не с точкой ОТРЫВА, а с ДОМОМ ЛАТЧА
                        (rth_ready.py латчит его позже, уже в свежей раме) — здесь
                        считается от отрыва, поэтому число завышено на дрейф,
                        накопленный до латча;
  ДРЕЙФ EKF             |(ekf − ekf₀) − (truth − truth₀)| на момент касания. Это
                        и есть цена возврата: промах по истине ≈ дрейф (плюс
                        сдвиг рамы, если FrameAnchor перелатчился в полёте).

Кадры: EKF ('map') и истина Gazebo ('world') — оба ENU и оба с курсом на север
(yaw EKF ведёт компас), поэтому вычитаем без поворота, привязавшись к моменту
отрыва. Штампы в заголовках — sim-время (ноды в use_sim_time).

Запуск С ХОСТА (ROS jazzy):
    python3 src/lab/rth_check.py docker/sim/output/joystick/<RUN>
В контейнере nav (там ещё и режимы FCU из /mavros/state — mavros_msgs есть):
    docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash;
      python3 /lab/rth_check.py /root/sim_ws/output/joystick/<RUN>'
"""
import bisect
import math
import os
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

EKF_TOPIC = '/mavros/local_position/pose'
TRUTH_TOPIC = '/model/iris_cam/odometry'
STATE_TOPIC = '/mavros/state'
GROUND = 0.3          # м над точкой старта: «на земле»


def find_bag(arg):
    for cand in (arg, os.path.join(arg, 'bag')):
        if os.path.isfile(os.path.join(cand, 'metadata.yaml')):
            return cand
    if arg.endswith('.db3') and os.path.isfile(arg):
        return os.path.dirname(arg)
    sys.exit(f'bag не найден: ни {arg}/metadata.yaml, ни {arg}/bag/metadata.yaml')


def read(bag):
    try:                                   # mavros_msgs есть только в контейнере
        from mavros_msgs.msg import State
    except ImportError:
        State = None
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    ekf, truth, modes = [], [], []
    while r.has_next():
        topic, data, _t = r.read_next()
        if topic == EKF_TOPIC:
            m = deserialize_message(data, PoseStamped)
            p = m.pose.position
            ekf.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z))
        elif topic == TRUTH_TOPIC:
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            truth.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z))
        elif topic == STATE_TOPIC and State is not None:
            m = deserialize_message(data, State)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if not modes or modes[-1][1:] != (m.mode, m.armed):
                modes.append((t, m.mode, m.armed))
    return ekf, truth, modes


def decimate(pts, step):
    """Проредить трек по пути: точки не ближе step метров друг к другу."""
    out = []
    for x in pts:
        if not out or math.hypot(x[1] - out[-1][1], x[2] - out[-1][2]) >= step:
            out.append(x)
    return out


def track_repeat(truth, t_off, t_split, t_land):
    """НАСКОЛЬКО ВОЗВРАТ ПОВТОРИЛ СЛЕД: для каждой точки обратного плеча — расстояние
    до ближайшей точки плеча «туда». Медиана/90-й перцентиль/максимум. У RTL это
    просто «насколько прямая домой совпала с траекторией ухода» (обычно метры), у
    SMART_RTL — мера того, что борт реально размотал свой путь."""
    out = decimate([x for x in truth if t_off <= x[0] <= t_split], 0.1)
    ret = decimate([x for x in truth if t_split < x[0] <= t_land], 0.2)
    if len(out) < 3 or len(ret) < 3:
        return None
    ds = []
    for r in ret:
        ds.append(min(math.hypot(r[1] - o[1], r[2] - o[2]) for o in out))
    ds.sort()
    return ds[len(ds) // 2], ds[int(0.9 * (len(ds) - 1))], ds[-1], len(ret)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    bag = find_bag(sys.argv[1])
    ekf, truth, modes = read(bag)
    if not ekf or not truth:
        sys.exit(f'в bag нет {EKF_TOPIC} или {TRUTH_TOPIC} — не тот прогон?')
    tt = [x[0] for x in truth]

    def at(s):                                   # ближайший по времени сэмпл истины
        i = min(max(bisect.bisect_left(tt, s), 0), len(truth) - 1)
        return truth[i]

    z0 = sorted(x[3] for x in truth[:20])[10]
    air = [x for x in truth if x[3] - z0 > GROUND]
    if not air:
        sys.exit('борт не отрывался (истина не поднялась выше 0.3 м) — судить нечего')
    t_off, t_land = air[0][0], air[-1][0]
    flying_at_end = t_land >= truth[-1][0] - 1.0
    e0 = next((x for x in ekf if x[0] >= t_off), ekf[0])
    e1 = next((x for x in reversed(ekf) if x[0] <= t_land), ekf[-1])
    r0, r1 = at(e0[0]), at(e1[0])

    def d(a, b):
        return math.hypot(a[1] - b[1], a[2] - b[2])

    miss_truth = d(r1, r0)
    miss_ekf = d(e1, e0)
    drift = math.hypot((e1[1] - e0[1]) - (r1[1] - r0[1]), (e1[2] - e0[2]) - (r1[2] - r0[2]))
    path, prev, far = 0.0, None, (0.0, 0.0)
    for x in truth:
        if not (t_off <= x[0] <= t_land):
            continue
        if prev is not None:
            path += math.hypot(x[1] - prev[1], x[2] - prev[2])
        prev = x
        far = max(far, (d(x, r0), x[0] - t_off))

    print(f'bag: {bag}')
    print(f'отрыв   t={t_off - truth[0][0]:6.1f} с  истина=({r0[1]:+.2f} {r0[2]:+.2f})  '
          f'EKF=({e0[1]:+.2f} {e0[2]:+.2f})')
    print(f'дальше всего от дома: {far[0]:.1f} м (t={far[1]:.1f} с после отрыва)')
    print(f'касание t={t_land - truth[0][0]:6.1f} с  истина=({r1[1]:+.2f} {r1[2]:+.2f})  '
          f'EKF=({e1[1]:+.2f} {e1[2]:+.2f})')
    print(f'в воздухе {t_land - t_off:.1f} с, путь {path:.1f} м')
    print()
    print(f'ПРОМАХ ПО ИСТИНЕ    {miss_truth:6.2f} м   ← вернулся ли борт в точку взлёта')
    print(f'ПРОМАХ ГЛАЗАМИ EKF  {miss_ekf:6.2f} м   ← у RTL ~0: летел к своей цифре home')
    print(f'ДРЕЙФ EKF           {drift:6.2f} м   ← цена возврата (дрейф + перелатчи рамы)')
    if flying_at_end:
        print('⚠️ bag кончился, а борт ещё в воздухе — посадка не записана '
              '(RTH_TIMEOUT? запись остановили раньше?)')
    # СЛЕД: сравниваем обратное плечо с плечом «туда». Границу берём по фронту
    # RTL/SMART_RTL, если режимы разобраны (в контейнере), иначе — по самой дальней
    # точке (для сортии «ушёл — вернулся» это тот же момент).
    # 'CMODE(21)' — это SMART_RTL: имени MAVROS не знает и отдаёт номер
    # (см. control_pkg/domain/modes.py, разбор полёта 200909)
    # GUIDED — наш возврат ПО ТРЕКУ (шаг RthTrack, cmd/rth_track): режим ставим мы,
    # уставки шлём сами. Без него прогон 2026-09-09 отчитывался «РЕЖИМ ВОЗВРАТА НЕ
    # ВКЛЮЧАЛСЯ», хотя борт вернулся и сел.
    RTH_MODES = ('RTL', 'SMART_RTL', 'CMODE(21)', 'GUIDED')
    t_split = None
    for t, m, _a in modes:
        if m in RTH_MODES:
            t_split = t
            break
    if t_split is None:
        t_split = t_off + far[1]
    rep = track_repeat(truth, t_off, t_split, t_land)
    if rep:
        med, p90, mx, n = rep
        print(f'СЛЕД (возврат против пути «туда», {n} точек обратного плеча):')
        print(f'    медиана {med:.2f} м, 90% {p90:.2f} м, максимум {mx:.2f} м')
        print('    у SMART_RTL это мера «размотал свой путь», у RTL — насколько '
              'прямая домой совпала с уходом')
    if modes:
        print('\nрежимы FCU (из /mavros/state):')
        for t, m, armed in modes:
            print(f'  t={t - truth[0][0]:7.1f} с  {m:<10} armed={int(armed)}')
        rtl = [(t, m) for t, m, _a in modes if m in RTH_MODES]
        if rtl:
            print(f'{rtl[0][1]} включился через {rtl[0][0] - t_off:.1f} с после отрыва, '
                  f'вернул и посадил за {t_land - rtl[0][0]:.1f} с')
        else:
            print('РЕЖИМ ВОЗВРАТА НЕ ВКЛЮЧАЛСЯ — смотри лог ноды (RTH_REFUSED?) и '
                  'mavros.log («Unknown mode»?)')
    else:
        print('\n(режимы FCU не разобраны: mavros_msgs нет — запусти в контейнере nav)')


if __name__ == '__main__':
    main()
