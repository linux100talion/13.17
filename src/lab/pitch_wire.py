#!/usr/bin/env python3
"""ПРОВОДКА продольного контура: сигнал → уставка → ошибка → PWM → истинный уход.

Вопрос, ради которого скрипт есть. После правки «опора замирает» (ToDo5) память держится
(сохранённых сегментов 96-100%), а борт всё равно уезжает — E2s3 улетел на 29 м, имея
НОЛЬ выбросов и НОЛЬ пересевов, то есть на идеально чистом сигнале. Значит ломается не
измерение, а что-то дальше по проводке. Различить можно только глядя на все звенья разом
в одном времени:

  kf_logs   (/flow_dbg3.x) — накопленное ПОЛОЖЕНИЕ от точки удержания, log-единиц;
  сегм/перес(/flow_dbg4)   — зачтено сегментов / пересевов с выброшенным сегментом:
                             показывают, наполняется накопитель или обнуляется;
  уставка   (/flow_dbg5.x) — точка удержания холдера (при cmd_gain=0 обязана СТОЯТЬ);
  ошибка    (/flow_dbg5.y) — то, что контур на самом деле отрабатывает;
  PWM       (/flow_dbg2.x) — выход на борт (минус = нос вниз = вперёд);
  уход      (одометрия)    — истина в метрах.

Что различается по таблице:
  • kf_logs растёт, ошибка ≈ 0  → уставка ЕДЕТ ЗА БОРТОМ, контур не видит, что ушёл;
  • ошибка растёт, PWM ≈ 0      → гейн мал или выход зажат;
  • PWM большой и в ту же сторону, что уход → знак/фаза;
  • kf_logs ≈ 0 при реальном уходе → канал слеп (крутизна ушла в ноль).

Запуск:
  REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
  docker run --rm -v $REPO/src/lab:/lab:ro \
    -v $REPO/docker/sim/output:/out:ro ros:humble-ros-base bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/pitch_wire.py /out/E2s3_bag'
"""
import math
import sys

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
HOVER_Z = 2.0
ROWS = 12


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, d2, d3, d4, d5 = [], [], [], [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            od.append((st(m), p.x, p.y, p.z, yaw_of(m.pose.pose.orientation)))
        elif topic in ('/flow_dbg2', '/flow_dbg3', '/flow_dbg4', '/flow_dbg5'):
            m = deserialize_message(raw, Vector3Stamped)
            row = (st(m), m.vector.x, m.vector.y, m.vector.z)
            {'/flow_dbg2': d2, '/flow_dbg3': d3, '/flow_dbg4': d4,
             '/flow_dbg5': d5}[topic].append(row)
    f = lambda a: np.array(a) if a else None
    return f(od), f(d2), f(d3), f(d4), f(d5)


EARLY = 5.0          # с — «первые секунды удержания», где родится разгон


def hover_start(od, d4):
    """Момент входа в шаг висения: ПЕРВОЕ обнуление счётчика сегментов (climb → hover).

    Обнуляет счётчик только `reset_keyframe()`, а его зовёт вход в шаг миссии
    (`plan/step.py`). Второе обнуление — уже вход в посадку."""
    if d4 is None or len(d4) < 5:
        return None
    segs = d4[:, 1]
    drops = np.where((segs[1:] == 0) & (segs[:-1] > 0))[0]
    return float(d4[drops[0] + 1, 0]) if len(drops) else None


def at(arr, ts, col):
    """Значение колонки col в моменты ts (ближайший отсчёт), NaN если потока нет."""
    if arr is None or len(arr) == 0:
        return np.full(len(ts), np.nan)
    idx = np.clip(np.searchsorted(arr[:, 0], ts), 0, len(arr) - 1)
    return arr[idx, col]


def main(bags):
    for bag in bags:
        name = bag.rstrip('/').split('/')[-1].replace('_bag', '')
        od, d2, d3, d4, d5 = load(bag)
        if od is None:
            print(f"{name}: одометрии нет")
            continue
        h = od[od[:, 3] > HOVER_Z]
        if len(h) < 10:
            print(f"{name}: висения нет")
            continue
        t0, yaw0 = h[0, 0], h[0, 4]
        t = h[:, 0] - t0
        # ⚠️ Ось «вперёд» берём по ТЕКУЩЕМУ курсу, а не по курсу на входе в висение.
        # Опорный канал меряет движение вдоль оси КАМЕРЫ, то есть в связанной системе;
        # если борт рыскает, мировая ось «вперёд на старте» и ось камеры расходятся, и
        # корреляция сигнала с уходом падает (вплоть до смены знака) НЕ из-за сигнала.
        # Поэтому продольный путь считаем интегралом проекции скорости на нос борта.
        dx, dy = np.diff(h[:, 1]), np.diff(h[:, 2])
        yaw_m = h[:-1, 4]
        fwd = np.concatenate([[0.0], np.cumsum(dx * np.cos(yaw_m) + dy * np.sin(yaw_m))])
        # для сравнения — старая мировая ось (курс зафиксирован на входе)
        fwd_w = (h[:, 1] - h[0, 1]) * math.cos(yaw0) + (h[:, 2] - h[0, 2]) * math.sin(yaw0)
        yaw_span = math.degrees(h[:, 4].max() - h[:, 4].min())
        sel = np.linspace(0, len(t) - 1, ROWS).astype(int)
        ts = h[sel, 0]
        print(f"\n=== {name} · висение {t[-1]:.0f} с, продольный путь {fwd[-1]:+.1f} м "
              f"(в мировой оси {fwd_w[-1]:+.1f} м, размах курса {yaw_span:.0f}°) ===")
        print(f"{'t,с':>5s} | {'уход,м':>7s} | {'kf_logs':>8s} | {'уставка':>8s} | "
              f"{'ошибка':>8s} | {'PWM':>6s} | {'сегм':>4s} | {'перес':>5s}")
        for k, i in enumerate(sel):
            print(f"{t[i]:5.1f} | {fwd[i]:+7.1f} | {at(d3, ts, 1)[k]:+8.4f} | "
                  f"{at(d5, ts, 1)[k]:+8.4f} | {at(d5, ts, 2)[k]:+8.4f} | "
                  f"{at(d2, ts, 1)[k]:+6.0f} | {at(d4, ts, 1)[k]:4.0f} | "
                  f"{at(d4, ts, 2)[k]:5.0f}")
        # крутизна канала: сколько log-единиц на метр даёт сигнал НА ЭТОМ прогоне
        sig = at(d3, h[:, 0], 1)
        ok = np.isfinite(sig) & np.isfinite(fwd)
        if ok.sum() > 10 and fwd[ok].ptp() > 1.0:
            k, _ = np.polyfit(fwd[ok], sig[ok], 1)
            kw, _ = np.polyfit(fwd_w[ok], sig[ok], 1)
            print(f"крутизна канала: {k:+.4f} log/м (паспорт −0.0121), "
                  f"corr {np.corrcoef(fwd[ok], sig[ok])[0, 1]:+.2f}"
                  f"  | в мировой оси {kw:+.4f}, corr "
                  f"{np.corrcoef(fwd_w[ok], sig[ok])[0, 1]:+.2f}")
        # Разгон рождается в ПЕРВЫЕ секунды удержания: пока смещение мало, сигнал тонет
        # в шуме и может показать знак НАОБОРОТ — контур тогда сам толкает борт наружу
        # (E2s3: уход +2.9 м, kf_logs −0.063, PWM −107 = нос вниз = вперёд). Поэтому
        # крутизну считаем отдельно на старте удержания и на остатке.
        t_h = hover_start(od, d4)
        if t_h is not None:
            m = h[:, 0] >= t_h
            for tag, sub in (('старт удержания (первые %.0f с)' % EARLY,
                              m & (h[:, 0] < t_h + EARLY)),
                             ('дальше', m & (h[:, 0] >= t_h + EARLY))):
                g = sub & ok
                if g.sum() > 10 and fwd[g].ptp() > 0.5:
                    kk, _ = np.polyfit(fwd[g], sig[g], 1)
                    print(f"  {tag}: {kk:+.4f} log/м, corr "
                          f"{np.corrcoef(fwd[g], sig[g])[0, 1]:+.2f}, "
                          f"уход {fwd[g][0]:+.1f} → {fwd[g][-1]:+.1f} м")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
