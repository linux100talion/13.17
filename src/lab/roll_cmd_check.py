"""КАЛИБРОВКА БОКОВОЙ КОМАНДЫ (`DpRollHold`, режим `rate`) — что отдаёт стик.

Аналог `yaw_seg_check.py` для крена, но проще: у крена сигнал — СКОРОСТЬ, поэтому
уставки-накопителя нет и мерить надо не «заказанный угол против отданного», а
ЗАКАЗАННУЮ СКОРОСТЬ против ОТДАННОЙ.

Что считает по каждому командному сегменту (`mv_right`/`mv_left`):
  заказ  = c_right · roll_cmd_gain / S_lat   — целевая скорость в м/с, где S_lat —
           крутизна канала (ед. сигнала на м/с), замер паспорта датчика;
  отдача = истинная боковая скорость по одометрии Gazebo;
  и калибровка gain_нов = gain_стар · (заказ/отдача), как S для рыскания в Y4.

Сегменты берутся из `/flow_dbg7` (цель, ошибка, PWM) — записи САМОЙ команды. Резать
по истинной скорости нельзя: замер R1 нашёл так три «проезда» на две команды миссии
(откат борта после торможения выглядит как новая команда) и дал разброс гейна 26%.

Запуск:
  REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
  docker run --rm --network none -v $REPO/docker/sim/output:/out:ro \
    -v $REPO/src/lab:/lab:ro sim-nav:latest bash -lc \
    "source /opt/ros/humble/setup.bash; BAG=/out/R11_bag python3 /lab/roll_cmd_check.py"
"""
import math, os
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

BAG = os.environ.get('BAG', '/out/R11_bag')
GAIN = float(os.environ.get('GAIN', '10.0'))     # roll_cmd_gain прогона
LEVEL = float(os.environ.get('LEVEL', '0.3'))    # cfg.mv_level — уровень стика токена
S_LAT = float(os.environ.get('S_LAT', '2.42'))   # ед./(м/с), паспорт канала после маски
MIN_V = float(os.environ.get('MIN_V', '0.35'))   # порог «едем», м/с
st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
od, d1, d7 = [], [], []
while r.has_next():
    t, raw, ts = r.read_next()
    if t == '/model/iris_cam/odometry':
        m = deserialize_message(raw, Odometry)
        p = m.pose.pose.position
        od.append((st(m), p.x, p.y, p.z, yw(m.pose.pose.orientation)))
    elif t == '/flow_dbg':
        m = deserialize_message(raw, Vector3Stamped)
        d1.append((st(m), m.vector.x, m.vector.y, m.vector.z))
    elif t == '/flow_dbg7':
        m = deserialize_message(raw, Vector3Stamped)
        d7.append((st(m), m.vector.x, m.vector.y, m.vector.z))
od, d1, d7 = np.array(od), np.array(d1), np.array(d7)
print(f'бэг {BAG}: одометрия {len(od)}, /flow_dbg {len(d1)}, /flow_dbg7 {len(d7)}')
if not len(d7):
    raise SystemExit('НЕТ /flow_dbg7 — бэг снят до записи команды крена (коммит d614c5d). '
                     'Резать по истинной скорости нельзя: см. шапку.')

g = np.arange(od[0, 0], od[-1, 0], 0.05)
x, y = np.interp(g, od[:, 0], od[:, 1]), np.interp(g, od[:, 0], od[:, 2])
z = np.interp(g, od[:, 0], od[:, 3])
hd = np.interp(g, od[:, 0], np.unwrap(od[:, 4]))
vx, vy = np.gradient(x, 0.05), np.gradient(y, 0.05)
v_left = -vx * np.sin(hd) + vy * np.cos(hd)      # ось ВЛЕВО (FLU), см. roll_sensor_check
pwm = np.interp(g, d1[:, 0], d1[:, 1]) if len(d1) else np.zeros_like(g)
sig = np.interp(g, d1[:, 0], d1[:, 2]) if len(d1) else np.zeros_like(g)

# КОМАНДНЫЙ СЕГМЕНТ = участок, где цель контура отлична от нуля (/flow_dbg7.x).
# Это сама команда, а не её последствие: откат борта после торможения сюда не попадёт.
tgt = np.interp(g, d7[:, 0], d7[:, 1])
err7 = np.interp(g, d7[:, 0], d7[:, 2])
pwm7 = np.interp(g, d7[:, 0], d7[:, 3])
on = np.abs(tgt) > 1e-6
edges = np.nonzero(np.diff(on.astype(int)))[0]
segs, a = [], None
for e in edges:
    if on[e + 1] and a is None:
        a = e + 1
    elif not on[e + 1] and a is not None:
        if g[e] - g[a] > 1.0:
            segs.append((a, e))
        a = None
if a is not None and g[-1] - g[a] > 1.0:
    segs.append((a, len(g) - 1))

print(f'\nS_lat {S_LAT} ед./(м/с) — крутизна канала из паспорта датчика\n')
print(f'{"сегмент":>18} | {"длит":>5} | {"цель":>7} | {"заказ":>8} | {"отдача":>8} | '
      f'{"PWM пик":>7} | {"потолок":>7} | {"gain нов":>8}')
gains = []
for i, j in segs:
    vs = v_left[i:j]
    v_mean = float(np.mean(vs))
    tg = float(np.mean(tgt[i:j]))            # цель в ЕДИНИЦАХ СИГНАЛА
    want = tg / S_LAT                        # она же в м/с
    side = 'ВЛЕВО' if tg > 0 else 'ВПРАВО'
    p = pwm7[i:j]
    sat = 100.0 * np.mean(np.abs(p) >= 149)
    gn = GAIN * abs(want) / abs(v_mean) if abs(v_mean) > 1e-3 else float('nan')
    gains.append(gn)
    print(f'{side:>8} t+{g[i]-g[0]:6.1f}с | {g[j]-g[i]:4.1f}с | {tg:+7.2f} | {want:+7.2f}м/с | '
          f'{v_mean:+7.2f}м/с | {np.max(np.abs(p)):7.0f} | {sat:6.0f}% | {gn:8.1f}')
if len(gains) > 1:
    a_, s_ = np.mean(gains), np.std(gains)
    print(f'\nroll_cmd_gain по сегментам: {a_:.1f} ± {s_:.1f} ({100*s_/a_:.0f}%) '
          f'— против стоящих {GAIN}')
print(f'\nсигнал flow_lateral на сегментах: '
      + ', '.join(f'{np.mean(sig[i:j]):+.2f}' for i, j in segs))
