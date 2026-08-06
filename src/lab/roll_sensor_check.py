"""ПАСПОРТ ДАТЧИКА боковой оси (tune.md, фаза 2) — офлайн, без полётов.

Считает по бэгу с ЗАДАННЫМ боковым движением (`mv_right`/`mv_left` под gz-оракулом,
петля крена не участвует): видит ли `flow_lateral` истинную боковую скорость и с
какой крутизной. Ворота методики: corr > 0.8, крутизна стабильна ±20% по сегментам.

⚠️ КОНВЕНЦИЯ ОСИ: проекция −vx·sin(курс)+vy·cos(курс) — это ось ВЛЕВО (тело в ROS =
FLU), а не вправо. Борт идёт влево → мир в кадре смещается вправо → flow_lat > 0.
Тот же разбор, что в комментарии к `roll_osign` (config.py).

⚠️ ГОДЕН ТОЛЬКО НА БЭГАХ ПОСЛЕ ПРАВКИ МАСКИ (`be8688f`, 2026-07-29). Единственный
боковой проезд в архиве, `L1_scale2ax`, снят на коммите СТАРШЕ маски, и паспорт по
нему даёт corr +0.09 при крутизне 0.414 — это ровно задокументированный «до маски»
S_lat=0.4 против 2.42 после, то есть замер слепого канала, а не приговор оси. Живьём
крен держит борт при ветре 5-7 м/с. Перед использованием сверять `# commit` в .env
бэга с `be8688f`.

Запуск (контейнер nav не нужен поднятым):
  docker run --rm --network none -v /root/13.17/docker/sim/output:/out:ro \
    -v /root/13.17/src/lab:/lab:ro sim-nav:latest bash -lc \
    "source /opt/ros/humble/setup.bash; BAG=/out/L1_scale2ax_bag python3 /lab/roll_sensor_check.py"
"""
import math, os
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

BAG = os.environ.get('BAG', '/out/L1_scale2ax_bag')
MIN_V = float(os.environ.get('MIN_V', '0.3'))   # м/с: ниже — стоянка, корреляцию не мерим
st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, d1 = [], []
    while r.has_next():
        t, raw, ts = r.read_next()
        if t == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p, v = m.pose.pose.position, m.twist.twist.linear
            od.append((st(m), p.x, p.y, p.z, yw(m.pose.pose.orientation), v.x, v.y))
        elif t == '/flow_dbg':
            m = deserialize_message(raw, Vector3Stamped)
            d1.append((st(m), m.vector.x, m.vector.y, m.vector.z))
    return np.array(od), np.array(d1)


od, d1 = load(BAG)
print(f'бэг: {BAG}   одометрия {len(od)}   /flow_dbg {len(d1)}')
if not len(od) or not len(d1):
    raise SystemExit('нет данных')

# истинная скорость ВЛЕВО в теле; скорость берём производной позиции — twist в
# gz-одометрии бывает в другой раме, а разность позиций однозначна
t = d1[:, 0]
x = np.interp(t, od[:, 0], od[:, 1])
y = np.interp(t, od[:, 0], od[:, 2])
hd = np.interp(t, od[:, 0], np.unwrap(od[:, 4]))
dt = np.gradient(t)
vx, vy = np.gradient(x) / dt, np.gradient(y) / dt
v_left = -vx * np.sin(hd) + vy * np.cos(hd)
sig = d1[:, 2]                                   # flow_lateral
conf = d1[:, 3]

mov = np.abs(v_left) > MIN_V
print(f'кадров всего {len(t)}, в движении (|v|>{MIN_V} м/с) {mov.sum()} '
      f'({100*mov.mean():.0f}%), conf медиана {np.median(conf):.2f}')

r_all = np.corrcoef(sig, v_left)[0, 1]
r_mov = np.corrcoef(sig[mov], v_left[mov])[0, 1] if mov.sum() > 10 else float('nan')
slope = np.polyfit(v_left[mov], sig[mov], 1)[0] if mov.sum() > 10 else float('nan')
print(f'\ncorr(flow_lateral, v_влево): всё {r_all:+.3f} | только в движении {r_mov:+.3f}'
      f'   ← ворота tune.md > 0.8')
print(f'крутизна: {slope:+.3f} ед./(м/с)')

# стабильность крутизны по СЕГМЕНТАМ движения (каждый проезд отдельно)
edges = np.nonzero(np.diff(mov.astype(int)))[0]
segs, a = [], None
for e in edges:
    if mov[e + 1] and a is None:
        a = e + 1
    elif not mov[e + 1] and a is not None:
        if t[e] - t[a] > 1.0:
            segs.append((a, e))
        a = None
print(f'\nсегментов движения: {len(segs)}')
sl = []
for i, j in segs:
    if j - i < 10:
        continue
    s = np.polyfit(v_left[i:j], sig[i:j], 1)[0]
    r = np.corrcoef(sig[i:j], v_left[i:j])[0, 1]
    sl.append(s)
    print(f'  t+{t[i]-t[0]:5.1f}с  {t[j]-t[i]:4.1f}с  v_ср {np.mean(v_left[i:j]):+5.2f} м/с '
          f'| corr {r:+.3f} | крутизна {s:+.3f}')
if len(sl) > 1:
    sl = np.array(sl)
    print(f'\nкрутизна по сегментам: {np.mean(sl):+.3f} ± {np.std(sl):.3f} '
          f'({100*np.std(sl)/abs(np.mean(sl)):.0f}%)   ← ворота ±20%')
