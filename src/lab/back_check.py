#!/usr/bin/env python3
"""back_check — почему борт уезжает НАЗАД под демпфером.

Печатает совмещённый ряд: истинная скорость в ТЕЛЕ (из позиции gz, повёрнутой на
курс), сигнал перцепта (flow_lon/flow_lat) и команду (pitch_off/roll_off). Ищем,
что чему предшествует: перцепт врёт (bias) → команда → скорость, или наоборот.

Запуск ВНУТРИ nav:
  docker exec -e BC_BAG=/root/sim_ws/output/D1c_damper_hi_bag p1317_nav bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/back_check.py'
"""
import math
import os

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

G_ = 9.80665
BAG = os.environ.get('BC_BAG', '/root/sim_ws/output/D1c_damper_hi_bag')


def euler(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    od, d1, d2, d3 = [], [], [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            ro, pi, ya = euler(m.pose.pose.orientation)
            od.append((stamp(m), p.x, p.y, p.z, ro, pi, ya))
        elif topic == '/flow_dbg':
            m = deserialize_message(raw, Vector3Stamped)
            d1.append((stamp(m), m.vector.x, m.vector.y, m.vector.z))
        elif topic == '/flow_dbg2':
            m = deserialize_message(raw, Vector3Stamped)
            d2.append((stamp(m), m.vector.x, m.vector.y, m.vector.z))
        elif topic == '/flow_dbg3':
            # ОПОРНЫЙ канал: (kf_logs, kf_vel, kf_n) — положение и скорость от опоры
            # ⚠️ в бэгах до J1c слот y = kf_dx (сдвиг X в px), а не kf_vel
            m = deserialize_message(raw, Vector3Stamped)
            d3.append((stamp(m), m.vector.x, m.vector.y, m.vector.z))
    return np.array(od), np.array(d1), np.array(d2), np.array(d3)


od, d1, d2, d3 = read(BAG)
t0 = od[0, 0]
t = od[:, 0] - t0
x, y, z, roll, pitch, yaw = od[:, 1], od[:, 2], od[:, 3], od[:, 4], od[:, 5], od[:, 6]

# скорость из позиции (world), затем в тело по курсу
vx = np.gradient(x, t)
vy = np.gradient(y, t)
v_fwd = vx * np.cos(yaw) + vy * np.sin(yaw)
# ⚠️ Тело в ROS — FLU, поэтому эта проекция даёт ось ВЛЕВО, а не вправо. Весь день
# она была подписана как v_right, и на этом построен неверный вывод о знаке крена
# (см. config.roll_osign). Имя v_left — чтобы не наступить снова.
v_left = -vx * np.sin(yaw) + vy * np.cos(yaw)
v_right = -v_left


def at(series_t, series_v, tq):
    return np.interp(tq, series_t, series_v)


td1 = d1[:, 0] - t0
td2 = d2[:, 0] - t0
print(f'bag={BAG}  odom {len(od)}  flow_dbg {len(d1)}  flow_dbg2 {len(d2)}')
print(f'высота: старт {z[0]:.2f}  макс {z.max():.2f}')
print()
print('  t    z     v_fwd  v_right | flow_lon flow_lat conf | pitch_cmd roll_cmd | pitch° roll°')
for tq in np.arange(0, t[-1], 1.0):
    print(f'{tq:5.1f} {at(t,z,tq):5.2f} {at(t,v_fwd,tq):6.2f} {at(t,v_right,tq):7.2f} |'
          f' {at(td2,d2[:,2],tq):8.2f} {at(td1,d1[:,2],tq):8.2f} {at(td1,d1[:,3],tq):5.2f} |'
          f' {at(td2,d2[:,1],tq):9.1f} {at(td1,d1[:,1],tq):8.1f} |'
          f' {math.degrees(at(t,pitch,tq)):6.2f} {math.degrees(at(t,roll,tq)):6.2f}')

# --- ОПОРНЫЙ КАНАЛ (/flow_dbg3): положение от опорного кадра ---
if len(d3):
    td3 = d3[:, 0] - t0
    A0 = float(os.environ.get('BC_A0', 13)); A1 = float(os.environ.get('BC_A1', 21))
    w3 = (td3 >= A0) & (td3 <= A1)
    print(f'\nОПОРНЫЙ КАНАЛ, окно {A0:.0f}..{A1:.0f}с (n={w3.sum()}):')
    print('   t   kf_logs  kf_vel  точек | v_fwd  путь от начала окна, м | pitch_cmd')
    x0, y0 = np.interp(A0, t, x), np.interp(A0, t, y)
    for tq in np.arange(A0, A1 + 0.01, 1.0):
        dd = math.hypot(np.interp(tq, t, x) - x0, np.interp(tq, t, y) - y0)
        print(f'{tq:5.1f} {np.interp(tq,td3,d3[:,1]):+8.3f} {np.interp(tq,td3,d3[:,2])*1e3:+6.1f} '
              f'{np.interp(tq,td3,d3[:,3]):6.0f} | {np.interp(tq,t,v_fwd):+5.2f} {dd:20.1f} | '
              f'{np.interp(tq,td2,d2[:,1]):+8.1f}')
    if w3.sum() > 10:
        tt3 = td3[w3]
        dist3 = np.array([math.hypot(np.interp(v,t,x)-x0, np.interp(v,t,y)-y0) for v in tt3])
        kfl = d3[w3, 1]
        print(f'  corr(kf_logs, удаление от начала окна) = {np.corrcoef(kfl, dist3)[0,1]:+.3f}; '
              f'kf_logs: среднее {kfl.mean():+.3f}, размах {kfl.min():+.3f}..{kfl.max():+.3f}')
        print(f'  точек опоры: медиана {np.median(d3[w3,3]):.0f} '
              f'(падение до порога = пересев, точка удержания сменилась)')

# --- перцепт на месте: пока борт стоит/висит почти без скорости ---
still = np.abs(v_fwd) < 0.3
if still.sum() > 5:
    lon_still = np.interp(t[still], td2, d2[:, 2])
    lat_still = np.interp(t[still], td1, d1[:, 2])
    print(f'\nперцепт при |v_fwd|<0.3 м/с (n={still.sum()}): '
          f'flow_lon {lon_still.mean():+.2f}±{lon_still.std():.2f}  '
          f'flow_lat {lat_still.mean():+.2f}±{lat_still.std():.2f}')

# --- равновесие: на что демпфер садится ---
seg = t > (t[-1] - 12)
print(f'\nхвост 12с: v_fwd {v_fwd[seg].mean():+.2f}  v_right {v_right[seg].mean():+.2f}  '
      f'путь {np.hypot(x[-1]-x[0], y[-1]-y[0]):.1f} м  '
      f'смещение x {x[-1]-x[0]:+.1f}  y {y[-1]-y[0]:+.1f}')

# --- гипотеза: НАБОР ВЫСОТЫ читается перцептом как «полёт вперёд» ---
vz = np.gradient(z, t)
lon_i = np.interp(t, td2, d2[:, 2])
lat_i = np.interp(t, td1, d1[:, 2])
cli = (vz > 0.3) & (z > 0.5)
if cli.sum() > 5:
    print(f'\nНАБОР (vz>0.3, n={cli.sum()}, t={t[cli][0]:.1f}..{t[cli][-1]:.1f}с): '
          f'vz {vz[cli].mean():+.2f} м/с, истинная v_fwd {v_fwd[cli].mean():+.2f} м/с, '
          f'перцепт flow_lon {lon_i[cli].mean():+.3f} px')
    print(f'  ожидание перцепта по S_lon=0.28: {0.28*v_fwd[cli].mean():+.3f} px  → '
          f'лишку {lon_i[cli].mean()-0.28*v_fwd[cli].mean():+.3f} px'
          f' = фантомные {(lon_i[cli].mean()-0.28*v_fwd[cli].mean())/0.28:+.2f} м/с')
    ok = np.isfinite(vz) & np.isfinite(lon_i)
    seg = cli & ok
    print(f'  corr(vz, flow_lon) на наборе: {np.corrcoef(vz[seg], lon_i[seg])[0,1]:+.3f}; '
          f'на всём полёте (z>0.5): ', end='')
    air = (z > 0.5) & ok
    print(f'{np.corrcoef(vz[air], lon_i[air])[0,1]:+.3f}')
    print(f'  corr(v_fwd, flow_lon) в воздухе: {np.corrcoef(v_fwd[air], lon_i[air])[0,1]:+.3f}')

# --- разделяем вклады: flow_lon = a·v_fwd + b·vz + c (МНК в воздухе) ---
air = (z > 0.5) & np.isfinite(v_fwd) & np.isfinite(vz)
if air.sum() > 20:
    A = np.column_stack([v_fwd[air], vz[air], np.ones(air.sum())])
    coef, *_ = np.linalg.lstsq(A, lon_i[air], rcond=None)
    pred = A @ coef
    r2 = 1 - np.var(lon_i[air] - pred) / np.var(lon_i[air])
    print(f'\nМНК flow_lon = {coef[0]:+.3f}·v_fwd {coef[1]:+.3f}·vz {coef[2]:+.3f}  (R²={r2:.2f}, n={air.sum()})')
    print(f'  → «вперёд» чувствительность {coef[0]:+.3f} px/(м/с); '
          f'подъём подмешивает {coef[1]:+.3f} px/(м/с) = эквивалент '
          f'{coef[1]/coef[0] if coef[0] else float("nan"):+.2f} м/с ложного хода на 1 м/с набора')
    A2 = np.column_stack([v_right[air], np.ones(air.sum())])
    c2, *_ = np.linalg.lstsq(A2, lat_i[air], rcond=None)
    print(f'  для сравнения flow_lat = {c2[0]:+.3f}·v_right {c2[1]:+.3f}')

# --- смыкание: наклон за окно набора ↔ набранная скорость назад ---
W0, W1 = float(os.environ.get('BC_W0', 10)), float(os.environ.get('BC_W1', 15))
w = (t >= W0) & (t <= W1)
if w.sum() > 3:
    # pitch>0 = НОС ВНИЗ (см. calib_check.euler) → ускорение вперёд = +g·tan(pitch)
    a_fwd = G_ * np.tan(pitch[w])
    dv = np.trapz(a_fwd, t[w])
    print(f'\nокно {W0:.0f}..{W1:.0f}с: средний тангаж {math.degrees(pitch[w].mean()):+.2f}° '
          f'(>0 = нос вниз), команда pitch {np.interp(t[w], td2, d2[:,1]).mean():+.1f} PWM')
    print(f'  интеграл g·tan(тангаж) = {dv:+.2f} м/с;  фактически v_fwd '
          f'{v_fwd[w][0]:+.2f} → {v_fwd[w][-1]:+.2f} (Δ {v_fwd[w][-1]-v_fwd[w][0]:+.2f} м/с)')

# --- ФОРМА СИГНАЛА: полное разрешение в окне [BC_F0..BC_F1] ---
F0, F1 = float(os.environ.get('BC_F0', 14)), float(os.environ.get('BC_F1', 19))
m2 = (td2 >= F0) & (td2 <= F1)
if m2.sum():
    print(f'\nФОРМА КОМАНДЫ, окно {F0:.0f}..{F1:.0f}с (полное разрешение, n={m2.sum()}):')
    print('   t    flow_lon  pitch_cmd |  v_fwd  тангаж°')
    for i in np.where(m2)[0]:
        tq = td2[i]
        print(f'{tq:6.2f} {d2[i,2]:9.2f} {d2[i,1]:10.1f} | {np.interp(tq,t,v_fwd):6.2f} '
              f'{math.degrees(np.interp(tq,t,pitch)):7.2f}')
    cmd = d2[m2, 1]
    print(f'  команда: среднее {cmd.mean():+.1f}  СКО {cmd.std():.1f}  '
          f'размах {cmd.min():+.0f}..{cmd.max():+.0f}  смен знака {int((np.diff(np.sign(cmd))!=0).sum())}')
    th = np.interp(td2[m2], t, pitch)
    print(f'  тангаж: среднее {math.degrees(th.mean()):+.2f}° → ускорение вперёд '
          f'{G_*np.tan(th.mean()):+.2f} м/с²; нужно для гашения {np.interp(F0,t,v_fwd):.1f} м/с за '
          f'{F1-F0:.0f}с: {-np.interp(F0,t,v_fwd)/(F1-F0):+.2f} м/с²')

# --- УПИРАЕТСЯ ЛИ КОМАНДА В СЛЮ-ЛИМИТ (и что остаётся от постоянной составляющей) ---
SLEW = float(os.environ.get('BC_SLEW', 100))
m3 = (td2 >= F0) & (td2 <= F1) & (np.interp(td2, t, z) > 3.0)
if m3.sum() > 5:
    tt, cc = td2[m3], d2[m3, 1]
    d = np.diff(cc)
    dtc = np.diff(tt)
    cap = SLEW * dtc
    hit = np.abs(d) >= 0.9 * cap
    print(f'\nСЛЮ-ЛИМИТ (окно {F0:.0f}..{F1:.0f}с, только z>3, n={m3.sum()}):')
    print(f'  шаг команды: медиана |Δ| {np.median(np.abs(d)):.1f} PWM при потолке шага '
          f'{np.median(cap):.1f} → в лимит упирается {100*hit.mean():.0f}% выборок')
    print(f'  команда: постоянная составляющая {cc.mean():+.1f} PWM, переменная (СКО) {cc.std():.1f} PWM')
    thh = np.interp(tt, t, pitch)
    print(f'  тангаж: среднее {math.degrees(thh.mean()):+.2f}° (размах '
          f'{math.degrees(thh.min()):+.1f}..{math.degrees(thh.max()):+.1f}) → чистое ускорение '
          f'{G_*math.tan(thh.mean()):+.3f} м/с²')
    v0 = np.interp(tt[0], t, v_fwd)
    a = G_ * math.tan(thh.mean())
    print(f'  при таком ускорении гашение {abs(v0):.1f} м/с заняло бы '
          f'{abs(v0/a) if a else float("inf"):.0f} с')
    lon = d2[m3, 2]
    print(f'  перцепт: среднее {lon.mean():+.2f} px при истинной v_fwd {np.interp(tt,t,v_fwd).mean():+.2f} м/с '
          f'(по S_lon=0.28 ждали бы {0.28*np.interp(tt,t,v_fwd).mean():+.2f}), СКО {lon.std():.2f}')

# --- ЗАДЕРЖКИ КОНТУРА: взаимная корреляция по окну полёта ---
def lag_of(ta, a, tb, b, lo=-2.0, hi=2.0, step=0.02):
    """сдвиг τ, при котором b(t+τ) лучше всего повторяет a(t) (τ>0 = b ОТСТАЁТ)."""
    best, bl = -2, 0.0
    grid = np.arange(max(ta[0], tb[0]) + abs(lo), min(ta[-1], tb[-1]) - abs(hi), 0.05)
    av = np.interp(grid, ta, a)
    av = av - av.mean()
    for L in np.arange(lo, hi, step):
        bv = np.interp(grid + L, tb, b)
        bv = bv - bv.mean()
        den = np.std(av) * np.std(bv)
        c = float(np.mean(av * bv) / den) if den > 1e-9 else 0.0
        if c > best:
            best, bl = c, L
    return bl, best


L0, L1 = float(os.environ.get('BC_L0', 9)), float(os.environ.get('BC_L1', 18))
seg_o = (t >= L0) & (t <= L1)
seg_d = (td2 >= L0) & (td2 <= L1)
if seg_o.sum() > 20 and seg_d.sum() > 20:
    rate = seg_d.sum() / (td2[seg_d][-1] - td2[seg_d][0])
    print(f'\nЗАДЕРЖКИ ({L0:.0f}..{L1:.0f}с), темп перцепта {rate:.1f} Гц:')
    for nm, ta, a, tb, b in (
        ('v_fwd → flow_lon  (перцепт)', t[seg_o], v_fwd[seg_o], td2[seg_d], d2[seg_d, 2]),
        ('flow_lon → команда (контур)', td2[seg_d], d2[seg_d, 2], td2[seg_d], d2[seg_d, 1]),
        ('команда → тангаж   (борт)  ', td2[seg_d], d2[seg_d, 1], t[seg_o], -pitch[seg_o]),
        ('v_fwd → тангаж     (ВСЕГО) ', t[seg_o], v_fwd[seg_o], t[seg_o], -pitch[seg_o])):
        L, c = lag_of(ta, a, tb, b)
        print(f'  {nm}: {L:+.2f} с (corr {c:+.2f})')

# --- СТРОКА В СВОДКУ СВИПА (BC_CSV=файл, BC_NAME=имя прогона) ---
CSV = os.environ.get('BC_CSV', '')
if CSV:
    NAME = os.environ.get('BC_NAME') or os.path.basename(BAG.rstrip('/')).replace('_bag', '')
    mW = (td2 >= F0) & (td2 <= F1) & (np.interp(td2, t, z) > 3.0)
    cc = d2[mW, 1]
    ll = d2[mW, 2]
    thw = np.interp(td2[mW], t, pitch)
    vfw = np.interp(td2[mW], t, v_fwd)
    vrw = np.interp(td2[mW], t, v_right)
    dd = np.abs(np.diff(cc))
    capw = SLEW * np.diff(td2[mW])
    hd = dict()
    for key, ta, a, tb, b in (
            ('perc', t[seg_o], v_fwd[seg_o], td2[seg_d], d2[seg_d, 2]),
            ('loop', td2[seg_d], d2[seg_d, 2], td2[seg_d], d2[seg_d, 1]),
            ('air', td2[seg_d], d2[seg_d, 1], t[seg_o], -pitch[seg_o]),
            ('tot', t[seg_o], v_fwd[seg_o], t[seg_o], -pitch[seg_o])):
        hd[key] = lag_of(ta, a, tb, b)[0]
    head = ('name,rate_hz,lag_perc,lag_loop,lag_air,lag_tot,slew_hit_pct,cmd_dc,cmd_ac,'
            'lon_mean,lon_sd,vfwd_mean,vabs_mean,vabs_max,tilt_deg\n')
    new = (not os.path.exists(CSV)) or os.path.getsize(CSV) == 0
    with open(CSV, 'a') as f:
        if new:
            f.write(head)
        f.write(f'{NAME},{seg_d.sum()/(td2[seg_d][-1]-td2[seg_d][0]):.1f},'
                f'{hd["perc"]:+.2f},{hd["loop"]:+.2f},{hd["air"]:+.2f},{hd["tot"]:+.2f},'
                f'{100*np.mean(dd >= 0.9*capw):.0f},{cc.mean():+.1f},{cc.std():.1f},'
                f'{ll.mean():+.2f},{ll.std():.2f},{vfw.mean():+.2f},'
                f'{np.hypot(vfw,vrw).mean():.2f},{np.hypot(vfw,vrw).max():.2f},'
                f'{math.degrees(thw.mean()):+.2f}\n')
    print(f'\nстрока дописана в {CSV}')
