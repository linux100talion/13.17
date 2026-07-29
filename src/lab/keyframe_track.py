#!/usr/bin/env python3
"""keyframe_track — что даёт УДЕРЖАНИЕ фич вместо переоткрытия каждый кадр.

Повод. `flow_estimator.py:137` в конце каждого `process()` делает
`self.prev_pts = self._detect(gray)` — фичи детектируются заново на КАЖДОМ кадре,
КЛТ живёт одну пару. Значит: (1) точка не помнит, где была секунду назад → доступна
только скорость, положение недоступно в принципе; (2) созвездие точек стирается →
масштаб, в котором живёт продольная ось, не читается; (3) набор углов меняется от
кадра к кадру → медиана скачет сама по себе.

Скрипт считает по УЖЕ СНЯТОМУ bag'у два измерения на одних и тех же кадрах:
  БАЗА    — как сейчас: детект каждый кадр, медиана сдвига между соседними;
  ОПОРА   — детект ОДИН раз (опорный кадр), дальше только трекинг; из соответствий
            опорный→текущий вынимается подобие: сдвиг (dx, dy) и масштаб s.

Печатает: сколько кадров живут фичи, как ведёт себя log(s) против истинного удаления
от точки опоры (одометрия gz) и какой шум у обоих измерений на неподвижном борту.

Гипотеза, которую он проверяет: продольный ход сидит в МАСШТАБЕ, а не в сдвиге. На
висении сцена в ~19 м, отъезд на метр меняет масштаб на 5.3% — точка в 200 px от
центра уезжает на 10 px, тогда как тот же ход за кадр даёт доли пикселя.

Запуск ВНУТРИ nav (нужен cv_bridge из overlay):
  docker exec -e KT_BAG=/root/sim_ws/output/G1_r25_slew300_bag p1317_nav bash -lc \
    'source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash; \
     source /root/sim_ws/install/setup.bash; python3 /lab/keyframe_track.py'

KT_PERIOD=N — ПЕРИОДИЧЕСКИЙ пересев каждые N кадров с НАКОПЛЕНИЕМ: каждый сегмент
даёт свой Δlog s, полное смещение = сумма. Проверяет вариант «пересевать всё каждые
30 кадров»: точки не успевают состариться и уползти в дальний план (в H1 из-за этого
масштаб замер), но ошибка сегментов копится — одиночная опора не дрейфует, сумма
дрейфует как √N. 0 = одна опора на весь прогон (как раньше).

Env: KT_BAG, KT_T0 (сек от начала bag'а, когда ставить опору; по умолчанию — выход
     на потолок высоты), KT_MAXF (макс. кадров, 0=все), KT_MIN_PTS (пересев опоры).
"""
import math
import os

import numpy as np

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image

BAG = os.environ.get('KT_BAG', '/root/sim_ws/output/G1_r25_slew300_bag')
T0 = os.environ.get('KT_T0', '')
MAXF = int(os.environ.get('KT_MAXF', 0))
MIN_PTS = int(os.environ.get('KT_MIN_PTS', 40))

LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
FEAT = dict(maxCorners=200, qualityLevel=0.01, minDistance=8, blockSize=7)


def euler(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read(bag):
    """→ кадры [(t, gray)], одометрия [(t, x, y, z, roll, pitch, yaw)]."""
    br = CvBridge()
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    frames, od = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/image_color':
            m = deserialize_message(raw, Image)
            img = br.imgmsg_to_cv2(m, desired_encoding='bgr8')
            frames.append((stamp(m), cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)))
        elif topic == '/model/iris_cam/odometry':
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose.position
            ro, pi, ya = euler(m.pose.pose.orientation)
            od.append((stamp(m), p.x, p.y, p.z, ro, pi, ya))
    frames.sort(key=lambda f: f[0])
    return frames, np.array(od)


def track(prev_gray, gray, pts):
    """КЛТ вперёд + обратная проверка (отсев уехавших/переприлипших точек)."""
    nxt, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, pts, None, **LK)
    back, st2, _ = cv2.calcOpticalFlowPyrLK(gray, prev_gray, nxt, None, **LK)
    ok = (st.reshape(-1) == 1) & (st2.reshape(-1) == 1)
    ok &= np.linalg.norm(back.reshape(-1, 2) - pts.reshape(-1, 2), axis=1) < 1.0
    return nxt, ok


frames, od = read(BAG)
if MAXF:
    frames = frames[:MAXF]
t0_bag = od[0, 0]
tf = np.array([f[0] for f in frames]) - t0_bag
t = od[:, 0] - t0_bag
x, y, z = od[:, 1], od[:, 2], od[:, 3]
print(f'bag={BAG}\nкадров {len(frames)}, одометрия {len(od)}, '
      f'кадры {tf[0]:.1f}..{tf[-1]:.1f}с ({len(frames)/max(1e-6, tf[-1]-tf[0]):.1f} Гц)')

# --- где ставить опору: по умолчанию выход на потолок высоты ---
t_key = float(T0) if T0 else float(t[np.argmax(z > 0.9 * z.max())])
k = int(np.argmin(np.abs(tf - t_key)))
print(f'опорный кадр: #{k} на {tf[k]:.1f}с (высота {np.interp(tf[k], t, z):.2f} м)')

# ---------------------------------------------------------------- БАЗА
# Как сейчас: на каждом кадре детект заново, меряем медиану сдвига к следующему.
base_lat, base_lon, base_n = [], [], []
for i in range(k, len(frames) - 1):
    p0 = cv2.goodFeaturesToTrack(frames[i][1], mask=None, **FEAT)
    if p0 is None:
        continue
    nxt, ok = track(frames[i][1], frames[i + 1][1], p0)
    if ok.sum() < 8:
        continue
    d = (nxt.reshape(-1, 2) - p0.reshape(-1, 2))[ok]
    base_lat.append(float(np.median(d[:, 0])))
    base_lon.append(float(np.median(d[:, 1])))
    base_n.append(int(ok.sum()))

# ---------------------------------------------------------------- ОПОРА
# Детект ОДИН раз; дальше только трекинг. Пересев — когда точек стало мало.
PERIOD = int(os.environ.get('KT_PERIOD', 0))
acc = 0.0            # накопленный log s по завершённым сегментам
seg_rows = []        # (t_начала, t_конца, Δlog s) каждого сегмента — БЕЗ накопления
rows = []
pts = cv2.goodFeaturesToTrack(frames[k][1], mask=None, **FEAT)
ref_pts, ref_gray, ref_idx = pts.copy(), frames[k][1], k
prev_gray, cur = frames[k][1], pts.copy()
alive0 = len(pts)
life = np.zeros(len(pts))          # сколько кадров прожила каждая точка первого посева
alive_idx = np.arange(len(pts))    # исходные индексы ещё живых точек (длина = len(cur))
reseeds = 0
reseed_t = []
life_on = True                     # копим жизнь точек до первого пересева любого рода
for i in range(k + 1, len(frames)):
    nxt, ok = track(prev_gray, frames[i][1], cur)
    if life_on:                    # статистика жизни ТОЛЬКО для первого посева
        alive_idx = alive_idx[ok]
        life[alive_idx] += 1
    if PERIOD and (i - ref_idx) >= PERIOD and ok.sum() >= 8:
        life_on = False
        seg_start = ref_idx        # НАЧАЛО сегмента: ниже ref_idx уедет на текущий кадр
        # плановый пересев: фиксируем Δ сегмента в накопитель и сеем заново
        M0, _ = cv2.estimateAffinePartial2D(
            ref_pts.reshape(-1, 2)[ok], nxt.reshape(-1, 1, 2).reshape(-1, 2)[ok],
            method=cv2.RANSAC, ransacReprojThreshold=2.0)
        if M0 is not None:
            s_seg = math.hypot(M0[0, 0], M0[0, 1])
            if s_seg > 1e-6:
                acc += math.log(s_seg)
        new_pts = cv2.goodFeaturesToTrack(frames[i][1], mask=None, **FEAT)
        if new_pts is None or len(new_pts) < 8:
            break
        ref_pts, ref_idx = new_pts, i
        cur, prev_gray = new_pts.copy(), frames[i][1]
        seg_rows.append((tf[seg_start], tf[i], math.log(s_seg) if s_seg > 1e-6 else 0.0))
        rows.append((tf[i], np.nan, np.nan, math.exp(acc), int(ok.sum()), 0))
        continue
    if ok.sum() < MIN_PTS:
        # пересев: опора переносится на текущий кадр, накопленное подобие фиксируется
        reseeds += 1
        life_on = False
        new_pts = cv2.goodFeaturesToTrack(frames[i][1], mask=None, **FEAT)
        if new_pts is None or len(new_pts) < MIN_PTS:
            print(f'  ⚠️ на {tf[i]:.1f}с сцена не даёт углов '
                  f'({0 if new_pts is None else len(new_pts)}) — трекинг остановлен')
            break
        ref_pts, ref_gray, ref_idx = new_pts, frames[i][1], i
        cur, prev_gray = new_pts.copy(), frames[i][1]
        reseed_t.append(tf[i])
        rows.append((tf[i], np.nan, np.nan, np.nan, int(ok.sum()), 1))
        continue
    cur = nxt.reshape(-1, 1, 2)
    # подобие опорный→текущий: сдвиг + масштаб (поворот кадра поглощается моделью)
    M, inl = cv2.estimateAffinePartial2D(
        ref_pts.reshape(-1, 2)[ok], cur.reshape(-1, 2)[ok],
        method=cv2.RANSAC, ransacReprojThreshold=2.0)
    if M is None:
        prev_gray = frames[i][1]
        continue
    s = math.hypot(M[0, 0], M[0, 1]) * math.exp(acc)   # acc=0 → как было
    rows.append((tf[i], float(M[0, 2]), float(M[1, 2]), s, int(ok.sum()), 0))
    # точки, потерянные КЛТ, из опорного набора тоже убираем — соответствие 1:1
    ref_pts = ref_pts.reshape(-1, 2)[ok].reshape(-1, 1, 2)
    cur = cur.reshape(-1, 2)[ok].reshape(-1, 1, 2)
    prev_gray = frames[i][1]

R = np.array([r[:5] for r in rows], dtype=float)
print(f'\nЖИЗНЬ ФИЧ (первый посев {alive0} точек, без пересева):')
print(f'  медиана {np.median(life):.0f} кадров, среднее {life.mean():.1f}, '
      f'максимум {life.max():.0f}; доживших до 20 кадров {100*np.mean(life >= 20):.0f}%')
print(f'  пересевов за прогон: {reseeds} (порог {MIN_PTS} точек)'
      + (f', на {", ".join(f"{v:.1f}" for v in reseed_t)}с' if reseed_t else ''))
print(f'  ⇒ сейчас каждая фича живёт РОВНО 1 кадр (детект в конце process()), '
      f'хотя могла бы {np.median(life):.0f}')

# --- масштаб против истинного удаления ---
print(f'\n   t    n   dx_px   dy_px  масштаб  log s | удаление от опоры, м  дальность×')
kx, ky, kz = (np.interp(tf[k], t, v) for v in (x, y, z))
Zref = kz / math.sin(math.radians(15.0))       # дальность до земли по оси камеры
for row in R[::max(1, len(R) // 18)]:
    ti, dx, dy, s, n = row
    xi, yi = np.interp(ti, t, x), np.interp(ti, t, y)
    d = math.hypot(xi - kx, yi - ky)
    print(f'{ti:6.1f} {n:4.0f} {dx:7.1f} {dy:7.1f} {s:8.3f} {math.log(s) if s>0 else float("nan"):+7.3f} |'
          f' {d:12.1f} {(Zref + d)/Zref:15.2f}')

# Статистика — по ЧИСТОМУ окну: от опоры до первого пересева (дальше сцена
# рассыпается на посадке и мешает в кучу трекинг и удар о землю).
W1 = reseed_t[0] if reseed_t else R[-1, 0]
ok = np.isfinite(R[:, 3]) & (R[:, 3] > 0) & (R[:, 0] <= W1)
print(f'\nчистое окно: {R[0,0]:.1f}..{W1:.1f}с (до первого пересева)')
if ok.sum() > 10:
    ti = R[ok, 0]
    logs = np.log(R[ok, 3])
    dist = np.array([math.hypot(np.interp(v, t, x) - kx, np.interp(v, t, y) - ky) for v in ti])
    rng = (Zref + dist) / Zref
    c = np.corrcoef(logs, -np.log(rng))[0, 1]
    A = np.column_stack([-np.log(rng), np.ones(len(logs))])
    coef, *_ = np.linalg.lstsq(A, logs, rcond=None)
    print(f'\nМАСШТАБ ↔ ДАЛЬНОСТЬ: corr(log s, −log(дальность/Zref)) = {c:+.3f}, '
          f'наклон {coef[0]:+.2f} (идеал +1.00)')

# --- ЧЕСТНОЕ СРАВНЕНИЕ: насколько каждый сигнал предсказывает то, что должен ---
if ok.sum() > 10:
    ti = R[ok, 0]
    vfwd_t = np.gradient(x, t) * np.cos(od[:, 6]) + np.gradient(y, t) * np.sin(od[:, 6])
    vf = np.interp(ti, t, vfwd_t)
    dist_s = np.array([math.hypot(np.interp(v, t, x) - kx, np.interp(v, t, y) - ky) for v in ti])
    logs_w = np.log(R[ok, 3])
    # БАЗА: покадровая медиана вертикального сдвига — это то, что ест DpPitchHold
    tb = np.array([tf[i] for i in range(k, k + len(base_lon))])
    m = (tb >= ti[0]) & (tb <= ti[-1])
    if m.sum() > 10:
        vb = np.interp(tb[m], t, vfwd_t)
        cb = np.corrcoef(np.array(base_lon)[m], vb)[0, 1]
        print(f'\nЧТО С ЧЕМ СВЯЗАНО (окно {ti[0]:.1f}..{ti[-1]:.1f}с):')
        print(f'  БАЗА  — покадровая медиана сдвига ↔ ИСТИННАЯ СКОРОСТЬ: corr {cb:+.3f}')
    print(f'  ОПОРА — log(масштаб) ↔ ИСТИННОЕ УДАЛЕНИЕ:            corr '
          f'{np.corrcoef(logs_w, dist_s)[0,1]:+.3f}')
    A = np.column_stack([dist_s, np.ones(len(logs_w))])
    cf, *_ = np.linalg.lstsq(A, logs_w, rcond=None)
    print(f'    крутизна {cf[0]*100:+.2f}% масштаба на метр удаления, '
          f'монотонность: {100*np.mean(np.diff(logs_w) < 0):.0f}% шагов в одну сторону')

# --- ПОСЕГМЕНТНО (вариант «опора живёт 30 кадров»): Δ за сегмент = СКОРОСТЬ ---
# Тут ничего не копится: каждый сегмент меряет смещение за свою секунду. Это и есть
# сигнал для «перестать улетать» — база в PERIOD кадров вместо одного.
if seg_rows:
    S = np.array(seg_rows)
    d_true, v_true = [], []
    for a, b, _ in S:
        xa, ya = np.interp(a, t, x), np.interp(a, t, y)
        xb, yb = np.interp(b, t, x), np.interp(b, t, y)
        # знак: проекция на курс в начале сегмента (вперёд +)
        ya_ = np.interp(a, t, od[:, 6])
        d_true.append((xb - xa) * math.cos(ya_) + (yb - ya) * math.sin(ya_))
        v_true.append(d_true[-1] / max(1e-6, b - a))
    d_true = np.array(d_true); v_true = np.array(v_true)
    m = (S[:, 0] >= t_key) & (S[:, 1] <= float(os.environ.get('KT_W1', 1e9)))
    if m.sum() > 3:
        c = np.corrcoef(S[m, 2], d_true[m])[0, 1]
        A = np.column_stack([d_true[m], np.ones(m.sum())])
        cf2, *_ = np.linalg.lstsq(A, S[m, 2], rcond=None)
        resid = S[m, 2] - A @ cf2
        print(f'\nПОСЕГМЕНТНО (опора живёт {PERIOD} кадров, n={m.sum()} сегментов):')
        print(f'  corr(Δlog s, истинное смещение за сегмент) = {c:+.3f}')
        print(f'  крутизна {cf2[0]*100:+.2f}% на метр; остаточный шум {np.std(resid):.4f} '
              f'= {np.std(resid)/max(1e-9, abs(cf2[0])):.2f} м на сегмент '
              f'= {np.std(resid)/max(1e-9, abs(cf2[0]))/(S[m,1]-S[m,0]).mean():.2f} м/с')
        print(f'  истинная скорость по сегментам: {v_true[m].min():+.2f}..{v_true[m].max():+.2f} м/с')

# --- ВАРИАНТ C: СКОЛЬЗЯЩЕЕ ОКНО по ТЕМ ЖЕ точкам ---
# Опора не «сменяется раз в N кадров», а всегда отстоит на N кадров назад, и сравниваем
# по ОДНОМУ И ТОМУ ЖЕ набору точек. Меряет СКОРОСТЬ (смещение за окно), а не положение:
# копить нечего → дрейфа нет; состав точек внутри окна постоянен → коэффициент
# масштаб→метры не прыгает (у пересева-с-суммой он гулял 1.2…5.9 %/м).
SLIDE = int(os.environ.get('KT_SLIDE', 0))
if SLIDE:
    hist = []          # позиции живых точек на последних SLIDE+1 кадрах (список (M,2))
    cur_s = None
    prev_g = None
    out_t, out_dlog, out_n = [], [], []
    reseeds_s = 0
    for i in range(k, len(frames)):
        g = frames[i][1]
        if cur_s is None:
            p_ = cv2.goodFeaturesToTrack(g, mask=None, **FEAT)
            if p_ is None:
                continue
            cur_s = p_.reshape(-1, 2)
            hist = [cur_s.copy()]
            prev_g = g
            continue
        nxt, okk = track(prev_g, g, cur_s.reshape(-1, 1, 2).astype(np.float32))
        if okk.sum() < MIN_PTS:
            reseeds_s += 1                      # набор выбыл — сеем заново, окно копится с нуля
            p_ = cv2.goodFeaturesToTrack(g, mask=None, **FEAT)
            if p_ is None:
                break
            cur_s = p_.reshape(-1, 2)
            hist = [cur_s.copy()]
            prev_g = g
            continue
        cur_s = nxt.reshape(-1, 2)[okk]
        hist = [h[okk] for h in hist]           # историю чистим тем же фильтром — строго 1:1
        hist.append(cur_s.copy())
        if len(hist) > SLIDE + 1:
            hist.pop(0)
        prev_g = g
        if len(hist) == SLIDE + 1:
            M_, _ = cv2.estimateAffinePartial2D(hist[0], hist[-1], method=cv2.RANSAC,
                                                ransacReprojThreshold=2.0)
            if M_ is not None:
                sc = math.hypot(M_[0, 0], M_[0, 1])
                if sc > 1e-6:
                    out_t.append(tf[i]); out_dlog.append(math.log(sc)); out_n.append(len(cur_s))
    if len(out_t) > 10:
        ot = np.array(out_t); od_ = np.array(out_dlog)
        dt_win = ot - np.array([tf[max(k, i0)] for i0 in
                                range(k + SLIDE, k + SLIDE + len(ot))])[:len(ot)] * 0  # плейсхолдер
        # истинное смещение за окно (проекция на курс в НАЧАЛЕ окна)
        dtrue, vtrue = [], []
        for v in ot:
            v0 = v - SLIDE / 30.4
            xa, ya_ = np.interp(v0, t, x), np.interp(v0, t, y)
            xb, yb = np.interp(v, t, x), np.interp(v, t, y)
            yw0 = np.interp(v0, t, od[:, 6])
            dd = (xb - xa) * math.cos(yw0) + (yb - ya_) * math.sin(yw0)
            dtrue.append(dd); vtrue.append(dd / (SLIDE / 30.4))
        dtrue = np.array(dtrue); vtrue = np.array(vtrue)
        c = np.corrcoef(od_, dtrue)[0, 1]
        A = np.column_stack([dtrue, np.ones(len(od_))])
        cf3, *_ = np.linalg.lstsq(A, od_, rcond=None)
        resid = od_ - A @ cf3
        print(f'\nВАРИАНТ C — СКОЛЬЗЯЩЕЕ ОКНО {SLIDE} кадров (n={len(ot)} кадров, '
              f'пересевов {reseeds_s}, точек медиана {np.median(out_n):.0f}):')
        print(f'  corr(Δlog s за окно, истинное смещение за окно) = {c:+.3f}')
        print(f'  крутизна {cf3[0]*100:+.2f}% на метр; остаточный шум {np.std(resid):.4f} '
              f'= {np.std(resid)/max(1e-9, abs(cf3[0])):.2f} м за окно '
              f'= {np.std(resid)/max(1e-9, abs(cf3[0]))/(SLIDE/30.4):.2f} м/с')
        print(f'  истинная скорость в окне: {vtrue.min():+.2f}..{vtrue.max():+.2f} м/с')
        # ПОЛНАЯ МОДЕЛЬ: продольное смещение + боковое + высота. Масштаб меняется от
        # всего, что меняет дальность до сцены, поэтому меряем вклад каждого и смотрим,
        # что остаётся необъяснённым. Со стандартными ошибками — иначе не понять,
        # крутизна это число или пожелание.
        dlat = []
        for v in ot:
            v0 = v - SLIDE / 30.4
            xa_, ya2 = np.interp(v0, t, x), np.interp(v0, t, y)
            xb_, yb2 = np.interp(v, t, x), np.interp(v, t, y)
            yw0_ = np.interp(v0, t, od[:, 6])
            dlat.append(-(xb_ - xa_) * math.sin(yw0_) + (yb2 - ya2) * math.cos(yw0_))
        dlat = np.array(dlat)
        dzz = np.array([math.log(max(0.1, np.interp(v, t, z))
                                 / max(0.1, np.interp(v - SLIDE / 30.4, t, z))) for v in ot])
        # ТАНГАЖ. Камера наклонена вниз на 15°, борт для разгона кладётся на ±10° —
        # угол визирования гуляет 5°..25°, а дальность до земли ∝ 1/sin(угол) меняется
        # при этом ВТРОЕ. Для точек на земле это перебивает трансляцию целиком, поэтому
        # проверяем: сколько остатка объясняет изменение тангажа за окно.
        # pitch>0 = нос ВНИЗ → угол визирования РАСТЁТ → земля БЛИЖЕ → масштаб больше.
        CAM = math.radians(15.0)
        dpit = []
        for v in ot:
            v0 = v - SLIDE / 30.4
            th1 = np.interp(v, t, od[:, 5]); th0 = np.interp(v0, t, od[:, 5])
            # логарифм отношения дальностей по модели плоской земли
            dpit.append(math.log(max(1e-3, math.sin(CAM + th1)) / max(1e-3, math.sin(CAM + th0))))
        dpit = np.array(dpit)
        A4_ = np.column_stack([dtrue, dlat, dzz, dpit, np.ones(len(od_))])
        cf6, *_ = np.linalg.lstsq(A4_, od_, rcond=None)
        r4 = od_ - A4_ @ cf6
        dof4 = max(1, len(od_) - A4_.shape[1])
        se4 = np.sqrt(np.diag(np.linalg.pinv(A4_.T @ A4_)) * np.sum(r4 ** 2) / dof4)
        print(f'  + ТАНГАЖ (дальность ∝ 1/sin(15°+тангаж)), R² {1 - np.var(r4)/np.var(od_):.2f}:')
        print(f'    продольное {cf6[0]*100:+.2f} ± {se4[0]*100:.2f} %/м   '
              f'тангаж {cf6[3]:+.3f} ± {se4[3]:.3f} (модель даёт +1.00)')
        print(f'    остаток {np.std(r4):.4f} = '
              f'{np.std(r4)/max(1e-9, abs(cf6[0]))/(SLIDE/30.4):.2f} м/с')
        A3 = np.column_stack([dtrue, dlat, dzz, np.ones(len(od_))])
        cf5, *_ = np.linalg.lstsq(A3, od_, rcond=None)
        r3 = od_ - A3 @ cf5
        dof = max(1, len(od_) - A3.shape[1])
        cov = np.var(r3) * dof / dof * np.linalg.pinv(A3.T @ A3)
        se = np.sqrt(np.diag(cov) * np.var(r3) * len(od_) / dof) / np.sqrt(np.var(r3) * len(od_) / dof) * np.sqrt(np.diag(cov))
        se = np.sqrt(np.diag(np.linalg.pinv(A3.T @ A3)) * np.sum(r3 ** 2) / dof)
        print(f'  ПОЛНАЯ МОДЕЛЬ (продольное + боковое + высота), R² {1 - np.var(r3)/np.var(od_):.2f}:')
        print(f'    продольное {cf5[0]*100:+.2f} ± {se[0]*100:.2f} %/м   '
              f'боковое {cf5[1]*100:+.2f} ± {se[1]*100:.2f} %/м   '
              f'высота {cf5[2]:+.2f} ± {se[2]:.2f}')
        print(f'    остаток {np.std(r3):.4f} = '
              f'{np.std(r3)/max(1e-9, abs(cf5[0]))/(SLIDE/30.4):.2f} м/с')
        # ВЫСОТА. Дальность до земли ∝ высоте (камера наклонена вниз), значит подъём
        # тоже меняет масштаб: −10% высоты = +10% масштаба = 0.095 в логарифме. Это
        # сравнимо со всем наблюдаемым разбросом, поэтому проверяем двухфакторной МНК.
        dz = np.array([math.log(max(0.1, np.interp(v, t, z))
                                / max(0.1, np.interp(v - SLIDE / 30.4, t, z))) for v in ot])
        A2_ = np.column_stack([dtrue, dz, np.ones(len(od_))])
        cf4, *_ = np.linalg.lstsq(A2_, od_, rcond=None)
        r2 = od_ - A2_ @ cf4
        print(f'  С УЧЁТОМ ВЫСОТЫ: Δlog s = {cf4[0]*100:+.2f}%/м·смещение '
              f'{cf4[1]:+.2f}·Δlog(высота) {cf4[2]:+.4f}')
        print(f'    остаточный шум {np.std(r2):.4f} = '
              f'{np.std(r2)/max(1e-9, abs(cf4[0]))/(SLIDE/30.4):.2f} м/с '
              f'(было {np.std(resid)/max(1e-9, abs(cf3[0]))/(SLIDE/30.4):.2f}); '
              f'R² {1 - np.var(r2)/np.var(od_):.2f}')

# --- шум обоих измерений на «стоячем» участке ---
# ШУМ НА НЕПОДВИЖНОМ БОРТУ: до отрыва (борт стоит, картинка обязана быть постоянной).
g0, g1 = 3.0, 8.0
gi = [i for i in range(len(frames) - 1) if g0 <= tf[i] <= g1]
if len(gi) > 10:
    bl, bn = [], 0
    for i in gi:
        p0 = cv2.goodFeaturesToTrack(frames[i][1], mask=None, **FEAT)
        if p0 is None:
            continue
        nxt, okk = track(frames[i][1], frames[i + 1][1], p0)
        if okk.sum() < 8:
            continue
        d = (nxt.reshape(-1, 2) - p0.reshape(-1, 2))[okk]
        bl.append((float(np.median(d[:, 0])), float(np.median(d[:, 1])), int(okk.sum())))
    bl = np.array(bl)
    gpts = cv2.goodFeaturesToTrack(frames[gi[0]][1], mask=None, **FEAT)
    gcur, gprev, gs = gpts.copy(), frames[gi[0]][1], []
    gref = gpts.copy()
    for i in gi[1:]:
        nxt, okk = track(gprev, frames[i][1], gcur)
        if okk.sum() < 8:
            break
        gcur = nxt.reshape(-1, 2)[okk].reshape(-1, 1, 2)
        gref = gref.reshape(-1, 2)[okk].reshape(-1, 1, 2)
        M, _ = cv2.estimateAffinePartial2D(gref.reshape(-1, 2), gcur.reshape(-1, 2),
                                           method=cv2.RANSAC, ransacReprojThreshold=2.0)
        if M is not None:
            gs.append(math.log(max(1e-6, math.hypot(M[0, 0], M[0, 1]))))
        gprev = frames[i][1]
    print(f'\nШУМ НА СТОЯЩЕМ БОРТУ ({g0:.0f}..{g1:.0f}с, дрон не взлетел):')
    print(f'  БАЗА  — медиана покадрового сдвига: σ_x {bl[:,0].std():.2f} px, '
          f'σ_y {bl[:,1].std():.2f} px (точек {bl[:,2].mean():.0f})')
    if gs:
        print(f'  в метрах (по крутизне {abs(cf[0])*100:.2f}%/м): шум опоры '
              f'{np.std(gs)/max(1e-9, abs(cf[0])):.2f} м, '
              f'а шум базы {bl[:,1].std():.2f} px = '
              f'{bl[:,1].std()/0.28:.1f} м/с по S_lon=0.28')
        print(f'  ОПОРА — log(масштаб) от опорного кадра: σ {np.std(gs):.4f}, '
              f'уход за {g1-g0:.0f}с {gs[-1]:+.4f} '
              f'(= {abs(gs[-1])/max(1e-9, abs(cf[0])):.2f} м ложного смещения)')
