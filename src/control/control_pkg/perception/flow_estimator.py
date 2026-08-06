#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FlowEstimator — чистая зрительная часть FLOW-DAMP: кадр + гироскоп → агрегаты потока.

КАНОНИЧНАЯ копия для control_pkg (борт self-contained: src/lab на Orin не монтируется).
Легаси-копия src/lab/flow_estimator.py остаётся для монолит-инструментов до их вывода.
БЕЗ ROS (только numpy + cv2) — перцепт-сервис домена восприятия.

Два канала на одних и тех же точках:

СКОРОСТЬ (как было, см. FAQ_vins.md 6-11):
  sparse LK (поток между соседними кадрами)
    → derotate: вычесть ВРАЩАТЕЛЬНЫЙ поток (ω_cam = R · ω_imu), формула Longuet-Higgins
    → остаточный ТРАНСЛЯЦИОННЫЙ поток
    → агрегаты: боковой (медиана горизонт.) + диагностика (RMS остатка/измерения, |ω|).

ПОЛОЖЕНИЕ (опорный кадр):
  точки НЕ переоткрываются каждый кадр, а ведутся, пока видны
    → подобие опорный→текущий (estimateAffinePartial2D)
    → kf_dx/kf_dy (сдвиг), kf_logs = log(масштаб), kf_rot (поворот).

Зачем. Раньше в конце каждого process() стоял безусловный _detect(): фича жила
РОВНО ОДИН кадр, и вместе с ней терялось положение — доступна была только скорость.
Замер (src/lab/keyframe_track.py по bag'у G1, посев 158 точек): медиана жизни точки
284 кадра = 9.4 с при 30 Гц, 97% доживают до 20 кадров. В окне 12.1..21.9 с покадровый
сдвиг связан с ИСТИННОЙ СКОРОСТЬЮ на corr −0.22 (то есть почти никак), а log(масштаб)
с ИСТИННЫМ УДАЛЕНИЕМ — на −0.75 при крутизне −1.8% на метр и шуме 0.14 м. Продольная
ось живёт в МАСШТАБЕ: камера смотрит почти горизонтально (наклон 15°), ход вперёд идёт
вдоль оптической оси и сдвига почти не даёт.

Осталось (не в этом слое): звать reset_keyframe() на входе в сегмент удержания —
опора и есть точка, к которой борт возвращается; и подключить kf_logs к DpPitchHold
вместо покадрового longitudinal.

rotflow_sign: множитель вращательной поправки. +1/−1 — два знака; 0 — БЕЗ derotation
(baseline). Перебором {R, Rᵀ}×{±1} оффлайн-тест выбирает верный вариант по минимуму
остатка на кадрах с большим |ω| (чистое вращение → правильная derotation → остаток≈0).
"""

import math

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None


class FlowEstimator:
    def __init__(self, fx, fy, cx, cy, R_cam_imu, rotflow_sign=1.0, max_feats=200,
                 roll_smooth_n=1, pitch_smooth_n=1, yaw_smooth_n=1, kf_min_pts=40,
                 kf_max_step=0.05, cam_tilt=0.26, kf_tilt_k=0.05, feat_lo=0.667,
                 kf_alt_max=0.06, kf_reject_max=10, kf_seg_max=0.027, kf_win=2.0,
                 kf_alt_hold=1.5):
        if cv2 is None:
            raise RuntimeError('cv2 не найден — FlowEstimator не работает')
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.R = np.asarray(R_cam_imu, dtype=np.float64).reshape(3, 3)
        self.rotflow_sign = float(rotflow_sign)
        self.max_feats = max_feats
        # ВРЕМЕННОЕ СГЛАЖИВАНИЕ: медиана по N кадрам, СВОЁ N на КАЖДУЮ ось (roll=lateral,
        # pitch=longitudinal, yaw). Шум потока БЕЛЫЙ (автокорр≈0, см. flow_calib) →
        # усреднение по N режет пол как √N, а сигнал (скорость) на низкой частоте почти
        # не смазывается. Лаг ~N/2 кадров мал и петля к нему нечувствительна (τ-развёртка
        # в flow_loop_sim). 1 = выкл.
        self.roll_smooth_n = max(1, int(roll_smooth_n))
        self._lat_buf = []
        self.pitch_smooth_n = max(1, int(pitch_smooth_n))
        self._lon_buf = []
        self._kf_buf = []               # то же сглаживание для опорного сигнала
        self.yaw_smooth_n = max(1, int(yaw_smooth_n))
        self._yaw_buf = []
        self.prev_gray = None
        self.prev_stamp = None
        # --- ДВА НАБОРА ТОЧЕК на одном кадре, у каждого своя работа ---
        # vel_pts — СВЕЖИЙ детект на каждом кадре: канал СКОРОСТИ (roll/yaw) считает
        #   медиану межкадрового сдвига, и идентичность точек ему не нужна вовсе.
        # kf_ref/kf_cur — ДОЛГОЖИВУЩИЙ набор: канал ПОЛОЖЕНИЯ (pitch), там идентичность
        #   и есть весь смысл.
        # Почему врозь. Когда опора забрала себе единственный набор, крен поехал: набор
        # стареет и смещается к дальним объектам, у которых боковой поток слабее, а
        # kp=16 подобран под свежие точки. Замер по 8-секундному окну висения — снос
        # вбок 1.4 м (G1, свежие точки) против 12.8 и 13.8 м (H1/H2, состарившиеся).
        # Цена разделения — второй вызов КЛТ на кадр; детект по-прежнему один.
        self.vel_pts = None
        # --- ГДЕ ИСКАТЬ ТОЧКИ ДЛЯ КАНАЛА СКОРОСТИ ---
        # feat_lo — доля высоты кадра, ВЫШЕ которой детектор не смотрит (0 = весь кадр).
        # Замер по L1_scale2ax (оракул ведёт, зрение смотрит), распределение точек:
        #   строки   0-90 (небо)          4.2%
        #   строки  90-180 (горизонт)    65.6%
        #   строки 180-270               27.7%
        #   строки 270-360                2.5%
        #   строки 360-540 (земля 7-11м)  0.0%
        # То есть 93% точек сидели в полосе горизонта, где поток почти нулевой, а до
        # ближней земли, где он впятеро сильнее, детектор не доходил ни разу: он тратит
        # бюджет в 200 углов на контрастную линию горизонта. Отсюда S_lat=0.4 вместо
        # расчётных 1.3-3.3 — мерили скорость по дальнему плану, который не движется.
        # Что даёт маска (тот же bag, та же обработка):
        #   весь кадр   S_lat +0.40 px/(м/с), шум 3.10 м/с, R² 0.02
        #   ниже 200    +1.17,               1.04 м/с,      0.14
        #   ниже 270    +1.65,               0.76 м/с,      0.23
        #   ниже 360    +2.42,               0.59 м/с,      0.32   ← 0.667·540
        # Углов на земле хватает: под маской набираются все 200, и снижение
        # qualityLevel (0.01→0.001) ничего не меняет — упор в лимит, не в качество.
        # ⚠️ Порог привязан к геометрии: при 5 м и наклоне 15° земля у нижнего края в
        # 7 м, у строки 360 — в 11 м. На другой высоте правильнее считать маску от
        # высоты и наклона; фиксированная доля — первое приближение.
        # Опорный набор маской НЕ ограничен: у него своя механика (долгая жизнь точек),
        # и менять оба канала одним прогоном значило бы не понять, что подействовало.
        self.feat_lo = float(feat_lo)
        self._mask_vel = None
        # --- ОПОРА (keyframe): точки живут, пока их видно ---
        # kf_ref[i] — где точка была в ОПОРНОМ кадре, kf_cur[i] — где она сейчас.
        # Массивы строго 1:1: потерянная КЛТ точка вычёркивается из обоих.
        self.kf_min_pts = int(kf_min_pts)   # ниже — опора потеряна, пересев
        # Отсечка выброса подобия. Физика: 3 м/с при 30 Гц = 0.1 м за кадр = 0.0018
        # в log(масштаба) (крутизна ~1.8%/м). Порог 0.05 — двадцатикратный запас, но
        # ловит срывы RANSAC на ложной гипотезе (в G1 один такой: −0.60 вместо −0.30).
        # Выброс не гасится и не сглаживается, а ОТБРАСЫВАЕТСЯ: держим прошлое значение.
        self.kf_max_step = float(kf_max_step)
        self._kf_logs_prev = None
        self.kf_rejects = 0
        # ЗАЩЁЛКА, которую эта переменная снимает. Отсечка выше молчит по одному кадру,
        # но на скорости точки вылетают из кадра, подобие скачет больше порога КАЖДЫЙ
        # кадр — и канал затыкается насовсем: значение держится прежнее, kf_valid=False,
        # регулятор молчит. Пересев при этом не срабатывает (точек хватает, высота
        # стабильна). Замер H8s1: kf_logs замер на 0.085 и не двигался пять секунд, борт
        # разогнался с 3.9 до 6.4 м/с и улетел на 20 м; вышло само собой, только когда
        # точек стало мало и опора пересеялась — контур тут же скомандовал верно.
        # Чем быстрее борт, тем надёжнее был заткнут канал — ровно когда он нужнее.
        # Поэтому: отбраковки ПОДРЯД дольше kf_reject_max кадров (10 ≈ 0.5 с) значат не
        # выброс, а протухшую опору → пересев.
        self.kf_reject_max = int(kf_reject_max)
        self._kf_reject_run = 0
        # --- КОРОТКАЯ ОПОРА С НАКОПЛЕНИЕМ ---
        # Канал линеен только вблизи опоры. Замер по H9s1/H9s2/H8s2, |kf_logs| против
        # истинного удаления: 0-2 м читается на 97/198/72%, 2-5 м на 72/56/44,
        # 5-9 м на 58/34/63, 9-15 м на 41/30/44, 15-30 м на 3%. Причина геометрическая:
        # масштаб чувствителен к БЛИЖНИМ точкам, а именно они первыми уходят из кадра
        # при смещении; остаётся дальний план, у которого метр почти ничего не меняет.
        # Контур из-за этого садился на устойчивое равновесие в 10-13 м: чем дальше
        # борт, тем меньше причин его возвращать.
        # Поэтому опору держим КОРОТКОЙ: набралось |Δ| больше kf_seg_max — сегмент
        # закрывается, его значение уходит в накопитель, опора сеется заново. Отчёт
        # наружу = накопитель + текущий сегмент, то есть работаем всегда в линейной зоне.
        # 0.027 ≈ 2 м при крутизне 1.35%/м.
        # Платим дрейфом счисления: шум сегмента 0.14 м, за 40 с при ~20 сегментах это
        # √20·0.14 ≈ 0.6 м — против 13 м нынешнего равновесия размен выгодный.
        self.kf_seg_max = float(kf_seg_max)
        self.kf_acc = 0.0                   # закрытые сегменты, log-единиц
        self.kf_segs = 0                    # сколько сегментов закрыто (диагностика)
        # --- СКОРОСТЬ ОПОРЫ: наклон МНК в окне kf_win секунд ---
        # Демпферу нужна производная положения. Считать её кадр-к-кадру нельзя: шаг
        # сигнала (медиана 0.0007, p95 0.0134 — замер J1b) на порядок больше полезного
        # приращения, и производная выходит чистым шумом. Замер J1b против истинной
        # v_fwd по одометрии: кадр-к-кадру corr +0.27, окно 0.25 с +0.46, 0.5 с +0.61,
        # 1.0 с +0.80, 1.5-2.0 с +0.84. Крутизна при этом стабильна (+11.1±0.5
        # m-log/(м/с) на всех окнах) — то есть окно не искажает масштаб, только режет шум.
        # ПЕРЕПРОВЕРЕНО переигрыванием J2 боевым оценщиком (kf_vel_check.py), окно
        # висения, 1213 кадров — там же опровергнута версия «виновата медиана»:
        #   окно  0.5с  corr +0.57  остаток 1.87 м/с
        #   окно  1.0с       +0.72           1.26
        #   окно  1.5с       +0.81           0.95
        #   окно  2.0с       +0.84           0.83   ← берём
        #   окно  3.0с       +0.77           1.07   (окно уже размазывает сам сигнал)
        # По СЫРОМУ значению сегмента вместо сглаженного — та же картина ±0.02, то есть
        # медианный фильтр производной НЕ мешает; дело в шуме самой оценки подобия.
        # Цена окна 2 с — своя задержка ≈ W/2 = 1 с поверх контурной 1.04 с. На периоде
        # автоколебания 22 с это 16° фазы; демпфирование того стоит (при 1 с остаток
        # 1.26 м/с сравним с самой скоростью борта, то есть демпфер работал по шуму).
        self.kf_win = float(kf_win)
        self._kf_hist = []                  # [(stamp, kf_logs)] в окне kf_win
        self.kf_vel = 0.0                   # log-единиц в секунду (≈ +0.0111·v_fwd)
        # --- КОМПЕНСАЦИЯ НАКЛОНА (главный источник шума опорного канала в полёте) ---
        # Камера смотрит вниз на cam_tilt (SDF: 0.26 рад = 14.9°; по горизонту в кадре
        # 12.9° ±2.4 — метод грубый, разница уходит в подобранный коэффициент). Борт для
        # разгона кладётся на ±10°, угол визирования гуляет, дальность до сцены ∝
        # 1/sin(наклон+тангаж) — и масштаб меняется БЕЗ всякого перемещения.
        # Замер (keyframe_track.py, окно 30 кадров): добавление этого члена в модель
        # подняло R² с 0.10 до 0.63 (K1c) и с 0.31 до 0.50 (G1), а остаток упал
        # 2.03 → 0.59 м/с. Коэффициент вышел +0.050 и +0.047 на двух независимых
        # прогонах — совпадение до третьего знака, поэтому берём 0.05.
        # Почему не 1.00, как у модели плоской земли: основная масса точек не на земле,
        # а на дальних объектах, чью дальность наклон почти не меняет.
        self.cam_tilt = float(cam_tilt)
        self.kf_tilt_k = float(kf_tilt_k)
        self._kf_pitch0 = 0.0               # тангаж на момент посева опоры
        # --- ОПОРА ЖИВЁТ ТОЛЬКО НА ПОСТОЯННОЙ ВЫСОТЕ ---
        # Дальность до сцены пропорциональна высоте, поэтому при наборе/снижении точка
        # отсчёта протухает: подобие пересобирается на ходу и kf_logs скачет вокруг нуля
        # с амплитудой 0.02-0.04 — не показывая движения. Замер H6_kd на наборе с 1.5 до
        # 4.8 м: kd берёт производную этого дребезга (скачок 0.02 за кадр = 1000 PWM),
        # слю-лимит размазывает её в устойчивые −70 PWM, рама кладётся на 6.9° и разгоняет
        # борт до 1.5 м/с — ВСЯ скорость на входе в висение оказалась самодельной
        # (до больших команд борт шёл 0.25 м/с).
        # Поэтому: ушла высота больше чем на kf_alt_max (в логарифме) — блок опоры
        # ЗАМИРАЕТ. Кадр помечается kf_valid=False (регулятор по нему НЕ командует), и
        # при этом НИЧЕГО не меняется: сегмент не закрывается, опора не пересевается,
        # `_kf_logs_prev`/медианный буфер держат последнее хорошее значение.
        # 0.06 ≈ 6% высоты: на 5 м это 30 см, меньше рабочего шага набора и больше
        # колебаний удержания высоты. НО летаем мы на 3 м, где 6% = 19 см при размахе
        # болтанки ALT_HOLD 0.2-0.4 м — порог срабатывает на штатном удержании высоты.
        #
        # Почему замирание, а не пересев (было: пересев, разбор в ToDo5.md).
        # Пересев по высоте выбрасывал накопленное смещение (`trust=False`) — 31-39 раз
        # за 20 с висения, то есть точка удержания переезжала дважды в секунду. Контур
        # от этого умел только гасить и не умел возвращать. Свип E1 (kf_alt_max
        # 0.06/0.15/0.25/0.40) это подтвердил: доля сохранённых сегментов 28/85/89/95%.
        # Пересев не нужен, потому что вклад высоты в масштаб МГНОВЕННЫЙ (считается по
        # текущей высоте) — вернулась высота, сигнал продолжился сам. А фича живёт
        # медиану 284 кадра ≈ 9.4 с против долей секунды болтанки, так что опора
        # переживает заморозку с запасом порядка.
        self.kf_alt_max = float(kf_alt_max)
        # Настоящий набор высоты пересев ТРЕБУЕТ — там высота не возвращается. Отличаем
        # по ДЛИТЕЛЬНОСТИ, а не по величине: не нужен дифференциатор высоты со своим
        # шумом, хватает одного счётчика. 1.5 с лежит между болтанкой (~0.3 с) и набором
        # (десятки секунд). На таком пересеве сегмент ЗАСЧИТЫВАЕТСЯ: `_kf_logs_prev` —
        # последнее достоверное значение (заморозка его не портила), ему верим.
        self.kf_alt_hold = float(kf_alt_hold)
        self._alt_out = 0.0                 # сколько секунд подряд высота вне порога
        self._kf_alt0 = None
        self.kf_ref = None
        self.kf_cur = None
        self.kf_n0 = 0                      # сколько точек было посеяно в опоре
        self.kf_age = 0                     # кадров с момента посева опоры
        self.kf_reseeds = 0                 # сколько раз опора терялась за прогон
        self._kf_pending = True             # первый кадр станет опорным
        self._lk = dict(winSize=(21, 21), maxLevel=3,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        self._feat = dict(maxCorners=max_feats, qualityLevel=0.01, minDistance=8, blockSize=7)

    def _detect(self, gray, mask=None):
        return cv2.goodFeaturesToTrack(gray, mask=mask, **self._feat)

    def _vel_mask(self, gray):
        """Маска для набора СКОРОСТИ: только нижняя часть кадра (ближняя земля)."""
        if self.feat_lo <= 0:
            return None
        if self._mask_vel is None or self._mask_vel.shape != gray.shape:
            m = np.zeros(gray.shape, np.uint8)
            m[int(self.feat_lo * gray.shape[0]):, :] = 255
            self._mask_vel = m
        return self._mask_vel

    def _rot_flow(self, p0, wx, wy, wz, dt):
        """Вращательный поток (пиксели/кадр) в точках p0 для ω камеры (rad/s)."""
        # нормированные координаты
        xn = (p0[:, 0] - self.cx) / self.fx
        yn = (p0[:, 1] - self.cy) / self.fy
        # Longuet-Higgins/Prazdny, нормир. плоскость, ×dt. TODO[sign]: знаки сверяет тест.
        u_rot_n = (xn * yn * wx - (1.0 + xn ** 2) * wy + yn * wz) * dt
        v_rot_n = ((1.0 + yn ** 2) * wx - xn * yn * wy - xn * wz) * dt
        u_rot = self.fx * u_rot_n
        v_rot = self.fy * v_rot_n
        return np.column_stack([u_rot, v_rot])

    # ---------------------------------------------------------------- опора
    def reset_keyframe(self):   # noqa: D401 — накопитель тоже обнуляется, см. ниже
        """Сбросить опору: следующий кадр станет новым опорным (точка удержания).

        Зовётся управляющим слоем на входе в сегмент удержания — как `enter()` у
        стабилизаторов. Позиция, которую отдаёт опорный канал, отсчитывается ОТ
        этого кадра; вернуться борт может только туда, что видит."""
        self._kf_pending = True
        self.kf_acc = 0.0       # новая точка удержания — счёт с нуля
        self.kf_segs = 0
        self._kf_hist = []      # положение отсчитывается заново → окно скорости тоже
        self.kf_vel = 0.0
        self._alt_out = 0.0     # заморозка по высоте — тоже заново

    def _kf_vel_update(self, stamp, kf_logs, kf_ok):
        """Скорость опоры = наклон МНК по окну kf_win секунд (см. ctor).

        Недостоверные кадры в окно НЕ кладём: их значение — задержанная копия
        прошлого, и наклон по ним занижен. Окно при этом не чистим — вернётся
        достоверность, вернётся и оценка, а пропуск в 2-3 кадра наклону не мешает.
        """
        if kf_ok:
            self._kf_hist.append((float(stamp), float(kf_logs)))
        self._kf_hist = [p for p in self._kf_hist if stamp - p[0] <= self.kf_win]
        if len(self._kf_hist) < 4:
            self.kf_vel = 0.0
            return
        ts = np.array([p[0] for p in self._kf_hist])
        vs = np.array([p[1] for p in self._kf_hist])
        span = ts[-1] - ts[0]
        if span < 0.5 * self.kf_win:     # окно ещё не набралось (вход в сегмент)
            self.kf_vel = 0.0
            return
        tc = ts - ts.mean()
        self.kf_vel = float(np.dot(tc, vs - vs.mean()) / np.dot(tc, tc))

    def _seed(self, gray):
        """Посеять точки и сделать текущий кадр опорным."""
        pts = self._detect(gray)
        if pts is None or len(pts) < 8:
            self.kf_ref = self.kf_cur = None
            return False
        self._kf_logs_prev = None                   # новая опора — новый отсчёт
        self._kf_reject_run = 0
        self._kf_buf = []
        self.kf_ref = pts.reshape(-1, 2).copy()     # где точки были в ОПОРНОМ кадре
        self.kf_cur = pts.reshape(-1, 2).copy()     # где они сейчас (1:1 с kf_ref)
        self.kf_n0 = len(pts)                       # сколько посеяли — база для conf
        self.kf_age = 0
        self._kf_pending = False
        return True

    def _similarity(self):
        """Подобие опорный→текущий: (dx, dy, масштаб, поворот) или None.

        ⚠️ dx/dy — НЕ чистое смещение борта: поворот камеры по тангажу/курсу даёт
        такой же сдвиг картинки. Масштаб к вращению нечувствителен (первый порядок),
        поэтому продольный канал по нему честен, а боковой требует компенсации по
        углам — она пока не сделана, `kf_rot` отдаётся для диагностики."""
        if self.kf_ref is None or len(self.kf_ref) < self.kf_min_pts:
            return None
        M, _ = cv2.estimateAffinePartial2D(self.kf_ref, self.kf_cur,
                                           method=cv2.RANSAC, ransacReprojThreshold=2.0)
        if M is None:
            return None
        s = float(math.hypot(M[0, 0], M[0, 1]))
        rot = float(math.atan2(M[1, 0], M[0, 0]))
        return float(M[0, 2]), float(M[1, 2]), s, rot

    def _tilt_term(self, pitch):
        """log отношения дальностей из-за наклона борта: log(sin(α+θ)/sin(α+θ_опоры))."""
        a1 = math.sin(self.cam_tilt + pitch)
        a0 = math.sin(self.cam_tilt + self._kf_pitch0)
        if a1 <= 1e-3 or a0 <= 1e-3:
            return 0.0
        return math.log(a1 / a0)

    def process(self, gray, stamp, omega_imu, pitch=0.0, alt=None):
        """gray: uint8 HxW; stamp: сек; omega_imu: ω в FLU (rad/s);
        pitch: тангаж борта, рад (>0 = нос ВНИЗ) — компенсация наклона;
        alt: высота (баро), м — опора действительна только пока она не ушла. → dict|None."""
        alt_drift = 0.0
        if alt is not None and self._kf_alt0 and alt > 0.2:
            alt_drift = abs(math.log(alt / self._kf_alt0))
        # ЗАМОРОЗКА: высота ушла — масштаб испорчен ЕЮ, а не движением (см. __init__).
        frozen = alt_drift > self.kf_alt_max
        dt_frame = 0.0 if self.prev_stamp is None else max(0.0, stamp - self.prev_stamp)
        self._alt_out = (self._alt_out + dt_frame) if frozen else 0.0
        out = None
        if self.prev_gray is not None and self.vel_pts is not None and len(self.vel_pts) > 0:
            dt = max(1e-3, stamp - self.prev_stamp)
            # --- НАБОР СКОРОСТИ: свежие точки предыдущего кадра → текущий ---
            vp = self.vel_pts.reshape(-1, 1, 2).astype(np.float32)
            nxt, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, vp, None, **self._lk)
            st = st.reshape(-1).astype(bool)
            p0 = vp.reshape(-1, 2)[st]
            p1 = nxt.reshape(-1, 2)[st]
            # --- НАБОР ОПОРЫ: свой КЛТ с обратной проверкой (отсев переприлипших) ---
            if self.kf_cur is not None and len(self.kf_cur) > 0:
                kp_prev = self.kf_cur.reshape(-1, 1, 2).astype(np.float32)
                knx, kst, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, kp_prev,
                                                       None, **self._lk)
                kback, kst2, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, knx,
                                                          None, **self._lk)
                kok = kst.reshape(-1).astype(bool) & kst2.reshape(-1).astype(bool)
                kok &= np.linalg.norm(kback.reshape(-1, 2) - kp_prev.reshape(-1, 2),
                                      axis=1) < 1.0
                # опора: выжившие точки — и в текущем наборе, и в опорном, строго 1:1
                self.kf_ref = self.kf_ref[kok]
                self.kf_cur = knx.reshape(-1, 2)[kok].copy()
                self.kf_age += 1
            n = len(p0)
            if n >= 8:
                flow = p1 - p0                                  # измеренный поток (px/кадр)
                # ω в фрейме камеры: ω_cam = R · ω_imu
                w = self.R @ np.asarray(omega_imu, dtype=np.float64)
                if self.rotflow_sign != 0.0:
                    rot = self.rotflow_sign * self._rot_flow(p0, w[0], w[1], w[2], dt)
                else:
                    rot = np.zeros_like(flow)                   # baseline: без derotation
                tr = flow - rot                                 # трансляционный остаток
                lateral_raw = float(np.median(tr[:, 0]))        # v0: прокси бокового сноса
                # временное сглаживание (медиана по N кадрам) — режет белый шум ~√N
                self._lat_buf.append(lateral_raw)
                if len(self._lat_buf) > self.roll_smooth_n:
                    self._lat_buf.pop(0)
                lateral = float(np.median(self._lat_buf)) if self.roll_smooth_n > 1 else lateral_raw
                # --- ВИЗУАЛЬНЫЙ YAW (фаза 2): derotate ТОЛЬКО roll+pitch (гиро x,y —
                # гравитация-референс, НЕ дрейфуют), yaw гиро НЕ вычитаем → остаток =
                # yaw-вращение + трансляция. В ДАЛЬНЕЙ сцене трансляция ≈0 (тот самый
                # depth, что убил боковую ось) → остаток ≈ чистый визуальный yaw.
                oi = np.asarray(omega_imu, dtype=np.float64)
                w_ny = self.R @ np.array([oi[0], oi[1], 0.0])   # FLU: yaw (z) обнулён
                rot_ny = self._rot_flow(p0, w_ny[0], w_ny[1], w_ny[2], dt)
                yaw_flow_raw = float(np.median((flow - rot_ny)[:, 0]))  # px/кадр ∝ визуальный yaw
                self._yaw_buf.append(yaw_flow_raw)                  # сглаживание (медиана по N)
                if len(self._yaw_buf) > self.yaw_smooth_n:
                    self._yaw_buf.pop(0)
                yaw_flow = float(np.median(self._yaw_buf)) if self.yaw_smooth_n > 1 else yaw_flow_raw
                # --- ПРОДОЛЬНАЯ ось (phase2, looming): два кандидата сигнала из tr.
                # longitudinal = медиана ВЕРТИКАЛЬНОГО остатка (для down-tilt камеры ∝
                # продольной скорости; прямой аналог lateral по оси Y).
                longitudinal_raw = float(np.median(tr[:, 1]))
                self._lon_buf.append(longitudinal_raw)          # сглаживание — как у roll
                if len(self._lon_buf) > self.pitch_smooth_n:
                    self._lon_buf.pop(0)
                longitudinal = (float(np.median(self._lon_buf)) if self.pitch_smooth_n > 1
                                else longitudinal_raw)
                # divergence = расширение поля из АФФИННОГО фита tr ~ [1, xn, yn]:
                # ∂u/∂xn + ∂v/∂yn ∝ Tz/Z (looming — движение вдоль оптической оси).
                xn = (p0[:, 0] - self.cx) / self.fx
                yn = (p0[:, 1] - self.cy) / self.fy
                M = np.column_stack([np.ones_like(xn), xn, yn])
                cu, *_ = np.linalg.lstsq(M, tr[:, 0], rcond=None)
                cv, *_ = np.linalg.lstsq(M, tr[:, 1], rcond=None)
                divergence = float(cu[1] + cv[2])
                # --- ОПОРНЫЙ КАНАЛ: подобие опорный→текущий (СМЕЩЕНИЕ, не скорость) ---
                sim = self._similarity()
                kf_dx, kf_dy, kf_logs, kf_rot = (0.0, 0.0, 0.0, 0.0)
                kf_ok = sim is not None
                if sim is not None:
                    kf_dx, kf_dy, s_kf, kf_rot = sim
                    kf_logs_raw = float(math.log(s_kf)) if s_kf > 1e-6 else 0.0
                    # вычитаем вклад собственного наклона — остаётся перемещение
                    kf_logs = kf_logs_raw - self.kf_tilt_k * self._tilt_term(pitch)
                    if frozen:
                        # Кадру не верим И НИЧЕГО НЕ МЕНЯЕМ: ни отбраковку, ни
                        # `_kf_logs_prev`, ни медианный буфер. Иначе испорченные высотой
                        # значения (а) отравят медиану на pitch_smooth_n кадров вперёд и
                        # (б) уедут в накопитель при закрытии сегмента — подъём запишется
                        # как перемещение, причём с пометкой «доверенный».
                        kf_ok = False
                        kf_logs = self._kf_logs_prev if self._kf_logs_prev is not None else 0.0
                    else:
                        if (self.kf_max_step > 0 and self._kf_logs_prev is not None
                                and abs(kf_logs - self._kf_logs_prev) > self.kf_max_step):
                            self.kf_rejects += 1
                            self._kf_reject_run += 1
                            kf_logs = self._kf_logs_prev  # выброс: держим прошлое значение
                            kf_ok = False                 # и помечаем кадр недостоверным
                        else:
                            self._kf_reject_run = 0
                        self._kf_logs_prev = kf_logs
                        # Сглаживание — тем же N, что у продольной оси. Сигнал медленный
                        # (положение), поэтому медиана почти не смазывает его, а шум режет:
                        # σ 0.0025 = 0.14 м на кадр (замер keyframe_track.py).
                        self._kf_buf.append(kf_logs)
                        if len(self._kf_buf) > self.pitch_smooth_n:
                            self._kf_buf.pop(0)
                    if self.pitch_smooth_n > 1 and self._kf_buf:
                        kf_logs = float(np.median(self._kf_buf))
                    kf_seg = kf_logs                  # смещение ВНУТРИ текущего сегмента
                    kf_logs = self.kf_acc + kf_seg    # наружу — полное от точки удержания
                self._kf_vel_update(stamp, kf_logs, kf_ok)
                out = dict(
                    lateral=lateral, lateral_raw=lateral_raw, yaw_flow=yaw_flow,
                    longitudinal=longitudinal, longitudinal_raw=longitudinal_raw,
                    divergence=divergence, n=n, dt=dt,
                    # conf считается по набору СКОРОСТИ (n из свежего детекта) — как и
                    # было исходно. Здоровье опоры отдельно, в kf_n/kf_age: мерить их
                    # одним числом нельзя, наборы живут по-разному.
                    conf=float(n) / float(self.max_feats),
                    # --- опора: положение относительно опорного кадра ---
                    kf_dx=kf_dx, kf_dy=kf_dy, kf_logs=kf_logs, kf_rot=kf_rot,
                    kf_vel=self.kf_vel,
                    kf_n=len(self.kf_ref) if self.kf_ref is not None else 0,
                    kf_age=self.kf_age, kf_reseeds=self.kf_reseeds,
                    kf_segs=self.kf_segs, kf_rejects=self.kf_rejects,
                    kf_valid=kf_ok,
                    # --- диагностика для flow_derotation_check ---
                    resid_rms=float(np.sqrt(np.mean(np.sum(tr ** 2, axis=1)))),
                    meas_rms=float(np.sqrt(np.mean(np.sum(flow ** 2, axis=1)))),
                    omega_norm=float(np.linalg.norm(omega_imu)),
                )
        # Набор СКОРОСТИ переоткрывается каждый кадр — ему так и надо (см. выше).
        self.vel_pts = self._detect(gray, self._vel_mask(gray))
        if self.vel_pts is not None:
            self.vel_pts = self.vel_pts.reshape(-1, 2)
        # Опора пересевается ТОЛЬКО когда точек не осталось (или попросили сбросить):
        # фича живёт медиану 284 кадра = 9.4 с при 30 Гц (замер keyframe_track.py по G1),
        # и в этих кадрах лежит положение, которого нет в межкадровом сдвиге.
        # На заморозке сегмент НЕ закрывается: `_kf_logs_prev` стоит на месте, значит и
        # seg_full сработать не может (проверка избыточна, но пишем явно — читателю).
        seg_full = (not frozen and self._kf_logs_prev is not None
                    and abs(self._kf_logs_prev) >= self.kf_seg_max)
        # Высота не вернулась за kf_alt_hold — это НАСТОЯЩИЙ набор, опора протухла честно.
        alt_stale = frozen and self._alt_out > self.kf_alt_hold
        if (self._kf_pending or self.kf_cur is None
                or len(self.kf_cur) < self.kf_min_pts
                or self._kf_reject_run > self.kf_reject_max
                or seg_full or alt_stale):
            if not self._kf_pending and self.kf_cur is not None:
                # Сегмент закрываем в накопитель, ЕСЛИ его значение чему-то верили:
                # плановый (набралось смещение), потеря точек или честный набор высоты
                # (`_kf_logs_prev` заморозка не портила — это последнее ДОСТОВЕРНОЕ
                # значение). Не верим только серии отбраковок: там измерение уже врало.
                trust = seg_full or alt_stale or (len(self.kf_cur) < self.kf_min_pts)
                if trust and self._kf_logs_prev is not None:
                    self.kf_acc += self._kf_logs_prev
                    self.kf_segs += 1
                else:
                    self.kf_reseeds += 1    # опора потеряна: точка удержания сменилась
            self._alt_out = 0.0             # повод отработан — счётчик заново
            if self._seed(gray):
                self._kf_pitch0 = pitch     # наклон опоры — от него считаем поправку
                if alt is not None and alt > 0.2:
                    self._kf_alt0 = alt     # высота опоры — от неё считаем протухание
        self.prev_gray = gray
        self.prev_stamp = stamp
        return out
