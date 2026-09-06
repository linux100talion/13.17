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

РАСКЛАДКА (2026-09-01, бывший монолит 1198 строк; класс ОДИН, состояние на self,
имена атрибутов/методов не менялись — стенды со своими `_ipm_update`/`_ipm_px` живут):
- `ipm.py` — миксин IpmChannel: канал ВИДА СВЕРХУ (проекция/полоса/варп/LK, фильтр
  скорости, ФВЧ-дебиасы ω_z и ускорения). ⚠️ cv2 для перехвата LK патчить ТАМ.
- `keyframe.py` — миксин KeyframeChannel: ОПОРНЫЙ канал (посев/ведение точек,
  подобие, гейты кадра, сегменты-накопитель, скорость опоры, пересев).
- здесь — фасад: ipm_dbg_z, конструктор (общая оптика + канал СКОРОСТИ/yaw)
  и process()-оркестратор (LK скорости, derotation, боковой/yaw/продольный
  агрегаты — и вызовы каналов).
⚠️ Импортировать ТОЛЬКО пакетом (`control_pkg.perception.flow_estimator`):
standalone `import flow_estimator` больше не работает (миксины — relative import).
"""

import math

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from .ipm import IpmChannel
from .keyframe import KeyframeChannel


def ipm_dbg_z(ok: bool, fail: int) -> float:
    """Кодировка достоверности IPM для /flow_dbg8.z и /flow_dbg9.z.

    Одна правда с таблицей кодов ipm_fail (см. FlowEstimator.__init__);
    живёт здесь, а не в ros_io, чтобы тестироваться без ROS. Годный кадр →
    1.0 (бит-в-бит со старыми bag); брак → −код (−1…−7: старые разборщики
    «z>0.5 = ок» живут, причина = round(−z)); фильтр ipm_vel_tau держит
    скорость на браке кадра → 1.0 + код/10 (1.1…1.7: для старых по-прежнему
    «ок», причина = round((z−1)·10)). В старых bag 0.0 = брак без причины."""
    if ok:
        return 1.0 + (fail / 10.0 if fail else 0.0)
    return float(-fail)


class FlowEstimator(IpmChannel, KeyframeChannel):
    def __init__(self, fx, fy, cx, cy, R_cam_imu, rotflow_sign=1.0, max_feats=200,
                 roll_smooth_n=1, pitch_smooth_n=1, yaw_smooth_n=1, kf_min_pts=40,
                 kf_max_step=0.05, cam_tilt=0.26, kf_tilt_k=0.05, feat_lo=0.667,
                 kf_alt_max=0.06, kf_reject_max=10, kf_seg_max=0.027, kf_win=2.0,
                 kf_alt_hold=1.5, yaw_trans_fix=True, kf_seg_min_sec=0.3, kf_seg_frac=0.30,
                 kf_seg_cap_sec=10.0, ipm=True, ipm_x0=3.0, ipm_x1=6.0,
                 ipm_yhalf=2.0, ipm_res=0.02, ipm_win=0.5, ipm_model='legacy',
                 ipm_derot=0.0, ipm_wz_tau=0.0, ipm_adapt=0.0, ipm_vel_tau=0.0,
                 ipm_alt_floor=0.0, ipm_scale_ref=0.0, ipm_acc_tau=0.0,
                 ipm_wz_gate=0.0, ipm_wz_bias_max=0.0):
        if cv2 is None:
            raise RuntimeError('cv2 не найден — FlowEstimator не работает')
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.R = np.asarray(R_cam_imu, dtype=np.float64).reshape(3, 3)
        self.rotflow_sign = float(rotflow_sign)
        # Вычитать ли вклад сноса из канала курса (подгонка по строке кадра, см. process).
        # False возвращает прежний закон — медиану горизонтального потока целиком.
        self.yaw_trans_fix = bool(yaw_trans_fix)
        # конфиг/состояние каналов — в их миксинах (комментарии-обоснования там):
        # канал вида сверху (IPM) — ipm.py, опорный канал (keyframe) — keyframe.py
        self._init_ipm(ipm, ipm_x0, ipm_x1, ipm_yhalf, ipm_res, ipm_win, ipm_model,
                       ipm_derot, ipm_wz_tau, ipm_adapt, ipm_vel_tau, ipm_alt_floor,
                       ipm_scale_ref, ipm_acc_tau, ipm_wz_gate,
                       ipm_wz_bias_max=ipm_wz_bias_max)
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
        # ⚠️ ВЫСОТА СОКРАЩАЕТСЯ — маска УЖЕ масштабно-инвариантна (замер 2026-08-26,
        # исправляет прежнюю запись «фиксированная доля — первое приближение, надо
        # считать от высоты»). Камера проективна: фиксированная СТРОКА = фиксированное
        # ОТНОШЕНИЕ X/h, а не фиксированная дистанция. Строки 360..540 при наклоне
        # 15° и fy=480 смотрят на землю:
        #     h = 0.26 м → 0.54…0.27 м     X/h = 2.08…1.04
        #     h = 1.00 м → 2.09…1.03 м     X/h = 2.09…1.03
        #     h = 5.00 м → 10.47…5.13 м    X/h = 2.09…1.03
        # То есть считать маску ОТ ВЫСОТЫ нечего — она даст ту же строку. Не
        # сокращается только НАКЛОН: тангаж +5° уводит полосу с 1.03-2.09h на
        # 0.88-1.75h (~16%), +10° — вдвое сильнее; маска должна ехать за горизонтом,
        # и вот это стоит доделать (за ручкой, с офлайн-замером).
        # Настоящий вопрос — ВЫБОР полосы, а не привязка к высоте: нынешняя сидит
        # вплотную (1-2h), и на 0.26 м при 1 м/с точки уезжают 29-60 px/кадр при
        # потолке LK (winSize 21, maxLevel 3) порядка 40-80 — на быстром низком
        # полёте канал начнёт сыпаться. Полоса подальше (1.5-4h) уполовинит сдвиг.
        # Опорный набор маской НЕ ограничен: у него своя механика (долгая жизнь точек),
        # и менять оба канала одним прогоном значило бы не понять, что подействовало.
        self.feat_lo = float(feat_lo)
        self._mask_vel = None
        self._init_keyframe(kf_min_pts, kf_max_step, cam_tilt, kf_tilt_k,
                            kf_alt_max, kf_reject_max, kf_seg_max, kf_win,
                            kf_alt_hold, kf_seg_min_sec, kf_seg_frac, kf_seg_cap_sec)
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
        self.ipm_fwd = self.ipm_lat = 0.0   # путь вида сверху — от новой точки удержания
        self._ipm_hist = []
        self._ipm_prev_t = None
        self._alt_out = 0.0     # заморозка по высоте — тоже заново

    def process(self, gray, stamp, omega_imu, pitch=0.0, alt=None, roll=0.0):
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
        # ω в FLU: z — ось ВВЕРХ, то есть ω_z и есть скорость разворота (см. ipm_derot).
        wz = float(np.asarray(omega_imu, dtype=np.float64).reshape(-1)[2])
        self._ipm_update(gray, stamp, alt, pitch, roll, wz)
        out = None
        if self.prev_gray is not None and self.vel_pts is not None and len(self.vel_pts) > 0:
            dt = max(1e-3, stamp - self.prev_stamp)
            # --- НАБОР СКОРОСТИ: свежие точки предыдущего кадра → текущий ---
            vp = self.vel_pts.reshape(-1, 1, 2).astype(np.float32)
            nxt, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, vp, None, **self._lk)
            st = st.reshape(-1).astype(bool)
            p0 = vp.reshape(-1, 2)[st]
            p1 = nxt.reshape(-1, 2)[st]
            self._kf_track(gray)
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
                # ⚠️ ПРЕДПОЛОЖЕНИЕ «трансляция ≈0» ЛОМАЕТСЯ ДВИЖЕНИЕМ. Замер (yaw_fidelity.py,
                # доля истинного разворота, увиденная сигналом): ось курса одна, соседи на
                # оракулах (борт стоит) — +0.96 ± 0.12; три оси на Dp, борт идёт 1-4 м/с
                # (E2) — −0.09 ± 0.02; без стабилизации вовсе — −10. Точки выстраиваются
                # по СКОРОСТИ борта, а не по числу осей: медиану горизонтального потока
                # заливает снос, контур нулит снос вместо курса, курс уходит на 23…360°.
                #
                # Разделяем их ГЕОМЕТРИЕЙ, а не гироскопом (вычесть yaw-гиро нельзя — тогда
                # визуальный курс выродится в гироскоп со всем его дрейфом, ради чего его и
                # не вычитали). Для наклонённой вниз камеры дальность до земли растёт к
                # горизонту, поэтому горизонтальный поток от ТРАНСЛЯЦИИ ∝ 1/Z ∝ (y − y_гор),
                # а от ВРАЩЕНИЯ почти не зависит от строки. Значит линейная подгонка
                # u(y) = a + b·(y − y_гор) разносит их: b — снос, свободный член a —
                # вращение (там, где трансляции нет по построению: на бесконечности).
                oi = np.asarray(omega_imu, dtype=np.float64)
                w_ny = self.R @ np.array([oi[0], oi[1], 0.0])   # FLU: yaw (z) обнулён
                rot_ny = self._rot_flow(p0, w_ny[0], w_ny[1], w_ny[2], dt)
                u_ny = (flow - rot_ny)[:, 0]                    # yaw-вращение + трансляция
                yaw_flow_raw = float(np.median(u_ny))           # старый закон — как fallback
                if self.yaw_trans_fix:
                    # строка горизонта: камера смотрит вниз на (cam_tilt + pitch)
                    a_dn = self.cam_tilt + pitch
                    if a_dn > 1e-3:
                        yh = self.cy - self.fy * math.tan(a_dn)
                        dy = p0[:, 1] - yh
                        # подгонка осмысленна только когда точки РАЗНЕСЕНЫ по строкам:
                        # на узкой полосе наклон не определён и вылезет мусор в свободном члене
                        if float(np.ptp(dy)) > 0.15 * self.cy:
                            b, a0 = np.polyfit(dy, u_ny, 1)
                            yaw_flow_raw = float(a0)
                        # ⚠️ ЭКСТРАПОЛЯЦИЯ ДЛИННАЯ. Точки канала скорости живут ПОД маской
                        # feat_lo (нижняя треть кадра), а горизонт — выше неё на сотни
                        # строк, поэтому свободный член берётся далеко за пределами данных
                        # и шум наклона попадает в него с плечом. На синтетике (шума нет)
                        # это не видно: yaw_trans_test даёт остаток сноса 0.014 px/кадр
                        # против 8.9 у прежнего закона. На реальных кадрах проверять
                        # отдельно — переигрыванием бэга с сохранёнными /image_color.
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
                kf_dx, kf_dy, kf_logs, kf_rot, kf_ok = self._kf_measure(pitch, frozen)
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
                    # --- канал вида сверху: МЕТРЫ и м/с ---
                    ipm_fwd=self.ipm_fwd, ipm_lat=self.ipm_lat,
                    ipm_vfwd=self.ipm_vfwd, ipm_vlat=self.ipm_vlat,
                    ipm_noise_fwd=self.ipm_noise_fwd, ipm_noise_lat=self.ipm_noise_lat,
                    ipm_ok=self.ipm_ok, ipm_fail=self.ipm_fail,
                    # --- диагностика для flow_derotation_check ---
                    resid_rms=float(np.sqrt(np.mean(np.sum(tr ** 2, axis=1)))),
                    meas_rms=float(np.sqrt(np.mean(np.sum(flow ** 2, axis=1)))),
                    omega_norm=float(np.linalg.norm(omega_imu)),
                )
        # Набор СКОРОСТИ переоткрывается каждый кадр — ему так и надо (см. выше).
        self.vel_pts = self._detect(gray, self._vel_mask(gray))
        if self.vel_pts is not None:
            self.vel_pts = self.vel_pts.reshape(-1, 2)
        self._kf_reseed(gray, stamp, pitch, alt, frozen)
        self.prev_gray = gray
        self.prev_stamp = stamp
        return out
