#!/usr/bin/env python3
"""RosPerception — адаптер: камера (mono8) + гироскоп (FLU) → FlowEstimator → flow_* в DroneState.

Зрительная часть боевого пре-VINS демпфера. Подписывается на кадр и ω, гоняет
FlowEstimator (control_pkg.perception), результат отдаёт домену через merge(state):
кладёт flow_lateral/flow_yaw/flow_conf/flow_dt и ИНКРЕМЕНТИРУЕТ flow_seq на КАЖДЫЙ
обработанный кадр (домен интегрирует PID покадрово по seq, а не по 20-Гц тику).

Интринсики берутся из разрешения камеры (pinhole 90° hfov: fx=fy=W/2, cx=W/2, cy=H/2).
R_cam_imu и rotflow_sign — из sim.yaml/монолита (подтверждены flow_derotation_check).
"""
import math

import numpy as np

from ..perception.attitude_buffer import AttitudeBuffer
from ..perception.flow_estimator import FlowEstimator


def attitude_at(pitch, roll, att_t, gyro_buf, stamp,
                extrap=True, extrap_max=0.2):
    """Ориентация, ДОТЯНУТАЯ гироскопом до штампа кадра. ⚠️ ПО ЗАМЕРУ ВЫКЛЮЧЕНА.

    ЗАМЫСЕЛ БЫЛ. ATTITUDE идёт 12.5 Гц, камера — 20-30 Гц, а `_pitch`/`_roll` берутся как
    «последнее пришедшее», без привязки ко времени кадра. Внутри интервала ATTITUDE угол
    держится СТУПЕНЬКОЙ, истинный уходит со скоростью ω — ошибка растёт линейно, полоса
    земли в `_ipm_rectify` уезжает, и в скорость подмешивается член ∝ угловой скорости.
    Геометрия давала 1.53 м/(рад/с), лётный замер по G3 — 0.85…2.13. Сходилось.

    ЗАМЕР ЭТО ОПРОВЕРГ. Стенд `att_extrap_test.py` по бэгу I1s1 (одни кадры, боковая ось):
        способ   утечка ωx, м
        hold        +3.31        как в полёте
        extrap      +3.39        дотяжка
        near        +3.42        ближайшее сообщение — НЕДОСТИЖИМЫЙ потолок
    Раз даже `near` (он заглядывает вперёд, лучше синхронизировать нельзя) утечку не
    убирает, причина не временнáя. Совпадение расчёта с замером оказалось случайным.
    Настоящей багой был ЗНАК КРЕНА в проекции — см. `FlowEstimator._ipm_px`.
    Продольный канал дотяжка при этом ПОРТИТ: R² 0.74 → 0.50, ошибка 0.40 → 0.65 м/с.

    Код оставлен под `att_extrap` (дефолт False): рассинхрон никуда не делся, он просто
    не объясняет утечку. Понадобится — включать вместе с новым замером, не «на всякий».

    ⚠️ ω тут НЕ та, что уходит в оценщик. Оценщику нужен угол, повёрнутый МЕЖДУ кадрами
    (`_omega_for` — среднее по интервалу), а нам — скорость НА МОМЕНТ сообщения
    ориентации, чтобы дотянуть её вперёд. Разные вопросы, разные выборки.

    Малые углы: φ̇ ≈ ωx, θ̇ ≈ ωy. Точные формулы Эйлера добавляют множители
    sinφ·tanθ / cosφ, но на рабочих ±15° это ≤5 %, а интервал дотяжки ≤80 мс."""
    if not extrap or att_t is None or len(gyro_buf) == 0:
        return pitch, roll
    dt = stamp - att_t
    if dt <= 0.0 or dt > extrap_max:
        return pitch, roll
    arr = np.asarray(gyro_buf)
    sel = arr[:, 0] >= att_t
    w = arr[sel, 1:4].mean(axis=0) if sel.sum() else arr[-1, 1:4]
    return pitch + float(w[1]) * dt, roll + float(w[0]) * dt


class RosPerception:
    def __init__(self, node, cam_w, cam_h, R_cam_imu, rotflow_sign=1.0,
                 roll_smooth_n=1, pitch_smooth_n=1, yaw_smooth_n=5,
                 kf_alt_max=None, kf_alt_hold=None, yaw_trans_fix=None,
                 kf_seg_min_sec=None, kf_seg_frac=None,
                 image_topic='/image_mono', imu_topic='/mavros/imu/data',
                 gyro_topic=None, att_extrap=True, att_extrap_max=0.2,
                 ipm_model=None, ipm_derot=None, ipm_wz_tau=None, ipm_win=None,
                 ipm_adapt=None, ipm_vel_tau=None, ipm_alt_floor=None,
                 ipm_scale_ref=None, ipm_acc_tau=None, alt_src='global',
                 alt_zero=False, ipm_wz_gate=None, att_interp=False, att_latency=0.0,
                 att_wait_max=0.15):
        # ⚠️ ИСТОЧНИК ω — НЕ /gz_imu/data_flu. Тот поток пропущен через low-pass 5 Гц
        # (src/sim/imu_frd_to_flu.py; фильтр нужен VINS — срезает лимит-цикл rate-loop
        # ~7.5 Гц, которого камера на 10 Гц не видит). Оценщик вычитает по ω ВРАЩАТЕЛЬНЫЙ
        # поток, и прожатая ω оставляет остаток вращения, который уезжает в оценку
        # перемещения. Замер (kf_vel_check.py, один бэг J3, одни кадры, один код, разница
        # только в источнике ω):
        #             положение corr | шаг/кадр | скорость corr (окно 2с) | остаток
        #   MAVROS         +0.89        0.0034          +0.68              1.43 м/с
        #   gz_imu_flu     +0.61        0.0070          +0.32              3.86 м/с
        # Полётная телеметрия того же прогона: +0.55 / +0.34 — то есть нода получала
        # ровно вторую строку. Ориентация в фильтре проходит НЕтронутой, поэтому дело
        # именно в угловой скорости, а не в тангаже.
        # Плюс это совпадает с боевым бортом: на Orin IMU приходит с полётника через
        # MAVROS (фильтр INS_GYRO_FILTER 20 Гц — вчетверо шире), gz-потока там нет вовсе.
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, Imu
        from std_msgs.msg import Float64
        fx = fy = cam_w / 2.0          # pinhole 90° hfov
        cx, cy = cam_w / 2.0, cam_h / 2.0
        # Затвор опоры по высоте — две ручки, обе наружу (свип E1, разбор в ToDo5.md):
        #   kf_alt_max  — ПОРОГ заморозки: ушла высота больше — кадр недостоверен,
        #                 регулятор по нему не командует;
        #   kf_alt_hold — сколько секунд держаться вне порога, чтобы признать это
        #                 настоящим набором и пересеять опору. Болтанка ALT_HOLD
        #                 (~0.3 с) до пересева не доживает — точка удержания стоит.
        # None = дефолты оценщика (0.06 и 1.5 с).
        extra = {}
        if kf_alt_max is not None:
            extra['kf_alt_max'] = float(kf_alt_max)
        if kf_alt_hold is not None:
            extra['kf_alt_hold'] = float(kf_alt_hold)
        # Вычитание сноса из канала курса. Наружу выведено ради ЧЕСТНОГО сравнения:
        # переигрывание требует прогона со СТАРЫМ законом и сохранёнными кадрами,
        # иначе мерить нечего — исправленная ось держит курс, и разворота в данных
        # не остаётся (E3f1: истинных 7° за висение, отношение тонет в шуме).
        if kf_seg_frac is not None:
            extra['kf_seg_frac'] = float(kf_seg_frac)
        if kf_seg_min_sec is not None:
            extra['kf_seg_min_sec'] = float(kf_seg_min_sec)
        if yaw_trans_fix is not None:
            extra['yaw_trans_fix'] = bool(yaw_trans_fix)
        if ipm_model is not None:
            extra['ipm_model'] = str(ipm_model)
        if ipm_derot is not None:
            extra['ipm_derot'] = float(ipm_derot)
        if ipm_wz_tau is not None:
            extra['ipm_wz_tau'] = float(ipm_wz_tau)
        if ipm_wz_gate is not None:
            extra['ipm_wz_gate'] = float(ipm_wz_gate)
        if ipm_win is not None:
            extra['ipm_win'] = float(ipm_win)
        if ipm_adapt is not None:
            extra['ipm_adapt'] = float(ipm_adapt)
        if ipm_vel_tau is not None:
            extra['ipm_vel_tau'] = float(ipm_vel_tau)
        if ipm_alt_floor is not None:
            extra['ipm_alt_floor'] = float(ipm_alt_floor)
        if ipm_scale_ref is not None:
            extra['ipm_scale_ref'] = float(ipm_scale_ref)
        if ipm_acc_tau is not None:
            extra['ipm_acc_tau'] = float(ipm_acc_tau)
        self._est = FlowEstimator(fx, fy, cx, cy, R_cam_imu, rotflow_sign,
                                  roll_smooth_n=roll_smooth_n, pitch_smooth_n=pitch_smooth_n,
                                  yaw_smooth_n=yaw_smooth_n, **extra)
        self._omega = np.zeros(3)
        self._ipm_fwd = self._ipm_lat = 0.0
        self._ipm_vfwd = self._ipm_vlat = 0.0
        self._ipm_noise = (0.0, 0.0)
        self._ipm_ok = False
        self._ipm_fail = 7   # причина брака кадра IPM (коды — FlowEstimator)
        # ω НЕ «последняя пришедшая», а СРЕДНЯЯ ЗА МЕЖКАДРОВЫЙ ИНТЕРВАЛ. Оценщик
        # умножает ω на dt, то есть ему нужен угол, повёрнутый МЕЖДУ кадрами, а не
        # мгновенная скорость в момент прихода сообщения. Телеметрия при этом реже
        # кадров: замер живьём — /mavros/imu/data 12.5 Гц, /mavros/imu/data_raw
        # 20.8 Гц, камера 19-30 Гц. С «последней пришедшей» ω запаздывает до 80 мс;
        # при 5°/с это 0.4° неснятого вращения = ~2 px ложного потока на fx=480 —
        # прямо в оценку подобия опоры.
        # ГДЕ БРАТЬ ГИРОСКОП. Пробовали data_raw (RAW_IMU, 20.8 Гц против 12.5 у
        # ATTITUDE) — стало ХУЖЕ: серия K2s/K1s дала остаток D-канала 3.1-3.3 м/с
        # против офлайновых 0.8 м/с на тех же кадрах, а лучший одиночный прогон серии
        # (J4, уход 7.7 м против базовых 11.9 ± 1.4) работал как раз на data.
        # Причина: RAW_IMU — СЫРОЙ гироскоп, он несёт тряску рамы ~7.5 Гц (лимит-цикл
        # rate-loop, см. docker/sim/FAQ_rate_loop.md), и усреднение по кадру 50 мс от
        # 7.5 Гц не спасает — за интервал укладывается меньше полупериода.
        # Поэтому ω и тангаж берём из ОДНОГО отфильтрованного ATTITUDE (gyro_topic=None
        # → буфер наполняет _on_imu). Параметр оставлен: gyro_topic='/mavros/imu/data_raw'
        # возвращает прежнее поведение, если понадобится сравнить.
        self._gyro_buf = []                 # [(t, ωx, ωy, ωz)] в окне последних кадров
        self._gyro_own = False              # пришёл ли data_raw (иначе ATTITUDE-фолбэк)
        self._prev_img_stamp = None
        self._pitch = 0.0
        self._roll = 0.0
        self._att_t = None                  # штамп САМОГО сообщения ориентации
        self._att_extrap = bool(att_extrap)
        self._att_extrap_max = float(att_extrap_max)
        # интерполяция ориентации на штамп кадра (AttitudeBuffer): кадр ждёт отсчёт
        # ориентации со штампом ≥ кадр + latency, не дольше wait_max (дальше — как было:
        # последнее пришедшее ± дотяжка)
        self._att_interp = bool(att_interp)
        self._att_latency = float(att_latency)
        self._att_wait_max = float(att_wait_max)
        self._att_buf = AttitudeBuffer()
        self._pending = []                   # кадры, ждущие ориентацию: (stamp, gray)
        self._att_waited = 0                 # диагностика: кадров, обработанных по таймауту
        self._alt = None
        # ЛАТЧ НУЛЯ ВЫСОТЫ ПЕРЦЕПЦИИ (alt_zero, источник 'local'): z EKF смещён
        # вниз на 0.2-0.3 м — на низком полёте это БОЛЬШЕ всей высоты, и гейт
        # земли IPM не открывается вовсе (прогоны 183305/185921, разбор в
        # config.perc_alt_zero). Смещение постоянное, дельта точная → на арме
        # запоминаем z земли и вычитаем. latch_alt_zero() зовёт нода на фронте
        # armed; если local_position ещё молчит (GPS-denied — обычное дело),
        # латч ОТЛОЖЕН и сработает на первом же сообщении.
        # только для 'local': у 'global'/'baro' высота и так отсчитывается
        # от дома/земли, и латч там был бы вторым нулём поверх первого
        self._alt_zero_on = bool(alt_zero) and alt_src == 'local'
        self._alt_zero = 0.0
        self._alt_raw = None          # последний СЫРОЙ z (для латча)
        self._alt_zero_pending = False
        self._lateral = self._longitudinal = self._yaw = self._conf = 0.0
        self._att_yaw = 0.0
        self._kf_dx = self._kf_dy = self._kf_logs = self._kf_rot = 0.0
        self._kf_vel = 0.0
        self._kf_n = self._kf_age = self._kf_reseeds = 0
        self._kf_segs = self._kf_rejects = 0
        self._kf_valid = False
        self._dt = 0.0
        self._seq = 0
        node.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        if gyro_topic and gyro_topic != imu_topic:
            node.create_subscription(Imu, gyro_topic,
                                     lambda m: self._on_gyro(m, own=True),
                                     qos_profile_sensor_data)
        # Высота для масштаба IPM/гейтов. Она входит в IPM НА КАЖДОМ КАДРЕ
        # (позиция адаптивной полосы + масштаб ректификации), поэтому критична
        # МЕЖКАДРОВАЯ ПРОИЗВОДНАЯ шума (скачок alt между кадрами = «зум»
        # ректификации = фантомная скорость ∝ Δalt/dt) и ЛАГ (на наборе врут
        # масштаб и полоса — улёты 4 прогонов 2026-08-19 на баро). Замер бок о
        # бок (bag lv2_replay_20260824_062957, п95 межкадровой |Δ| на сетке
        # 30 Гц / σ шума в ховере / лаг):
        #   global (EKF)  6.5 см / 4.2 см / ~0.03 с
        #   local_z (EKF) 7.6 см / 3.5 см / +0.03 с
        #   baro_raw     25.6 см / 11 см  / +0.10 с  ← фантомы производной
        #   baro_ema      5.2 см / 5.1 см / ~0.35 с  ← лаг EMA, срыв на наборе
        # 'global' — rel_alt из GLOBAL_POSITION_INT (жив, пока EKF с origin
        # публикует global; при чистой потере aiding замерзает); 'local' —
        # z из /mavros/local_position/pose: тот же EKF-канал (баро-POSZ +
        # IMU-предсказание: гладко И без лага), но живёт без GPS и без global —
        # профиль LV=2/боевой борт; вертикаль EKF переживает смерть VINS
        # (баро-фьюжн продолжается). 'baro' — BaroAlt (сырой баро + EMA):
        # полностью независим от EKF, но лаг 0.35 с — только для экспериментов.
        if alt_src == 'baro':
            from .baro_alt import BaroAlt
            self._baro = BaroAlt(node, self._set_alt)
        elif alt_src == 'local':
            from geometry_msgs.msg import PoseStamped
            node.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                     self._on_lpos_alt, qos_profile_sensor_data)
        else:
            node.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                     self._on_alt, qos_profile_sensor_data)
        node.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)

    def _on_imu(self, m):
        # ATTITUDE — только ФОЛБЭК: пока data_raw молчит, ω берём отсюда. Смешивать
        # два источника в одном буфере нельзя — у них разный темп, и среднее за кадр
        # перекосило бы в сторону того, кто чаще.
        if not self._gyro_own:
            self._on_gyro(m)
        # тангаж из ориентации того же сообщения — для компенсации наклона камеры
        # (на борту это оценка FCU; её дрейф 0.19°/с за секундное окно даёт 0.2°,
        # что мало против рабочих ±10°)
        q = m.orientation
        self._pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        # крен — только для канала вида сверху (выпрямление полосы земли)
        self._roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                                1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        # ENU-курс — для поворота body→ENU при отдаче скорости IPM в EKF
        self._att_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._att_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self._att_buf.push(self._att_t, self._pitch, self._roll)
        if self._att_interp and self._pending:
            self._flush_pending()

    def _on_gyro(self, m, own=False):
        if own and not self._gyro_own:
            self._gyro_own = True
            self._gyro_buf = []             # выкидываем фолбэк-сэмплы, начинаем чисто
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        w = m.angular_velocity
        self._gyro_buf.append((t, w.x, w.y, w.z))
        if len(self._gyro_buf) > 120:        # ~6 с при 20 Гц — с запасом на кадр
            del self._gyro_buf[:-120]

    def _omega_for(self, stamp):
        """Средняя ω за интервал (прошлый кадр, этот кадр]. Нет попавших сэмплов —
        берём ближайший по времени (лучше, чем последний пришедший: он в прошлом)."""
        if not self._gyro_buf:
            return self._omega
        arr = np.asarray(self._gyro_buf)
        t0 = self._prev_img_stamp
        if t0 is not None:
            sel = (arr[:, 0] > t0) & (arr[:, 0] <= stamp)
            if sel.sum():
                return arr[sel, 1:4].mean(axis=0)
        i = int(np.argmin(np.abs(arr[:, 0] - stamp)))
        return arr[i, 1:4]

    def _att_for(self, stamp):
        """Ориентация на штамп кадра. Логика — в `attitude_at` (см. её док):
        вынесена функцией модуля, чтобы офлайн-стенд `att_extrap_test.py` гонял
        РОВНО тот же код, а не свою копию."""
        return attitude_at(self._pitch, self._roll, self._att_t, self._gyro_buf,
                           stamp, self._att_extrap, self._att_extrap_max)

    def _on_alt(self, m):
        self._alt = float(m.data)

    def _on_lpos_alt(self, m):
        # EKF local z ≈ высота над точкой арма (origin EKF); на ровной сцене
        # эквивалент rel_alt. Отрицательный дребезг у земли клампим нулём.
        z = float(m.pose.position.z)
        self._alt_raw = z
        if self._alt_zero_pending:            # латч ждал первого сообщения
            self._alt_zero = z
            self._alt_zero_pending = False
        self._alt = max(0.0, z - self._alt_zero)

    def latch_alt_zero(self):
        """Запомнить текущую высоту как НОЛЬ земли (зовётся на фронте armed).

        Возвращает залатченный z (м) или None, если сырой высоты ещё не было —
        тогда латч отложен до первого сообщения. Выключенный alt_zero и любой
        источник кроме 'local' — no-op (там высота и так от дома/земли)."""
        if not self._alt_zero_on:
            return None
        if self._alt_raw is None:
            self._alt_zero_pending = True
            return None
        self._alt_zero = self._alt_raw
        self._alt_zero_pending = False
        self._alt = max(0.0, self._alt_raw - self._alt_zero)
        return self._alt_zero

    def _set_alt(self, alt):
        self._alt = float(alt)

    def _on_image(self, m):
        if m.encoding not in ('mono8', '8UC1'):
            return
        gray = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self._att_interp:
            pitch, roll = self._att_for(stamp)
            self._process(gray, stamp, pitch, roll)
            return
        self._pending.append((stamp, gray))
        self._flush_pending()

    def _flush_pending(self):
        """Обработать по порядку кадры, для которых уже есть ориентация со штампом
        ≥ кадр + latency (интерполяция); кадры старше wait_max (относительно самого
        свежего кадра) или лишние в очереди — по прежнему правилу (удержание/дотяжка)."""
        newest = self._pending[-1][0] if self._pending else None
        while self._pending:
            stamp, gray = self._pending[0]
            tq = stamp + self._att_latency
            if self._att_buf.ready(tq):
                pitch, roll = self._att_buf.at(tq)
            elif newest - stamp > self._att_wait_max or len(self._pending) > 5:
                pitch, roll = self._att_for(stamp)
                self._att_waited += 1
            else:
                break
            self._pending.pop(0)
            self._process(gray, stamp, pitch, roll)

    def _process(self, gray, stamp, pitch, roll):
        omega = self._omega_for(stamp)
        self._prev_img_stamp = stamp
        res = self._est.process(gray, stamp, omega, pitch, self._alt,
                                roll=roll)
        if res is None:
            return
        self._lateral = res['lateral']
        self._longitudinal = res['longitudinal']   # looming → DpPitchHold
        self._yaw = res['yaw_flow']
        self._conf = res['conf']
        # ОПОРА: смещение картинки от опорного кадра (не скорость)
        self._kf_dx, self._kf_dy = res['kf_dx'], res['kf_dy']
        self._kf_logs, self._kf_rot = res['kf_logs'], res['kf_rot']
        self._kf_vel = res['kf_vel']
        self._kf_n, self._kf_age = res['kf_n'], res['kf_age']
        self._kf_reseeds, self._kf_valid = res['kf_reseeds'], res['kf_valid']
        self._kf_segs, self._kf_rejects = res['kf_segs'], res['kf_rejects']
        self._ipm_fwd, self._ipm_lat = res['ipm_fwd'], res['ipm_lat']
        self._ipm_vfwd, self._ipm_vlat = res['ipm_vfwd'], res['ipm_vlat']
        self._ipm_noise = (res.get('ipm_noise_fwd', 0.0), res.get('ipm_noise_lat', 0.0))
        self._ipm_ok = res['ipm_ok']
        self._ipm_fail = res['ipm_fail']
        self._dt = res['dt']
        self._seq += 1               # НОВЫЙ кадр → домен продвинет PID

    def merge(self, s):
        """Влить свежие агрегаты потока в снапшот телеметрии (как pilot_* в ноде)."""
        s.flow_lateral = self._lateral
        s.flow_longitudinal = self._longitudinal
        s.flow_yaw = self._yaw
        s.flow_conf = self._conf
        s.flow_dt = self._dt
        s.flow_seq = self._seq
        s.kf_dx, s.kf_dy = self._kf_dx, self._kf_dy
        s.kf_logs, s.kf_rot = self._kf_logs, self._kf_rot
        s.kf_vel = self._kf_vel
        s.kf_n, s.kf_age = self._kf_n, self._kf_age
        s.kf_reseeds, s.kf_valid = self._kf_reseeds, self._kf_valid
        s.kf_segs, s.kf_rejects = self._kf_segs, self._kf_rejects
        s.ipm_fwd, s.ipm_lat = self._ipm_fwd, self._ipm_lat
        s.ipm_vfwd, s.ipm_vlat = self._ipm_vfwd, self._ipm_vlat
        s.ipm_noise_fwd, s.ipm_noise_lat = self._ipm_noise
        s.ipm_ok = self._ipm_ok
        s.ipm_fail = self._ipm_fail
        s.perc_alt = self._alt        # по НЕЙ судит гейт земли IPM (не rel_alt)
        s.att_yaw = self._att_yaw
        return s

    def reset_keyframe(self):
        """Назначить точку удержания: следующий кадр станет опорным."""
        self._est.reset_keyframe()
