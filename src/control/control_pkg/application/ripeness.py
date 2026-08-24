#!/usr/bin/env python3
"""VinsRipeness — детектор зрелости VINS по самой одометрии + баро (2-я ступень
гейта зрелости; 1-я ступень — время потока ripe_sec, страховка плохого дня).

Два независимых признака (замер по bag'ам lv2_replay 041803/050600, где есть
истина Gazebo для валидации):

1. RESIDUAL «поза против скорости» |Δpose/Δt − twist| — пока солвер перекраивает
   скользящее окно, публикуемая поза «переписывает историю» несогласованно с
   публикуемой скоростью: в первом 2-с окне после init 0.5-0.7 м/с, после
   успокоения — пол шума 0.05-0.10 и на манёврах не растёт. Порог по EMA
   (сглаживание одиночных выбросов пар), «тихо» = ниже порога quiet_sec подряд.
   Слепое пятно: НЕ ловит масштаб-ошибку, которую окно уже узаконило, — для
   масштаба второй признак.

2. Вертикальный RATIO (pos.z − z0)/(ref_alt − ref0) — метрический тест масштаба:
   референс высоты (баро при alt_src=baro; global на GPS-профилях) метричен, и
   набор высоты сразу после init (init на отрыве, климб ~5 м) даёт сигнал.
   Оценивается, когда референс ушёл от точки init на ratio_min_dz и больше;
   защёлкивается ПОСЛЕДОВАТЕЛЬНЫМИ попаданиями в полосу (ratio_n подряд) —
   одиночное чтение на 1.5 м при шуме баро ~0.3 м даёт до 20% ошибки.

ready = ratio защёлкнут И residual тих quiet_sec подряд. Порог полосы широкий
([0.8, 1.25]): ловим кратные ошибки масштаба (×10 полёта №3 эпохи сломанного
солвера), а не проценты. Разрыв потока >1 с сбрасывает пару и тишину (рестарт
VINS начинает тишину заново; ratio-защёлку не трогаем — масштаб после рестарта
проверит уже гейт свежести/времени).

Чистый python без ROS: кормится RosTelemetry._on_odom, тестируется оффлайн
(test_vins_ripeness.py) — тот же паттерн, что hud_status/handover.
"""


class VinsRipeness:
    def __init__(self, quiet_res: float = 0.15, quiet_sec: float = 4.0,
                 ratio_lo: float = 0.8, ratio_hi: float = 1.25,
                 ratio_min_dz: float = 1.5, ratio_n: int = 3,
                 ema_a: float = 0.3):
        self.quiet_res = quiet_res
        self.quiet_sec = quiet_sec
        self.ratio_lo = ratio_lo
        self.ratio_hi = ratio_hi
        self.ratio_min_dz = ratio_min_dz
        self.ratio_n = ratio_n
        self.ema_a = ema_a
        self._prev = None             # (t, pos) прошлой одометрии
        self._quiet_since = None      # старт непрерывной тишины residual
        self._z0 = None               # z VINS на первой одометрии
        self._ref0 = None             # референс-высота на первой одометрии
        self._ratio_hits = 0          # попаданий в полосу подряд
        self.res = None               # EMA residual, м/с (None до 2-й одометрии)
        self.ratio = None             # последний вертикальный ratio (None до dz)
        self.ratio_ok = False         # защёлка масштаба
        self._t = None

    def on_odom(self, t: float, pos, vel, ref_alt) -> None:
        """t — часы вызывающего (sim); pos=(x,y,z) и vel=(vx,vy,vz) — поза и
        twist одометрии (мировой фрейм VINS); ref_alt — метрическая высота
        (rel_alt снапшота) или None."""
        self._t = t
        if self._z0 is None:
            self._z0 = pos[2]
            self._ref0 = ref_alt
        elif self._ref0 is None:
            self._ref0 = ref_alt      # референс мог опоздать к первой одометрии
        if self._prev is not None:
            dt = t - self._prev[0]
            if 0.0 < dt <= 1.0:
                fd = [(b - a) / dt for a, b in zip(self._prev[1], pos)]
                r = sum((f - v) ** 2 for f, v in zip(fd, vel)) ** 0.5
                self.res = r if self.res is None else \
                    (1.0 - self.ema_a) * self.res + self.ema_a * r
                if self.res >= self.quiet_res:
                    self._quiet_since = None
                elif self._quiet_since is None:
                    self._quiet_since = t
            else:                     # разрыв/рестарт потока — тишина заново
                self._quiet_since = None
        self._prev = (t, tuple(pos))
        if (not self.ratio_ok and self._ref0 is not None
                and ref_alt is not None):
            dz_ref = ref_alt - self._ref0
            if abs(dz_ref) >= self.ratio_min_dz:
                self.ratio = (pos[2] - self._z0) / dz_ref
                if self.ratio_lo <= self.ratio <= self.ratio_hi:
                    self._ratio_hits += 1
                    if self._ratio_hits >= self.ratio_n:
                        self.ratio_ok = True
                else:
                    self._ratio_hits = 0

    @property
    def ready(self) -> bool:
        return (self.ratio_ok and self._quiet_since is not None
                and self._t is not None
                and self._t - self._quiet_since >= self.quiet_sec)
