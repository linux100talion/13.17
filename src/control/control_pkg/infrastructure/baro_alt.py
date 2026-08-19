#!/usr/bin/env python3
"""BaroAlt — относительная высота из СЫРОГО барометра (GPS-независимая).

Зачем: /mavros/global_position/rel_alt питается GLOBAL_POSITION_INT и при
отсутствии GPS замерзает (GPS-denied прогон 2026-08-19: rel_alt застрял на
0.02 при реальном взлёте на 3 м → CLIMB_FAIL + гейт перцепции «на земле» →
vision-фид слал нули в полёте → EKF разнесло). На боевом Orin GPS нет вообще —
там баро единственный источник относительной высоты для миссии/перцепции.

Базлайн: медиана первых WARMUP давлений — нода ВСЕГДА стартует на земле
(стенд: стек поднимается до арма; борт: питание подаётся на земле).
Дрейф баро за минуты полёта — доли метра, для гейтов и climb-контроля хватает.

Источник: /mavros/imu/static_pressure (SCALED_PRESSURE, stream RAW_SENSORS —
уже запрошен nav_up.sh, ~80 Гц, живёт без GPS). Па → метры барометрической
формулой (изотермическое приближение, как в AP_Baro).
"""
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure

WARMUP = 50          # сэмплов на базлайн (~0.6 с при 80 Гц)
_EXP = 0.190295      # показатель барометрической формулы
# EMA-сглаживание: шум давления SITL ~2.8 Па ≈ ±0.23 м дрожания высоты на 80 Гц.
# Сырой сигнал валит контур Climb (alt_hold тормозит по производной rel_alt:
# прогон 2026-08-19 — подскок до 1.6 м и просадка в дизарм). ALPHA=0.05 при
# 80 Гц ≈ постоянная времени 0.25 с — шум ×0.15, лаг сопоставим с EKF-каналом.
ALPHA = 0.05


class BaroAlt:
    """Подписка + пересчёт; каждое обновление отдаёт сглаженный rel_alt (м) в on_alt."""

    def __init__(self, node, on_alt):
        self._on_alt = on_alt
        self._warm = []
        self._p0 = None
        self._alt_f = None
        node.create_subscription(FluidPressure, '/mavros/imu/static_pressure',
                                 self._on_pressure, qos_profile_sensor_data)

    def _on_pressure(self, m):
        p = float(m.fluid_pressure)          # Па
        if p <= 0.0:
            return
        if self._p0 is None:
            self._warm.append(p)
            if len(self._warm) >= WARMUP:
                s = sorted(self._warm)
                self._p0 = s[len(s) // 2]
                self._warm = None
            return
        alt = 44330.0 * (1.0 - (p / self._p0) ** _EXP)
        self._alt_f = alt if self._alt_f is None else \
            (1.0 - ALPHA) * self._alt_f + ALPHA * alt
        self._on_alt(self._alt_f)
