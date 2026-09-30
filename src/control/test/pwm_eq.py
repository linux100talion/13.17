#!/usr/bin/env python3
"""Тестам: ручки стабилизаторов в PWM-ЭКВИВАЛЕНТЕ → СИ (перевод домена, 2026-09-30).

Ожидания тестов посчитаны в µs (выход носителя RcCommand и тримы в валюте каналов — µs),
поэтому и гейны удобнее писать в той же валюте: kp 40 PWM на м/с = 1.6 °/(м/с) =
tilt_from_us(40) рад на м/с. `si(**kw)` переводит известные ключи наклона (kp/ki/imax/
max/ff…) и переименовывает max_pwm → max_tilt (DpVins); остальные проходят как есть.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.units import tilt_from_us, us_from_tilt   # noqa: E402,F401

# DpVins: kp_fwd/kp_lat/ki/ki_trim/imax/ff; оси демпфера IPM: kp/ki/kd/ki_trim/imax/max_out;
# VinsHold/GzHold: kp/kd/ki/imax (+ max_pwm → max_tilt)
TILT_KEYS = ('kp_fwd', 'kp_lat', 'ki', 'ki_trim', 'imax', 'ff', 'kp', 'kd', 'max_out')


def si(**kw):
    out = {}
    for k, v in kw.items():
        if k == 'max_pwm':
            out['max_tilt'] = tilt_from_us(v)
        elif k in TILT_KEYS:
            out[k] = tilt_from_us(v)
        else:
            out[k] = v
    return out


def us(rad):
    """Наклон/трим, рад → µs (для ожиданий в PWM-эквиваленте)."""
    return us_from_tilt(rad)


# ── носитель команды в СИ (фаза A, 2026-09-30) ────────────────────────────────────────
# Выход стабилизаторов/стека/шагов/арбитра — AttitudeCommand (СИ). Ожидания тестов в µs:
# rc_of() переводит выход в RcCommand тем же путём, что нода отдаёт в провод
# (command_wire.to_rc, эталонная карта газа); не-команды проходят как есть (обёртка
# безопасна вокруг любого .update()/.tick()). cmd_pwm() — заглушкам и входам тестов:
# PWM каналов → команда домена.
from control_pkg.application.command_wire import from_rc, to_rc   # noqa: E402
from control_pkg.domain.attitude import AttitudeCommand             # noqa: E402
from control_pkg.domain.rc import RC_CENTER                         # noqa: E402


def rc_of(x):
    if isinstance(x, AttitudeCommand):
        return to_rc(x)
    inner = getattr(x, "rc", None)            # StepResult: команда шага → µs на месте
    if isinstance(inner, AttitudeCommand):
        x.rc = to_rc(inner)
    return x


def cmd_pwm(roll=RC_CENTER, pitch=RC_CENTER, throttle=RC_CENTER, yaw=RC_CENTER):
    return from_rc(roll, pitch, throttle, yaw)
