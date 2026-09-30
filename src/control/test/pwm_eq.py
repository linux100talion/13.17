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
