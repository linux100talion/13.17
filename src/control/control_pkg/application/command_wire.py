#!/usr/bin/env python3
"""command_wire — AttitudeCommand (СИ) ↔ RcCommand (µs каналов): единственный переход домена
в провод override и обратно (носитель в СИ, фазы A/B 2026-09-30).

Туда (to_rc): наклон и курс — фиксированным масштабом domain/units.py (20° и 90 °/с на
500 µs, усечение к целому µs, как прежнее int() стабилизаторов), газ — обратной картой ALT_HOLD
полётника ThrottleMap (domain/control/altitude.py, по его параметрам — фаза B). Override шлёт
эти µs (реверс каналов — в адаптере); канал углов берёт команду напрямую
(att_convert.cmd_to_attitude: climb прямо в thrust). PWM-эквивалент — ещё и отладке/статусу.

Обратно (from_rc): стики пилота (DroneState.pilot_*, сырой PWM входа) → команда домена —
арбитр в MANUAL, проход стиков шагами плана (Freefly до арма, LOITER, отмены возврата).
Круг pwm → СИ → pwm точен для наклона и курса; газ — с точностью до µs вне мёртвой зоны
THR_DZ полётника (внутри — центр: полётник там всё равно держит высоту сам).
"""
from ..domain.attitude import AttitudeCommand
from ..domain.control.altitude import ThrottleMap
from ..domain.rc import RC_CENTER, RcCommand
from ..domain.units import rc_off_tilt, rc_off_yaw, tilt_of_pwm, yaw_of_pwm

DEFAULT_THR = ThrottleMap()


def to_rc(cmd: AttitudeCommand, thr: ThrottleMap = DEFAULT_THR) -> RcCommand:
    return RcCommand(roll=RC_CENTER + rc_off_tilt(cmd.roll),
                     pitch=RC_CENTER + rc_off_tilt(cmd.pitch),
                     throttle=thr.pwm(cmd.climb),
                     yaw=RC_CENTER + rc_off_yaw(cmd.yaw_rate))


def from_rc(roll, pitch, throttle, yaw, thr: ThrottleMap = DEFAULT_THR) -> AttitudeCommand:
    """Сырые PWM стиков (roll, pitch, throttle, yaw) → команда домена."""
    return AttitudeCommand(roll=tilt_of_pwm(roll), pitch=tilt_of_pwm(pitch),
                           yaw_rate=yaw_of_pwm(yaw), climb=thr.climb(throttle))


def pilot_cmd(s, thr: ThrottleMap = DEFAULT_THR) -> AttitudeCommand:
    """Все четыре стика пилота как есть → команда домена."""
    return from_rc(s.pilot_roll, s.pilot_pitch, s.pilot_throttle, s.pilot_yaw, thr)
