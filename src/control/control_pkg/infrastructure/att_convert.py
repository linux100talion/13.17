#!/usr/bin/env python3
"""RcCommand (PWM) → AttitudeCommand (углы, СИ) и скорость набора → thrust — чистое ядро.

ЗАЧЕМ. Шаг 1 переезда на углы (laptop_move.md §5.7): домен и гейны остаются в PWM, а выход
идёт в полётник не override'ом, а SET_ATTITUDE_TARGET (GUIDED_NOGPS). Чтобы A/B «override
против углов» что-то доказывал, пересчёт обязан давать ТОТ ЖЕ угол, темп и набор, что
ALT_HOLD делал из нашего override. Формулы — по исходникам ArduCopter (образ
sim-simulator, a824813), параметры — прочитанные у полётника:

  крен/тангаж  форма ALT_HOLD (rc_input_to_roll_pitch_rad, AP_Math/control.cpp:856): вектор
               горизонтальной тяги, тангаж = A·norm_p, крен = atan(cos(тангаж)·tan(A·norm_r)),
               но МАСШТАБ ФИКСИРОВАН: norm = (pwm − 1500)/500, A = ANGLE_SPAN_DEG (20°) —
               0.040 °/µs, не ATC_ANGLE_MAX/ход стика полётника (решение 2026-09-30, см. ниже).
  курс         YAW_SPAN_DPS (90 °/с) на 500 µs — тоже фиксировано, не PILOT_Y_RATE.
  газ          Copter::get_pilot_desired_climb_rate_ms (Attitude.cpp): control = pwm_to_range
               (RC3, с мёртвой зоной RC3_DZ у низа), центр = get_control_mid, мёртвая зона
               THR_DZ вокруг центра, вверх PILOT_SPD_UP, вниз PILOT_SPD_DN (0 = как вверх).
  thrust       GCS_MAVLink_Copter.cpp (SET_ATTITUDE_TARGET, GUID_OPTIONS бит 3 = 0):
               0.5 = висеть, 1.0 = +WP_SPD_UP, 0.0 = −WP_SPD_DN, линейно.

ЕДИНИЦЫ НЕ ЗАВИСЯТ ОТ БОРТА (решение пилота 2026-09-30, вариант A). Масштаб PWM → угол —
константа ядра, а не параметры полётника: одинаковый PWM домена даёт одинаковый угол в симе
и на борту, гейн переносится как есть. 20°/500 µs = как сим сегодня (ATC_ANGLE_MAX 20) и почти
как борт на override (20°/491 µs); курс 90 °/с = PILOT_Y_RATE борта. Параметры полётника здесь
только ПОТОЛОК наклона (ATC_ANGLE_MAX) и газ (ниже). Полный перевод домена в СИ (гейны в
рад/(м/с)) — позже, делением на тот же коэффициент.

СОЗНАТЕЛЬНЫЕ ОТЛИЧИЯ от ALT_HOLD (решение 2026-09-30): на крен/тангаж/курс НЕ накладываются
мёртвая зона RCn_DZ, реверс RCn_REVERSED и expo PILOT_Y_EXPO. Это обработка СТИКА ПИЛОТА, а
выход домена — команда стабилизатора: мёртвая зона съедала его поправки у центра (на борту
DZ 20), реверс в канал углов не входит вовсе (SET_ATTITUDE_TARGET идёт мимо обработки
пульта — зеркалить тангаж здесь НЕЛЬЗЯ). Их место — на ВХОДЕ, при чтении стиков пилота. Газ
пока берётся по параметрам полётника с мёртвой зоной THR_DZ: в него идёт только стик пилота
(или защёлка) — это вход, а не поправка стабилизатора.

ПОТОЛОК. Полётник к углам из SET_ATTITUDE_TARGET ATC_ANGLE_MAX НЕ применяет (разведка
2026-09-30: в angle_control_run/input_quaternion нет клампа наклона) — потолок держит это
ядро тем же ограничением вектора тяги, что ALT_HOLD.
"""
import math

from ..domain.attitude import AttitudeCommand

# что нужно прочитать у полётника; RC3_DZ — мёртвая зона у НИЗА канала газа (pwm_to_range)
PARAMS = ('ATC_ANGLE_MAX', 'RC3_MIN', 'RC3_MAX', 'RC3_DZ',
          'THR_DZ', 'PILOT_SPD_UP', 'PILOT_SPD_DN', 'WP_SPD_UP', 'WP_SPD_DN')

# ФИКСИРОВАННЫЙ МАСШТАБ домена (µs от центра 1500 → угол/темп), вариант A 2026-09-30 —
# одно число на весь домен, живёт в domain/units.py (им же стабилизаторы переводят СИ в µs)
from ..domain.units import ANGLE_SPAN_DEG, PWM_SPAN, YAW_SPAN_DPS   # noqa: E402,F401
PWM_CENTER = 1500


class FcuStickParams:
    """Кэш параметров полётника для пересчёта (заполняется из событий/опроса MAVROS)."""

    def __init__(self):
        self.p = {}

    def on_param(self, name: str, value) -> bool:
        """True — наш параметр, и значение изменилось."""
        if name not in PARAMS:
            return False
        v = float(value)
        changed = self.p.get(name) != v
        self.p[name] = v
        return changed

    def missing(self):
        return [n for n in PARAMS if n not in self.p]

    def ready(self) -> bool:
        return not self.missing()

    def describe(self) -> str:
        p = self.p
        return (f"масштаб {ANGLE_SPAN_DEG:g}°/{YAW_SPAN_DPS:g} °/с на {PWM_SPAN:g} µs, "
                f"потолок {p['ATC_ANGLE_MAX']:g}°, "
                f"набор ↑{p['PILOT_SPD_UP']:g}/↓{_spd_dn(p):g} м/с (THR_DZ {p['THR_DZ']:g}), "
                f"thrust ↑{p['WP_SPD_UP']:g}/↓{p['WP_SPD_DN']:g} м/с")


def _norm(pwm) -> float:
    """PWM домена → −1..+1 по ФИКСИРОВАННОМУ масштабу (без мёртвой зоны и реверса)."""
    return max(-1.0, min(1.0, (pwm - PWM_CENTER) / PWM_SPAN))


def _spd_dn(p) -> float:
    return p['PILOT_SPD_DN'] if p['PILOT_SPD_DN'] > 0 else p['PILOT_SPD_UP']


def climb_from_throttle(pwm, p) -> float:
    """Copter::get_pilot_desired_climb_rate_ms: PWM газа → м/с (вверх +)."""
    lo, hi, dz3 = p['RC3_MIN'], p['RC3_MAX'], p['RC3_DZ']
    low = lo + dz3
    r_in = max(lo, min(hi, pwm))
    ctrl = 1000.0 * (r_in - low) / (hi - low) if r_in > low else 0.0
    ctrl = max(0.0, min(1000.0, ctrl))
    r_mid, lo_i = int(lo + hi) // 2, int(low)                 # get_control_mid: int16, как в C
    mid = float(int(1000 * (r_mid - lo_i) / (int(hi) - lo_i)))
    dz = max(0.0, min(400.0, p['THR_DZ']))
    top, bot = mid + dz, mid - dz
    if ctrl < bot:
        return _spd_dn(p) * (ctrl - bot) / bot
    if ctrl > top:
        return p['PILOT_SPD_UP'] * (ctrl - top) / (1000.0 - top)
    return 0.0


def thrust_from_climb(climb, p) -> float:
    """Скорость набора → поле thrust SET_ATTITUDE_TARGET (GUID_OPTIONS бит 3 = 0)."""
    if climb >= 0:
        t = 0.5 + 0.5 * climb / p['WP_SPD_UP'] if p['WP_SPD_UP'] > 0 else 0.5
    else:
        t = 0.5 + 0.5 * climb / p['WP_SPD_DN'] if p['WP_SPD_DN'] > 0 else 0.5
    return max(0.0, min(1.0, t))


def rc_to_attitude(rc, p) -> AttitudeCommand:
    """RcCommand (стандартная конвенция ArduPilot, как при RCn_REVERSED 0) → углы.
    Масштаб фиксирован (ANGLE_SPAN_DEG/YAW_SPAN_DPS), потолок — ATC_ANGLE_MAX."""
    a = math.radians(ANGLE_SPAN_DEG)
    nr, np_, ny = _norm(rc.roll), _norm(rc.pitch), _norm(rc.yaw)
    # форма rc_input_to_roll_pitch_rad: вектор горизонтальной тяги
    pitch = a * np_
    roll = math.atan(math.cos(pitch) * math.tan(a * nr))
    cmd = AttitudeCommand(roll=roll, pitch=pitch,
                          yaw_rate=math.radians(YAW_SPAN_DPS) * ny,
                          climb=climb_from_throttle(rc.throttle, p))
    return limit_tilt(cmd, p)


def limit_tilt(cmd, p) -> AttitudeCommand:
    """Потолок наклона ATC_ANGLE_MAX для ЛЮБОЙ команды углом (полётник его к
    SET_ATTITUDE_TARGET не применяет): тот же предел вектора горизонтальной тяги, что
    в rc_input_to_roll_pitch_rad, — направление сохраняется, длина режется."""
    a = math.radians(min(p['ATC_ANGLE_MAX'], 85.0))
    tx = -math.tan(cmd.pitch)
    ty = math.tan(cmd.roll) / max(math.cos(cmd.pitch), 1e-6)
    lim = math.tan(a)
    n = math.hypot(tx, ty)
    if n <= lim:
        return cmd
    tx, ty = tx * lim / n, ty * lim / n
    pitch = -math.atan(tx)
    return AttitudeCommand(roll=math.atan(math.cos(pitch) * ty), pitch=pitch,
                           yaw_rate=cmd.yaw_rate, climb=cmd.climb)
