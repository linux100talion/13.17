#!/usr/bin/env python3
"""AltHold — внешний контур ВЫСОТЫ: ошибка по баро → скорость набора, м/с (носитель — climb).

Зачем он появился (замер J1b, «висение на 3 м»):
  - шаг набора выходил, как только баро показало цель, и отдавал throttle в центр.
    Вертикальная скорость на этом моменте +1.58 м/с, а ALT_HOLD гасит её ~2 с →
    ПЕРЕЛЁТ 3.0 → 5.2 м. То есть висели на 5 м вместо 3 всегда, без исключений;
  - дальше борт болтало 4.7..6.2 м (±0.7 м) вокруг этой точки.
Опорный кадр зрения при этом живёт только на постоянной высоте: затвор
`kf_alt_max=0.06` при 5 м = ±0.30 м, то есть ТУЖЕ собственной болтанки, и он
срабатывал каждые 2.6 с, каждый раз выбрасывая накопленное смещение (22 выброса
из 25 пересевов в висении). Пока высота не держится, счисление положения по зрению
не имеет смысла — отсюда этот контур.

Каскад (внутренний контур vz — в ArduPilot, мы даём ему уставку):
  err = target − rel_alt  [м]
    → climb = clamp(kp·err, ±rate_max)  [м/с] — это и есть команда (AttitudeCommand.climb)
В провод (фаза B переезда носителя, 2026-09-30): канал углов — climb прямо в thrust; override
— ThrottleMap ниже, обратной картой ALT_HOLD САМОГО ПОЛЁТНИКА (по его параметрам), поэтому
полётник исполняет ровно ту скорость, что просил контур.

До фазы B газ считался своей картой (мёртвая зона 100, 3.16 м/с на полный ход — замер сима
на throttle 1800 → +1.58 м/с): она расходилась с картой полётника (1.2 м/с команды → ~1.36 у
полётника), и канал углов шёл по третьей. Теперь карта одна — полётника.

Высота берётся ИЗ БАРО (`rel_alt`) — на реальном борту GPS нет, а баро есть; тот же
источник, что у затвора опоры (`flow_estimator.kf_alt_max`), поэтому контур и
затвор видят одну и ту же высоту, а не расходятся.
"""

from ..rc import RC_CENTER, clamp

# Параметры ALT_HOLD полётника, из которых строится карта газа (читает адаптер MAVROS).
PARAMS = ('RC3_MIN', 'RC3_MAX', 'RC3_DZ', 'THR_DZ', 'PILOT_SPD_UP', 'PILOT_SPD_DN')
# Значения SITL сима (прочитаны у полётника 2026-09-30) — стендам, тестам и отладочному
# PWM-эквиваленту, пока параметры борта не прочитаны. Провод без прочитанных НЕ работает.
SIM_DEFAULTS = {'RC3_MIN': 1100.0, 'RC3_MAX': 1900.0, 'RC3_DZ': 30.0, 'THR_DZ': 100.0,
                'PILOT_SPD_UP': 2.5, 'PILOT_SPD_DN': 0.0}
# «Газ в пол» (арм, касание): скорость заведомо ниже любой PILOT_SPD_DN — pwm() даёт RC3_MIN,
# thrust канала углов — 0.
CLIMB_FLOOR = -100.0


class ThrottleMap:
    """Газ канала, µs ↔ скорость набора, м/с — карта ALT_HOLD ПОЛЁТНИКА
    (Copter::get_pilot_desired_climb_rate_ms по его параметрам RC3_*, THR_DZ, PILOT_SPD_*).

    climb(pwm) — прямая (стик пилота на входе, константы газа плана); pwm(climb) — обратная
    (провод override: полётник, прочитав этот газ, исполнит ровно climb с точностью до µs).
    params — ЖИВОЙ словарь (адаптер MAVROS дописывает прочитанные значения); недостающие
    берутся из SIM_DEFAULTS, ready() — все прочитаны."""

    def __init__(self, params=None):
        self.params = params if params is not None else {}

    def ready(self) -> bool:
        return all(k in self.params for k in PARAMS)

    def _p(self):
        return {**SIM_DEFAULTS, **self.params}

    @staticmethod
    def _geom(p):
        lo, hi = p['RC3_MIN'], p['RC3_MAX']
        low = lo + p['RC3_DZ']
        r_mid, lo_i = int(lo + hi) // 2, int(low)              # get_control_mid: int16, как в C
        mid = float(int(1000 * (r_mid - lo_i) / (int(hi) - lo_i)))
        dz = max(0.0, min(400.0, p['THR_DZ']))
        up = p['PILOT_SPD_UP']
        dn = p['PILOT_SPD_DN'] if p['PILOT_SPD_DN'] > 0 else up
        return lo, hi, low, mid + dz, mid - dz, up, dn

    def climb(self, pwm) -> float:
        """µs газа → м/с (вверх +): pwm_to_range с мёртвой зоной RC3_DZ у низа, зона THR_DZ
        вокруг середины, вверх PILOT_SPD_UP, вниз PILOT_SPD_DN (0 = как вверх)."""
        lo, hi, low, top, bot, up, dn = self._geom(self._p())
        r_in = max(lo, min(hi, pwm))
        ctrl = 1000.0 * (r_in - low) / (hi - low) if r_in > low else 0.0
        ctrl = max(0.0, min(1000.0, ctrl))
        if ctrl < bot:
            return dn * (ctrl - bot) / bot
        if ctrl > top:
            return up * (ctrl - top) / (1000.0 - top)
        return 0.0

    def pwm(self, climb) -> int:
        """м/с → µs газа (обратная к climb, округление до µs): 0 = центр 1500 (в зоне THR_DZ),
        ниже −PILOT_SPD_DN — RC3_MIN (газ в пол), выше PILOT_SPD_UP — RC3_MAX."""
        if climb == 0.0:
            return RC_CENTER
        lo, hi, low, top, bot, up, dn = self._geom(self._p())
        if climb > 0:
            ctrl = top + climb / up * (1000.0 - top)
            if ctrl >= 1000.0:
                return int(hi)
        else:
            ctrl = bot + climb / dn * bot
            if ctrl <= 0.0:
                return int(lo)
        return int(round(low + ctrl * (hi - low) / 1000.0))


class AltHold:
    def __init__(self, kp=0.6, rate_max=1.2, tol=0.10):
        self.kp = float(kp)                  # м/с на метр ошибки
        self.rate_max = float(rate_max)      # потолок командной vz, м/с
        self.tol = float(tol)                # мёртвая зона ПО ОШИБКЕ, м
        self.target = None                   # уставка высоты, м (ставит шаг миссии)

    def set_target(self, alt) -> None:
        self.target = None if alt is None else float(alt)

    def climb(self, s):
        """Командная скорость набора, м/с (вверх +); 0.0 — держать (у цели в пределах tol);
        None — контур молчит (нет уставки или высоты)."""
        if self.target is None or s.rel_alt is None:
            return None
        err = self.target - float(s.rel_alt)
        if abs(err) < self.tol:
            return 0.0
        return clamp(self.kp * err, -self.rate_max, self.rate_max)

    def climb_cmd(self, s) -> float:
        """То же для носителя команды: молчит → 0.0 (держать)."""
        vz = self.climb(s)
        return 0.0 if vz is None else vz
