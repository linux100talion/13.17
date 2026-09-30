#!/usr/bin/env python3
"""AltHold — внешний контур ВЫСОТЫ: ошибка по баро → командная vz → PWM throttle.

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
    → vz_cmd = clamp(kp·err, ±rate_max)  [м/с]
    → PWM: центр + знак·(dz + |vz|/rate_full·span)

Про мёртвую зону. В ALT_HOLD стик throttle задаёт СКОРОСТЬ, а не тягу, и вокруг
центра есть зона THR_DZ (~100 PWM), внутри которой автопилот держит высоту сам.
Команда меньше зоны не делает НИЧЕГО, поэтому её нельзя выдавать пропорционально:
контур перескакивает зону разом и дальше работает линейно. Отсюда и правило
«|err| < tol → отдать ровно центр»: у самой цели правильная команда — молчать,
а не давить в край зоны.

rate_full откалиброван по прогону: throttle 1800 (+300 PWM, то есть 200 за зоной
из 400) давал vz = +1.58 м/с → полный размах ≈ 3.16 м/с. PILOT_SPEED_UP на FCU не
трогаем: пересчёт живёт здесь, в одной формуле, и правится замером.

Высота берётся ИЗ БАРО (`rel_alt`) — на реальном борту GPS нет, а баро есть; тот же
источник, что у затвора опоры (`flow_estimator.kf_alt_max`), поэтому контур и
затвор видят одну и ту же высоту, а не расходятся.
"""
from ..rc import RC_CENTER, clamp


class ThrottleMap:
    """Скорость набора, м/с → смещение газа, µs — карта ALT_HOLD полётника, как её видит домен.

    Домен управления в СИ (перевод 2026-09-30): высотный контур и мягкая посадка считают
    СКОРОСТЬ НАБОРА (м/с), а газ канала — её PWM-эквивалент на носителе RcCommand. Карта одна
    на AltHold и SoftLand: мёртвая зона THR_DZ (dz) перескакивается разом, дальше линейно
    rate_full м/с на span µs (калибровка замером — см. AltHold). Возвращает МОДУЛЬ смещения
    (float, без округления): округление и знак у каждого потребителя свои, как было."""

    def __init__(self, dz=100.0, span=400.0, rate_full=3.16):
        self.dz = float(dz)                  # мёртвая зона стика (THR_DZ), µs
        self.span = float(span)              # µs от края зоны до полного отклонения
        self.rate_full = float(rate_full)    # vz при полном отклонении, м/с

    def off(self, vz) -> float:
        return self.dz + abs(vz) / self.rate_full * self.span

    def pwm(self, climb, center=RC_CENTER) -> int:
        """Скорость набора → газ канала, µs: 0 = центр («держать»), иначе центр ± округлённое
        смещение за мёртвой зоной (как считали AltHold и SoftLand)."""
        if climb == 0.0:
            return int(center)
        off = int(round(self.off(climb)))
        return int(center) + (off if climb > 0 else -off)

    def climb(self, pwm, center=RC_CENTER) -> float:
        """Газ канала, µs → скорость набора, м/с (обратная к pwm): внутри мёртвой зоны — 0
        (полётник там высоту держит сам), за ней линейно. pwm → climb → pwm точен вне зоны."""
        e = float(pwm) - float(center)
        if abs(e) < self.dz:
            return 0.0
        # ровно на краю зоны — исчезающе малая скорость со знаком: pwm() вернёт тот же край
        v = max((abs(e) - self.dz) / self.span * self.rate_full, 1e-12)
        return v if e > 0 else -v


class AltHold:
    def __init__(self, kp=0.6, rate_max=1.2, tol=0.10, dz=100.0, span=400.0,
                 rate_full=3.16, center=RC_CENTER, out_max=350.0):
        self.kp = float(kp)                  # м/с на метр ошибки
        self.rate_max = float(rate_max)      # потолок командной vz, м/с
        self.tol = float(tol)                # мёртвая зона ПО ОШИБКЕ, м
        self.map = ThrottleMap(dz, span, rate_full)   # м/с → µs газа (PWM-эквивалент)
        self.center = int(center)
        self.out_max = float(out_max)
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
        """Скорость набора для носителя команды (м/с) — на сетке газа: climb() после
        потолка out_max и округления до µs, как ушло бы в провод. 0.0 — держать."""
        return self.map.climb(self.throttle(s), self.center)

    def throttle(self, s) -> int:
        """Газ, µs: PWM-эквивалент climb() по карте ThrottleMap. Молчит/держит → центр."""
        vz = self.climb(s)
        if not vz:
            return self.center
        off = self.map.off(vz)
        # округляем ВЕЛИЧИНУ, а знак ставим после: int() рубит к нулю, и «вверх» с
        # «вниз» разошлись бы на 1 PWM при одинаковой по модулю ошибке
        off = int(round(clamp(off, 0.0, self.out_max)))
        return self.center + (off if vz > 0 else -off)
