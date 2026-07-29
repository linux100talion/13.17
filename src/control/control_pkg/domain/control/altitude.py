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

Высота берётся ИЗ БАРО (`rel_alt`) — на боевом борту GPS нет, а баро есть; тот же
источник, что у затвора опоры (`flow_estimator.kf_alt_max`), поэтому контур и
затвор видят одну и ту же высоту, а не расходятся.
"""
from ..rc import RC_CENTER, clamp


class AltHold:
    def __init__(self, kp=0.6, rate_max=1.2, tol=0.10, dz=100.0, span=400.0,
                 rate_full=3.16, center=RC_CENTER, out_max=350.0):
        self.kp = float(kp)                  # м/с на метр ошибки
        self.rate_max = float(rate_max)      # потолок командной vz, м/с
        self.tol = float(tol)                # мёртвая зона ПО ОШИБКЕ, м
        self.dz = float(dz)                  # мёртвая зона стика (THR_DZ), PWM
        self.span = float(span)              # PWM от края зоны до полного отклонения
        self.rate_full = float(rate_full)    # vz при полном отклонении, м/с
        self.center = int(center)
        self.out_max = float(out_max)
        self.target = None                   # уставка высоты, м (ставит шаг миссии)

    def set_target(self, alt) -> None:
        self.target = None if alt is None else float(alt)

    def throttle(self, s) -> int:
        """PWM throttle для текущего состояния. Нет уставки/высоты → центр (как было)."""
        if self.target is None or s.rel_alt is None:
            return self.center
        err = self.target - float(s.rel_alt)
        if abs(err) < self.tol:
            return self.center
        vz = clamp(self.kp * err, -self.rate_max, self.rate_max)
        off = self.dz + abs(vz) / self.rate_full * self.span
        # округляем ВЕЛИЧИНУ, а знак ставим после: int() рубит к нулю, и «вверх» с
        # «вниз» разошлись бы на 1 PWM при одинаковой по модулю ошибке
        off = int(round(clamp(off, 0.0, self.out_max)))
        return self.center + (off if vz > 0 else -off)
