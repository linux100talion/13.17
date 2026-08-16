#!/usr/bin/env python3
"""ThrottleLatch — защёлка газа живого пилота (чистый домен, ноль импортов).

Газ — единственная ось, где сырой стик опасен В МОМЕНТ ПЕРЕДАЧИ управления:
roll/pitch/yaw в центре = «ничего не делать», а газ в ALT_HOLD — это вертикальная
скорость, и стик вне центра в момент seize = немедленный уход по высоте. Живой
случай (полёт 2026-08-16): пилот 5 с держал газ на −1.0, считая, что уже в MANUAL —
от снижения в землю с 3 м спасла только перепутанная сторона тумблера.

Правило: после reset() газ пилота ЗАПЕРТ — pass_through() возвращает None, пока
стик впервые не окажется в центре (±deadzone). После этого стик вне центра =
команда пилота (сырой PWM), в центре = None («газа нет» — вызывающий сам решает,
чем держать высоту: центр стика или контур AltHold).

Используется двумя потребителями с РАЗНЫМ моментом reset():
- Arbiter: reset() на каждом входе в MANUAL (щелчок тумблера);
- Control-шаг миссии (assisted): reset() на входе в шаг.
"""

from ..rc import RC_CENTER


class ThrottleLatch:
    def __init__(self, deadzone: int = 30):
        self.deadzone = int(deadzone)
        self._open = False

    def reset(self) -> None:
        """Запереть: газ не проходит, пока стик не побывает в центре."""
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def pass_through(self, throttle: int):
        """Сырой PWM газа пилота — или None (защёлка закрыта / стик в центре)."""
        centered = abs(throttle - RC_CENTER) <= self.deadzone
        if not self._open:
            if centered:
                self._open = True
            return None
        return None if centered else throttle
