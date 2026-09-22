#!/usr/bin/env python3
"""PilotLink — СТОРОЖ СВЕЖЕСТИ канала пилота. Чистый домен, ноль импортов.

Зачем. Пока нода пишет `/mavros/rc/override`, полётник считает нашу команду
командой пилота: его собственный RC-failsafe не сработает (`radio_in` жив нашей
же рукой), а адаптер пульта при пропаже источника держит ПОСЛЕДНИЕ значения
(докстринг `JoyPilot`) — борт полетит с замороженными стиками и без failsafe.
Сторож ловит ровно это: источник молчит дольше `stale_sec` → вердикт «протух»,
и актуатор ОТПУСКАЕТ каналы (`RC_RELEASE` = 0) вместо публикации команды. FCU
мгновенно возвращается к физическому приёмнику (борт) / остаётся без override
(сим). Разбор и лестница — `docker/sim/laptop_move.md` §5.2–5.3.

Часы — МОНОТОННЫЕ (не sim-время): канал пилота физический, его свежесть от RTF
не зависит; даже на RTF 0.07 `/joy` идёт 20 Гц стеночных (`autorepeat_rate` в
`bootstrap_arch2.sh`), а реплей — 50 Гц и продолжает публиковать после конца
сценария («держу последнее состояние»), так что ложных срабатываний нет.

`age=None` — у источника нет внешнего канала (`ScriptedPilot`): сторожить нечего.
`stale_sec=0` — сторож ВЫКЛЮЧЕН (поведение до 2026-09-22).

⚠️ Порог обязан быть больше периода источника с запасом: `/joy` 20 Гц → 0.5 с =
10 кадров. Для `RosPilot` (`/mavros/rc/in`) период задаёт стрим-рейт FCU
(`MAV1_*`) — он бывает куда реже, см. память `fcu-telemetry-streams-race`.
"""


class PilotLink:
    """Сторож свежести источника стиков: возраст семпла → вердикт live/stale."""

    def __init__(self, stale_sec: float = 0.0):
        self.stale_sec = float(stale_sec)
        self.stale = False      # текущий вердикт: True = канал пилота протух
        self.changed = False    # был ли ФРОНТ на последнем update (для лога)
        self.age = None         # возраст последнего семпла, с (None = источника нет)

    @property
    def enabled(self) -> bool:
        return self.stale_sec > 0.0

    @property
    def alive(self) -> bool:
        """Для статуса: канал жив (или сторож выключен/источника нет)."""
        return not self.stale

    def update(self, age) -> bool:
        """age — секунды с последнего семпла источника (None = внешнего канала нет).
        Возвращает вердикт «протух» (True = override пора отпускать)."""
        self.age = age
        new = bool(self.enabled and age is not None and age >= self.stale_sec)
        self.changed = (new != self.stale)
        self.stale = new
        return new
