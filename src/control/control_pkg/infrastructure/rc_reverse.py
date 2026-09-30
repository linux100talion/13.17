#!/usr/bin/env python3
"""RcReverse — компенсация реверса каналов ПОЛЁТНИКА на override (чистое ядро, без ROS).

ЗАЧЕМ (разведка 2026-09-29, docker/sim/laptop_move.md §5.7). Override ArduPilot кладёт
в `radio_in` ТОЧНО ТАК ЖЕ, как приёмник (`RC_Channel::update`), и дальше к нему
применяется реверс канала `RCn_REVERSED`. Реверс — настройка ПУЛЬТА («этот пульт шлёт
канал наоборот»): на реальном борту TX12 по ELRS шлёт тангаж «вперёд = 2011», поэтому
в полётнике стоит `RC2_REVERSED 1`. А нода шлёт override уже в стандартной конвенции
ArduPilot («вперёд = ниже центра», `_PITCH_RC_SIGN`): причуду пульта нода исправляет
сама на входе. Без компенсации FCU перевернул бы тангаж ноды ещё раз — демпфер и VINS
по тангажу в положительной обратной связи. В симе `RC2_REVERSED 0` — зеркалить нечего.

ЧТО ДЕЛАЕТ. Домен всегда говорит в стандартной конвенции (как при `REVERSED 0`); на
выходе канал с `RCn_REVERSED 1` зеркалится вокруг `RCn_TRIM`: v' = 2·TRIM − v. FCU,
перевернув, получает (v − TRIM) — ровно то же, что без реверса. Нода НЕ решает «сим или
борт» по этому параметру — просто компенсирует реверс каждого канала, какой бы он ни был.

Газ (канал 3) — канал-ДИАПАЗОН: FCU переворачивает его вокруг середины MIN/MAX, не
TRIM, и реверс газа — не наш случай. Реверс газа = отказ (`apply` → None), как и
«параметры ещё не прочитаны»: адаптер тогда отпускает override (release), и на борту
управление остаётся у физического приёмника — лучше, чем слать канал с неизвестным знаком.
Спецзначения RC_RELEASE (0) и RC_NOCHANGE (65535) не зеркалятся.
"""
from ..domain.rc import RC_CENTER, RC_NOCHANGE, RC_RELEASE

# каналы ArduCopter 1..4 = roll/pitch/throttle/yaw; газ (3) — диапазон, не угол
ANGLE_CHANNELS = (1, 2, 4)
THROTTLE_CHANNEL = 3
PARAMS = tuple(f'RC{i}_REVERSED' for i in range(1, 5)) + \
    tuple(f'RC{i}_TRIM' for i in ANGLE_CHANNELS)


class RcReverse:
    def __init__(self):
        self._rev = {}      # канал → bool (реверс FCU)
        self._trim = {}     # канал → PWM центра FCU

    def on_param(self, name: str, value) -> bool:
        """Значение параметра FCU (из события MAVROS или опроса). True — если это
        наш параметр и он поменял состояние (адаптер пишет лог)."""
        if name not in PARAMS:
            return False
        ch = int(name[2])
        if name.endswith('_REVERSED'):
            v = bool(int(round(float(value))))
            changed = self._rev.get(ch) != v
            self._rev[ch] = v
        else:
            v = int(round(float(value)))
            changed = self._trim.get(ch) != v
            self._trim[ch] = v
        return changed

    def missing(self):
        """Параметры, которые ещё нужны: все REVERSED + TRIM реверсных угловых каналов
        (TRIM нереверсных не нужен — их не трогаем)."""
        need = [f'RC{i}_REVERSED' for i in range(1, 5) if i not in self._rev]
        need += [f'RC{i}_TRIM' for i in ANGLE_CHANNELS
                 if self._rev.get(i) and i not in self._trim]
        return need

    def ready(self) -> bool:
        return not self.missing()

    def throttle_reversed(self) -> bool:
        return bool(self._rev.get(THROTTLE_CHANNEL))

    def reversed_channels(self):
        return [i for i in range(1, 5) if self._rev.get(i)]

    def apply(self, channels):
        """channels — список ch1..ch4 (PWM стандартной конвенции). → тот же список
        с зеркальными реверсными каналами, или None: реверс ещё не прочитан либо
        перевёрнут газ (адаптер тогда отпускает override)."""
        if not self.ready() or self.throttle_reversed():
            return None
        out = list(channels)
        for i in ANGLE_CHANNELS:
            v = out[i - 1]
            if self._rev.get(i) and v not in (RC_RELEASE, RC_NOCHANGE):
                out[i - 1] = 2 * self._trim.get(i, RC_CENTER) - int(v)
        return out

    def describe(self) -> str:
        """Одна строка для лога ноды."""
        rev = ' '.join(f'RC{i}={int(self._rev[i])}' for i in range(1, 5) if i in self._rev)
        ch = {1: 'крен', 2: 'тангаж', 3: 'газ', 4: 'рыскание'}
        mir = [ch[i] for i in self.reversed_channels()]
        if self.throttle_reversed():
            return f'{rev} — РЕВЕРС ГАЗА не поддержан, override не шлю'
        return f'{rev} — зеркалю: {", ".join(mir) if mir else "нечего"}'
