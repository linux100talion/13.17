#!/usr/bin/env python3
"""Маршрутизация режима FCU при выходе в УГЛАХ (config.att_out, laptop_move.md §5.7).

Схема (решение пилота 2026-09-30): ALT_HOLD — ТОЛЬКО ПИЛОТ (стики с RC-входа полётника,
нода вне цепочки, override нет вообще); нода управляет только в GUIDED_NOGPS углами
(SET_ATTITUDE_TARGET, порт AttitudeOutput). LOITER/LAND/RTL/SMART_RTL/GUIDED-возврат —
штатные режимы полётника, нода в них молчит (в LOITER пилот ведёт стиками с RC-входа).

План миссии по-прежнему заявляет режим «держать» = ALT_HOLD (keep); этот модуль решает,
что это значит в воздухе:
  * пилот держит SF не вверх (Арбитр — MANUAL) или борт не заармлен → ALT_HOLD:
    руддер-арм, взлёт газом, полёт руками — всё по радио;
  * заармлен, SF вверх, высота ≥ ALT_ON → GUIDED_NOGPS и держим его до касания и
    дизарма (на земле полётник сам глушит моторы по детектору посадки; руддер-дизарм
    ArduCopter разрешает в любом режиме при land_complete — AP_Arming_Copter::disarm);
    обратно в ALT_HOLD — после дизарма или по SF.
Прочие заявленные режимы проходят как есть.

Почему GUIDED_NOGPS только с высоты: на земле полётник в GUIDED_NOGPS не раскрутит
моторы, пока набор ≤ 0 (mode_guided.cpp angle_control_run), а руддер-арм в GUIDED
запрещён (allows_arming) — взлёт по радио в ALT_HOLD надёжнее. Взлёт и арм нодой — отдельный
следующий шаг.
"""
ALT_ON = 0.7                # м (высота перцепции, иначе rel_alt): с неё — GUIDED_NOGPS
GUIDED_NOGPS = "GUIDED_NOGPS"
KEEP = "ALT_HOLD"


def effective_mode(requested, s, manual: bool) -> str:
    """Какой режим FCU на самом деле держать вместо заявленного планом."""
    if requested != KEEP:
        return requested
    if manual or not s.armed:
        return KEEP
    if s.mode == GUIDED_NOGPS:
        return GUIDED_NOGPS                  # уже в воздухе под нами — до дизарма
    alt = s.perc_alt if s.perc_alt is not None else s.rel_alt
    return GUIDED_NOGPS if alt is not None and alt >= ALT_ON else KEEP


class AttModeProxy:
    """FlightMode-прокси: set_mode("ALT_HOLD") из шагов плана → effective_mode по
    последнему снапшоту (update() на каждом тике ноды). Остальное — как у адаптера."""

    def __init__(self, inner):
        self._inner = inner
        self._s = None
        self._manual = False

    def update(self, s, manual: bool) -> None:
        self._s, self._manual = s, manual

    def effective(self, mode):
        return mode if self._s is None else effective_mode(mode, self._s, self._manual)

    def set_mode(self, mode) -> None:
        self._inner.set_mode(self.effective(mode))

    def arm(self, value: bool = True) -> None:
        self._inner.arm(value)

    def ready(self) -> bool:
        return self._inner.ready()

    def __getattr__(self, name):             # force_disarm и прочие — к адаптеру
        return getattr(self._inner, name)
