#!/usr/bin/env python3
"""Маршрутизация режима FCU при выходе в УГЛАХ (config.att_out, laptop_move.md §5.7).

Схема (решение пилота 2026-09-30): ALT_HOLD — ТОЛЬКО ПИЛОТ (стики с RC-входа полётника,
нода вне цепочки, override нет вообще); нода управляет только в GUIDED_NOGPS углами
(SET_ATTITUDE_TARGET, порт AttitudeOutput). LOITER/LAND/RTL/SMART_RTL/GUIDED-возврат —
штатные режимы полётника, нода в них молчит (в LOITER пилот ведёт стиками с RC-входа).

План миссии по-прежнему заявляет режим «держать» = ALT_HOLD (keep); этот модуль решает,
что он значит:
  * пилот держит SF не вверх (Арбитр — MANUAL) → ALT_HOLD «только пилот» по радио;
  * иначе → GUIDED_NOGPS ВСЕГДА — и на земле: арм НОДОЙ (application/node_arm.py, жест
    пилота + готовность ноды), взлёт газом пилота (набор > 0 раскручивает моторы,
    mode_guided.cpp angle_control_run), посадка до касания; дизарм — нодой по жесту
    (полётник по GCS дизармит только при land_complete). Арм со стиков у полётника
    запрещён (ARMING_RUDDER 0) — борт не взлетит без готовой ноды.
Прочие заявленные режимы проходят как есть.
"""
GUIDED_NOGPS = "GUIDED_NOGPS"
KEEP = "ALT_HOLD"


def effective_mode(requested, s, manual: bool) -> str:
    """Какой режим FCU на самом деле держать вместо заявленного планом."""
    if requested != KEEP:
        return requested
    return KEEP if manual else GUIDED_NOGPS


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
