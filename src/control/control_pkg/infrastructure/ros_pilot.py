#!/usr/bin/env python3
"""Адаптеры порта PilotInput.

- RosPilot — БОЕВОЙ/SITL: читает /mavros/rc/in (стики двигает человек). Тумблер
  режима — на канале 6 (порог 1700). Это и есть «переключить управление на реальный
  пульт»: домен (RcTransmitter/PilotPassthrough) не меняется, меняется только источник.
- ScriptedPilot — СИМ (headless, без живого пульта): детерминированный профиль стиков
  по sim-времени. Валидирует пилот-пайплайн воспроизводимо. Drop-in замена RosPilot.

Оба реализуют один порт: sticks()->RcCommand, mode_switch()->int.
"""
from ..domain.rc import RC_CENTER, RcCommand


class RosPilot:
    def __init__(self, node):
        from mavros_msgs.msg import RCIn
        from rclpy.qos import qos_profile_sensor_data
        self._r = self._p = self._t = self._y = RC_CENTER
        self._sw = 0
        node.create_subscription(RCIn, '/mavros/rc/in', self._on, qos_profile_sensor_data)

    def _on(self, m):
        ch = m.channels
        if len(ch) >= 4:
            self._r, self._p, self._t, self._y = ch[0], ch[1], ch[2], ch[3]
        if len(ch) >= 6:
            self._sw = 1 if ch[5] > 1700 else 0   # тумблер авто/ручной на ch6

    def sticks(self) -> RcCommand:
        return RcCommand(self._r, self._p, self._t, self._y)

    def mode_switch(self) -> int:
        return self._sw


class ScriptedPilot:
    """Профиль стиков по sim-времени. segments: список (t_until, roll, pitch, yaw) —
    первый сегмент с t_until > t выигрывает; после последнего — центр. switch_segments:
    (t_until, value) для тумблера. Время — с первого вызова (базируется лениво)."""

    def __init__(self, clock, segments, switch_segments=None):
        self._clock = clock
        self._seg = segments
        self._sw = switch_segments or []
        self._t0 = None

    def _t(self) -> float:
        now = self._clock.now_sim()
        if self._t0 is None:
            self._t0 = now
        return now - self._t0

    def sticks(self) -> RcCommand:
        t = self._t()
        for tu, r, p, y in self._seg:
            if t < tu:
                return RcCommand(r, p, RC_CENTER, y)
        return RcCommand(RC_CENTER, RC_CENTER, RC_CENTER, RC_CENTER)

    def mode_switch(self) -> int:
        t = self._t()
        for tu, val in self._sw:
            if t < tu:
                return val
        return 0

    def total(self) -> float:
        """Длительность профиля (для триггера land в пилот-режимах)."""
        return self._seg[-1][0] if self._seg else 0.0
