#!/usr/bin/env python3
"""Адаптеры порта PilotInput.

- JoyPilot — ЖИВОЙ ПУЛЬТ: читает /joy (TX в режиме USB-джойстика → joy_linux_node),
  МИМО FCU. Единственный корректный источник живых стиков, пока нода публикует
  /mavros/rc/override (см. докстринг класса).
- RosPilot — ЛЕГАСИ (⚠️ петля): читает /mavros/rc/in. Под активным override ArduPilot
  отдаёт в RC_CHANNELS уже ПОДМЕНЁННЫЕ значения → нода читает СОБСТВЕННУЮ команду как
  «стик пилота» (в assist — самораскачка, в MANUAL — защёлка). Годен только когда нода
  НЕ оверрайдит ch1..4. Тумблер режима — канал 6, порог 1700.
- ScriptedPilot — СИМ (headless, без живого пульта): детерминированный профиль стиков
  по sim-времени. Валидирует пилот-пайплайн воспроизводимо. Drop-in замена JoyPilot.

Все реализуют один порт: sticks()->RcCommand, mode_switch()->int.
"""
from ..domain.rc import RC_CENTER, RcCommand

# Карта /joy для EdgeTX (RadioMaster TX12/TX16S): в режиме USB-джойстика пульт отдаёт
# ВЫХОДНЫЕ КАНАЛЫ МИКШЕРА как HID-оси → та же конвенция, что у RC_CHANNELS:
# axes[0..3] = CH1..CH4 (roll/pitch/throttle/yaw при модели AETR),
# axes[4]    = CH5 (FLTMODE_CH — не трогаем, это FCU-уровень safety),
# axes[5]    = CH6 — тумблер MANUAL-seize Арбитра (назначить в микшере на SC/SD).
# Порог 0.5 — зеркало «ch6 > 1700» у RosPilot: (1700−1500)/400 = 0.5.
JOY_AXIS_ROLL, JOY_AXIS_PITCH, JOY_AXIS_THROTTLE, JOY_AXIS_YAW = 0, 1, 2, 3
JOY_AXIS_SWITCH = 5
JOY_SWITCH_THRESHOLD = 0.5
_JOY_SPAN = 400          # ось ±1 → PWM 1500±400 (конвенция pilot_full)
# Знаки осей TX12 (roll,pitch,throttle,yaw) — выверены ЖИВЫМИ ПОЛЁТАМИ
# (assisted/MANUAL, 2026-08-16): в EdgeTX-HID зеркальны нашей RC-конвенции
# roll, yaw И throttle; прямой только pitch — потому что провод сам инвертирован:
# «стик от себя → HID минус → PWM 1300» и есть «вперёд» на RC2 (низ = нос вниз).
# Газ подтверждён первым полётом, где он дошёл до провода (до этого ось
# игнорировалась и знак был непроверенным предположением «прямой»).
JOY_SIGNS_DEFAULT = (-1.0, 1.0, -1.0, -1.0)


def joy_sticks(axes, signs=(1.0, 1.0, 1.0, 1.0)):
    """Чистое ядро JoyPilot: axes [-1..1] → (roll, pitch, throttle, yaw, switch) PWM/бит.
    Отсутствующая ось (короткий axes) → центр; тумблер без оси → 0 (AUTO)."""
    def pwm(idx, sign):
        if idx >= len(axes):
            return RC_CENTER
        v = max(-1.0, min(1.0, sign * axes[idx]))
        return int(round(RC_CENTER + v * _JOY_SPAN))
    sw = 0
    if JOY_AXIS_SWITCH < len(axes):
        sw = 1 if axes[JOY_AXIS_SWITCH] > JOY_SWITCH_THRESHOLD else 0
    return (pwm(JOY_AXIS_ROLL, signs[0]), pwm(JOY_AXIS_PITCH, signs[1]),
            pwm(JOY_AXIS_THROTTLE, signs[2]), pwm(JOY_AXIS_YAW, signs[3]), sw)


class JoyPilot:
    """PilotInput из /joy (sensor_msgs/Joy) — живые стики МИМО FCU.

    Почему не /mavros/rc/in: пред-override RC в MAVLink-телеметрии НЕ СУЩЕСТВУЕТ —
    пока нода пишет /mavros/rc/override, FCU в RC_CHANNELS отдаёт её же команду
    (замкнутая петля). Поэтому живой пульт входит только напрямую:
    TX (EdgeTX, USB-джойстик) → joy_linux_node → /joy → сюда. Нода остаётся
    ЕДИНСТВЕННЫМ писателем override.

    signs — знаки осей (roll,pitch,throttle,yaw): сверять НА ЗЕМЛЕ, по одному стику
    за раз (у HID свои конвенции направлений; зеркальный знак уже стоил разбора).
    Потеря джойстика: /joy замолкает → держим последние значения (как RosPilot при
    потере радио); барьер на этот случай — FCU-уровень (FLTMODE_CH), не адаптер.
    """

    def __init__(self, node, signs=JOY_SIGNS_DEFAULT):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Joy
        self._signs = signs
        self._r = self._p = self._t = self._y = RC_CENTER
        self._sw = 0
        node.create_subscription(Joy, '/joy', self._on, qos_profile_sensor_data)

    def _on(self, m):
        self._r, self._p, self._t, self._y, self._sw = joy_sticks(m.axes, self._signs)

    def sticks(self) -> RcCommand:
        return RcCommand(self._r, self._p, self._t, self._y)

    def mode_switch(self) -> int:
        return self._sw


class RosPilot:
    """⚠️ ЛЕГАСИ. /mavros/rc/in под активным override — эхо собственной команды ноды
    (петля). Для живого пульта использовать JoyPilot; этот адаптер оставлен для
    сценариев, где нода не оверрайдит ch1..4."""

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
