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
# axes[5]    = CH6 — ТРЁХПОЗИЦИОННЫЙ селектор (в микшере на SC/SD):
#   +1 (>0.5)  → MANUAL-seize Арбитра (сырые стики, миссия отстранена);
#   −1 (<−0.5) → наш стабилизатор (Control-шаг ставит BS_STAB-стек);
#    0 (центр) → чистый ALT_HOLD (стабилизаторов нет, стики = наклоны).
# Порог 0.5 — зеркало «ch6 > 1700» у RosPilot: (1700−1500)/400 = 0.5.
JOY_AXIS_ROLL, JOY_AXIS_PITCH, JOY_AXIS_THROTTLE, JOY_AXIS_YAW = 0, 1, 2, 3
JOY_AXIS_SWITCH = 5
JOY_SWITCH_THRESHOLD = 0.5
# --- Схема «SF-мастер» (опт-ин: cfg.sf_master / BS_SF_MASTER) ---
# axes[6] = CH7 — SF-мастер: ВВЕРХ (>0.5) = стабилизация разрешена; центр/вниз/
# оси нет = СЫРЫЕ СТИКИ (MANUAL-seize Арбитра) при ЛЮБОМ SC. Выделенный
# выключатель: перехват — один бинарный щелчок из любого состояния, без проезда
# через центр SC (в легаси сырые стики — крайнее положение селектора).
# CH6 (SC) при SF-вверх выбирает ПОТОЛОК лесенки зрелости (Freefly._ladder_*):
#   −1 (SC физически ВВЕРХ) → 0: только демпфер (свапа на VinsHold НЕТ);
#    0 (центр)              → 1: демпфер → VinsHold по готовности VINS;
#   +1 (ВНИЗ)               → 2: демпфер → VinsHold → штатный LOITER по зрелости.
# Борт всегда на лучшей ДОСТУПНОЙ ступени ≤ потолка, деградация симметрична.
# ⚠️ СОВМЕСТИМОСТЬ: старые записи/реплеи несут 6 осей (axes[6] нет → «SF не
# вверх») и CH6=+1 в них означал MANUAL — под sf_master они летели бы целиком
# на сырых стиках. Поэтому семантика ТОЛЬКО опт-ином: старые jsonl гонять в
# легаси, новые записи несут 7 осей (joy_timeline пишет axes[:7]).
JOY_AXIS_MASTER = 6
# --- Кнопка ПОСАДКИ (SA на TX12): источник задаёт config.land_joy / --land-joy ---
# Живой /joy TX12 (bag lv2_joy_20260830_101919): 7 осей + 24 кнопки, ни одна
# кнопка за полёт не нажималась — SA в миксере ещё ни к чему не привязан.
# Куда приедет SA, решает микшер EdgeTX: канал-кнопка → buttons[i] ('b<i>'),
# канал-ось → axes[i] ('a<i>', нажата = > порога). Дефолт 'b0' = первая кнопка
# (CH8 при 7 осях). Проверка на земле: щёлкнуть SA и посмотреть, что меняется
# (joy_timeline пишет фронты кнопок в ленту; ros2 topic echo /joy). '' = выкл.
JOY_LAND_SRC_DEFAULT = 'b0'
_JOY_SPAN = 400          # ось ±1 → PWM 1500±400 (конвенция pilot_full)
# Знаки осей TX12 (roll,pitch,throttle,yaw) — выверены ЖИВЫМИ ПОЛЁТАМИ
# (assisted/MANUAL, 2026-08-16): в EdgeTX-HID зеркальны нашей RC-конвенции
# roll, yaw И throttle; прямой только pitch — потому что провод сам инвертирован:
# «стик от себя → HID минус → PWM 1300» и есть «вперёд» на RC2 (низ = нос вниз).
# Газ подтверждён первым полётом, где он дошёл до провода (до этого ось
# игнорировалась и знак был непроверенным предположением «прямой»).
JOY_SIGNS_DEFAULT = (-1.0, 1.0, -1.0, -1.0)


def joy_sticks(axes, signs=(1.0, 1.0, 1.0, 1.0)):
    """Чистое ядро JoyPilot: axes [-1..1] → (roll, pitch, throttle, yaw, switch) PWM/поз.
    switch — трёхпозиционник: +1 MANUAL / 0 ALT_HOLD / −1 наш стабилизатор.
    Отсутствующая ось (короткий axes) → центр; тумблер без оси → 0 (ALT_HOLD)."""
    def pwm(idx, sign):
        if idx >= len(axes):
            return RC_CENTER
        v = max(-1.0, min(1.0, sign * axes[idx]))
        return int(round(RC_CENTER + v * _JOY_SPAN))
    sw = 0
    if JOY_AXIS_SWITCH < len(axes):
        v = axes[JOY_AXIS_SWITCH]
        sw = 1 if v > JOY_SWITCH_THRESHOLD else (-1 if v < -JOY_SWITCH_THRESHOLD else 0)
    return (pwm(JOY_AXIS_ROLL, signs[0]), pwm(JOY_AXIS_PITCH, signs[1]),
            pwm(JOY_AXIS_THROTTLE, signs[2]), pwm(JOY_AXIS_YAW, signs[3]), sw)


def joy_master(axes):
    """Чистое ядро схемы SF-мастер: axes → (switch, level).
    switch — для Арбитра: +1 (MANUAL), пока SF не вверх; −1 (авто) при SF-вверх
    (в Control-шагах −1 = наш стек — деградация не-freefly миссий осмысленна).
    level 0..2 — потолок лесенки по SC (карта у JOY_AXIS_MASTER выше).
    Отсутствующая ось SF (короткий axes, старые записи) → MANUAL: безопасный
    дефолт «мастер выключен = сырые стики»."""
    sc = 0
    if JOY_AXIS_SWITCH < len(axes):
        v = axes[JOY_AXIS_SWITCH]
        sc = 1 if v > JOY_SWITCH_THRESHOLD else (-1 if v < -JOY_SWITCH_THRESHOLD else 0)
    sf_up = (JOY_AXIS_MASTER < len(axes)
             and axes[JOY_AXIS_MASTER] > JOY_SWITCH_THRESHOLD)
    return (-1 if sf_up else 1), sc + 1


def parse_land_src(spec):
    """'b0' → ('b', 0), 'a7' → ('a', 7); ''/None/'0'/'off' → None (кнопки нет).
    Кривой spec — ValueError (лучше упасть на старте, чем лететь без посадки)."""
    spec = (spec or '').strip().lower()
    if spec in ('', '0', 'off', 'none'):
        return None
    if len(spec) >= 2 and spec[0] in ('a', 'b') and spec[1:].isdigit():
        return spec[0], int(spec[1:])
    raise ValueError(f"land_joy: ожидал 'b<i>' (кнопка) или 'a<i>' (ось), получил {spec!r}")


def joy_land(axes, buttons, src) -> bool:
    """Чистое ядро кнопки посадки: нажата ли по /joy. src — parse_land_src();
    отсутствующий индекс (короткий массив, старые записи) → False."""
    if src is None:
        return False
    kind, i = src
    if kind == 'b':
        return i < len(buttons) and int(buttons[i]) != 0
    return i < len(axes) and float(axes[i]) > JOY_SWITCH_THRESHOLD


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

    def __init__(self, node, signs=JOY_SIGNS_DEFAULT, sf_master=False,
                 land_src=JOY_LAND_SRC_DEFAULT):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Joy
        self._signs = signs
        self._sf_master = sf_master
        self._land_src = parse_land_src(land_src)
        self._r = self._p = self._t = self._y = RC_CENTER
        # дефолты до первого /joy: MANUAL в схеме SF-мастер (руки пилота —
        # безопасный старт), потолок 0 (только демпфер) — эскалация лишь по
        # явному положению SC
        self._sw = 1 if sf_master else 0
        self._lvl = 0
        self._land = False
        node.create_subscription(Joy, '/joy', self._on, qos_profile_sensor_data)

    def _on(self, m):
        self._r, self._p, self._t, self._y, sw = joy_sticks(m.axes, self._signs)
        if self._sf_master:
            self._sw, self._lvl = joy_master(m.axes)
        else:
            self._sw = sw
        self._land = joy_land(m.axes, m.buttons, self._land_src)

    def sticks(self) -> RcCommand:
        return RcCommand(self._r, self._p, self._t, self._y)

    def mode_switch(self) -> int:
        return self._sw

    def stab_level(self) -> int:
        return self._lvl

    def land_switch(self) -> bool:
        return self._land


class RosPilot:
    """⚠️ ЛЕГАСИ. /mavros/rc/in под активным override — эхо собственной команды ноды
    (петля). Для живого пульта использовать JoyPilot; этот адаптер оставлен для
    сценариев, где нода не оверрайдит ch1..4."""

    def __init__(self, node, sf_master=False):
        from mavros_msgs.msg import RCIn
        from rclpy.qos import qos_profile_sensor_data
        self._sf_master = sf_master
        self._r = self._p = self._t = self._y = RC_CENTER
        self._sw = 1 if sf_master else 0     # дефолты — как у JoyPilot
        self._lvl = 0
        self._land = False
        node.create_subscription(RCIn, '/mavros/rc/in', self._on, qos_profile_sensor_data)

    def _on(self, m):
        ch = m.channels
        if len(ch) >= 4:
            self._r, self._p, self._t, self._y = ch[0], ch[1], ch[2], ch[3]
        if len(ch) >= 6:
            # трёхпозиционник ch6 (зеркало joy_sticks): >1700 / <1300 / центр
            sc = 1 if ch[5] > 1700 else (-1 if ch[5] < 1300 else 0)
            if self._sf_master:
                # зеркало joy_master в PWM: SF на CH7, >1700 = вверх
                sf_up = len(ch) >= 7 and ch[6] > 1700
                self._sw, self._lvl = (-1 if sf_up else 1), sc + 1
            else:
                self._sw = sc
        # кнопка посадки — CH8 (>1700 = нажата), зеркало joy_land в PWM
        self._land = len(ch) >= 8 and ch[7] > 1700

    def sticks(self) -> RcCommand:
        return RcCommand(self._r, self._p, self._t, self._y)

    def mode_switch(self) -> int:
        return self._sw

    def stab_level(self) -> int:
        return self._lvl

    def land_switch(self) -> bool:
        return self._land


class ScriptedPilot:
    """Профиль стиков по sim-времени. segments: список (t_until, roll, pitch, yaw) —
    первый сегмент с t_until > t выигрывает; после последнего — центр. switch_segments:
    (t_until, value) для тумблера. Время — с первого вызова (базируется лениво)."""

    def __init__(self, clock, segments, switch_segments=None, land_at=None):
        self._clock = clock
        self._seg = segments
        self._sw = switch_segments or []
        self._land_at = land_at     # sim-сек профиля, с которых «кнопка нажата»
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

    def stab_level(self) -> int:
        # схема SF-мастер живому пилоту; scripted лесенку не ведёт (freefly и так
        # требует живого пилота), потолок 0 = только демпфер
        return 0

    def land_switch(self) -> bool:
        return self._land_at is not None and self._t() >= self._land_at

    def total(self) -> float:
        """Длительность профиля (для триггера land в пилот-режимах)."""
        return self._seg[-1][0] if self._seg else 0.0
