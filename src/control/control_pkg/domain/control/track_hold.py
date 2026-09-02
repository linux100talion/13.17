#!/usr/bin/env python3
"""TrackHold — стики LOITER в ОСЯХ МИРА: yaw вращает нос, а не траекторию.

Выделен под ярус LOITER лесенки (агро-профиль «прямые галсы»), 2026-09-01.
"""
import math

from ..rc import RC_CENTER, RcCommand, clamp
from ..setpoint import Setpoint
from ..state import DroneState
from .base import StabilizationStrategy

_SPAN = 400            # c=±1 → ±400 PWM — конвенция pilot_full, зеркало базы ControlStack
_PITCH_RC_SIGN = -1.0  # «вперёд» = НИЖЕ центра на RC2 (см. _PITCH_RC_SIGN в control_stack)


class TrackHold(StabilizationStrategy):
    """Контр-вращение стик-вектора roll/pitch на Δψ от латча — уставка скорости
    LOITER постоянна В МИРЕ, пока пилот крутит yaw.

    ЗАЧЕМ (разбор eagle/4, 2026-09-01). В LOITER стики roll/pitch — уставки
    скорости в осях БОРТА: yaw на ходу вращает вектор уставки, борт летит по
    дуге, а дуга требует крена φ = atan(v·ω/g) — замер сошёлся с формулой нос в
    нос (5.8 м/с × 24°/с → крен 13.8° при предсказании 14.0°), пики упирались в
    PSC_ANGLE_MAX. Физику не отменить настройками FCU: квадрокоптер ускоряется
    вбок только наклоном. Убирается сама ПРИЧИНА — искривление: здесь стик-вектор
    поворачивается ПРОТИВ носа на накопленный Δψ, FCU получает body-frame
    команду, неподвижную в мире → траектория прямая, нос (камера) вращается
    свободно, крен виража исчезает по построению. Агро-семантика: pitch+yaw =
    «лететь прямо и осматриваться», а не «заворачивать».

    ЛАТЧ. Мировая рама берётся от курса В МОМЕНТ ОТКЛОНЕНИЯ стика (переход
    центр → жив) и держится, пока стик жив; стики в центре — рама сброшена.
    Итог: каждое новое нажатие = «вперёд там, куда сейчас смотрю», и скачка
    уставки при отпускании yaw нет (рама не перелатчивается под живым стиком).
    Галс «стоп → разворот на месте → вперёд» работает как раньше.

    Курс — s.att_yaw (ENU, /mavros/imu/data): та же ориентация, которой EKF сам
    поворачивает body→ENU, — поворот согласован с трактовкой стиков контроллером
    LOITER. att_yaw молчит (нет IMU) → 0.0 констант → Δψ=0 → деградация в сырой
    passthrough, как без TrackHold.

    ГРАНИЦЫ. (1) Оба стика в упоре: |вектор| = √2, повёрнутые компоненты клампятся
    ±_SPAN — направление у диагонали слегка искажается (модуль важнее). (2) slew
    стека (BS_SLEW 300 PWM/с) отстаёт от полного темпа контр-вращения (400·ω ≈
    460 PWM/с при 66°/с) — остаточная кривизна мала и гаснет с концом разворота.
    (3) Живёт ТОЛЬКО в ярусе LOITER (стик = скорость); в ALT_HOLD-ярусах стик =
    наклон, там смысла не имеет. При Δψ=0 выход бит-в-бит равен базе ControlStack
    (int(clamp(...)), те же span и знак pitch) — включение ручки без разворота
    ничего не меняет."""
    axes = frozenset({"roll", "pitch"})

    def __init__(self, dz=0.02):
        self.dz = float(dz)     # порог «стик жив» (траектория уже занулила свою
                                # мёртвую зону — это страховочный пол, не ручка)
        self._psi0 = None

    def enter(self, s: DroneState) -> None:
        self._psi0 = None       # вход в ярус = рама с первого нажатия

    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand:
        rc = RcCommand(throttle=RC_CENTER)
        f, r = float(sp.c_fwd), float(sp.c_right)
        if abs(f) < self.dz and abs(r) < self.dz:
            self._psi0 = None   # центр = «стоять» (держит FCU), рама сброшена
            return rc
        if self._psi0 is None:
            self._psi0 = float(s.att_yaw)
        d = float(s.att_yaw) - self._psi0   # sin/cos периодичны — wrap ±π не важен
        c, si = math.cos(d), math.sin(d)
        f2 = f * c - r * si     # мировой вектор рамы латча в осях ТЕКУЩЕГО носа
        r2 = f * si + r * c
        rc.roll = int(clamp(RC_CENTER + r2 * _SPAN,
                            RC_CENTER - _SPAN, RC_CENTER + _SPAN))
        rc.pitch = int(clamp(RC_CENTER + _PITCH_RC_SIGN * f2 * _SPAN,
                             RC_CENTER - _SPAN, RC_CENTER + _SPAN))
        return rc
