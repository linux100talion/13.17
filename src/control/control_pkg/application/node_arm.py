#!/usr/bin/env python3
"""Арм и дизарм НОДОЙ при выходе в углах (config.att_out, решение пилота 2026-09-30).

ЗАЧЕМ. Полётнику арм со стиков запрещён (ARMING_RUDDER 0): борт не должен взлететь, пока
лётная нода не готова (Linux не загрузился, EKF не прогрет, нет камеры). Пилот делает ТОТ ЖЕ
жест, что для руддер-арма (газ в пол + курс вправо), нода его видит в своих стиках и армит
сервисом, только если готова; дизарм — зеркальный жест (курс влево) на земле. Касается и
ALT_HOLD «только пилот» (SF не вверх): иначе защиту обходил бы один щелчок SF.

Чистое ядро, без ROS: ArmGesture — жест по стикам (удержание HOLD_SEC, как у ArduPilot:
ARM_DELAY 2 с в arm_motors_check), ready_reasons — чего не хватает для арма.
Пороги жеста — те же, что у страховки дизарма Freefly (≤ 1160 / ≥ 1840 PWM).
"""
HOLD_SEC = 2.0
THR_LOW = 1160          # газ «в полу»
YAW_LOW = 1160          # курс «в упор влево»
YAW_HIGH = 1840         # курс «в упор вправо»
FRESH_SEC = 1.0         # свежесть телеметрии/EKF/кадров для готовности


class ArmGesture:
    """Жест арма/дизарма → импульс 'arm'/'disarm' через HOLD_SEC удержания и ПОВТОР
    каждые HOLD_SEC, пока жест держат (а условие ещё в силе: не заармлен / заармлен на
    земле). Повтор нужен, когда нода на первом импульсе не готова (EKF греется): пилот
    держит жест — и нода армит, как только готова; реплей держит жест до арма непрерывно."""

    def __init__(self, hold=HOLD_SEC):
        self.hold = hold
        self._kind = None       # какой жест держат сейчас
        self._since = None

    def update(self, now, thr, yaw, armed, landed):
        kind = None
        if thr <= THR_LOW:
            if yaw >= YAW_HIGH and not armed:
                kind = 'arm'
            elif yaw <= YAW_LOW and armed and landed:
                kind = 'disarm'
        if kind != self._kind:
            self._kind, self._since = kind, now
            return None
        if kind is None or now - self._since < self.hold:
            return None
        self._since = now               # следующий импульс — ещё через HOLD_SEC
        return kind


def ready_reasons(s, att_ready: bool, cam_fresh: bool):
    """Список причин НЕ армить (пусто = готов). VINS не требуется: он инициализируется
    только в полёте."""
    why = []
    if s.now_sim - s.tel_last_sim >= FRESH_SEC:
        why.append("нет телеметрии FCU")
    if s.now_sim - s.ekf_pos_last_sim >= FRESH_SEC:
        why.append("EKF не прогрет (нет local_position)")
    if not att_ready:
        why.append("не прочитаны параметры пересчёта в углы")
    if not cam_fresh:
        why.append("нет кадров камеры")
    return why
