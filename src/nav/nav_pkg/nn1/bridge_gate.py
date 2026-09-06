#!/usr/bin/env python3
# ============================================================================
# bridge_gate.py — ГЕЙТ ЗДОРОВЬЯ МОСТА VINS→EKF (ray_tracer). Чистая логика без
# ROS — тестируется офлайн (src/nav/test/test_bridge_gate.py).
#
# ЗАЧЕМ (полёт lv2_joy_20260906_142811, cmd/6): VINS разнёсся сразу после init на
# висении (|v| 1.3 → 49 м/с за 18 с, 111 скачков позы), а мост ray_tracer, не зная
# о здоровье, 687 раз «подтянул» якорь кадра (расход > 1 м на каждой одометрии) и
# на каждой одометрии публиковал /mavros/vision_pose/pose = позиция EKF в момент
# подтяжки + мусорный прирост, ориентация VINS повёрнутая на Δyaw до +19°. EKF3
# отравился: ориентация AHRS против истины ушла на 6–12° по крену/тангажу и
# 20–33° по курсу, и НЕ вернулась после выздоровления VINS (якорь остался с Δyaw
# +18°, свежего латча не было). На кривой ориентации не работает ничего: канал
# IPM считает дерото по ней (продольная ось ослепла), ALT_HOLD держит ложный
# горизонт (авторитет демпфера 150 PWM ≈ 9–13° съеден ошибкой целиком) — DpHold
# унесло на 8 м/с. Гейт здоровья лётной ноды (handover.vins_sane) защищал только
# ЯРУС; мост кормил полётник без всякой проверки.
#
# ЧТО ДЕЛАЕТ. Мост ЗАКРЫТ (vision_pose не публикуется, якорь заморожен), если:
#   - |twist| > v_max (12 м/с — потолок гейта здоровья ноды, честный борт столько
#     не летает);
#   - ПЕРЕРОЖДЕНИЕ потока: дыра штампов > gap_sec или скачок позы быстрее v_jump
#     между соседними одометриями (детект VinsTrack лётной ноды, 1:1) — новая
#     рама → якорь надо латчить ЗАНОВО (relatch_pending), не подтягивать;
#   - ШТОРМ ПОДТЯЖЕК: ≥ relatch_n жёстких подтяжек за relatch_win с — расход
#     > relatch_m на каждой одометрии бывает только у разноса (норма: 0–2 за
#     полёт); рама после шторма — мусор → латч заново;
#   - внешний вердикт лётной ноды (/vins/sane = False) — когда нода есть и её
#     сообщение свежее; на Orin без ноды мост живёт только своими проверками.
# Закрытие держится hold_sec после последней причины (латч гейта); открытие —
# только при здоровом потоке. Счётчики closes/rebirths — в /nn1/bridge и
# статус (brg=/brw=/brl=/brc=).
# ============================================================================
import math


class BridgeGate:
    def __init__(self, v_max=12.0, v_jump=12.0, gap_sec=1.0, relatch_n=3,
                 relatch_win=5.0, hold_sec=5.0):
        self.v_max = float(v_max)
        self.v_jump = float(v_jump)
        self.gap_sec = float(gap_sec)
        self.relatch_n = int(relatch_n)
        self.relatch_win = float(relatch_win)
        self.hold_sec = float(hold_sec)
        # состояние
        self._prev = None               # (t, x, y) предыдущей одометрии
        self._closed_until = -math.inf  # мост закрыт до этого времени
        self._relatch_ts = []           # штампы подтяжек в окне
        self.reason = '-'               # причина последнего закрытия
        self.closes = 0                 # закрытий за полёт
        self.rebirths = 0               # перерождений потока
        self.relatch_pending = False    # якорь надо латчить заново

    # --- события ------------------------------------------------------------
    def on_odom(self, t, x, y, speed, ext_sane=None) -> bool:
        """Одометрия VINS: t — штамп (с), x/y — поза, speed — |twist_xy| (м/с),
        ext_sane — свежий вердикт лётной ноды (None = ноды нет/протух).
        Возвращает: мост ОТКРЫТ."""
        reborn = False
        if self._prev is not None:
            pt, px, py = self._prev
            dt = t - pt
            if dt > 0.0:
                if dt > self.gap_sec or math.hypot(x - px, y - py) / dt > self.v_jump:
                    reborn = True
        self._prev = (t, x, y)
        if reborn:
            self.rebirths += 1
            self.relatch_pending = True
            self._close(t, 'reborn')
        if speed > self.v_max:
            self._close(t, f'v{speed:.0f}')
        if ext_sane is False:
            self._close(t, 'ext')
        return self.is_open(t)

    def on_relatch(self, t) -> bool:
        """Жёсткая подтяжка якоря. Возвращает: мост ЗАКРЫЛСЯ штормом подтяжек."""
        self._relatch_ts = [x for x in self._relatch_ts if t - x <= self.relatch_win]
        self._relatch_ts.append(t)
        if len(self._relatch_ts) >= self.relatch_n:
            self.relatch_pending = True
            self._relatch_ts = []
            self._close(t, 'relatch')
            return True
        return False

    def take_relatch(self) -> bool:
        """Снять флаг «латчить заново» (один раз)."""
        p = self.relatch_pending
        self.relatch_pending = False
        return p

    def is_open(self, t) -> bool:
        return t >= self._closed_until

    def reset(self):
        """Наш /restart VINS: поток объявлен новым (следующая одометрия не
        сравнивается с прошлой — она из другой рамы), якорь заново."""
        self._prev = None
        self.relatch_pending = True

    # --- внутреннее -----------------------------------------------------------
    def _close(self, t, reason):
        if self.is_open(t):
            self.closes += 1
        self.reason = reason
        self._closed_until = max(self._closed_until, t + self.hold_sec)

    def state_line(self, t) -> str:
        """Строка /nn1/bridge: 'open|closed <причина> <подтяжек в окне> <закрытий> <перерождений>'."""
        return (f"{'open' if self.is_open(t) else 'closed'} {self.reason} "
                f"{len(self._relatch_ts)} {self.closes} {self.rebirths}")
