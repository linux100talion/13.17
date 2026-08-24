#!/usr/bin/env python3
# ============================================================================
# frame_anchor.py — якорь кадра VINS на кадр EKF: РЫСКАНЬЕ + трансляция.
# Чистая математика (без ROS) — тестируется офлайн (test_frame_anchor.py).
#
#   corrected = Rz(yaw_off) @ p_vins + t
#
# ЗАЧЕМ ПОВОРОТ (прогоны lv1_joy_20260824_212409/213830). Мир монокулярного
# VINS рождается с курсом ПЕРВОГО кадра камеры (yaw у VIO ненаблюдаем), кадр
# EKF выровнен компасом. Прежний якорь был только трансляционным (допущение
# «рамки выровнены по рысканью» — оно выполнялось, пока борт стартовал носом
# на восток). Спавн с курсом −169° дал мир VINS, повёрнутый к ENU на ~170°:
# каждое смещение VINS входило в EKF почти перевёрнутым, обратная связь
# LOITER стала ПОЛОЖИТЕЛЬНОЙ — разнос с центральных стиков до 15 м/с при
# идеальном масштабе VINS (0.94–1.04). Замер по bag: угол трека VINS к истине
# +169..171° = ровно курс спавна; контрольный прогон со штатным курсом
# (211215) держал LOITER (+8.6°, 3 м).
#
# КАК ЛАТЧИТСЯ Δyaw: по паре ориентаций (yaw EKF − yaw VINS) в момент якоря.
# Обе описывают ОДНО физическое тело в своих кадрах, разность и есть поворот
# кадров; оба кадра гравитационно выровнены — roll/pitch не участвуют.
#
# СЛЕЖЕНИЕ трансляции — прежнее (полёт 2026-08-21 №7): расход > relatch_m —
# жёсткая подтяжка (заново И ПОВОРОТ: расход мог накопиться именно из-за
# ошибки курса, а на GPS-фазе EKF — истина); меньше — мягкий дожим с τ.
# Засечка NN1 правит ТОЛЬКО трансляцию (fix_translation) — yaw-коррекция по
# ориентиру остаётся отдельным шагом (см. nn1_anchor_howto.txt).
# ============================================================================
import math

import numpy as np


def quat_yaw(x, y, z, w):
    """Рысканье ROS-кватерниона (ZYX-эйлер): курс носа в world-кадре, рад."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class FrameAnchor:
    def __init__(self, relatch_m=1.0, tau_sec=5.0):
        self.relatch_m = float(relatch_m)
        self.tau = float(tau_sec)
        self.yaw_off = 0.0            # поворот кадра VINS → кадр EKF, рад
        self.t = np.zeros(3)          # трансляция после поворота, м
        self.latched = False
        self.relatch_n = 0            # счётчик жёстких подтяжек (для лога)
        self._last_wall = 0.0

    def rotate(self, p):
        """Rz(yaw_off) @ p — для позиций и world-скоростей VINS."""
        c, s = math.cos(self.yaw_off), math.sin(self.yaw_off)
        return np.array([c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]])

    def map(self, p):
        """Поза VINS → кадр EKF/ENU."""
        return self.rotate(p) + self.t

    def rotate_quat(self, x, y, z, w):
        """Ориентация VINS → кадр EKF: премножение на Rz(yaw_off).
        Раскрытое умножение кватернионов q_z ⊗ q при чистом Rz."""
        h = 0.5 * self.yaw_off
        zs, zc = math.sin(h), math.cos(h)
        return (zc * x - zs * y,
                zc * y + zs * x,
                zc * z + zs * w,
                zc * w - zs * z)

    def update(self, vins_pos, vins_yaw, ekf_pos, ekf_yaw, now):
        """Свежая пара поз (EKF жив, засечки NN1 нет) → двигаем якорь.
        Возвращает 'latch' | 'relatch' | None (событие для лога)."""
        if not self.latched:
            self.latched = True
            self.yaw_off = _wrap(ekf_yaw - vins_yaw)
            self.t = ekf_pos - self.rotate(vins_pos)
            self._last_wall = now
            return 'latch'
        delta = (ekf_pos - self.rotate(vins_pos)) - self.t
        dn = float(np.linalg.norm(delta))
        dt = min(max(now - self._last_wall, 0.0), 0.5)
        self._last_wall = now
        if self.relatch_m > 0 and dn > self.relatch_m:
            self.yaw_off = _wrap(ekf_yaw - vins_yaw)
            self.t = ekf_pos - self.rotate(vins_pos)
            self.relatch_n += 1
            return 'relatch'
        if self.tau > 0 and dt > 0:
            self.t = self.t + (1.0 - math.exp(-dt / self.tau)) * delta
        return None

    def fix_translation(self, target, vins_pos, alpha=1.0):
        """Засечка NN1 (сброс дрейфа): t так, чтобы map(vins_pos) == target;
        alpha — сглаживание (1 = жёсткий сброс). Поворот НЕ трогаем."""
        new_t = np.asarray(target, dtype=float) - self.rotate(vins_pos)
        self.t = alpha * new_t + (1.0 - alpha) * self.t
