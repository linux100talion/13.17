"""Поправка высоты по зуму земли в канале вида сверху (BS_IPM_ALT_EST, 2026-10-02).

Основная высота — барометрическая цепочка (высота перцепции + клиренс корпуса + вынос
камеры → `h_a`, по ней канал строит геометрию). Камера меряет ПОПРАВКУ к ней: если
истинная высота камеры над землёй `h_a + δ`, то при наборе/снижении земля «зумится» не
так, как ждёт геометрия, и продольный ход точек полосы растёт с их дальностью:

    ход_i = t + ε·X_i,   ε = Δln(h_a + δ) − Δln h_a ≈ δ·g,   g = 1/h_a(сейчас) − 1/h_a(прошлый)

(t — настоящий ход борта; ε мерит канал, см. ipm._ipm_metric_step). δ — линейный МНК
по кадрам с экспоненциальным забыванием τ (дрейф барометра, смена площадки), шум ε —
из невязок, отсюда собственная погрешность σ_δ. На висении g ≈ 0: δ не наблюдается,
оценка держится (и медленно забывается), σ_δ растёт. Лучший сигнал — взлёт: h мала.

Пока ТОЛЬКО НАБЛЮДЕНИЕ: демпфер летит на h_a, оценка уходит в HUD (vis/vsig) — смотреть
расхождение барометрической цепочки с камерой. Ровная земля под полосой — допущение:
уклон/кочка выглядят как изменение δ (для канала это, скорее, правильно)."""
import math
from collections import deque

import numpy as np


class GroundOffsetEstimator:
    """δ — поправка высоты камеры над землёй к принятой геометрией, м.

    Нелинейный МНК: ε_k = ln((h1_k+δ)/(h0_k+δ)) − ln(h1_k/h0_k), веса exp(−возраст/τ);
    решается несколькими шагами Гаусса — Ньютона по кадрам окна (≤ 3τ) на каждом
    обновлении. Линейная версия с накопленными суммами (регрессор у принятой высоты)
    занижала |δ| на 10–20 % при δ/h ≈ 0.2 (синтетика test_ipm_alt_est)."""

    def __init__(self, tau_forget=60.0, eps_max=0.2, iters=3, h_min=0.6, model_sd=0.07):
        self.tau_forget = float(tau_forget)
        self.eps_max = float(eps_max)      # |ε| за кадр выше — сбой LK, не зум
        self.iters = int(iters)
        # у земли зум меряется плохо: полёт bl_vinit_vis_20261002_134434 — на посадке
        # поправка ушла +0.125 → +0.25 за последние секунды. Кадры ниже h_min не берём.
        self.h_min = float(h_min)
        # ОШИБКА МОДЕЛИ в σ: оцениваем ПОСТОЯННУЮ поправку, а истинная (ошибка высоты EKF)
        # блуждает за секунды — СКО 0.04–0.076 м по bag vzph_s1/vzph_sc/bl_vinit (2026-10-02).
        # Без неё покрытие |ошибка| < 2σ было 9–19 % (σ ~0.01 при реальной ±0.1).
        self.model_sd = float(model_sd)
        # МИНИМУМ ИНФОРМАЦИИ Σw·J²: без хода по высоте δ не наблюдается, и Гаусс — Ньютон
        # на одних висячих кадрах (J ≈ 0) уходил в бесконечность (реплей с середины висения).
        # 1e-3 ≈ несколько кадров набора 0.5 м/с на метре. Ниже — оценки нет (HUD vis --).
        self.info_min = 1e-3
        self.reset()

    def reset(self):
        self._buf = deque()                 # (t, ε, h0, h1)
        self.delta = None
        self.sigma = None
        self.rho = 0.0
        self.n = 0

    def update(self, t, eps, h0, h1):
        """Кадр: ε (мера зума), h0/h1 — принятая высота камеры опорного/текущего кадра."""
        if eps is None or not math.isfinite(eps) or abs(eps) > self.eps_max:
            return
        if min(h0, h1) < self.h_min:
            return
        self._buf.append((float(t), float(eps), float(h0), float(h1)))
        while self._buf and t - self._buf[0][0] > 3.0 * self.tau_forget:
            self._buf.popleft()
        self.n = len(self._buf)
        b = np.array(self._buf)
        w = np.exp(-(t - b[:, 0]) / self.tau_forget)
        e, h0a, h1a = b[:, 1], b[:, 2], b[:, 3]
        base = np.log(h1a / h0a)
        d = self.delta if self.delta is not None else 0.0
        for _ in range(self.iters):
            d = max(d, -0.9 * float(min(h0a.min(), h1a.min())))   # h + δ > 0
            r = e - (np.log((h1a + d) / (h0a + d)) - base)
            J = 1.0 / (h1a + d) - 1.0 / (h0a + d)
            info = float(np.sum(w * J * J))
            if info < self.info_min:
                return                         # δ не наблюдается — прежняя оценка (или нет)
            d += max(-0.5, min(0.5, float(np.sum(w * J * r)) / info))   # шаг ≤ 0.5 м
        r = e - (np.log((h1a + d) / (h0a + d)) - base)
        J = 1.0 / (h1a + d) - 1.0 / (h0a + d)
        info = float(np.sum(w * J * J))
        var_e = float(np.sum(w * r * r) / max(np.sum(w), 1e-9))   # шум ε за кадр
        # ЧЕСТНАЯ σ: соседние кадры не независимы (LK по почти той же картинке, сглаженные
        # углы) — без поправки σ выходила 0.007 при реальной ошибке ±0.1 (реплей). Невязки
        # как AR(1): ρ — их автокорреляция с шагом в кадр, дисперсия оценки ×(1+ρ)/(1−ρ).
        self.rho = 0.0
        if len(r) > 10:
            num = float(np.sum(w[1:] * r[1:] * r[:-1]))
            den = float(np.sum(w[1:] * r[1:] * r[1:]))
            if den > 0.0:
                self.rho = min(max(num / den, 0.0), 0.98)
        infl = (1.0 + self.rho) / (1.0 - self.rho)
        self.delta = d
        self.sigma = (math.sqrt(var_e * infl / info + self.model_sd ** 2)
                      if info > 0.0 else None)
