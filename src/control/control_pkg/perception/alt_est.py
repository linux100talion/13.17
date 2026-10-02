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

    def __init__(self, tau_forget=60.0, eps_max=0.2, iters=3):
        self.tau_forget = float(tau_forget)
        self.eps_max = float(eps_max)      # |ε| за кадр выше — сбой LK, не зум
        self.iters = int(iters)
        self.reset()

    def reset(self):
        self._buf = deque()                 # (t, ε, h0, h1)
        self.delta = None
        self.sigma = None
        self.n = 0

    def update(self, t, eps, h0, h1):
        """Кадр: ε (мера зума), h0/h1 — принятая высота камеры опорного/текущего кадра."""
        if eps is None or not math.isfinite(eps) or abs(eps) > self.eps_max:
            return
        if h0 <= 0.05 or h1 <= 0.05:
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
            if info <= 0.0:
                return
            d += float(np.sum(w * J * r)) / info
        r = e - (np.log((h1a + d) / (h0a + d)) - base)
        J = 1.0 / (h1a + d) - 1.0 / (h0a + d)
        info = float(np.sum(w * J * J))
        var_e = float(np.sum(w * r * r) / max(np.sum(w), 1e-9))   # шум ε за кадр
        self.delta = d
        self.sigma = math.sqrt(var_e / info) if info > 0.0 else None
