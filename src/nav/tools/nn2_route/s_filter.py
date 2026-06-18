#!/usr/bin/env python3
# ============================================================================
# s_filter.py — ПОСЛЕДОВАТЕЛЬНЫЙ фильтр прогресса по маршруту s∈[0,1] (HMM/гистограмма).
#
# Зачем (XVIII, разбор top-1 vs kNN): засечка s по одному кадру шумна и АЛЯЙСИТСЯ —
# два похожих места маршрута дают «телепорт» s. Один кадр это не различит. Но s
# вдоль маршрута меняется ПЛАВНО и ВПЕРЁД, поэтому время разводит дубли:
#   - вера живёт РАСПРЕДЕЛЕНИЕМ по s (а не точкой) -> может быть мультимодальной,
#     пока неоднозначно; форвард-движение (predict) сдвигает её и разрешает;
#   - робастное правдоподобие (пол p_outlier) НЕ даёт аляйснутому «телепорту»
#     обнулить правильную область -> выброс отвергается, если вера уже сошлась.
# Вход s — из route-головы s(descriptor) или проекции метрики; шум σ раздуваем,
# когда страж kNN сработал (msrc=top1-fallback) или mconf низкий / разброс велик.
#
# Гистограммный фильтр (Байес на сетке): predict = сдвиг вперёд + диффузия,
# update = умножение на гауссово правдоподобие. Чистый numpy — тестируется.
#
# Куда встаёт: в relocalizer_field перед использованием s — фильтруем сырое s,
# берём s_filt для поля −∇V/засечки и гейтим по conf (пиковости) веры.
# ============================================================================
import numpy as np


def _norm(b):
    return b / (b.sum() + 1e-12)


def _shift(b, ds, grid, clamp):
    """Сдвиг распределения ВПЕРЁД на ds (в долях s): new(s)=old(s-ds). Интерполяцией.
    clamp -> масса за краями [0,1] прижимается к границе (маршрут не циклический)."""
    src = grid - ds
    if clamp:
        src = np.clip(src, grid[0], grid[-1])
    return _norm(np.interp(src, grid, b))


def _gauss_blur(b, sigma_bins):
    """Диффузия = свёртка с гауссом (края — edge-pad)."""
    if sigma_bins <= 1e-6:
        return b
    r = int(max(1, round(3 * sigma_bins)))
    x = np.arange(-r, r + 1)
    k = np.exp(-0.5 * (x / sigma_bins) ** 2)
    k /= k.sum()
    return np.convolve(np.pad(b, r, mode="edge"), k, "valid")


class SFilter:
    """Гистограммный фильтр s∈[0,1]. belief — распределение по nbins.

    predict(ds): прогресс вперёд (ds — ожидаемая доля маршрута за шаг = v·dt/L) +
                 диффузия (неопределённость модели);
    update(z,σ): байес-апдейт гауссовым правдоподобием вокруг засечки z; робастный
                 пол p_outlier держит мультимодальность и гасит аляйснутый выброс;
    estimate():  s (среднее), s_map (мода), std, conf (пиковость), bimodal.
    """
    def __init__(self, nbins=200, diffusion=0.01, p_outlier=0.05, clamp=True):
        self.nb = int(nbins)
        self.grid = (np.arange(self.nb) + 0.5) / self.nb     # центры бинов в [0,1]
        self.belief = np.ones(self.nb) / self.nb             # старт: не знаем где
        self.diffusion = float(diffusion)                    # σ диффузии за шаг (доли s)
        self.p_outlier = float(p_outlier)                    # робастный пол правдоподобия
        self.clamp = bool(clamp)

    def reset(self):
        self.belief = np.ones(self.nb) / self.nb

    def predict(self, ds=0.0, extra_diffusion=0.0):
        b = _shift(self.belief, ds, self.grid, self.clamp)
        b = _gauss_blur(b, (self.diffusion + extra_diffusion) * self.nb)
        self.belief = _norm(b)

    def update(self, z, sigma):
        like = np.exp(-0.5 * ((self.grid - float(z)) / max(float(sigma), 1e-3)) ** 2)
        like = (1.0 - self.p_outlier) * like + self.p_outlier   # робастная смесь
        self.belief = _norm(self.belief * like)

    def step(self, z, conf, ds=0.0, sigma_base=0.02, alias=False, extra_diffusion=0.0):
        """predict+update+estimate за раз. σ засечки растёт при низком conf и при
        срабатывании стража алиасинга (alias=True -> ещё шире)."""
        self.predict(ds, extra_diffusion)
        sigma = sigma_base / max(float(conf), 0.05)
        if alias:
            sigma *= 3.0
        self.update(z, sigma)
        return self.estimate()

    # σ равномерного распределения на [0,1] (= 1/√12); им нормируем «пиковость».
    _STD_UNIFORM = 1.0 / np.sqrt(12.0)

    def estimate(self):
        b = self.belief
        s_mean = float((b * self.grid).sum())
        s_map = float(self.grid[int(np.argmax(b))])
        std = float(np.sqrt(max((b * (self.grid - s_mean) ** 2).sum(), 0.0)))
        # уверенность = насколько вера у́же равномерной: 1 (острый пик) .. 0 (плоско).
        conf = float(np.clip(1.0 - std / self._STD_UNIFORM, 0.0, 1.0))
        return {"s": s_mean, "s_map": s_map, "std": std, "conf": conf,
                "bimodal": self._bimodal()}

    def _bimodal(self):
        """Груб. детектор двух мод: ≥2 кластера бинов выше 0.5·max, разделённых
        провалом. Сигнал «неоднозначно, ждём движение»."""
        b = self.belief
        hi = b > 0.5 * b.max()
        groups = np.diff(np.concatenate([[0], hi.astype(int), [0]]))
        return int((groups == 1).sum()) >= 2


# --- самопроверка: трекинг сквозь АЛИАСИНГ (только numpy) --------------------
def _selftest():
    rng = np.random.default_rng(0)
    N = 120
    s_true = np.clip(np.linspace(0.0, 1.0, N), 0, 1)     # дрон идёт по маршруту 0->1
    ds = 1.0 / (N - 1)                                   # известный прогресс/шаг (v·dt/L)

    f = SFilter(nbins=200, diffusion=0.01, p_outlier=0.05)
    raw_err, filt_err, rejected = [], [], 0
    for k in range(N):
        z = s_true[k] + rng.normal(0, 0.01)             # обычная шумная засечка
        aliased = rng.random() < 0.15                   # 15% — перцептуальный алиасинг:
        if aliased:
            z = (s_true[k] + 0.5) % 1.0                  # «телепорт» в похожее место
        out = f.step(z, conf=0.9, ds=ds, sigma_base=0.02, alias=False)
        raw_err.append(abs(((z - s_true[k] + 0.5) % 1.0) - 0.5))   # цикл. ошибка сырого
        filt_err.append(abs(out["s"] - s_true[k]))
        if aliased and abs(out["s"] - s_true[k]) < 0.1:
            rejected += 1

    raw_mae = float(np.mean(raw_err))
    filt_mae = float(np.mean(filt_err))
    assert filt_mae < raw_mae * 0.5, (raw_mae, filt_mae)   # фильтр кратно лучше
    assert filt_err[-1] < 0.05, filt_err[-1]               # в конце сошёлся к s≈1
    print("s_filter selftest: OK")
    print(f"  сырое s MAE: {raw_mae:.3f} (15% кадров аляйснуты на 0.5)")
    print(f"  фильтр  MAE: {filt_mae:.3f}  ({raw_mae/max(filt_mae,1e-9):.1f}× точнее)")
    print(f"  аляйснутых выбросов отвергнуто (|ошибка|<0.1): {rejected}")

    # неоднозначный старт: равномерная вера -> bimodal/низкий conf, пока не сошлась
    g = SFilter(nbins=200)
    e0 = g.estimate()
    assert e0["conf"] < 0.2, e0                  # старт: ничего не знаем (плоско)
    for _ in range(8):
        g.step(0.30, conf=0.9, ds=0.0)           # повторные согласные засечки
    e1 = g.estimate()
    assert e1["conf"] > 0.7 and abs(e1["s"] - 0.30) < 0.03, e1
    print(f"  сходимость: conf {e0['conf']:.2f} -> {e1['conf']:.2f}, s_map={e1['s_map']:.2f}")


if __name__ == "__main__":
    _selftest()
