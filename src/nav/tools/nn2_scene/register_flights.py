#!/usr/bin/env python3
# ============================================================================
# register_flights.py — НАБРОСОК: регистрация N облётов в ОДНУ рамку (XXI).
#
# Зачем (XXI): VINS каждого облёта — в СВОЕЙ рамке (свой origin + курс). Чтобы из
# 100 облётов собрать ОДНУ метрическую карту топографа, поз каждого облёта надо
# свести в общий мир (ENU, начало = датум взлёта), чтобы одни и те же места легли
# друг на друга. Иначе метки топографа противоречивы (одно место — N координат).
#
# ── ДАТА-ПАЙПЛАЙН (8 стадий) ────────────────────────────────────────────────
#   1. RECORD     — 100 bag'ов (image_color + VINS odom + IMU + NN1-детекции),
#                   утро/вечер/пасмурно/солнечно, ракурсы, ВСЯ карта.
#   2. EXTRACT    — на облёт: ~2 Гц кадры -> (DINOv2-дескриптор, ЛОКАЛЬНАЯ поза
#                   VINS, t, попадания якорей). [reuse build_scene_map/extract_bags]
#   3. CORRESPOND — связи: (a) якоря NN1 кадр<->георефернс (АБСОЛЮТНЫЕ); (b) кросс-
#                   облётные совпадения мест (ретривал DINOv2/топограф + геом-проверка);
#                   (c) датум взлёта (origin'ы совпадают).
#   4. REGISTER   — грубо: на облёт жёсткое T_i (umeyama по связям) -> все в одну
#                   рамку.  ← ЭТОТ ФАЙЛ (ядро).
#   5. OPTIMIZE   — тонко: pose-graph (одометрия + loop-closure + якоря) -> глобально
#                   согласованные траектории, дрейф выдавлен. (набросок-документация ниже)
#   6. BUILD MAP  — сэмпл из ОПТИМИЗИРОВАННЫХ глобальных поз -> обучающий набор
#                   топографа (дескриптор, ГЛОБАЛЬНАЯ позиция) с согласованными метками.
#   7. TRAIN      — топограф (метрика) -> потом route-головы (вариант c).
#   8. QC         — невязки: кросс-облётные пары мест должны совпасть в пределах X м;
#                   облёты с битым VINS (большая невязка) — пометить и дропнуть/переснять.
#
# Ядро здесь — стадия 4 (грубая регистрация через Umeyama) + QC-невязка. Стадия 5
# (pose-graph) — отдельный солвер (g2o/GTSAM/ceres), здесь только структура графа
# в комментарии. Чистый numpy — тестируется.
# ============================================================================
import numpy as np


def _rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def umeyama_2d(src, dst, with_scale=False):
    """Жёсткое (+опц. масштаб) выравнивание 2D: ищет R,t,c так, что dst ≈ c·R·src+t
    (least squares, Umeyama 1991). src,dst — (M,2) соответствия. -> (R(2,2),t(2,),c).
    Это ядро грубой регистрации: src=локальные позы облёта в точках-связях,
    dst=их же позиции в общей рамке (из якорей/уже-сведённых облётов)."""
    src = np.asarray(src, np.float64)
    dst = np.asarray(dst, np.float64)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    cov = (D.T @ S) / len(src)                     # Σ (dst)(src)^T
    U, sv, Vt = np.linalg.svd(cov)
    W = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:   # запрет отражения
        W[1, 1] = -1.0
    R = U @ W @ Vt
    c = float((sv * np.diag(W)).sum() / (S ** 2).sum() * len(src)) if with_scale else 1.0
    t = mu_d - c * (R @ mu_s)
    return R, t, c


def apply_se2(R, t, pts, c=1.0):
    """Применить (R,t,c) к точкам (N,2): out = c·(pts·Rᵀ) + t."""
    return c * (np.asarray(pts, np.float64) @ R.T) + t


def register_to_global(local_at_corr, global_at_corr, with_scale=False):
    """Грубая регистрация ОДНОГО облёта в общую рамку по соответствиям.
    local_at_corr — локальные позы VINS в точках-связях (M,2) (кадры с якорями
    или кросс-облётными совпадениями); global_at_corr — их позиции в общей рамке
    (M,2): из георефернса якорей или из УЖЕ зарегистрированных облётов.
    Возвращает T=(R,t,c); применяй apply_se2(R,t,all_local,c) ко ВСЕМ позам облёта.
    Нужно ≥2 неколлинеарных связи (лучше 3+)."""
    if len(local_at_corr) < 2:
        raise ValueError("нужно ≥2 соответствия для жёсткой регистрации")
    return umeyama_2d(local_at_corr, global_at_corr, with_scale=with_scale)


def cross_flight_residual(registered, place_ids):
    """QC (стадия 8): средняя/макс невязка между копиями ОДНОГО места из разных
    облётов после регистрации. registered — list of (poses(Ni,2), ids(Ni,)) уже в
    общей рамке; place_ids — какие id мест проверять. Большая невязка = облёт не
    сошёлся (битый VINS / мало связей) -> кандидат на дроп/переснять."""
    bucket = {}
    for poses, ids in registered:
        for p, i in zip(poses, ids):
            bucket.setdefault(int(i), []).append(p)
    errs = []
    for i in place_ids:
        pts = np.array(bucket.get(int(i), []))
        if len(pts) >= 2:
            errs.append(np.linalg.norm(pts - pts.mean(0), axis=1).max())
    errs = np.array(errs) if errs else np.array([0.0])
    return float(errs.mean()), float(errs.max())


# ── СТАДИЯ 5 (тонко): POSE-GRAPH — структура (солвер внешний: GTSAM/g2o/ceres) ──
#   Узлы:  кейфреймы поз ВСЕХ облётов (SE2/SE3).
#   Рёбра: (1) ОДОМЕТРИЯ внутри облёта (из VINS) — последовательные;
#          (2) LOOP-CLOSURE внутри/между облётами (ретривал мест + геом-проверка)
#              — «эти два кейфрейма = одно место»;
#          (3) ЯКОРЬ NN1 — «кейфрейм в этой АБСОЛЮТНОЙ георефернс-позиции»;
#          (4) ДАТУМ взлёта — фиксирует калибровку (gauge).
#   Минимизируем сумму невязок рёбер на многообразии -> согласованные глобальные
#   позы. Грубая регистрация (этот файл) даёт ХОРОШУЮ НАЧАЛЬНУЮ ТОЧКУ для этого
#   нелинейного МНК (иначе оптимизатор уйдёт в локальный минимум).


def _selftest():
    rng = np.random.default_rng(0)
    # ГЛОБАЛЬНАЯ истина: 60 мест на карте 200x120 м
    G = rng.uniform([0, 0], [200, 120], (60, 2))
    anchors = rng.choice(60, 5, replace=False)             # 5 георефернс-якорей (NN1)

    def make_flight(seed, m=25):
        r = np.random.default_rng(seed)
        idx = np.unique(np.concatenate([r.choice(60, m, replace=False), anchors[:3]]))
        Rf, tf = _rot(r.uniform(-np.pi, np.pi)), r.uniform([-40, -40], [40, 40])
        # ЛОКАЛЬНЫЕ позы VINS = глобальные, повёрнутые/сдвинутые в рамку облёта + шум
        local = (G[idx] - tf) @ Rf + r.normal(0, 0.5, (len(idx), 2))
        return idx, local

    flights = [make_flight(s) for s in (1, 2, 3, 4, 5)]

    # БЕЗ регистрации: локальные координаты разных облётов разбросаны
    raw = [(loc, idx) for idx, loc in flights]
    raw_mean, raw_max = cross_flight_residual(raw, range(60))

    # РЕГИСТРАЦИЯ: каждый облёт -> общая рамка по ЯКОРЯМ (local<->глобальный георефернс)
    reg = []
    for idx, loc in flights:
        amask = np.isin(idx, anchors)                      # кадры облёта, видевшие якоря
        R, t, c = register_to_global(loc[amask], G[idx[amask]])
        reg.append((apply_se2(R, t, loc, c), idx))         # ВСЕ позы облёта -> в мир
    reg_mean, reg_max = cross_flight_residual(reg, range(60))

    # точность абсолютной привязки: зарег. место vs глобальная истина
    abs_err = np.mean([np.linalg.norm(p - G[i])
                       for poses, ids in reg for p, i in zip(poses, ids)])

    assert reg_mean < 2.0 and reg_max < 4.0, (reg_mean, reg_max)
    assert raw_mean > 30.0, raw_mean
    print("register_flights selftest: OK")
    print(f"  кросс-облётная невязка ДО:    среднее {raw_mean:6.1f} м, макс {raw_max:6.1f} м")
    print(f"  кросс-облётная невязка ПОСЛЕ: среднее {reg_mean:6.2f} м, макс {reg_max:6.2f} м")
    print(f"  абс. ошибка vs глоб. истина: {abs_err:.2f} м (по якорям, 5 облётов)")
    print("  -> 5 облётов сведены в одну рамку; QC-невязка мала (битые отсеялись бы)")


if __name__ == "__main__":
    _selftest()
