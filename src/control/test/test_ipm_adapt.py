#!/usr/bin/env python3
"""Юнит-тест АДАПТИВНОЙ ПОЛОСЫ IPM (`ipm_adapt`).

Зачем. У статичной полосы 3-6 м есть потолок высоты x1·tan(cam_tilt+vfov/2) ≈ 5.85 м:
выше полоса целиком уходит под нижний край кадра, ipm_ok=0, демпферы крена/тангажа
слепы, стики пилота на этих осях мертвы. Полёт 2026-08-18: провал ipm_ok длиной
63.7 с — набор на 10 м, зависание на 6.0 м (15 см выше потолка), канал ожил на 5.2 м.

Адаптив сдвигает начало окна за границу видимости (длина окна та же), а известный
сдвиг окна между кадрами вычитает из продольного накопителя — иначе скольжение окна
на наборе читалось бы как ход вперёд.

Сцена — ЧЕСТНЫЙ синтетический рендер: текстура земли проецируется в кадр гомографией,
построенной через тот же `_ipm_px`, что и выпрямление. Значит тест проверяет реальную
геометрию канала, а не согласие функции с самой собой по одной ветке.

Запуск:  python3 src/control/test/test_ipm_adapt.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2                                                            # noqa: E402
import numpy as np                                                    # noqa: E402

from control_pkg.perception.flow_estimator import FlowEstimator       # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


FX = FY = 640.0
CX, CY = 640.0, 360.0
FPS_DT = 1.0 / 30.0

# Земля: 1 см/пиксель, X 0..22 м (строки), Y ±8.5 м (столбцы) — все окна внутри
rng = np.random.default_rng(1317)
GROUND = rng.integers(0, 255, (2200, 1700), dtype=np.uint8)
GROUND = cv2.GaussianBlur(GROUND, (3, 3), 0)    # LK любит градиенты, не соль-перец


def g_px(X, Y):
    return [(Y + 8.5) / 0.01, (22.0 - X) / 0.01]


def make(adapt):
    return FlowEstimator(FX, FY, CX, CY, np.eye(3), ipm_adapt=adapt)


def render(e, h, dx=0.0):
    """Кадр камеры на высоте h над землёй; dx — уход борта вперёд (сцена назад)."""
    src, dst = [], []
    for X, Y in ((3.0, -8.0), (3.0, 8.0), (20.0, 8.0), (20.0, -8.0)):
        dst.append(e._ipm_px(X, Y, h, 0.0, 0.0))
        src.append(g_px(X + dx, Y))
    M = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
    return cv2.warpPerspective(GROUND, M, (1280, 720))


def feed(e, h, t, dx=0.0):
    e._ipm_update(render(e, h, dx), t, h, 0.0, 0.0)


# --- 1. ПОТОЛОК: на 6.5 м статичная полоса мертва, адаптивная жива ---
e0 = make(0.0)
feed(e0, 6.5, 0.0)
feed(e0, 6.5, FPS_DT)
check("статичная полоса на 6.5 м мертва (полоса под кадром, ipm_ok=False)",
      not e0.ipm_ok)
ea = make(1.05)
feed(ea, 6.5, 0.0)
feed(ea, 6.5, FPS_DT)
check("адаптивная полоса на 6.5 м жива (ipm_ok=True)", ea.ipm_ok)
check("окно отодвинуто за базовое (x0 > 6 м)", ea._ipm_prev_x0 > 6.0)
check("неподвижный борт → путь ~0",
      abs(ea.ipm_fwd) < 0.05 and abs(ea.ipm_lat) < 0.05)

# --- 2. РЕГРЕСС на рабочей высоте 3 м: оба режима живы, окно почти на месте ---
for adapt, name in ((0.0, "статичная"), (1.05, "адаптивная")):
    e = make(adapt)
    feed(e, 3.0, 0.0)
    feed(e, 3.0, FPS_DT)
    check(f"{name} полоса на 3 м жива, путь ~0",
          e.ipm_ok and abs(e.ipm_fwd) < 0.05)
ea3 = make(1.05)
feed(ea3, 3.0, 0.0)
check("на 3 м адаптив сдвигает окно лишь на сантиметры (<0.4 м)",
      3.0 <= ea3._ipm_prev_x0 < 3.4)

# --- 3. КОМПЕНСАЦИЯ СДВИГА ОКНА: честный набор 5→6 м на месте не рождает хода ---
# 30 кадров по 3.3 см набора (1 м/с). Окно уезжает на ~1.07 м; без вычитания
# (x0 − prev_x0) накопитель показал бы ровно этот фантом «вперёд».
ec = make(1.05)
t, h = 0.0, 5.0
for k in range(31):
    feed(ec, h, t)
    t += FPS_DT
    h += 1.0 * FPS_DT
drift = ec.ipm_fwd
check(f"набор 5→6 м на месте: фантом хода |{drift:+.3f}| < 0.2 м (без вычитания ~1.07)",
      abs(drift) < 0.2)

# --- 4. МЕТРИКА НА ВЫСОТЕ: на 6.5 м (выше старого потолка) ход меряется метрами ---
em = make(1.05)
t, dx = 0.0, 0.0
for k in range(16):
    feed(em, 6.5, t, dx)
    t += FPS_DT
    dx += 1.0 * FPS_DT          # 1 м/с вперёд, итог 0.5 м
check(f"ход 0.50 м на 6.5 м высоты: намеряно {em.ipm_fwd:+.3f} (±30%)",
      0.35 < em.ipm_fwd < 0.65)
check("боковой канал при этом молчит", abs(em.ipm_lat) < 0.1)

# --- 5. КОМПЛЕМЕНТАРНЫЙ ФИЛЬТР СКОРОСТИ (ipm_vel_tau) ---
# Прогноз наклоном тяги тикает каждый кадр (мостит провалы), МНК-наклон корректирует.
# Знаки наклона здесь НЕ проверяются (это полётная валидация) — проверяется механика:
# провал мостится, свежесть держится _VEL_HOLD, коррекция стягивает к измерению.
BLACK = np.zeros((720, 1280), dtype=np.uint8)
ef = FlowEstimator(FX, FY, CX, CY, np.eye(3), ipm_adapt=1.05, ipm_vel_tau=0.4)
t = 0.0
for _ in range(20):                                   # разгон фильтра на годных кадрах
    ef._ipm_update(render(ef, 5.0), t, 5.0, 0.0, 0.0)
    t += FPS_DT
check("фильтр: статичная сцена → скорость ~0", ef.ipm_ok and abs(ef.ipm_vfwd) < 0.1)
for _ in range(18):                                   # провал 0.6с + наклон нос-вниз 0.05
    ef._ipm_update(BLACK, t, 5.0, 0.05, 0.0)
    t += FPS_DT
check(f"фильтр: провал 0.6с мостится прогнозом (ipm_ok=True, v={ef.ipm_vfwd:+.2f})",
      ef.ipm_ok and 0.15 < ef.ipm_vfwd < 0.45)
for _ in range(18):                                   # провал тянется дальше 1.0с
    ef._ipm_update(BLACK, t, 5.0, 0.05, 0.0)
    t += FPS_DT
check("фильтр: провал дольше _VEL_HOLD → ipm_ok=False (слепые не командуют)",
      not ef.ipm_ok)
v_drift = ef._ipm_v[0]
for _ in range(45):                                   # измерения вернулись, наклон 0
    ef._ipm_update(render(ef, 5.0), t, 5.0, 0.0, 0.0)
    t += FPS_DT
check(f"фильтр: коррекция стянула дрейф {v_drift:+.2f} → {ef.ipm_vfwd:+.2f} (<0.15)",
      ef.ipm_ok and abs(ef.ipm_vfwd) < 0.15)
ef._ipm_update(BLACK, t, 0.3, 0.0, 0.0)              # сели: alt < 0.5
check("фильтр: на земле состояние сброшено в 0", ef._ipm_v == [0.0, 0.0])
# tau=0 (дефолт) — прежнее поведение: все тесты выше по файлу шли без фильтра

# --- 6. КОД ПРИЧИНЫ БРАКА (ipm_fail): какой return бьёт — видно из bag ---
# Мотив: разбор LV2/1 (2026-08-25) — 31% слепоты канала на 1.1 м, причины по
# бинарному ipm_ok неразличимы. Каждый ранний return _ipm_update помечен кодом;
# здесь каждая ветка вызывается НАРОЧНО и проверяется её код.
from control_pkg.perception.flow_estimator import ipm_dbg_z           # noqa: E402

er = make(1.05)
check("код: до первого кадра — 7 (нет опоры)", er.ipm_fail == 7)
er._ipm_update(render(er, 3.0), 0.00, 0.3, 0.0, 0.0)
check("код: alt 0.3 < 0.5 → 1 (гейт высоты)",
      not er.ipm_ok and er.ipm_fail == 1)
er._ipm_update(render(er, 3.0), 0.03, None, 0.0, 0.0)
check("код: alt None → 1 (гейт высоты)", er.ipm_fail == 1)
er._ipm_update(render(er, 3.0), 0.06, 3.0, 0.0, 0.0)
check("код: первый кадр после сброса → 7 (нет опоры)", er.ipm_fail == 7)
er._ipm_update(render(er, 3.0), 0.09, 3.0, 0.0, 0.0)
check("код: годный кадр → 0, ipm_ok=True", er.ipm_ok and er.ipm_fail == 0)
er._ipm_update(BLACK, 0.12, 3.0, 0.0, 0.0)
check("код: поток из текстуры в черноту → 5 (мало выживших LK)",
      not er.ipm_ok and er.ipm_fail == 5)
er._ipm_update(BLACK, 0.15, 3.0, 0.0, 0.0)
check("код: опора чёрная → 4 (мало фич)", er.ipm_fail == 4)
er._ipm_update(render(er, 3.0), 0.18, 3.0, -0.8, 0.0)
check("код: взгляд у горизонта → 2 (окно не видно)", er.ipm_fail == 2)
e3 = make(0.0)   # adapt=0: окно не двигается — до варпа доходим и за камерой
e3._ipm_update(render(e3, 3.0), 0.0, 3.0, -1.2, 0.0)
check("код: точка полосы за камерой → 3 (варп за кадром)", e3.ipm_fail == 3)
e6 = FlowEstimator(FX, FY, CX, CY, np.eye(3), ipm=False)
e6._ipm_update(BLACK, 0.0, 3.0, 0.0, 0.0)
check("код: канал выключен → 6", e6.ipm_fail == 6)

# кодировка в /flow_dbg8.z (одна правда с ros_io.publish_axes): годный 1.0
# бит-в-бит; брак −код; фильтр держит ok на браке → 1.0+код/10. Совместимость:
# «z>0.5 = ок» верна на старых (0/1) и новых значениях.
check("z: годный кадр = ровно 1.0", ipm_dbg_z(True, 0) == 1.0)
check("z: брак → −код (гейт высоты −1.0)", ipm_dbg_z(False, 1) == -1.0)
check("z: все коды брака ниже порога 0.5",
      all(ipm_dbg_z(False, c) < 0.5 for c in range(1, 8)))
check("z: фильтр мостит брак → 1.0+код/10 (>0.5, код восстановим)",
      ipm_dbg_z(True, 5) == 1.5 and ipm_dbg_z(True, 5) > 0.5
      and round((ipm_dbg_z(True, 5) - 1.0) * 10.0) == 5)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ АДАПТИВНАЯ ПОЛОСА IPM OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
