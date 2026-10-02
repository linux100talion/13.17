#!/usr/bin/env python3
"""Юнит-тест ВЫНОСА КАМЕРЫ в канале вида сверху (`cam_lever`, BS_IPM_LEVER).

Камера стоит не в точке борта (IMU), а на выносе t (борт: 0.14 вперёд, 0.06 ниже).
Сцена — ЧЕСТНЫЙ рендер: камера ставится туда, где она физически находится при данных
курсе/тангаже борта (центр борта + R·t), текстура земли проецируется в кадр через
тот же `_ipm_px`, что и выпрямление. Оценщик получает углы и высоту БОРТА.

  A. Разворот на месте (борт стоит): вынесенная вперёд камера едет вбок на t_x·dψ.
     Без учёта выноса канал видит ложный снос, с учётом — ноль.
  B. Ход вперёд на малой высоте: камера на 6 см ниже — без учёта масштаб завышен на
     h_борта/h_камеры, с учётом — метр в метр.
  C. Формула выноса в осях курса и высота геометрии (рендером клевок не проверить —
     см. в тексте).
  D. Нулевой вынос — бит-в-бит прежний канал.

Модель — лётная (`exact`), деротация +1 (лётная), камера горизонтально (борт).

Запуск:  python3 src/control/test/test_ipm_lever.py
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
DT = 1.0 / 30.0
LEVER = (0.14, 0.0, -0.06)

# Земля 30×30 м, 1 см/пиксель, центр текстуры = начало мира. Многооктавная, как в
# test_ipm_adapt (LK любит градиенты всех масштабов, а не соль-перец).
rng = np.random.default_rng(1317)


def _octave(k):
    n = rng.integers(0, 255, (3000, 3000)).astype(np.float32)
    return cv2.GaussianBlur(n, (k, k), 0)


GROUND = 0.3 * _octave(3) + 2.0 * _octave(25) + 4.0 * _octave(75)
GROUND = ((GROUND - GROUND.min()) / (GROUND.max() - GROUND.min()) * 255).astype(np.uint8)


def g_px(xw, yw):
    """Мир (x вперёд-север, y ВЛЕВО) → пиксель текстуры."""
    return [1500.0 - yw / 0.01, 1500.0 - xw / 0.01]


def lever_level(pitch, roll, t=LEVER):
    """R(тангаж, крен)·t в горизонтированных осях курса — НЕЗАВИСИМО от оценщика."""
    R = (np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],
                   [-math.sin(pitch), 0, math.cos(pitch)]])
         @ np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)],
                     [0, math.sin(roll), math.cos(roll)]]))
    return R @ np.array(t)


def render(e, xb, yb, psi, alt, pitch=0.0, roll=0.0, t=LEVER):
    """Кадр борта в (xb, yb) с курсом psi (влево +) на высоте alt: камера — на выносе t."""
    c = lever_level(pitch, roll, t)
    cxw = xb + math.cos(psi) * c[0] - math.sin(psi) * c[1]
    cyw = yb + math.sin(psi) * c[0] + math.cos(psi) * c[1]
    h = alt + c[2]
    src, dst = [], []
    for X, Y in ((2.5, -5.0), (2.5, 5.0), (11.0, 5.0), (11.0, -5.0)):  # Y вправо
        p = e._ipm_px(X, Y, h, pitch, roll)
        dst.append(p)
        L = -Y
        src.append(g_px(cxw + math.cos(psi) * X - math.sin(psi) * L,
                        cyw + math.sin(psi) * X + math.cos(psi) * L))
    M = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
    return cv2.warpPerspective(GROUND, M, (1280, 720))


def make(lever):
    return FlowEstimator(FX, FY, CX, CY, np.eye(3), cam_tilt=0.0, ipm_model='exact',
                         ipm_derot=1.0, cam_lever=lever)


def fly(lever, frames):
    """frames: [(xb, yb, psi, alt, pitch, wz)] — рендер всегда с настоящим выносом."""
    e = make(lever)
    r = make(LEVER)          # рендер-ник: геометрия _ipm_px от наклона не зависит от выноса
    for i, (xb, yb, psi, alt, pitch, wz) in enumerate(frames):
        e._ipm_update(render(r, xb, yb, psi, alt, pitch), i * DT, alt, pitch, 0.0, wz)
    return e


print('A. разворот на месте, ω 0.5 рад/с, 0.8 с, высота 1 м')
WZ, N = 0.5, 24
spin = [(0.0, 0.0, WZ * i * DT, 1.0, 0.0, WZ) for i in range(N + 1)]
dpsi = WZ * N * DT
off, on = fly((0.0, 0.0, 0.0), spin), fly(LEVER, spin)
lat_true = LEVER[0] * dpsi          # ход камеры вбок (влево) за разворот
print(f'     без учёта: вбок {off.ipm_lat:+.4f} м (ход выноса {lat_true:+.4f}), '
      f'с учётом: {on.ipm_lat:+.4f} м')
check('без учёта канал видит ход выноса как снос (знак и 70–130 %)',
      0.7 * lat_true < off.ipm_lat < 1.3 * lat_true)
check('с учётом бокового сноса нет (|вбок| < 25 % хода выноса)',
      abs(on.ipm_lat) < 0.25 * lat_true)

print('B. ход вперёд 0.5 м на высоте 0.8 м (камера на 0.74)')
# Эталон — ТОТ ЖЕ канал без выноса на высоте камеры: у горизонтальной камеры у земли
# канал и сам занижает ход на 3-6 % (скользящий взгляд), это не предмет теста.
fwd = [(0.5 * i / 30, 0.0, 0.0, 0.8, 0.0, 0.0) for i in range(31)]
off, on = fly((0.0, 0.0, 0.0), fwd), fly(LEVER, fwd)
ref = make((0.0, 0.0, 0.0))
r0 = make((0.0, 0.0, 0.0))
for i, (xb, yb, psi, alt, pitch, wz) in enumerate(fwd):
    # камера на 0.74 прямо над точкой борта: тот же кадр, что даёт вынос (сдвиг вперёд
    # 0.14 — константа, ходу не мешает), оценщик знает честную высоту
    ref._ipm_update(render(r0, xb + LEVER[0], yb, psi, 0.74, pitch, t=(0.0, 0.0, 0.0)),
                    i * DT, 0.74, pitch, 0.0, wz)
print(f'     без учёта {off.ipm_fwd:.4f} м, с учётом {on.ipm_fwd:.4f} м, '
      f'эталон на высоте камеры {ref.ipm_fwd:.4f} м')
# геометрия даёт 0.8/0.74 = +8 %, на скользящем взгляде LK добавляет своё (замер +11.5 %)
check('без учёта ход завышен к эталону (> +5 %)', off.ipm_fwd / ref.ipm_fwd > 1.05)
check('с учётом = эталону (±1 %)', abs(on.ipm_fwd / ref.ipm_fwd - 1.0) < 0.01)

print('C. вынос в осях курса (формула оценщика против независимой матрицы)')
# Рендером не проверить: синтетический клевок и без выноса даёт ±5-11 см ошибки
# (горизонтальная камера), а ход выноса на клевке 0.2 рад — 1.5 см.
e = make(LEVER)
worst = max(float(np.max(np.abs(np.array(e._lever_level(p, r)) - lever_level(p, r))))
            for p in (-0.4, -0.1, 0.0, 0.2, 0.5) for r in (-0.5, 0.0, 0.3))
check(f'совпадает с Ry(θ)·Rx(φ)·t (худшее {worst:.1e} м)', worst < 1e-12)
check('клевок (θ>0 = нос вниз) опускает вынесенную вперёд камеру',
      e._lever_level(0.2, 0.0)[2] < e._lever_level(0.0, 0.0)[2])
check('высота геометрии = борт + вынос (0.8 → 0.74 на ровном)',
      abs(e._ipm_geom_h(0.8, 0.0, 0.0) - 0.74) < 1e-12)

print('D. нулевой вынос = прежний канал (бит-в-бит)')
a = FlowEstimator(FX, FY, CX, CY, np.eye(3), cam_tilt=0.0, ipm_model='exact', ipm_derot=1.0)
b = make((0.0, 0.0, 0.0))
r = make(LEVER)
for i, (xb, yb, psi, alt, pitch, wz) in enumerate(spin):
    fr = render(r, xb, yb, psi, alt, pitch)
    a._ipm_update(fr, i * DT, alt, pitch, 0.0, wz)
    b._ipm_update(fr, i * DT, alt, pitch, 0.0, wz)
check('ipm_fwd/ipm_lat совпадают точно', a.ipm_fwd == b.ipm_fwd and a.ipm_lat == b.ipm_lat)

bad = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(bad)}/{len(results)} OK')
sys.exit(1 if bad else 0)
