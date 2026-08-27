#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Картинка КАНАЛА ВИДА СВЕРХУ: кадр с полосой земли + выпрямленный варп.

ОДИН рендерер на двух потребителей — ровно та же развязка, что у HUD
(`nav_pkg/hud_renderer.py`: живой стример и пост-рендер `hud_video.py` рисуют
одним кодом):
  * `ipm_alt_replay.py` — исследовательский стенд A/B/C по высоте перцепции;
  * `ipm_video.py`      — артефакт прогона `scene_ipm.mp4` рядом с `scene_hud.mp4`.
Две копии рисовалки означали бы, что видео стенда и видео прогона показывают
РАЗНОЕ при одном и том же коде канала — поэтому копия одна.

⚠️ Геометрия НЕ пересчитывается: углы полосы проецируются БОЕВЫМ `_ipm_px` по
`_ipm_prev_geo` оценщика — той самой полосе, которую канал только что обработал.
Рисуем то, что мерилось, а не то, что должно было мериться.
"""
import numpy as np

import cv2

FAIL_NAME = {0: 'годен', 1: 'гейт высоты', 2: 'окно не видно', 3: 'варп за кадром',
             4: 'мало фич (<20)', 5: 'мало выживших LK (<15)', 6: 'канал выключен',
             7: 'нет опорного кадра'}
# то же для КАРТИНКИ: cv2.putText рисует Hershey-шрифтом, кириллица выходит «?»
FAIL_ASCII = {0: 'OK', 1: 'ALT GATE', 2: 'NO WINDOW', 3: 'WARP OOB',
              4: 'FEW PTS', 5: 'FEW LK', 6: 'OFF', 7: 'NO REF'}
GREEN, RED, YELLOW, GREY = (60, 200, 60), (50, 50, 230), (0, 255, 255), (190, 190, 190)


def _text(img, s, org, scale, color, thick=2):
    """Подпись с чёрной обводкой: без неё светлые строки тонут в небе кадра."""
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2,
                cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick,
                cv2.LINE_AA)


def dbg_z_decode(z):
    """(ok, код) из `/flow_dbg8.z` — обратная `flow_estimator.ipm_dbg_z`."""
    if z >= 0.95:
        return True, int(round((z - 1.0) * 10.0))
    if z <= -0.5:
        return False, int(round(-z))
    return False, 0                      # старые бэги: брак без причины


def warp_panel(gray, est, alt, pitch, roll, t, zoom=3, agl=None, extra=()):
    """Композит «что видит канал»: слева кадр с полосой земли, справа варп.

    `extra` — дополнительные строки заголовка (серым, под первой): туда
    потребитель кладёт своё (записанные в полёте скорости, истина Gazebo).
    """
    view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    # ⚠️ ПРОЕЦИРОВАТЬ ПО ТОЙ ЖЕ ВЫСОТЕ, ЧТО СЧИТАЛА ВАРП: `_ipm_update` применяет
    # пол ipm_alt_floor ВНУТРИ, и рамка, нарисованная по сырой высоте, легла бы
    # не туда (у земли промах в разы). Одна строка — но без неё картинка врёт.
    h_geom = max(alt, est.ipm_alt_floor) if est.ipm_alt_floor > 0.0 else alt
    geo, rect = est._ipm_prev_geo, est._ipm_prev
    if geo is not None:
        x0, length, yhalf, _res = geo
        pts = []
        for X, Y in ((x0, -yhalf), (x0, yhalf), (x0 + length, yhalf),
                     (x0 + length, -yhalf)):
            p = est._ipm_px(X, Y, h_geom, pitch, roll)
            pts.append(None if p is None else (int(round(p[0])), int(round(p[1]))))
        if all(p is not None for p in pts):
            cv2.polylines(view, [np.array(pts, np.int32)], True, YELLOW, 2)
            _text(view, f'{x0:.2f}..{x0+length:.2f}m',
                  (pts[0][0], min(pts[0][1] + 22, view.shape[0] - 6)), 0.6, YELLOW)
    ok = est.ipm_fail == 0
    floor = ' (floor)' if h_geom > alt + 1e-9 else ''
    head = f't{t:5.1f}s  '
    if agl is not None:
        head += f'AGL {agl:.2f}m  '
    head += (f'alt {alt:.2f}m  geom {h_geom:.2f}m{floor}  '
             f'{FAIL_ASCII.get(est.ipm_fail, "?")}')
    _text(view, head, (10, 26), 0.7, GREEN if ok else RED)
    for i, line in enumerate(extra):
        _text(view, line, (10, 26 + 24 * (i + 1)), 0.55, GREY, 1)
    h, w = view.shape[:2]
    # ⚠️ РАЗМЕР ПАНЕЛИ — ФИКСИРОВАННЫЙ (квадрат по высоте кадра). Ширина варпа
    # ПЛАВАЕТ вместе с адаптивной полосой (ipm_adapt: x0/длина зависят от высоты
    # и тангажа), а cv2.VideoWriter МОЛЧА выбрасывает кадры не своего размера —
    # первое же видео доехало 182 кадрами из 1924 (уцелели ровно те, что совпали
    # с размером первого). Поэтому варп вписывается в панель, а не задаёт её.
    panel = np.zeros((h, h, 3), np.uint8)
    if rect is None:
        _text(panel, 'NO WARP', (10, h // 2), 1.0, RED)
        return np.hstack([view, panel])
    big = cv2.resize(rect, (max(1, rect.shape[1] * zoom), max(1, rect.shape[0] * zoom)),
                     interpolation=cv2.INTER_NEAREST)
    k = min(1.0, h / big.shape[1], h / big.shape[0])       # вписать, не растягивая
    if k < 1.0:
        big = cv2.resize(big, (max(1, int(big.shape[1] * k)), max(1, int(big.shape[0] * k))),
                         interpolation=cv2.INTER_AREA)
    big = cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)
    # подписи осей сетки: строка 0 — ДАЛЬНИЙ край полосы, столбец растёт ВПРАВО
    _text(big, 'far', (6, 22), 0.6, YELLOW)
    _text(big, 'near', (6, big.shape[0] - 10), 0.6, YELLOW)
    y0, x0p = (h - big.shape[0]) // 2, (h - big.shape[1]) // 2
    panel[y0:y0 + big.shape[0], x0p:x0p + big.shape[1]] = big
    return np.hstack([view, panel])
