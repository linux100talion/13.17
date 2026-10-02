#!/usr/bin/env python3
"""Юнит-тест: ПРОТУХШАЯ высота перцепции не уходит в канал вида сверху — без ROS.

Зачем. Источники высоты 'local'/'global' живут, пока EKF держит позицию; теряет — топик
замолкает, `_alt` застывает на последнем значении. Страховка `alt_stale` (BS_PERC_ALT_STALE)
закрывала только `perc_alt` снапшота (гейты, HUD palt=--), а сам ОЦЕНЩИК получал `self._alt`
как есть и выпрямлял землю по замёрзшей высоте. Прогон lever1_20261002_081136: на 64.3 с EKF
потерял позицию (мост VINS→EKF закрыт по insane, GPS нет), высота застыла на 0.9 м, борт
висел на 0.5 м — масштаб канала завышен почти вдвое при ipm=1 до самой посадки.
Теперь протухшая высота уходит в оценщик как None: гейт высоты честно закрывает канал
(код 1), опорный кадр не пересчитывает высоту. «Лучше слепой канал, чем врущий».

Адаптер создаём через object.__new__, оценщик — заглушка, запоминающая высоту.

Запуск:  python3 src/control/test/test_alt_stale_frozen.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.infrastructure.ros_perception import RosPerception    # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


class Est:
    """Заглушка FlowEstimator: запоминает высоту, кадр не считает."""

    def __init__(self):
        self.alt = 'не звали'

    def process(self, gray, stamp, omega, pitch, alt, roll=0.0):
        self.alt = alt
        return None


def mk(alt, age, stale):
    p = object.__new__(RosPerception)
    p._est = Est()
    p._alt = alt
    p._alt_wall = time.time() - age
    p._alt_stale = stale
    p._omega_for = lambda stamp: (0.0, 0.0, 0.0)
    p._prev_img_stamp = None
    return p


p = mk(0.9, age=0.1, stale=2.0)
p._process(None, 1.0, 0.0, 0.0)
check('свежая высота доходит до оценщика как есть', p._est.alt == 0.9)

p = mk(0.9, age=5.0, stale=2.0)
p._process(None, 1.0, 0.0, 0.0)
check('протухшая (5 с при пороге 2) — в оценщик уходит None', p._est.alt is None)
check('снапшот согласован: perc_alt тоже неизвестна', not p._alt_fresh())

p = mk(0.9, age=5.0, stale=0.0)
p._process(None, 1.0, 0.0, 0.0)
check('страховка выключена (0) — прежнее поведение: высота как есть', p._est.alt == 0.9)

p = mk(None, age=0.0, stale=2.0)
p._process(None, 1.0, 0.0, 0.0)
check('высоты ещё не было — None', p._est.alt is None)

bad = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(bad)}/{len(results)} OK')
sys.exit(1 if bad else 0)
