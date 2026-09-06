#!/usr/bin/env python3
"""WindTrim — ветровой трим ОДИН НА ВСЕ ЯРУСЫ (2026-09-06).

До него трим жил в двух местах: у демпфера — мировой вектор внутри StationFrame
(пре-osign И-члены крена/тангажа), у DpVins — свой мировой вектор в раме VINS
(_itx/_ity), и между ними ходил ПОСЕВ в одну сторону (0 → 1, только на девственный
трим): обратно 1 → 0 демпфер учил ветер заново первым брейком (DpHold.enter
сбрасывал раму), после LOITER — тоже, перерождение VINS обнуляло трим DpVins
(его рама умирала), а сам посев дал провал в 46 м (читал идле-копию, cmd/3).

Здесь: мировой вектор в ВАЛЮТЕ PWM КАНАЛОВ (pitch_off, roll_off — то, что стаб
добавляет в RcCommand, знаки уже применены: у демпфера osign, у DpVins psign/
rsign), в раме ENU по курсу AHRS att_yaw — он есть у обоих ярусов и не зависит
от рамы VINS. (pitch_off, roll_off) поворачивается как вектор тела (вперёд,
влево): физический наклон = (−pitch_off, −roll_off), а смена знака обеих
компонент с поворотом коммутирует — результат тот же. Каждая ось читает свою
компоненту вдоль ТЕКУЩЕГО курса, учится своим законом и пишет обратно (другая
компонента не тронута) — семантика StationFrame.trim_body/set_trim_body. Флаг
`learned` («ветер выучен», был trim_armed у станции и у DpVins по отдельности)
— тоже общий: выучил один ярус — другой не проходит фазу захвата (ki_trim /
первый брейк). Сброс — только на фронте арма (новый полёт); переключения
ярусов, LOITER и перерождение VINS трим не трогают — ветер физический.
"""
import math

from ..rc import clamp


class WindTrim:
    def __init__(self, imax=150.0):
        self.imax = float(imax)
        self.reset()

    def reset(self) -> None:
        self.x = 0.0                 # мировые компоненты (ENU), PWM каналов
        self.y = 0.0
        self.learned = False         # ветер выучен (первый гвоздь / первый брейк прошёл)

    @staticmethod
    def _rot(psi):
        return math.cos(psi), math.sin(psi)

    def channel(self, psi):
        """(pitch_off, roll_off) в теле под курсом psi (рад, ENU как att_yaw)."""
        c, s = self._rot(psi)
        return (self.x * c + self.y * s, -self.x * s + self.y * c)

    def set_channel(self, psi, pitch_off, roll_off) -> None:
        c, s = self._rot(psi)
        p = clamp(float(pitch_off), -self.imax, self.imax)
        r = clamp(float(roll_off), -self.imax, self.imax)
        self.x = p * c - r * s
        self.y = p * s + r * c

    def channel_axis(self, psi, axis) -> float:
        p, r = self.channel(psi)
        return p if axis == "pitch" else r

    def set_channel_axis(self, psi, axis, value) -> None:
        p, r = self.channel(psi)
        if axis == "pitch":
            p = value
        else:
            r = value
        self.set_channel(psi, p, r)

    def magnitude(self) -> float:
        return math.hypot(self.x, self.y)
