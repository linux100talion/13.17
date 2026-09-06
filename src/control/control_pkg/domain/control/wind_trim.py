#!/usr/bin/env python3
"""WindTrim — ветровой трим ОДИН НА ВСЕ ЯРУСЫ (2026-09-06, возвращён с добавками).

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

ТРИ ДОБАВКИ ПОСЛЕ ОТКАТА (первая версия 17ccce8 откачена 0c3c82b по полётам
185004/191610 «стрелка крутится»; разбор показал, что на DpHold тот же симптом
дал капкан оценки нуля ω_z (память ipm-wz-bias-trap), а на DpVins — п.1):

1. ВЛАДЕЛЕЦ (`owner`). В ярусе 1 композит DpHold остаётся в стеке «тенью»
   (handover.vins_stabs: keep + [DpVins], выход тени перезаписывает DpVins),
   и его оси продолжали ПИСАТЬ свой И-член в общий трим параллельно с DpVins —
   два интегратора на одном векторе, ошибка IPM против собственного гвоздя тени.
   Теперь пишет только владелец: рама демпфера (ярус 0) либо DpVins (ярус 1);
   владельца назначает вход в ярус (handover), чужие записи, наблюдения и
   «выучен» игнорируются. Чтение — всем (тень читает трим DpVins в свой _i —
   безвредно, выход её отброшен).

2. СНИМОК УСТОЙЧИВОГО HOLD (`observe`/`handover`). Живой трим = ветер + смещение
   датчика активного яруса (195742: 150 PWM фантома деротации IPM при истинном
   ветре 2–5). Ярусу-приёмнику отдаётся не что попало, а последнее значение,
   которое источник ДЕРЖАЛ УСТОЙЧИВО: активный ярус каждый тик сообщает, устойчив
   ли он (гвоздь, фаза hold без брейка, стики в центре, трим учится, |v| < steady_v
   по своему датчику И по чужому, если тот жив — фантом своего датчика станция
   сама не видит); серия ≥ steady_sec → снимок. Вход в ярус: источник устойчив
   прямо сейчас → живой трим как есть (L); иначе → откат к снимку (S); снимка
   ещё нет → живой как есть (N — ровно старый посев: начало полёта, первый брейк
   демпфера ещё идёт, а нулевой трим при ki 8 = унос трим/ki, 17 м на ветре 10).
   Откат теряет ровно то, что источник выучил НЕ в устойчивом hold — на возврате
   и в выбеге (уползание против ветра ~3 м, разбор cmd/3). Вердикт идемпотентен
   по тику (композит и DpVins входят одним тиком).

3. Стрелка ветра HUD гаснет при |трим| < 15 PWM (рендерер, не здесь): ниже
   направление — шум (191610: базовый ветер 1 м/с ≈ 6 PWM).
"""
import math

from ..rc import clamp


class WindTrim:
    def __init__(self, imax=150.0, steady_sec=3.0, steady_v=0.5):
        self.imax = float(imax)
        self.steady_sec = float(steady_sec)   # серия устойчивого hold до снимка, с
        self.steady_v = float(steady_v)       # |v| «стоим», м/с (свой и чужой датчик)
        self.reset()

    def reset(self) -> None:
        self.x = 0.0                 # мировые компоненты (ENU), PWM каналов — ЖИВОЙ трим
        self.y = 0.0
        self.learned = False         # ветер выучен (первый гвоздь / первый брейк прошёл)
        self.owner = None            # кто пишет: рама демпфера (ярус 0) | DpVins (ярус 1)
        self.gx = 0.0                # снимок последнего устойчивого hold
        self.gy = 0.0
        self.has_good = False
        self.verdict = "-"           # последний вердикт входа: L живой / S снимок / N ноль
        self._steady_since = None    # начало текущей серии устойчивости
        self._obs_t = -1e9           # sim-время последнего наблюдения владельца
        self._hand_t = -1e9          # sim-время последнего вердикта (идемпотентность)

    # --- владелец: пишет только активный ярус ---
    def acquire(self, who) -> None:
        self.owner = who

    def _mine(self, who) -> bool:
        """Запись разрешена: владелец не назначен, вызов без подписи (тесты/стенды)
        либо подпись совпадает с владельцем. Тень (композит DpHold в ярусе 1) —
        нет."""
        return who is None or self.owner is None or who is self.owner

    @staticmethod
    def _rot(psi):
        return math.cos(psi), math.sin(psi)

    def channel(self, psi):
        """(pitch_off, roll_off) в теле под курсом psi (рад, ENU как att_yaw)."""
        c, s = self._rot(psi)
        return (self.x * c + self.y * s, -self.x * s + self.y * c)

    def set_channel(self, psi, pitch_off, roll_off, who=None) -> None:
        if not self._mine(who):
            return
        c, s = self._rot(psi)
        p = clamp(float(pitch_off), -self.imax, self.imax)
        r = clamp(float(roll_off), -self.imax, self.imax)
        self.x = p * c - r * s
        self.y = p * s + r * c

    def channel_axis(self, psi, axis) -> float:
        p, r = self.channel(psi)
        return p if axis == "pitch" else r

    def set_channel_axis(self, psi, axis, value, who=None) -> None:
        if not self._mine(who):
            return
        p, r = self.channel(psi)
        if axis == "pitch":
            p = value
        else:
            r = value
        self.set_channel(psi, p, r)

    def mark_learned(self, who=None, value=True) -> None:
        if self._mine(who):
            self.learned = bool(value)

    def magnitude(self) -> float:
        return math.hypot(self.x, self.y)

    # --- устойчивый hold → снимок; вход в ярус → вердикт ---
    def observe(self, now, steady, who=None) -> None:
        """Каждый тик активного яруса: устойчив ли он сейчас. Серия ≥ steady_sec
        → снимок живого трима (обновляется каждый тик серии — следит за ветром)."""
        if not self._mine(who):
            return
        if steady:
            if self._steady_since is None:
                self._steady_since = now
            if now - self._steady_since >= self.steady_sec:
                self.gx, self.gy, self.has_good = self.x, self.y, True
        else:
            self._steady_since = None
        self._obs_t = now

    def steady_now(self, now) -> bool:
        """Серия устойчивости набрана и наблюдение свежее (< 0.5 с: LOITER не
        наблюдает — после него серия протухает)."""
        return (self._steady_since is not None
                and now - self._steady_since >= self.steady_sec
                and now - self._obs_t <= 0.5)

    def handover(self, now, who, force_learned=False) -> str:
        """Вход в ярус (enter стаба): назначить владельца и решить, чему верить.
        L — источник устойчив прямо сейчас: живой трим как есть; S — откат к снимку
        последнего устойчивого hold, «выучен»; N — снимка ещё нет: живой как есть
        (старый посев). Идемпотентно по now: композит и DpVins входят одним тиком,
        второй вызов лишь переназначает владельца. force_learned — DpVins: его
        фаза захвата (ki_trim до гвоздя) на голом входе раскачивала борт (T 7 с,
        полёт 220204; посев всегда взводил «выучен» — повторяем)."""
        if now != self._hand_t:
            self._hand_t = now
            if self.steady_now(now):
                self.verdict = "L"
            elif self.has_good:
                self.x, self.y = self.gx, self.gy
                self.learned = True
                self.verdict = "S"
            else:
                self.verdict = "N"
            self._steady_since = None
        self.owner = who
        if force_learned:
            self.learned = True
        return self.verdict

    def status(self, now) -> str:
        """Поле `wt=` статуса: <устойчивость>/<вердикт>/<снимок PWM>/<выучен>.
        Устойчивость: S — серия набрана (снимок идёт), s — серия копится, - — нет."""
        if self.steady_now(now):
            st = "S"
        elif self._steady_since is not None and now - self._obs_t <= 0.5:
            st = "s"
        else:
            st = "-"
        snap = math.hypot(self.gx, self.gy) if self.has_good else -1.0
        return f"{st}/{self.verdict}/{snap:.0f}/{int(self.learned)}"
