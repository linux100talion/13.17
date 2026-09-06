#!/usr/bin/env python3
"""StationFrame — рама станции в осях курса, общая крену и тангажу.

Выделен из stabilization.py (там — реэкспорт).
"""
import math


class StationFrame:
    """РАМА СТАНЦИИ В ОСЯХ КУРСА — общая для крена и тангажа.

    Зачем. Станция и трим ветра жили в осях БОРТА: путь ipm_lat/ipm_fwd копится
    покомпонентно без поворота, гвоздь — точка на этом пути, И-член каждой оси —
    компонента трима вдоль оси борта. Разворот всё это ломает: полёт
    lv2_joy_20260829_153405, разворот 200° за 4 с в 5 м/с — стиков крена/тангажа
    нет, цели станции 0, а борт разгоняется с 0.06 до 1.38 м/с: трим (−50 PWM в
    тангаже) после разворота смотрит В ОБРАТНУЮ сторону и толкает ПО ветру, гвоздь
    сбрасывался каждые 17° курса, точка терялась. Здесь всё три вещи — мировые:
    - позиция (x, y): приращения пути IPM (тело: вперёд/влево) поворачиваются
      курсом ψ и суммируются — раз на кадр, идемпотентно по flow_seq;
    - гвоздь (px, py): ОДИН на обе оси, ставится осью, которая только что
      затормозила (set_pin); ошибка оси = компонента (гвоздь − позиция) вдоль
      её оси ТЕКУЩЕГО курса — после разворота точка на месте;
    - трим (tx, ty): вектор PWM в мировых осях; ось читает компоненту вдоль
      себя (trim_body), интегрирует по своему закону и пишет обратно
      (set_trim_body — другая компонента не тронута). После разворота трим
      сам поворачивается в оси борта — толчка по ветру нет.
    КУРС — ПОДКЛЮЧАЕМЫЙ ВХОД: `heading(s) → рад (ENU, как att_yaw)`. Сейчас —
    курс FCU (гиро + компас EKF; в симе идеален, им же считается порог 17°);
    для борта без компаса сюда встанет визуальный курс (лобовой или поворот
    полосы IPM) — станцию при этом переделывать не надо.
    Условности осей: тело = (вперёд, влево) — как ipm_vfwd/ipm_vlat; мир = ψ от
    оси x против часовой (ENU-курс). Сброс пути перцепцией (ipm_* = 0 на новом
    сегменте) распознаётся по точному нулю пары и приращения не даёт."""

    def __init__(self, heading=None):
        self.heading = heading if heading is not None else (lambda s: s.att_yaw)
        self.reset()

    def reset(self):
        self._seq = -1
        self._prev = None
        self.x = self.y = 0.0
        self.psi = 0.0
        self.pin = None
        self.trim = [0.0, 0.0]
        self._live = {}

    def _rot(self):
        return math.cos(self.psi), math.sin(self.psi)

    def advance(self, s) -> None:
        if s.flow_seq == self._seq:
            return
        self._seq = s.flow_seq
        self.psi = float(self.heading(s))
        cur = (float(s.ipm_fwd), float(s.ipm_lat))
        if self._prev is not None and not (cur[0] == 0.0 and cur[1] == 0.0):
            df, dl = cur[0] - self._prev[0], cur[1] - self._prev[1]
            c, si = self._rot()
            self.x += df * c - dl * si
            self.y += df * si + dl * c
        self._prev = cur

    def dbg(self):
        """(x, y, гвоздь|None) для телеметрии — или None, пока рама не видела ни
        одного кадра («чего нет в источниках, того нет и в строке»).

        ПУЛЛ-модель, как hold_dbg/rate_dbg демпфера: раму читает НОДА и сама кладёт
        поля st_* в снапшот перед hud_status (→ /mission/status: sf/sx/sy/spx/spy).
        До 2026-09-01 advance() писал st_* прямо в DroneState — единственное место,
        где домен использовал входной снапшот как выходную шину; заодно после ухода
        рамы из стека (свап на VinsHold) sf=1 залипал в персистентном снапшоте со
        stale-координатами."""
        if self._seq < 0:
            return None
        return (self.x, self.y, self.pin)

    def set_pin(self) -> None:
        self.pin = (self.x, self.y)

    def drop_pin(self) -> None:
        self.pin = None

    def body_err(self, axis):
        """Компонента (гвоздь − позиция) вдоль оси тела: 'pitch' → вперёд, 'roll' → влево."""
        if self.pin is None:
            return None
        ex, ey = self.pin[0] - self.x, self.pin[1] - self.y
        c, si = self._rot()
        return ex * c + ey * si if axis == "pitch" else -ex * si + ey * c

    def trim_body(self, axis) -> float:
        tx, ty = self.trim
        c, si = self._rot()
        return tx * c + ty * si if axis == "pitch" else -tx * si + ty * c

    def set_trim_body(self, axis, value) -> None:
        f, l = self.trim_body("pitch"), self.trim_body("roll")
        if axis == "pitch":
            f = float(value)
        else:
            l = float(value)
        c, si = self._rot()
        self.trim = [f * c - l * si, f * si + l * c]

    def stick(self, axis, live: bool) -> None:
        self._live[axis] = bool(live)

    def any_stick(self) -> bool:
        return any(self._live.values())
