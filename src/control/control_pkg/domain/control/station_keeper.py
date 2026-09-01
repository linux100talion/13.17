#!/usr/bin/env python3
"""StationKeeper — конечный автомат СТАНЦИИ-КИПИНГА rate-оси, выделен из _FlowDamper1D.

До выделения (2026-09-01) автомат жил в демпфере пятью флагами (_pos_sp, _pos_wait_t,
_pos_brake, _trim_armed, _i_hold), размазанными по update()/_station_target(). Здесь те
же переходы собраны в один класс с ИМЕНОВАННЫМИ фазами (`phase`):

    RELEASED ──стик в центре──► SETTLING ──|v|<pin_v или pin_t──► HOLD ◄──► BRAKE
        ▲                                                          │
        └──────────────── живой стик / ход высоты ─────────────────┘

- RELEASED — стик живой (или станции нет, kp=0): гвоздя нет, цель оси = команда пилота.
- SETTLING — стик отпущен, борт ещё несёт: цель 0 (чистое торможение демпфером),
  гвоздь — где остановится («сначала тормози, потом гвоздь», механика LOITER).
- HOLD — гвоздь взят: цель = RETURN-закон (kp·err, кламп vmax, √-кап acc).
- BRAKE — уход от гвоздя быстрее порога: цель = −brake·v (скоростной брейк).

Ортогональные защёлки (не фазы — переживают переходы, см. докстринги в _FlowDamper1D):
- `trim_armed` — первый брейк после reset() ещё не отработан (набор трима ветра);
- `i_hold` — И-член демпфера заморожен от живого стика до гвоздя (_TRIM_LATCH).

Обоснование каждого порога/правила (полёты, стенды) — В КОММЕНТАРИЯХ КОНСТАНТ
_FlowDamper1D (_POS_PIN_V, _POS_BRAKE_EXIT, _POS_BRAKE_REFIRE*, _TRIM_LATCH,
_POS_PIN_T, _POS_YAW_TOL) и ручек pos_* его __init__: демпфер отдаёт их сюда при
конструировании, история решений осталась при них. Поведение выверено A/B-прогоном
против монолита бит-в-бит; стенды — test_station_brake.py (§1-10), test_ipm_gates.py,
test_soft_alt.py.
"""
import math

from ..rc import clamp

# Фазы (str, не Enum: телеметрии и логам нужна строка, сравнение — по идентичности)
RELEASED = "released"
SETTLING = "settling"
HOLD = "hold"
BRAKE = "brake"


class StationKeeper:
    """Владеет состоянием станции; законы читают конфиг, снятый с демпфера.

    `hold()` — кадр со стиком в центре (станция ведёт, возвращает цель скорости);
    `release()` — кадр с живым стиком (гвоздь отпущен);
    `target()` — закон цели (BRAKE/RETURN) с переходами фазы брейка.
    Демпфер зовёт target() через свой хук _station_target — стендовые сабклассы
    подменяют закон, не трогая автомат."""

    def __init__(self, kp=0.0, vmax=1.0, brake=0.0, brake_vmax=0.0, acc=0.0,
                 brake_v=0.0, alt_gate=None, yaw_tol=0.3, pin_v=0.3,
                 brake_exit=0.1, refire=2.0, refire_dist=0.5, pin_t=3.0,
                 kp_exp=0.5):
        # --- конфиг (см. pos_* и _POS_* в _FlowDamper1D — там обоснования) ---
        self.kp, self.vmax = kp, vmax
        self.brake, self.brake_vmax = brake, brake_vmax
        self.acc, self.brake_v = acc, brake_v
        self.alt_gate = alt_gate          # _AltSettled | None: станция только на
                                          # установившейся высоте (pos_alt_band)
        self.yaw_tol, self.pin_v = yaw_tol, pin_v
        self.brake_exit, self.refire, self.refire_dist = brake_exit, refire, refire_dist
        self.pin_t = pin_t
        self.kp_exp = kp_exp              # показатель мягкости для kp (_SOFT_KP_EXP)
        # --- состояние ---
        self.pin = None                   # (путь в точке захвата, курс захвата) | None
        self.wait_t = None                # начало торможения (принудительный гвоздь)
        self.braking = False              # фаза BRAKE
        self.trim_armed = True            # первый брейк после reset() не отработан
        self.i_hold = False               # И-член демпфера заморожен (_TRIM_LATCH)

    @property
    def phase(self) -> str:
        if self.pin is not None:
            return BRAKE if self.braking else HOLD
        return SETTLING if self.wait_t is not None else RELEASED

    def reset(self) -> None:
        self.pin = None
        self.wait_t = None
        self.braking = False
        self.trim_armed = True
        self.i_hold = False
        if self.alt_gate is not None:
            self.alt_gate.reset()

    def release(self, latch: bool) -> None:
        """Живой стик (или станции нет): точка отпущена, брейк погашен.
        latch — заморозить И-член до следующего гвоздя (_TRIM_LATCH и станция есть)."""
        self.pin = None
        self.wait_t = None
        self.braking = False
        self.i_hold = latch

    def hold(self, s, pos, vmeas, fr, axis, soft, law) -> float:
        """Кадр со стиком в центре → цель скорости оси.

        pos — накопленный путь оси; vmeas — измеренная скорость (_signal);
        fr — StationFrame (оси курса) или None (оси борта); law — закон цели
        (хук _station_target демпфера). Логика 1:1 из монолита:
        ход высоты → гвоздь отпущен; уход курса → перезахват; гвоздь по
        |v| < pin_v/soft или таймауту pin_t; дальше цель по закону."""
        if self.alt_gate is not None \
                and self.alt_gate.update(s.now_sim, s.rel_alt) is False:
            # высота идёт / ещё не установилась — гвоздь отпущен, чистый
            # демпфер (см. pos_alt_band в _FlowDamper1D: фантом набора в пути)
            self.pin = None
            self.wait_t = None
            self.braking = False
            return 0.0
        # Перезахват при уходе курса: путь копится в body-осях. В осях курса (рама)
        # точка мировая и с курсом не едет — перезахват нужен только пока ДРУГАЯ
        # ось везёт пилота (эта держит «линию», и линия поворачивается с ходом).
        if (self.pin is not None
                and (fr is None or fr.any_stick())
                and abs(math.atan2(math.sin(s.att_yaw - self.pin[1]),
                                   math.cos(s.att_yaw - self.pin[1])))
                > self.yaw_tol):
            self.pin = None
        if self.pin is None:
            if self.wait_t is None:
                self.wait_t = s.now_sim
            if (abs(vmeas) < self.pin_v / soft
                    or s.now_sim - self.wait_t > self.pin_t):
                self.pin = (pos, s.att_yaw)
                self.wait_t = None
                self.i_hold = False           # гвоздь взят — трим снова учится
                if fr is not None:
                    fr.set_pin()              # ОДИН 2D-гвоздь на обе оси
        if self.pin is not None:
            err = fr.body_err(axis) if fr is not None else self.pin[0] - pos
            if err is None:                   # рама без гвоздя (сброшен) — взять
                fr.set_pin()
                err = 0.0
            return law(err, vmeas)
        self.braking = False                  # ещё тормозим — гвоздь позже
        return 0.0

    def target(self, err, v, soft=1.0) -> float:
        """Цель скорости по ошибке пути `err` (точка − путь) и скорости `v`.

        Одна ручка (brake = 0): clamp(kp·err, ±vmax) — как было.
        Две фазы — см. pos_brake в _FlowDamper1D.__init__: BRAKE, пока уходим от
        точки (v·err < 0) после входа по |v| > порога, до измеренного нуля — цель
        −brake·v (скоростной брейк, без позиционного члена); RETURN — всё
        остальное, с √-капом тормозного пути при acc > 0. Пороги скорости делятся
        на soft (шум канала растёт с высотой — _IpmGated.soft_alt), kp умножается."""
        away = v * err < 0.0                  # скорость направлена ОТ точки
        if self.brake > 0.0:
            if self.braking:
                if not away or abs(v) < self.brake_exit / soft:   # стоп состоялся
                    self.braking = False
                    self.trim_armed = False   # первый брейк отработан: трим ветра
                                              # выучен, дальше порог входа ×refire
            elif away and abs(v) > ((self.brake_v or self.pin_v) / soft
                                    * (1.0 if self.trim_armed else self.refire)) \
                    and (self.trim_armed or abs(err) >= self.refire_dist):
                self.braking = True
        if self.braking:
            vmax = self.brake_vmax if self.brake_vmax > 0.0 else self.vmax
            return clamp(-self.brake * v, -vmax, vmax)
        t = clamp(self.kp * soft ** self.kp_exp * err, -self.vmax, self.vmax)
        if self.acc > 0.0:
            cap = math.sqrt(2.0 * self.acc * abs(err))
            t = clamp(t, -cap, cap)
        return t
