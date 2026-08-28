#!/usr/bin/env python3
"""Юнит-тест ДВУХ ЗАКОНОВ СТАНЦИИ («тормози жёстко, возвращайся мягко») + anti-windup.

Зачем. Прогон BS_ROLL_POS_KP/ab_pos13 (2026-08-28, pos_kp=1.3 одной ручкой): стоп за
1 с — и маятник ±1.2…1.8 м/с с периодом 5.4 с, ζ −0.08 (растёт). Одна линейная pos_kp
одинаково жёстко тянет ОТ точки (нужно) и К точке (перелёт). Пилот успокаивал руками
(ab_pos13_me) — стик отпускает точку, перезахват в покое сбрасывает ошибку. Здесь тот
же результат получает автомат: фаза BRAKE (уходим от точки быстрее 0.3 м/с — цель
−pos_brake·v_изм, гасим СКОРОСТЬ с авторитетом ∝ скорости, упор как при 1.3), фаза
RETURN (стоим/идём к точке — pos_kp 0.3, потолок 0.3, √-кап тормозного пути) и
anti-windup И-члена (в упоре не наматывать). Логика фаз доказывается ДО полёта.

⚠️ Первая редакция брейка была ПОЗИЦИОННОЙ (цель pos_kp_brake·err, как 1.3, но только
пока уходим) и на этом же стенде дала предельный цикл ±0.6 м/с: на измеренном нуле
цель прыгала с −1.0 на −0.4, привод ещё толкал, борт проходил точку на 0.6, уход
> 0.3 снова будил брейк — и так бесконечно. Скоростной брейк заканчивается сам (цель
→ 0 вместе со скоростью), ступеньки на переходе нет. Секция 2b это помнит.

Плант — синтетика по идентификации gain_sim на ab_pos13: v̇ = −α·PWM + γ, α = 0.0125
м/с² на PWM (150 PWM = 1.9 м/с²), ветер γ = 0.65 м/с² (52 PWM), старт с толчка
0.25 м/с (как на отрыве). Два лага, которых нет в gain_sim и из-за которых он слеп к
этой раскачке (занижал пик 3.4×): канал видит скорость через апериодику τ_s = 0.3 с,
привод (контур FCU по углу) отрабатывает PWM через 0.2 с. Путь станции — интеграл
ИЗМЕРЕННОЙ скорости (как ipm_lat). Кадры 30 Гц, гвоздь — на первом кадре (|v_изм| < 0.3
при отрыве), как в полёте. Робастность — τ_s 0.5 и канал, видящий 0.6 истины
(боковой gain выше пола 0.5 м, см. память damper-low-alt).

Что проверяем:
1. регресс: pos_brake = 0 → бит-в-бит прежний закон (одна ручка);
2. одна ручка 1.3/1.0 в этой модели ЗВЕНИТ: проходит точку быстрее 0.8 м/с и не
   успокаивается — модель воспроизводит полёт; 2b: позиционный брейк — предельный цикл;
3. два закона (0.3/0.3 + brake 3 + acc 0.15 + awu): стоп быстрее, чем при 1.3
   (нуль скорости ≤ 1.5 с), точка НЕ проходится быстрее 0.15 м/с, борт возвращается
   (|путь| < 0.1 м) и стоит (|v| < 0.05 последние 5 с), пики убывают монотонно, ветер
   держит интегратор, фаза BRAKE была (упор) и погасла; √-кап у точки;
4. робастность: канал видит 0.6 истины — сходится; лаг 0.5 с — не растёт, а БЕЗ
   anti-windup на том же лаге разносит до ±1.7 м/с (awu — не косметика);
5. гистерезис: в RETURN у точки малые скорости (< 0.3) BRAKE не будят, в BRAKE до нуля;
6. живой стик отпускает точку и гасит фазу BRAKE.

Запуск:  python3 src/control/test/test_station_brake.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.stabilization import DpRollRate, clamp   # noqa: E402
from control_pkg.domain.rc import RC_CENTER                          # noqa: E402
from control_pkg.domain.setpoint import Setpoint                     # noqa: E402
from control_pkg.domain.state import DroneState                      # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


ALPHA, WIND, V0 = 0.0125, 0.65, 0.25     # плант: идентификация gain_sim по ab_pos13
DT = 1.0 / 30.0                          # кадры камеры
BRAKE = dict(pos_kp=0.3, pos_vmax=0.3, pos_brake=3.0, pos_brake_vmax=1.0, pos_acc=0.15,
             anti_windup=True)           # кандидат на полёт ab_brake


def axis(cls=DpRollRate, **kw):
    kw.setdefault('kp', 90.0)
    kw.setdefault('ki', 60.0)
    kw.setdefault('kd', 0.0)
    kw.setdefault('imax', 150.0)
    kw.setdefault('max_speed', 0.0)
    kw.setdefault('alt_band', 0.0)
    kw.setdefault('arm_frames', 0)
    return cls(**kw)


def fly(ax, sec=20.0, tau_s=0.3, tau_a=0.2, gain=1.0, stick=None):
    """Замкнутый контур ось+плант; строки (t, v_ист, v_изм, путь, pwm, И-член)."""
    v, vm, pwm_act, path = V0, 0.0, 0.0, 0.0
    ax.enter(DroneState(flow_seq=-1))
    rows, t = [], 0.0
    for k in range(int(round(sec / DT))):
        t += DT
        # что видит канал: скорость через апериодику (и с долей gain), путь = её интеграл
        vm += (gain * v - vm) * (1.0 - math.exp(-DT / tau_s))
        path += vm * DT
        sp = Setpoint()
        if stick is not None:
            sp.c_right = stick(t)
        rc = ax.update(DroneState(flow_seq=k + 1, now_sim=t, flow_dt=DT, rel_alt=0.3,
                                  ipm_ok=True, flow_conf=0.5, ipm_vlat=vm, ipm_lat=path),
                       sp, DT)
        pwm = rc.roll - RC_CENTER
        pwm_act += (pwm - pwm_act) * (1.0 - math.exp(-DT / tau_a))   # привод (FCU по углу)
        v += (-ALPHA * pwm_act + WIND) * DT
        rows.append((t, v, vm, path, pwm, ax._i))
    return rows


def peaks(rows):
    """Экстремумы истинной скорости (|v| > 0.08, зазор 1 с)."""
    out = []
    for i in range(1, len(rows) - 1):
        v0, v1, v2 = rows[i - 1][1], rows[i][1], rows[i + 1][1]
        if (v1 - v0) * (v2 - v1) < 0 and abs(v1) > 0.08:
            if out and rows[i][0] - out[-1][0] < 1.0:
                if abs(v1) > abs(out[-1][1]):
                    out[-1] = (rows[i][0], v1)
            else:
                out.append((rows[i][0], v1))
    return out


def first_zero(rows):
    return next((r[0] for r in rows if r[1] <= 0.0), None)


def crossing_speed(rows):
    """Скорость, с которой борт проходит точку (путь меняет знак) — макс по проходам."""
    return max([abs(b[1]) for a, b in zip(rows[:-1], rows[1:]) if a[3] * b[3] < 0.0],
               default=0.0)


def tail_v(rows, sec=5.0):
    return max(abs(r[1]) for r in rows[-int(sec / DT):])


def summary(tag, rows):
    pk = peaks(rows)
    z = first_zero(rows)
    print(f"    {tag}: ноль v @ {'--' if z is None else f'{z:.1f}'} с, проход точки "
          f"{crossing_speed(rows):.2f} м/с, пики " + ' '.join(f'{p[1]:+.2f}' for p in pk[:7])
          + f", путь в конце {rows[-1][3]:+.2f} м, |v| хвост {tail_v(rows):.2f}, "
          f"И-член макс {max(r[5] for r in rows):.0f} / в конце {rows[-1][5]:.0f} PWM")
    return pk, z


# --- 1. регресс: без brake-ручек закон прежний ---
a = axis(pos_kp=0.5, pos_vmax=0.4)
b = axis(pos_kp=0.5, pos_vmax=0.4, pos_brake=0.0, pos_brake_vmax=0.0, pos_acc=0.0,
         anti_windup=False)
ra, rb = fly(a, 6.0), fly(b, 6.0)
check("регресс: pos_brake=0 → тот же выход бит-в-бит",
      all(x[4] == y[4] for x, y in zip(ra, rb)))
check("регресс: _station_target при одной ручке = clamp(pos_kp·err, ±vmax)",
      a._station_target(3.0, 0.0) == 0.4 and a._station_target(-0.2, 0.0) == -0.1)

# --- 2. одна ручка 1.3/1.0: модель воспроизводит полёт ab_pos13 (звон) ---
print("  одна ручка pos_kp=1.3 vmax=1.0 (как ab_pos13):")
one = fly(axis(pos_kp=1.3, pos_vmax=1.0))
pk1, z1 = summary('1.3', one)
check("одна ручка 1.3: стоп быстрый (ноль скорости ≤ 2.5 с)", z1 is not None and z1 <= 2.5)
check(f"одна ручка 1.3: точку проходит быстро ({crossing_speed(one):.2f} ≥ 0.8 м/с — слэм)",
      crossing_speed(one) >= 0.8)
check("одна ручка 1.3: не успокаивается за 20 с (есть пик > 0.5 м/с после 8 с)",
      any(p[0] > 8.0 and abs(p[1]) > 0.5 for p in pk1))


# --- 2b. позиционный брейк (первая редакция) — предельный цикл; не возвращать ---
class PosBrake(DpRollRate):
    def _station_target(self, err, v):
        away = v * err < 0.0
        if self._pos_brake:
            if not away:
                self._pos_brake = False
        elif away and abs(v) > self._POS_PIN_V:
            self._pos_brake = True
        if self._pos_brake:
            return clamp(1.3 * err, -1.0, 1.0)
        return clamp(clamp(self.pos_kp * err, -self.pos_vmax, self.pos_vmax),
                     -math.sqrt(2 * self.pos_acc * abs(err)), math.sqrt(2 * self.pos_acc * abs(err)))


print("  позиционный брейк (первая редакция, для памяти):")
pb = fly(axis(PosBrake, pos_kp=0.5, pos_vmax=0.4, pos_acc=0.3, anti_windup=True))
summary('pos-brake', pb)
check(f"позиционный брейк: предельный цикл (|v| хвост {tail_v(pb):.2f} > 0.4) — "
      "поэтому брейк СКОРОСТНОЙ", tail_v(pb) > 0.4)

# --- 3. два закона: стоп как при 1.3, возврат мягкий, без звона ---
print("  два закона 0.3/0.3 + brake 3 + acc 0.15 + awu (кандидат ab_brake):")
two_ax = axis(**BRAKE)
two = fly(two_ax)
pk2, z2 = summary('brake', two)
check(f"два закона: стоп быстрее одной ручки (ноль скорости {z2:.1f} ≤ 1.5 с; у 1.3 — {z1:.1f})",
      z2 is not None and z2 <= 1.5)
check(f"два закона: точка не проходится быстрее 0.15 м/с ({crossing_speed(two):.2f})",
      crossing_speed(two) <= 0.15)
check(f"два закона: вернулся к точке (|путь| {abs(two[-1][3]):.2f} < 0.10 м к 20 с)",
      abs(two[-1][3]) < 0.10)
check(f"два закона: стоит (|v| хвост {tail_v(two):.2f} < 0.05)", tail_v(two) < 0.05)
amp2 = [abs(p[1]) for p in pk2]
check("два закона: пики убывают монотонно " + ' '.join(f'{x:.2f}' for x in amp2),
      len(amp2) >= 3 and all(y < x for x, y in zip(amp2[:-1], amp2[1:])))
check(f"два закона: ветер держит интегратор (И-член в конце {two[-1][5]:.0f} ≈ "
      f"{WIND / ALPHA:.0f} PWM)", abs(two[-1][5] - WIND / ALPHA) < 8.0)
check("два закона: фаза BRAKE была (упор ≥149 PWM в первые 2 с) и погасла",
      not two_ax._pos_brake and any(abs(r[4]) >= 149 for r in two[:int(2.0 / DT)]))
t_near = two_ax._station_target(0.05, 0.0)
check(f"√-кап: на 0.05 м цель {t_near:.3f} ≤ √(2·0.15·0.05) = {math.sqrt(0.015):.3f}",
      0.0 < t_near <= math.sqrt(0.015) + 1e-9)

# --- 4. робастность ---
print("  робастность:")
g6 = fly(axis(**BRAKE), gain=0.6)
pk6, z6 = summary('gain 0.6', g6)
amp6 = [abs(p[1]) for p in pk6]
check(f"канал видит 0.6 истины: стоп {z6:.1f} ≤ 2.0 с, проход {crossing_speed(g6):.2f} ≤ 0.3, "
      f"хвост {tail_v(g6):.2f} < 0.15, пики убывают",
      z6 <= 2.0 and crossing_speed(g6) <= 0.3 and tail_v(g6) < 0.15
      and all(y < x for x, y in zip(amp6[:-1], amp6[1:])))
l5 = fly(axis(**BRAKE), tau_s=0.5)
pk5, _ = summary('лаг 0.5', l5)
late5 = [abs(p[1]) for p in pk5 if p[0] > 8.0]
check(f"лаг канала 0.5 с: не растёт (пики после 8 с ≤ 0.35: {max(late5):.2f}), "
      f"хвост {tail_v(l5):.2f} < 0.35", max(late5) <= 0.35 and tail_v(l5) < 0.35)
l5n = fly(axis(**dict(BRAKE, anti_windup=False)), tau_s=0.5)
summary('лаг 0.5 БЕЗ awu', l5n)
check(f"лаг 0.5 БЕЗ anti-windup: разносит (хвост {tail_v(l5n):.2f} > 1.0) — awu обязателен",
      tail_v(l5n) > 1.0)
imax_awu = max(r[5] for r in two)
check(f"anti-windup: в упоре И-член не наматывается (макс {imax_awu:.0f} ≤ "
      f"{WIND / ALPHA + 25:.0f} PWM)", imax_awu <= WIND / ALPHA + 25.0)

# --- 5. гистерезис фаз ---
h = axis(**BRAKE)
h.enter(DroneState(flow_seq=-1))
h._pos_sp = (0.0, 0.0)
tgt = h._station_target(0.2, -0.2)          # точка в +0.2, уходим на 0.2 м/с (< 0.3)
check("гистерезис: уход 0.2 м/с — RETURN, цель по pos_kp (0.06), не по брейку",
      not h._pos_brake and abs(tgt - 0.06) < 1e-9)
tgt = h._station_target(0.5, -0.4)          # уходим на 0.4 м/с → BRAKE: −3·(−0.4)=1.2→1.0
check("гистерезис: уход 0.4 м/с — BRAKE, цель −3·v = 1.2 → кламп 1.0",
      h._pos_brake and abs(tgt - 1.0) < 1e-9)
tgt = h._station_target(0.6, -0.1)          # всё ещё уходим, медленно — BRAKE держится
check("гистерезис: в BRAKE до нуля скорости, даже когда |v| < 0.3 (цель 0.3)",
      h._pos_brake and abs(tgt - 0.3) < 1e-9)
tgt = h._station_target(0.6, 0.05)          # пошли к точке → RETURN
check("гистерезис: измеренный ноль → RETURN (цель min(0.3·0.6, √(0.3·0.6), 0.3) = 0.18)",
      not h._pos_brake and abs(tgt - 0.18) < 1e-9)

# --- 6. живой стик отпускает точку и гасит BRAKE ---
s6 = axis(**BRAKE)
fly(s6, 3.0, stick=lambda t: 0.5 if t > 1.5 else 0.0)
check("стик живой: точка отпущена и BRAKE погашен",
      s6._pos_sp is None and not s6._pos_brake)
check("стик живой: цель = стик·cmd_gain (станция не мешает пилоту)",
      abs(s6._target - (-0.5 * s6.cmd_gain)) < 1e-9)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ СТАНЦИЯ: ДВА ЗАКОНА OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
