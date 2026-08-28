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
4. робастность: канал видит 0.6 истины — сходится; лаг 0.5 с — не растёт; БЕЗ
   anti-windup в сильный ветер интегратор в упор 150 и проход точки 0.8 м/с
   (awu — не косметика);
   4b: смещённый канал (+0.08 «ухода» при истинном нуле, как ab_brake_win10 в 10 м/с)
   — брейк выходит по |v| < 0.1, а не виснет; порог входа pos_brake_v — ручка;
   4d: перевзвод — после первого брейка порог входа ×2: качание у точки брейк не будит,
   порыв ≥0.6 м/с — будит (у порога ×1 стенд на лаге 0.5 качал 0.29 → 0.74);
   4c: сильный ветер 104 PWM (10 м/с, толчок 0.55): со старым правилом anti-windup
   И-член в упоре брейка стоит (полёт 3/hover: 31 на стопе, пауза 3 с); с тримом в
   упоре BRAKE (_BRAKE_TRIM) трим копится в упоре, набран раньше, возврат раньше,
   перебора нет; в слабый ветер (упора почти нет) база не меняется;
5. гистерезис: в RETURN у точки малые скорости (< 0.3) BRAKE не будят, в BRAKE до нуля;
6. живой стик отпускает точку и гасит фазу BRAKE;
7. зеркало на тангаж: станция вяжет гвоздь только на установившейся высоте
   (pos_alt_band): на наборе гвоздя и брейка нет (фантом хода по высоте в ipm_fwd),
   после — перезахват, фантом прощён; болтанка в полосе гвоздь не сбрасывает.

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
             anti_windup=True)           # лётный набор (ab_brake_v025)


def axis(cls=DpRollRate, **kw):
    kw.setdefault('kp', 90.0)
    kw.setdefault('ki', 60.0)
    kw.setdefault('kd', 0.0)
    kw.setdefault('imax', 150.0)
    kw.setdefault('max_speed', 0.0)
    kw.setdefault('alt_band', 0.0)
    kw.setdefault('arm_frames', 0)
    return cls(**kw)


def fly(ax, sec=20.0, tau_s=0.3, tau_a=0.2, gain=1.0, stick=None, bias=0.0, wind=WIND,
        v0=V0):
    """Замкнутый контур ось+плант; строки (t, v_ист, v_изм, путь, pwm, И-член, brake).
    bias — постоянное смещение измеренной скорости (канал в ветер «видит уход»),
    wind — ветер, м/с² (0.65 = 52 PWM как 5 м/с; 1.3 = 104 PWM как 10 м/с),
    v0 — толчок на отрыве (0.25; в 10 м/с — 0.55, как ab_brake_win10)."""
    v, vm, pwm_act, path = v0, 0.0, 0.0, 0.0
    ax.enter(DroneState(flow_seq=-1))
    rows, t = [], 0.0
    for k in range(int(round(sec / DT))):
        t += DT
        # что видит канал: скорость через апериодику (и с долей gain), путь = её интеграл
        vm += (gain * v + bias - vm) * (1.0 - math.exp(-DT / tau_s))
        path += vm * DT
        sp = Setpoint()
        if stick is not None:
            sp.c_right = stick(t)
        rc = ax.update(DroneState(flow_seq=k + 1, now_sim=t, flow_dt=DT, rel_alt=0.3,
                                  ipm_ok=True, flow_conf=0.5, ipm_vlat=vm, ipm_lat=path),
                       sp, DT)
        pwm = rc.roll - RC_CENTER
        pwm_act += (pwm - pwm_act) * (1.0 - math.exp(-DT / tau_a))   # привод (FCU по углу)
        v += (-ALPHA * pwm_act + wind) * DT
        rows.append((t, v, vm, path, pwm, ax._i, ax._pos_brake))
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
# без anti-windup: с порогом перевзвода ×2 (4d) раскачка уже не разносит, но в
# сильный ветер интегратор в упоре брейка наматывается до 150 и борт проходит точку
# на 0.8 м/с вместо 0.07 — awu по-прежнему обязателен, просто ловится в другом месте
l5n = fly(axis(**dict(BRAKE, anti_windup=False)), tau_s=0.5)
summary('лаг 0.5 БЕЗ awu', l5n)
check(f"лаг 0.5 БЕЗ anti-windup: хуже, чем с ним (хвост {tail_v(l5n):.2f} > {tail_v(l5):.2f})",
      tail_v(l5n) > tail_v(l5))
w10n = fly(axis(**dict(BRAKE, anti_windup=False)), 25.0, gain=0.6, wind=1.3, v0=0.55)
summary('сильный ветер БЕЗ awu', w10n)
check(f"сильный ветер БЕЗ anti-windup: интегратор в упор (макс {max(r[5] for r in w10n):.0f} ≥ 140) "
      f"и проход точки {crossing_speed(w10n):.2f} ≥ 0.5 м/с — awu обязателен",
      max(r[5] for r in w10n) >= 140.0 and crossing_speed(w10n) >= 0.5)
# 4b. СМЕЩЁННЫЙ канал (ab_brake_win10: после стопа +0.01…+0.12 «ухода» при истинном
# нуле): без выхода по |v| < _POS_BRAKE_EXIT брейк висел бы с целью −3·v ≈ −0.1 и
# держал борт в 2 м от точки; с ним — стоп, выход, возврат.
bx = axis(**BRAKE)
b8 = fly(bx, gain=0.6, bias=0.08)
summary('bias +0.08', b8)
in_brake = [r for r in b8 if abs(r[4]) >= 149]
# остаточный крип = bias/gain (0.13 м/с): станция держит ИЗМЕРЕННЫЙ путь у гвоздя, а
# он ползёт со смещением — свойство смещённого канала («мягкая точка»), не брейка
check(f"смещённый канал +0.08: брейк был (упор) и погас, борт вернулся "
      f"(|путь| {abs(b8[-1][3]):.2f} < 0.3), крип ≤ bias/gain+0.1 (хвост {tail_v(b8):.2f} < 0.25)",
      in_brake and not bx._pos_brake and abs(b8[-1][3]) < 0.3 and tail_v(b8) < 0.25)
hb = axis(**BRAKE)
hb.enter(DroneState(flow_seq=-1))
hb._pos_sp = (0.0, 0.0)
hb._station_target(1.0, -0.5)               # BRAKE
hb._station_target(1.0, -0.08)              # |v| < 0.1, знак ещё «уход» → выход
check("выход из BRAKE по |v| < 0.1 без смены знака", not hb._pos_brake)
# порог входа — ручка (ab_brake_win5: пик канала ровно 0.30 при 0.62 истинных)
hv = axis(**dict(BRAKE, pos_brake_v=0.25))
hv.enter(DroneState(flow_seq=-1))
hv._pos_sp = (0.0, 0.0)
hv._station_target(0.5, -0.28)
check("порог входа pos_brake_v=0.25: уход 0.28 будит BRAKE (при дефолте 0.3 — нет)",
      hv._pos_brake)
# 4c. СИЛЬНЫЙ ВЕТЕР (104 PWM ≈ 10 м/с, толчок 0.55, канал 0.6): в упоре брейка
# anti-windup замораживал И-член, трим ветра не набирался, после стопа борт стоял
# ~3 с (3/hover: И-член −5…10 весь брейк, 31 на стопе). _BRAKE_TRIM: в упоре БРЕЙКА
# И-член копит чистую скорость (трим), вне упора — прежняя ошибка. Стенд упор держит
# короче полёта (канал слабее), поэтому сверяем ОТНОШЕНИЯ старое/новое, не абсолют.


def trim_stats(rows, trim):
    """(И-член на выходе из брейка, t достижения 90% трима, путь к 8 с)."""
    i_exit = next((rows[k][5] for k in range(1, len(rows)) if rows[k - 1][6] and not rows[k][6]),
                  None)
    t90 = next((r[0] for r in rows if r[5] >= 0.9 * trim), None)
    p8 = next(r[3] for r in rows if r[0] >= 8.0)
    return i_exit, t90, p8


print("  сильный ветер 104 PWM (как 10 м/с), толчок 0.55, канал 0.6:")
W10 = 1.3
DpRollRate._BRAKE_TRIM = False
old = fly(axis(**BRAKE), 25.0, gain=0.6, wind=W10, v0=0.55)
DpRollRate._BRAKE_TRIM = True
new = fly(axis(**BRAKE), 25.0, gain=0.6, wind=W10, v0=0.55)
summary('без трима (старое)', old)
summary('трим в упоре BRAKE', new)
io, t90o, p8o = trim_stats(old, W10 / ALPHA)
i_n, t90n, p8n = trim_stats(new, W10 / ALPHA)
print(f"    И-член на выходе из брейка {io:.0f}/{i_n:.0f}, 90% трима ({0.9 * W10 / ALPHA:.0f}) "
      f"к {t90o:.1f}/{t90n:.1f} с, путь к 8 с {p8o:.2f}/{p8n:.2f} м (старое/новое)")
check(f"сильный ветер: в упоре брейка трим копится (на выходе {i_n:.0f} ≥ {io:.0f}+10)",
      i_n >= io + 10.0)
check(f"сильный ветер: трим набран раньше (90% к {t90n:.1f} ≤ {t90o:.1f}−0.3 с)",
      t90n <= t90o - 0.3)
check(f"сильный ветер: возврат раньше (путь к 8 с {p8n:.2f} < {p8o:.2f})", p8n < p8o)
check(f"сильный ветер: перебора трима нет (И-член макс {max(r[5] for r in new):.0f} ≤ "
      f"{W10 / ALPHA + 15:.0f}), вернулся (|путь| {abs(new[-1][3]):.2f} < 0.3), стоит "
      f"(хвост {tail_v(new):.2f} < 0.15)",
      max(r[5] for r in new) <= W10 / ALPHA + 15.0 and abs(new[-1][3]) < 0.3
      and tail_v(new) < 0.15)
base_old_trim = DpRollRate._BRAKE_TRIM
DpRollRate._BRAKE_TRIM = False
base_off = fly(axis(**BRAKE))
DpRollRate._BRAKE_TRIM = base_old_trim
check("слабый ветер (упор — пара кадров): трим-правило меняет выход ≤ 5 PWM — база прежняя",
      max(abs(x[4] - y[4]) for x, y in zip(two, base_off)) <= 5)
# 4d. ПЕРЕВЗВОД: после первого брейка порог входа ×_POS_BRAKE_REFIRE (0.3 → 0.6):
# качание у точки с амплитудой у порога брейк не будит (стенд, лаг 0.5: с порогом
# ×1 брейк бил на каждом качке и качал до 0.74; см. секцию «лаг 0.5» выше — там уже
# ×2), а порыв/отпущенный стик на ≥0.6 м/с — будит.
rf = axis(**BRAKE)
rf.enter(DroneState(flow_seq=-1))
rf._pos_sp = (0.0, 0.0)
rf._station_target(0.5, -0.4)               # первый брейк
rf._station_target(0.5, 0.05)               # вышли
check("перевзвод: после первого брейка уход 0.4 м/с брейк НЕ будит (порог ×2 = 0.6)",
      not rf._trim_armed and not (rf._station_target(0.5, -0.4) < -0.15 or rf._pos_brake))
rf._station_target(0.8, -0.7)
check("перевзвод: уход 0.7 м/с (порыв / отпущенный стик) — брейк будит",
      rf._pos_brake)
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

# --- 7. ЗЕРКАЛО НА ТАНГАЖ: станция только на установившейся высоте ---
# Ход по высоте канал читает как ход вперёд (фантом в ipm_fwd ~0.5 м/м: замер
# ab_brake_trim). Гвоздь, переживший набор, тянул бы к фантомной точке, а брейк бил бы
# по фантомной скорости на отрыве. pos_alt_band: высота идёт → гвоздь отпущен, цель 0.
from control_pkg.domain.control.stabilization import DpPitchRate   # noqa: E402

print("  тангаж: станция на установившейся высоте (pos_alt_band 0.2):")
px = DpPitchRate(kp=90.0, ki=60.0, kd=0.0, imax=150.0, max_speed=0.0, alt_band=0.0,
                 arm_frames=0, pos_alt_band=0.2, pos_alt_still=0.5, **BRAKE)
px.enter(DroneState(flow_seq=-1))
seq, t, path = 0, 0.0, 0.0


def pframe(alt, vf, dpath):
    """один кадр тангажа: высота, скорость канала, приращение пути (фантом или ход)"""
    global seq, t, path
    seq += 1
    t += DT
    path += dpath
    rc = px.update(DroneState(flow_seq=seq, now_sim=t, flow_dt=DT, rel_alt=alt, ipm_ok=True,
                              flow_conf=0.5, ipm_vfwd=vf, ipm_fwd=path), Setpoint(), DT)
    return rc.pitch - RC_CENTER


# земля → набор 0.1→0.4 м за 0.6 с с фантомом 0.8 м/с вперёд (как на отрыве)
for k in range(6):
    pframe(0.1, 0.0, 0.0)
outs = [pframe(0.1 + 0.3 * k / 18, 0.8, 0.8 * DT) for k in range(18)]
check("тангаж, набор: гвоздя нет и брейка нет (цель 0 — чистый демпфер по фантому)",
      px._pos_sp is None and not px._pos_brake and px._target == 0.0)
check(f"тангаж, набор: команда — только демпфер kp·0.8+И ({max(abs(o) for o in outs)} ≤ 110, "
      "не упор брейка 150)", max(abs(o) for o in outs) <= 110)
# высота встала: 0.5 с покоя → гвоздь ТУТ (фантом 0.48 м прощён)
for k in range(20):
    pframe(0.4, 0.0, 0.0)
check(f"тангаж, высота установилась: гвоздь перезахвачен у текущего пути "
      f"({px._pos_sp[0] if px._pos_sp else float('nan'):.2f} ≈ {path:.2f}), цель 0",
      px._pos_sp is not None and abs(px._pos_sp[0] - path) < 0.02 and abs(px._target) < 1e-9)
# болтанка высоты ±0.1 (в полосе) — гвоздь держится
sp0 = px._pos_sp[0]
for k in range(30):
    pframe(0.4 + 0.1 * math.sin(k / 3.0), 0.0, 0.0)
check("тангаж, болтанка ±0.1 м в полосе: гвоздь на месте", px._pos_sp is not None
      and px._pos_sp[0] == sp0)
# пилот поднял на 1 м с фантомом +0.5 м → гвоздь отпущен, после — перезахват, тяги нет
for k in range(30):
    pframe(0.4 + 1.0 * k / 30, 0.5, 0.5 * DT)
check("тангаж, набор 1 м: гвоздь отпущен, цель 0 (к фантому не тянет)",
      px._pos_sp is None and px._target == 0.0)
for k in range(20):
    pframe(1.4, 0.0, 0.0)
check(f"тангаж, после набора: перезахват у нового пути ({px._pos_sp[0]:.2f} ≈ {path:.2f}), "
      "фантом прощён", px._pos_sp is not None and abs(px._pos_sp[0] - path) < 0.02)
# без высотной логики (крен): гвоздь пережил бы набор и тянул к фантому
rx = axis(**BRAKE)
rx.enter(DroneState(flow_seq=-1))
check("крен (pos_alt_band=0): высотной логики нет — поведение прежнее",
      rx._pos_alt is None)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ СТАНЦИЯ: ДВА ЗАКОНА OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
