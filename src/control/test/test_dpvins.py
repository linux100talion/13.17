#!/usr/bin/env python3
"""Оффлайн-тест DpVins (velocity-каскад на опоре VINS, чистый python).

Проверяет velocity-семантику и каскад:
- стик = цель СКОРОСТИ (внутренний контур kp·(v_цель − v)); ошибка скорости →
  наклон, знак верный;
- на цели (v = v_цель) — выход около нуля (не долг позиции, как у VinsHold);
- стик отпущен на ходу → тормозим к нулю (внешней точки ещё нет);
- ГВОЗДЬ по остановке: встал (|v|<0.3) → уставка = точка стопа, ошибка позиции
  даёт цель скорости к ней (внешний √-кап контур);
- √-кап: далеко от гвоздя цель ограничена vmax, близко — √(2·acc·e);
- латч трима: И-член заморожен на живом стике;
- геометрия: курс 90° — «вперёд» = мировая Y;
- vsmooth сглаживает ВНУТРЕННИЙ сигнал (главная петля), выход ровнее при шуме;
- ki_trim: до первого гвоздя ветер учится быстро, после — рабочим ki;
- trim_keep: трим переживает enter() (вход в ярус), сброс — только reset_trim();
- замкнутый контур: унос обучения = нужный трим / ki обучения (≈17 м при
  ki 6 и «ветре 10» — как в полётах wind_* 2026-09-03; ki_trim 60 — в разы короче).

Запуск:  python3 src/control/test/test_dpvins.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.vins_axes import DpVins                    # noqa: E402
from control_pkg.domain.rc import RC_CENTER                                # noqa: E402
from control_pkg.domain.setpoint import Setpoint                           # noqa: E402
from control_pkg.domain.state import DroneState                            # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


DT = 0.05


def make(**kw):
    d = dict(kp_fwd=200.0, kp_lat=120.0, ki=0.0, imax=100.0, max_pwm=150.0,
             cmd_gain=4.0, pos_kp=0.3, pos_vmax=0.3, pos_acc=0.15,
             vsmooth=0.0, i_latch=True)
    d.update(kw)
    vh = DpVins(**d)
    vh.enter(DroneState(now_sim=100.0, vins_x=0.0, vins_y=0.0))
    return vh


def st(vx=0.0, vy=0.0, x=0.0, y=0.0, yaw=0.0, t=100.05):
    return DroneState(now_sim=t, vins_x=x, vins_y=y, vins_yaw=yaw,
                      vins_vx=vx, vins_vy=vy, vins_valid=True)


# знак (конвенция VinsHold/демпфера): вперёд-стик → −po (разгон вперёд),
# торможение движения вперёд → +po. err = v − цель.
# --- 1. velocity-семантика: стик вперёд, борт стоит → разгон вперёд (−150) ---
# err_fwd = 0 − 4 = −4 м/с; kp·−4 = −800 → упор −150
rc = make().update(st(vx=0.0), Setpoint(c_fwd=1.0), DT)
check("стик вперёд, v=0: цель 4 м/с, разгон вперёд (−150)",
      rc.pitch == RC_CENTER - 150)

# --- 2. на цели (v = v_цель): выход ноль (НЕ долг позиции) ---
rc = make().update(st(vx=4.0), Setpoint(c_fwd=1.0), DT)
check("v=v_цель=4: ошибка скорости 0 → выход центр (нет долга)",
      rc.pitch == RC_CENTER)

# --- 3. знак: слишком быстро → тормоз (+150) ---
rc = make().update(st(vx=5.0), Setpoint(c_fwd=1.0), DT)   # err=+1 → +200 → +150
check("v > цель (5>4): тормоз (+150)", rc.pitch == RC_CENTER + 150)

# --- 4. стик отпущен на ходу, точки нет: тормозим к нулю (+150) ---
rc = make().update(st(vx=3.0), Setpoint(), DT)            # цель 0, err +3 → +150
check("стик отпущен, v=3, гвоздя нет: тормоз к нулю (+150)",
      rc.pitch == RC_CENTER + 150)

# --- 5. ГВОЗДЬ по остановке + внешний контур ---
vh = make()
vh.update(st(vx=3.0), Setpoint(c_fwd=1.0), DT)            # стик жил → pin_pending
rc = vh.update(st(vx=0.0, x=6.0), Setpoint(), DT)         # встал в x=6 → гвоздь тут
check("встал (v=0): гвоздь в точке стопа → ошибка позиции 0 → центр",
      rc.pitch == RC_CENTER)
# уехал на 6.5 от гвоздя 6.0: e_fwd = pin−pos = −0.5, цель назад
# = −min(0.3·0.5, 0.3, √(2·0.15·0.5)) = −0.15 м/с; err = v−цель = 0−(−0.15)
# = +0.15 → kp·0.15 = +30 (тянет назад к гвоздю)
rc = vh.update(st(vx=0.0, x=6.5), Setpoint(), DT)
check("уход 0.5 м вперёд от гвоздя: возврат назад (+30 PWM)",
      rc.pitch == RC_CENTER + 30)

# --- 6. √-кап: далеко от гвоздя цель = vmax ---
vh = make()
vh.update(st(vx=3.0), Setpoint(c_fwd=1.0), DT)
vh.update(st(vx=0.1, x=10.0), Setpoint(), DT)             # гвоздь в 10
rc = vh.update(st(vx=0.0, x=0.0), Setpoint(), DT)         # борт в 0, гвоздь в 10
# e_fwd = pin−pos = +10: цель вперёд min(0.3·10,0.3,√3)=0.3 (vmax); err = v−цель
# = 0−0.3 = −0.3 → kp·−0.3 = −60 (разгон вперёд к гвоздю)
check("уход 10 м назад от гвоздя: цель vmax вперёд → −60 PWM",
      rc.pitch == RC_CENTER - 60)

# --- 7. латч трима: И-член заморожен на живом стике ---
vh = make(ki=20.0)
r1 = None
for i in range(20):
    r1 = vh.update(st(vx=4.0, t=100.05 + i * DT), Setpoint(c_fwd=1.0), DT)
check("живой стик, v=цель: И-член заморожен (выход центр)",
      r1.pitch == RC_CENTER)

# --- 7a. трим: ПЕРВОЕ торможение учит ветер, после гвоздя морозится ---
# дилемма lv2_joy_065026/ab_dpv_pinfix: всегда мотать → унос назад; всегда
# морозить → дедлок (без ветра не остановиться). _trim_armed: первый брейк
# интегрирует, дальше морозит на торможении.
vh = make(ki=20.0, kp_fwd=40.0)
vh.update(st(vx=1.0, t=100.05), Setpoint(c_fwd=1.0), DT)   # живой стик → pin_pending
r0 = None
for i in range(30):                          # отпустили, ПЕРВОЕ торможение (не armed)
    r0 = vh.update(st(vx=1.0, t=100.1 + i * DT), Setpoint(), DT)
check("первое торможение (не armed): трим ИНТЕГРИРУЕТ (выход > чистый kp·v 40)",
      r0.pitch > RC_CENTER + 40)
vh.update(st(vx=0.0, x=1.0, t=102.0), Setpoint(), DT)      # встал → гвоздь, armed
vh.update(st(vx=1.0, t=102.05), Setpoint(c_fwd=1.0), DT)   # стик оживил → гвоздь снят
i0 = vh._itx
for i in range(20):                          # торможение armed: трим ЗАМОРОЖЕН
    vh.update(st(vx=1.0, t=102.1 + i * DT), Setpoint(), DT)
check("торможение после первого гвоздя (armed): трим ЗАМОРОЖЕН (не растёт)",
      abs(vh._itx - i0) < 1e-6)

# --- 7d. АНТИ-ВИНДАП: выход в упоре + ошибка глубже → трим НЕ мотается ---
# imax высокий, но в насыщении торможения трим замерзает (momentum не копится)
vh = make(ki=20.0, kp_fwd=40.0, imax=120.0, pos_kp=0.0)
vh.update(st(vx=5.0, t=100.05), Setpoint(c_fwd=1.0), DT)   # стик → pin_pending
i0 = vh._itx
for i in range(20):                          # тормозим v=5: kp·5=200 → упор, sat
    vh.update(st(vx=5.0, x=1.0, t=100.1 + i * DT), Setpoint(), DT)
check("выход в упоре (тормоз v=5): анти-виндап морозит трим",
      abs(vh._itx - i0) < 1e-6)
# не в упоре (малый снос): трим ИНТЕГРИРУЕТ (учит ветер)
for i in range(10):
    vh.update(st(vx=0.5, x=1.0, t=101.2 + i * DT), Setpoint(), DT)
check("малый снос (не упор): трим растёт (учит ветер)", abs(vh._itx) > 0.01)

# --- 7c. ТРИМ В ОСЯХ МИРА: набран на курсе 0, после разворота 90° гасит ту же
# мировую ось (не устаревает по телу) — фикс сноса при развороте (lv2_joy_075118)
vh = make(ki=20.0, kp_fwd=40.0, pos_kp=0.0)   # pos_kp=0: без внешнего контура
vh.update(st(vx=0.5, t=100.05), Setpoint(c_fwd=1.0), DT)   # стик → pin_pending
for i in range(30):                            # первый брейк: копим трим (мир +x)
    vh.update(st(vx=0.5, x=0.5, t=100.1 + i * DT), Setpoint(), DT)
vh.update(st(vx=0.0, x=0.5, t=102.0), Setpoint(), DT)      # гвоздь → armed
tx = vh._itx
check("трим набран в мировом +x (itx > 0)", tx > 0.5)
# теперь борт на КУРСЕ 90° держит ту же точку: мировой +x трим = боковой у тела
# → должен пойти в КРЕН (roll), тангаж почти чист
rc = vh.update(st(vx=0.0, x=0.5, yaw=math.pi / 2, t=102.05), Setpoint(), DT)
check("на курсе 90°: мировой трим +x → в КРЕН (roll ≠ центр)",
      abs(rc.roll - RC_CENTER) > 5)

# --- 7b. знак крена: правый стик → вправо (+ro), как VinsHold ---
rc = make(ki=0.0).update(st(vx=0.0, vy=0.0, yaw=0.0), Setpoint(c_right=1.0), DT)
check("правый стик, v=0: разгон вправо (+150, как VinsHold)",
      rc.roll == RC_CENTER + 150)

# --- 8. геометрия: курс 90° — «вперёд» = мировая Y ---
yaw = math.pi / 2
rc = make().update(st(vy=4.0, yaw=yaw), Setpoint(c_fwd=1.0), DT)
check("yaw=90°: цель вперёд = мировая Y, v_y=4=цель → центр",
      rc.pitch == RC_CENTER)

# --- 9. vsmooth сглаживает внутренний сигнал (главную петлю) ---
import random
def noisy(vsmooth, seed=1, n=40):
    rng = random.Random(seed)
    vh = make(vsmooth=vsmooth)
    out = []
    vn = 0.0
    for i in range(1, n + 1):
        if i % 2 == 1:
            vn = rng.uniform(-0.4, 0.4)
        rc = vh.update(st(vx=4.0 + vn, t=100.05 + i * DT), Setpoint(c_fwd=1.0), DT)
        out.append(rc.pitch - RC_CENTER)
    return out
raw = noisy(0.0)
sm = noisy(0.3)
mraw = max(abs(raw[i + 1] - raw[i]) for i in range(len(raw) - 1))
msm = max(abs(sm[i + 1] - sm[i]) for i in range(len(sm) - 1))
check("vsmooth сглаживает внутренний контур (макс шаг меньше)", msm < mraw)

# --- 10. vsmooth=0 воспроизводимо ---
check("vsmooth=0 воспроизводимо бит-в-бит", noisy(0.0) == raw)


# --- 11. ki_trim: быстрый захват ветра ДО первого гвоздя, после — рабочий ki ---
def learn(**kw):
    vh = DpVins(**dict(kp_fwd=40.0, kp_lat=32.0, ki=6.0, imax=120.0,
                       max_pwm=150.0, cmd_gain=4.0, pos_kp=0.3, pos_vmax=0.3,
                       pos_acc=0.15, vsmooth=0.0, i_latch=True, **kw))
    vh.enter(DroneState(now_sim=100.0))
    vh.update(st(vx=0.5, t=100.05), Setpoint(c_fwd=1.0), DT)  # стик → pin_pending
    for i in range(10):                       # первый брейк: учим ветер
        vh.update(st(vx=0.5, t=100.1 + i * DT), Setpoint(), DT)
    return vh

fast, slow = learn(ki_trim=60.0), learn()
check("ki_trim 60 против ki 6: захват ветра на первом брейке ×10",
      abs(fast._itx - 10.0 * slow._itx) < 1e-6 and slow._itx > 0.0)
fast.update(st(vx=0.0, x=1.0, t=101.0), Setpoint(), DT)   # встал → гвоздь, armed
i0 = fast._itx
fast.update(st(vx=0.5, x=1.0, t=101.05), Setpoint(), DT)  # удержание: рабочий ki
check("после первого гвоздя обучение падает до рабочего ki (6)",
      abs((fast._itx - i0) - 0.5 * DT * 6.0) < 1e-6)

# --- 12. trim_keep: трим переживает enter() (повторный вход в ярус) ---
vh = learn(ki_trim=60.0)
vh.update(st(vx=0.0, x=1.0, t=101.0), Setpoint(), DT)     # гвоздь → armed
t0 = vh._itx
vh.enter(st(t=200.0))                                     # повторный вход в ярус
check("enter(): трим и «ветер выучен» живы (ветер не исчез на переключении)",
      vh._itx == t0 and vh._trim_armed)
vh.reset_trim()                                           # фактический /restart VINS
check("reset_trim(): трим обнулён и разоружён (рама мира переродилась)",
      vh._itx == 0.0 and not vh._trim_armed)
vh2 = learn(ki_trim=60.0, trim_keep=False)
vh2.enter(st(t=200.0))
check("trim_keep=False: старое поведение — сброс на enter()", vh2._itx == 0.0)


# --- 13. ЗАМКНУТЫЙ КОНТУР: унос обучения = нужный трим / ki обучения ---
# план: «ветер» разгоняет 1 м/с², 100 PWM выхода = 1 м/с² противодействия →
# равновесный трим 100 PWM (как ветер 10 м/с в полётах wind_* 2026-09-03:
# унос 16-17.5 м при ki 6, формула 100/6 = 16.7). Меряем путь по ветру.
def carry(**kw):
    vh = DpVins(**dict(kp_fwd=40.0, kp_lat=32.0, ki=6.0, imax=120.0,
                       max_pwm=150.0, cmd_gain=4.0, pos_kp=0.3, pos_vmax=0.3,
                       pos_acc=0.15, vsmooth=0.0, i_latch=True, **kw))
    vh.enter(DroneState(now_sim=100.0))
    # стик НЕ трогаем — как в полётах: после входа в ярус гвоздь не заказан
    # (pin_pending=False), борт свободно тормозится к нулю, трим учит ветер
    x = v = 0.0
    t = 100.05
    for _ in range(4000):                     # 200 сим-секунд
        rc = vh.update(st(vx=v, x=x, t=t), Setpoint(), DT)
        v += (1.0 - (rc.pitch - RC_CENTER) / 100.0) * DT      # ветер − управление
        x += v * DT
        t += DT
    return x

c_slow, c_fast = carry(), carry(ki_trim=60.0)
check(f"унос при ki=6 ≈ 100/6 м (полёты 16-17.5; получено {c_slow:.1f})",
      14.0 < c_slow < 20.0)
check(f"ki_trim=60: унос в ≥4 раза короче ({c_fast:.1f} м)",
      c_fast < c_slow / 4.0)


# --- 14. голый вход в ярус (стика нет ВООБЩЕ): первый стоп после движения
#         вяжет гвоздь и заканчивает быстрое обучение. Полёт lv2_joy_220204:
#         выход из фазы ki_trim был привязан к гвоздю, гвоздь — к стику; пилот
#         стик не трогал → ki_trim=60 молотил все 41 с яруса (см. тест 15).
vh14 = make(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0)
vh14.update(st(vx=0.1, t=100.05), Setpoint(), DT)        # вход на висении
check("вход без стика на висении: гвоздь НЕ вяжется (движения ещё не было)",
      vh14._pinx is None and not vh14._trim_armed)
vh14.update(st(vx=0.6, t=100.10), Setpoint(), DT)        # ветер понёс (>_PIN_V)
vh14.update(st(vx=0.2, x=1.4, t=100.15), Setpoint(), DT)  # первый стоп
check("первый стоп после движения: гвоздь в точке стопа + ветер выучен",
      vh14._pinx == 1.4 and vh14._trim_armed)


# --- 15. ЗАМКНУТЫЙ КОНТУР С ЛАГОМ НАКЛОНА: голое висение на ветру ---
# Полёт lv2_joy_20260903_220204 (ветер 5, стики центр весь ярус): контур трима
# = скрытый позиционный член (∫v = путь), ωn=√(ki/100), ζ=(kp/100)/(2·ωn);
# при ki_trim 60 ζ≈0.26 (T≈8.1 c) — и лаги канала (наклон FCU, vsmooth 0.3,
# латентность VINS) съедают остаток запаса: рост, период 7.1 с, ±2 м и
# 2.2 м/с к 40-й секунде яруса. Модель: наклон — апериодика τ=0.5 с.
def hover_lag(**kw):
    vh = DpVins(**dict(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0,
                       imax=120.0, max_pwm=150.0, cmd_gain=4.0, pos_kp=0.3,
                       pos_vmax=0.3, pos_acc=0.15, vsmooth=0.3, i_latch=True,
                       **kw))
    vh.enter(DroneState(now_sim=100.0))
    x = v = a = 0.0
    t = 100.05
    tail = []
    for _ in range(int(80.0 / DT)):            # 80 сим-секунд висения
        rc = vh.update(st(vx=v, x=x, t=t), Setpoint(), DT)
        cmd = -(rc.pitch - RC_CENTER) / 100.0  # PWM → м/с² (100 PWM = 1 м/с²)
        a += (DT / 0.5) * (cmd - a)            # лаг наклона τ = 0.5 с
        v += (0.5 + a) * DT                    # «ветер 5» ≈ 0.5 м/с²
        x += v * DT
        t += DT
        if t - 100.0 > 40.0:
            tail.append(abs(v))
    return max(tail)

v_tail = hover_lag()
check(f"хвост висения (40-80 с) спокоен: max|v| < 0.3 (получено {v_tail:.2f})",
      v_tail < 0.3)


# --- 16. ПОСЕВ трима от демпфера (seed_trim): канал → psign → мир ---
# Валюта — PWM каналов (после osign демпфера / до psign DpVins): обращение
# собственного уравнения выхода po = psign·(kp·err + i_fwd), никаких
# рассуждений о конвенциях (три зеркальных знака в истории проекта).
vh16 = make(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0)
check("посев принят (девственный трим)",
      vh16.seed_trim(-40.0, 10.0, st(t=100.0)))
rc = vh16.update(st(vx=0.0, t=100.05), Setpoint(), DT)
check("v=0: выход = ровно посеянные каналы (тангаж −40, крен +10)",
      rc.pitch == RC_CENTER - 40 and rc.roll == RC_CENTER + 10)
check("трим уже нажит (≥1 PWM) → повторный посев отказан",
      not vh16.seed_trim(99.0, 0.0, st()))
vh16a = make(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0)
vh16a._trim_armed = True
check("ветер выучен (armed) → посев отказан (своё свежее)",
      not vh16a.seed_trim(99.0, 0.0, st()) and vh16a._itx == 0.0)

# трим МИРОВОЙ: посеян на курсе 0, после разворота на 90° проецируется
# в другие оси тела (ветер не вращается вместе с бортом)
vh17 = make(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0)
vh17.seed_trim(-40.0, 10.0, st(t=100.0))
rc = vh17.update(st(vx=0.0, yaw=math.pi / 2, t=100.05), Setpoint(), DT)
# допуск ±1 PWM: cos(π/2)=6e-17 даёт 9.999…, int() срезает вниз
check("разворот 90° после посева: трим следует за курсом (каналы ≈ +10, +40)",
      abs(rc.pitch - (RC_CENTER + 10)) <= 1
      and abs(rc.roll - (RC_CENTER + 40)) <= 1)


# --- 17. замкнутый контур с посевом: унос ≈ 0 (п.5.3 dpvins.txt) ---
def carry_seed(seed, **kw):
    vh = DpVins(**dict(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0,
                       imax=120.0, max_pwm=150.0, cmd_gain=4.0, pos_kp=0.3,
                       pos_vmax=0.3, pos_acc=0.15, vsmooth=0.0, i_latch=True,
                       **kw))
    vh.enter(DroneState(now_sim=100.0))
    vh.seed_trim(seed, 0.0, st(t=100.0))
    x = v = 0.0
    t = 100.05
    for _ in range(4000):
        rc = vh.update(st(vx=v, x=x, t=t), Setpoint(), DT)
        v += (1.0 - (rc.pitch - RC_CENTER) / 100.0) * DT
        x += v * DT
        t += DT
    return x

c_seed = carry_seed(100.0)
check(f"посев = нужный трим (100): унос ≈ 0 (получено {c_seed:.2f} м)",
      abs(c_seed) < 0.5)
c_part = carry_seed(70.0)
# Посев ВЗВОДИТ armed (2026-09-05): остаток 30 PWM доучивается РАБОЧИМ ki, не ki_trim —
# унос ≈ 30/ki: при ki 6 стенда ~4 м (было <1 с ki_trim), в лётных профилях ki 15 → ~1.6 м.
# Плата осознанная: ki_trim после посева за секунду переписывал трим скоростью возврата
# демпфера (вход в ярус на ходу 0.85 м/с → −56 при +57, cmd_3/wind_right, унос 46 м).
check(f"посев неточный (70 из 100): остаток учит рабочий ki — унос ≈ 30/ki ({c_part:.2f} м, ki 6)",
      2.5 < abs(c_part) < 6.0)

# --- 18. trim_pwm(): обратная к seed_trim (стрелка ветра HUD) ---
# круговой проход: посев каналов → trim_pwm отдаёт их же; после разворота
# на 90° — те же числа, что выход теста 17 (мировой трим глазами нового тела)
vh18 = make(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0)
vh18.seed_trim(-40.0, 10.0, st(t=100.0))
vh18.update(st(vx=0.0, t=100.05), Setpoint(), DT)
p18, r18 = vh18.trim_pwm()
check(f"trim_pwm = ровно посеянные каналы ({p18:.1f}, {r18:.1f})",
      abs(p18 - (-40.0)) < 1e-6 and abs(r18 - 10.0) < 1e-6)
vh18.update(st(vx=0.0, yaw=math.pi / 2, t=100.10), Setpoint(), DT)
p18b, r18b = vh18.trim_pwm()
check(f"trim_pwm после разворота 90°: мировой трим в новом теле "
      f"({p18b:.1f}, {r18b:.1f} ≈ +10, +40)",
      abs(p18b - 10.0) < 1e-9 + 1e-6 and abs(r18b - 40.0) < 1e-6)
# ветка LOITER: DpVins не тикает (кэш yaw заморожен), нода даёт ТЕКУЩИЙ курс
# явным аргументом — проекция по нему, кэш игнорируется
p18c, r18c = vh18.trim_pwm(0.0)
check(f"trim_pwm(yaw=0) при замороженном кэше 90°: проекция по данному курсу "
      f"({p18c:.1f}, {r18c:.1f} ≈ −40, +10)",
      abs(p18c - (-40.0)) < 1e-6 and abs(r18c - 10.0) < 1e-6)

# ============ ФАЗА BRAKE внешнего контура (закон станции демпфера, 2026-09-05) ============
# Серия dphold_vs_dpvins + cmd/1–2: без брейка DpVins пропускал порыв на 6–9 м против
# 2.5 у DpHold — при уходе от гвоздя станция ставит цель −brake·v (ошибка ×(1+brake)).
# 16. brake=0 — цель станции РОВНО прежний _return_target (регресс бит-в-бит)
vh16 = make(kp_fwd=40.0, kp_lat=32.0, ki=6.0)
ok16 = all(abs(vh16._st_fwd.target(e, v) - vh16._return_target(e)) < 1e-12
           for e in (-3.0, -1.0, -0.2, 0.0, 0.05, 0.4, 2.0) for v in (-1.0, 0.0, 0.7))
check("brake=0: target() станции == _return_target (RETURN, √-кап) при любых e/v", ok16)


def hover_then_push(brake, v_push=0.6, n_push=20, **kw):
    """Гвоздь в нуле (стоп после движения), затем борт УНОСИТ от гвоздя вперёд со
    скоростью v_push (VINS x растёт). Возвращает (стаб, список |po|)."""
    vh = make(kp_fwd=40.0, kp_lat=32.0, ki=6.0, brake=brake, **kw)
    t = 100.05
    for i in range(5):                                # движение → _moved
        vh.update(st(vx=0.6, x=0.1 * i, t=t), Setpoint(), DT); t += DT
    for i in range(5):                                # стоп → гвоздь
        vh.update(st(vx=0.0, x=0.5, t=t), Setpoint(), DT); t += DT
    outs = []
    x = 0.5
    for i in range(n_push):                           # унос от гвоздя (порыв)
        x += v_push * DT
        rc = vh.update(st(vx=v_push, x=x, t=t), Setpoint(), DT); t += DT
        outs.append(rc.pitch - RC_CENTER)
    return vh, outs, x, t


vh_a, out_a, xa, ta = hover_then_push(0.0)
vh_b, out_b, xb, tb = hover_then_push(3.0)
check("гвоздь по стопу связан (оба)", vh_a._pinx is not None and vh_b._pinx is not None)
check("brake=3: при уносе 0.6 м/с фаза BRAKE активна", vh_b.braking and not vh_a.braking)
# цель без брейка: RETURN к гвоздю ≈ −min(0.3·e, 0.3, √(2·0.15·e)) ≈ −0.3 (к точке);
# с брейком: −3·0.6 = −1.8 → кап −1.0. Ошибка v−цель: 0.9 против 1.6 → выход ×1.8
ratio = abs(out_b[-1]) / max(abs(out_a[-1]), 1e-9)
check(f"brake=3: выход торможения в {ratio:.2f} раза больше (ожидание ~1.6–1.8)",
      1.5 < ratio < 2.0)
check("brake=3: знак выхода — против уноса (торможение вперёд-хода = +po)",
      out_b[-1] > 0 and out_a[-1] > 0)
# 17. выход из BRAKE: скорость развернулась к гвоздю → RETURN (цель к точке, мягкая)
vh_c, _, xc, tc = hover_then_push(3.0)
for i in range(10):
    xc -= 0.4 * DT
    rc = vh_c.update(st(vx=-0.4, x=xc, t=tc), Setpoint(), DT); tc += DT
check("разворот к гвоздю: BRAKE погашен (RETURN)", not vh_c.braking)
# 18. трим на торможении после первого гвоздя ЗАМОРОЖЕН (как _BRAKE_TRIM демпфера)
vh_d = make(kp_fwd=40.0, kp_lat=32.0, ki=30.0, brake=3.0)
t = 100.05
for i in range(5):
    vh_d.update(st(vx=0.6, x=0.1 * i, t=t), Setpoint(), DT); t += DT
for i in range(5):
    vh_d.update(st(vx=0.0, x=0.5, t=t), Setpoint(), DT); t += DT
check("подготовка: ветер «выучен» (первый гвоздь прошёл)", vh_d._trim_armed)
itx0 = vh_d._itx
x = 0.5
for i in range(20):
    x += 0.6 * DT
    vh_d.update(st(vx=0.6, x=x, t=t), Setpoint(), DT); t += DT
check("BRAKE после первого гвоздя: трим не мотается (заморожен)",
      vh_d.braking and abs(vh_d._itx - itx0) < 1e-9)
# без брейка тот же унос интегрировал бы трим
vh_e = make(kp_fwd=40.0, kp_lat=32.0, ki=30.0, brake=0.0)
t = 100.05
for i in range(5):
    vh_e.update(st(vx=0.6, x=0.1 * i, t=t), Setpoint(), DT); t += DT
for i in range(5):
    vh_e.update(st(vx=0.0, x=0.5, t=t), Setpoint(), DT); t += DT
ite0 = vh_e._itx; x = 0.5
for i in range(20):
    x += 0.6 * DT
    vh_e.update(st(vx=0.6, x=x, t=t), Setpoint(), DT); t += DT
check("без брейка тот же унос учит трим (контроль)", abs(vh_e._itx - ite0) > 1.0)
# 19. живой стик гасит BRAKE и снимает гвоздь
vh_f, _, xf, tf = hover_then_push(3.0)
vh_f.update(st(vx=0.6, x=xf, t=tf), Setpoint(c_fwd=1.0), DT)
check("стик: BRAKE погашен, гвоздь снят", not vh_f.braking and vh_f._pinx is None)
# 20. enter() сбрасывает фазу станции
vh_g, _, _, _ = hover_then_push(3.0)
vh_g.enter(DroneState(now_sim=200.0))
check("enter(): фаза BRAKE сброшена", not vh_g.braking)

# 21. ЗАПИРАНИЕ BRAKE и страховка brake_t (cmd_3/wind_right/1): ошибочный трим ПО ветру
# + боковая ось kp 32: тормоз развернуть снос не может, фаза не выходит, трим заморожен —
# унос вечен. brake_t: после N с непрерывного BRAKE трим снова учится.
def locked(brake_t, secs=12.0):
    vh = make(kp_fwd=32.0, kp_lat=32.0, ki=15.0, brake=5.0, brake_vmax=2.0, brake_t=brake_t)
    t = 100.05
    for i in range(5):
        vh.update(st(vx=0.6, x=0.1 * i, t=t), Setpoint(), DT); t += DT
    for i in range(5):
        vh.update(st(vx=0.0, x=0.5, t=t), Setpoint(), DT); t += DT       # гвоздь
    vh._itx = -56.0                                                    # ошибочный трим
    x, it0 = 0.5, vh._itx
    seen_brake = False
    for i in range(int(secs / DT)):                                    # secs с уноса 0.5 м/с
        x += 0.5 * DT
        vh.update(st(vx=0.5, x=x, t=t), Setpoint(), DT); t += DT
        seen_brake |= vh.braking
    return vh, seen_brake, it0
vh21a, br_a, it_a = locked(0.0)
check("brake_t=0: BRAKE активен весь унос, трим заморожен (запирание как в полёте)",
      br_a and vh21a.braking and abs(vh21a._itx - it_a) < 1e-9)
vh21b, br_b, it_b = locked(8.0)
check("brake_t=8: через 8 с непрерывного BRAKE трим снова учится (страховка)",
      br_b and abs(vh21b._itx - it_b) > 5.0)
# brake_t < 0 — хвост брейка как у демпфера: трим учится с первой секунды BRAKE (только
# анти-виндап в упоре), таймера нет; за 3 с уноса при brake_t 8 трим ещё стоит
vh21c, br_c, it_c = locked(-1.0, secs=3.0)
vh21d, br_d, it_d = locked(8.0, secs=3.0)
check("brake_t=-1: трим учится в BRAKE сразу (правило демпфера), при 8 — ещё заморожен",
      br_c and br_d and abs(vh21c._itx - it_c) > 5.0 and abs(vh21d._itx - it_d) < 1e-9)
check("brake_t=-1: трим в брейке ПРОТИВ уноса (ошибка ×6 по знаку скорости)",
      (vh21c._itx - it_c) > 0.0)
# 22. посев взводит armed → обучение рабочим ki, не ki_trim
vh22 = make(kp_fwd=40.0, kp_lat=32.0, ki=6.0, ki_trim=60.0)
vh22.seed_trim(-40.0, 10.0, st(t=100.05))
check("seed_trim: armed (ветер известен)", vh22._trim_armed)
it0 = vh22._itx; t = 100.05
for i in range(20):                                # 1 с движения 0.85 м/с (возврат демпфера)
    vh22.update(st(vx=0.85, x=0.85 * DT * i, t=t), Setpoint(), DT); t += DT
d22 = abs(vh22._itx - it0)
check(f"после посева 1 с хода 0.85 м/с меняет трим на {d22:.1f} PWM (ki 6, не ki_trim 60 ≈ 51)",
      d22 < 10.0)

# 23. ПО-ОСЕВАЯ ЗАЩЁЛКА (latch_axis, полёт 113224): крейсер стиком тангажа с боковым
# сносом 0.5 м/с (курс 0: fwd = x, right = −y… для DpVins v_rgt = −vx·sinψ + vy·cosψ = vy).
# Старое: любой стик морозит обе оси — боковой трим стоит. Новое: свободная (боковая) ось
# учится рабочим ki, движимая (продольная) заморожена; после отпускания хвост до гвоздя
# только у движимой; оба стика — обе заморожены.
def cruise(latch_axis, secs=5.0, c_right=0.0):
    vh = make(kp_fwd=40.0, kp_lat=32.0, ki=8.0, ki_trim=60.0, latch_axis=latch_axis)
    vh.seed_trim(0.0, 0.0, st()); vh._trim_armed = True          # ветер «выучен»
    t = 100.05
    for i in range(int(secs / DT)):                              # стик вперёд, снос вправо
        vh.update(st(vx=1.5, vy=0.5, x=1.5 * DT * i, y=0.5 * DT * i, t=t),
                  Setpoint(c_fwd=-0.4, c_right=c_right), DT); t += DT
    return vh, t
vh23a, _ = cruise(False)
check("latch_axis=0: стик тангажа морозит ОБЕ оси (боковой трим стоит)",
      abs(vh23a._ity) < 1e-9 and abs(vh23a._itx) < 1e-9)
vh23b, t23 = cruise(True)
check(f"latch_axis=1: свободная боковая ось учится на стике тангажа (ity {vh23b._ity:.1f} PWM ≈ 0.5·8·5 = 20)",
      15.0 < abs(vh23b._ity) < 25.0)
check("latch_axis=1: движимая продольная ось заморожена (itx 0)", abs(vh23b._itx) < 1e-9)
vh23c, _ = cruise(True, c_right=0.3)
check("latch_axis=1: оба стика — обе оси заморожены",
      abs(vh23c._itx) < 1e-9 and abs(vh23c._ity) < 1e-9)
# отпустили стик тангажа: продольная ось в хвосте защёлки (до гвоздя) — стоит,
# боковая — учится дальше; торможение вперёд 1.0 м/с с боковым 0.5
ity0 = vh23b._ity
for i in range(20):
    vh23b.update(st(vx=1.0, vy=0.5, x=10.0 + 1.0 * DT * i, y=3.0 + 0.5 * DT * i, t=t23),
                 Setpoint(), DT); t23 += DT
check("latch_axis=1: после отпускания хвост до гвоздя только у движимой оси "
      f"(itx 0, ity растёт {ity0:.1f} → {vh23b._ity:.1f})",
      abs(vh23b._itx) < 1e-9 and abs(vh23b._ity) > abs(ity0) + 2.0)

# 24. ГВОЗДЬ СРАЗУ НА ВХОДЕ (pin_armed, bag 130326): трим посеян (armed) и борт стоит →
# гвоздь первым кадром; без посева (девственный трим) — как раньше, только по стопу после
# движения; вход на ходу — гвоздь по стопу.
vh24a = make(kp_fwd=40.0, kp_lat=32.0, ki=8.0, ki_trim=60.0, pin_armed=True)
vh24a.seed_trim(-20.0, 30.0, st())
vh24a.update(st(vx=0.05, t=100.05), Setpoint(), DT)
check("pin_armed + посев + стоим: гвоздь первым кадром", vh24a._pinx is not None)
vh24b = make(kp_fwd=40.0, kp_lat=32.0, ki=8.0, ki_trim=60.0, pin_armed=True)
vh24b.update(st(vx=0.05, t=100.05), Setpoint(), DT)
check("pin_armed без посева (девственный): гвоздя нет — фаза ki_trim ждёт движения",
      vh24b._pinx is None and not vh24b._trim_armed)
vh24c = make(kp_fwd=40.0, kp_lat=32.0, ki=8.0, ki_trim=60.0, pin_armed=False)
vh24c.seed_trim(-20.0, 30.0, st())
vh24c.update(st(vx=0.05, t=100.05), Setpoint(), DT)
check("pin_armed=0 (старое): посев есть, стоим — гвоздя нет до движения", vh24c._pinx is None)
vh24d = make(kp_fwd=40.0, kp_lat=32.0, ki=8.0, ki_trim=60.0, pin_armed=True)
vh24d.seed_trim(-20.0, 30.0, st()); t = 100.05
for i in range(10):
    vh24d.update(st(vx=0.85, x=0.85 * DT * i, t=t), Setpoint(), DT); t += DT
moving = vh24d._pinx is None
for i in range(3):
    vh24d.update(st(vx=0.1, x=0.5, t=t), Setpoint(), DT); t += DT
check("pin_armed на ходу 0.85 м/с: гвоздя нет, по стопу — есть", moving and vh24d._pinx is not None)

# 25. ЛИНИЯ НА ПЛЕЧЕ (line_hold, плечи 150448): стик тангажа — гвоздь остаётся, свободная
# (боковая) ось держит проекцию гвоздя: уход вправо → цель боковой скорости к линии
# (RETURN) / BRAKE при уходе быстрее brake_v; старое: гвоздь снят, цель 0.
def leg(line_hold, vy=0.0, y=0.0, yaw=0.0, n=10):
    vh = make(kp_fwd=40.0, kp_lat=32.0, ki=8.0, ki_trim=60.0, pin_armed=True, line_hold=line_hold,
              brake=5.0, brake_vmax=2.0, brake_t=-1.0)
    vh.seed_trim(0.0, 0.0, st()); vh.update(st(vx=0.05, t=100.05), Setpoint(), DT)   # гвоздь (0,0)
    t = 100.1
    for i in range(n):
        vh.update(st(vx=2.0, vy=vy, x=2.0 * DT * i, y=y + vy * DT * i, yaw=yaw, t=t),
                  Setpoint(c_fwd=-0.5), DT); t += DT
    return vh, t
vh25a, _ = leg(False, y=1.0)
check("line_hold=0: стик тангажа снимает гвоздь", vh25a._pinx is None)
vh25b, t25 = leg(True, y=1.0)
check("line_hold=1: гвоздь остаётся на плече", vh25b._pinx is not None)
# курс 0: боковая ось тела = +y (влево); борт в y=+1 (слева от линии) → цель вправо (−)
# RETURN: kp_pos 0.3·(−1) = −0.3, √-кап 0.15 → −0.55 → −0.3
ph = vh25b.station_phase()
check(f"line_hold=1: фаза rel/hold (движимая rel, свободная hold) — {ph[:2]}", ph[:2] == ('rel', 'hold'))
ro_b = vh25b.update(st(vx=2.0, vy=0.0, x=3.0, y=1.0, t=t25), Setpoint(c_fwd=-0.5), DT).roll - RC_CENTER
vh25c, t25c = leg(False, y=1.0)
ro_c = vh25c.update(st(vx=2.0, vy=0.0, x=3.0, y=1.0, t=t25c), Setpoint(c_fwd=-0.5), DT).roll - RC_CENTER
check(f"line_hold=1: слева от линии в покое поперёк — крен К линии ({ro_b:+d}), старое — 0 ({ro_c:+d})",
      ro_b != 0 and ro_c == 0)
vh25d, _ = leg(True, vy=0.6, y=0.0, n=20)          # уход влево 0.6 м/с от линии → BRAKE боковой
check("line_hold=1: уход от линии 0.6 м/с — BRAKE свободной оси", vh25d._st_rgt.braking)
vh25e, _ = leg(True, y=1.0, yaw=0.5)               # курс ушёл на 0.5 рад > 0.3 → перезахват
check("line_hold=1: уход курса > 17° — линия перезахвачена в текущей точке",
      vh25e._pinx is not None and abs(vh25e._pin_yaw - 0.5) < 1e-9)
# отпускание после плеча: гвоздь снят, по стопу — новый
vh25b.update(st(vx=1.0, x=4.0, y=1.0, t=t25 + DT), Setpoint(), DT)
check("line_hold=1: стик отпущен — гвоздь плеча снят (нет возврата на всё плечо)", vh25b._pinx is None)
for i in range(3):
    vh25b.update(st(vx=0.1, x=4.5, y=1.0, t=t25 + DT * (2 + i)), Setpoint(), DT)
check("line_hold=1: по стопу — новый гвоздь в точке стопа",
      vh25b._pinx is not None and abs(vh25b._pinx - 4.5) < 1e-9)
# 26. ПРЯМАЯ ПЕРЕДАЧА СТИКА (ff): при v = цель P даёт 0, ff даёт наклон к цели; на цели
# станции (висение) ff не действует
vh26a = make(kp_fwd=40.0, kp_lat=32.0, ki=0.0, ff=10.0)
vh26b = make(kp_fwd=40.0, kp_lat=32.0, ki=0.0, ff=0.0)
sp26 = Setpoint(c_fwd=-0.5)                          # цель −2 м/с (назад? знак: c_fwd·gain = −2)
pa = vh26a.update(st(vx=-2.0, t=100.05), sp26, DT).pitch - RC_CENTER
pb = vh26b.update(st(vx=-2.0, t=100.05), sp26, DT).pitch - RC_CENTER
p_lag = vh26b.update(st(vx=-1.5, t=100.10), sp26, DT).pitch - RC_CENTER   # v медленнее цели → P
check(f"ff=10: на цели ±2 м/с выход {pa:+d} (ff·2 = 20 PWM), без ff {pb:+d}; знак как у P при v<цели ({p_lag:+d})",
      pb == 0 and abs(abs(pa) - 20) <= 1 and (pa * p_lag > 0))
vh26c = make(kp_fwd=40.0, kp_lat=32.0, ki=0.0, ff=10.0, pin_armed=True)
vh26c.seed_trim(0.0, 0.0, st()); vh26c._trim_armed = True
vh26c.update(st(vx=0.05, t=100.05), Setpoint(), DT)
pc = vh26c.update(st(vx=0.0, x=0.5, t=100.10), Setpoint(), DT).pitch - RC_CENTER   # RETURN-цель, стик центр
vh26d = make(kp_fwd=40.0, kp_lat=32.0, ki=0.0, ff=0.0, pin_armed=True)
vh26d.seed_trim(0.0, 0.0, st()); vh26d._trim_armed = True
vh26d.update(st(vx=0.05, t=100.05), Setpoint(), DT)
pd = vh26d.update(st(vx=0.0, x=0.5, t=100.10), Setpoint(), DT).pitch - RC_CENTER
check(f"ff не действует на цель станции (висение): {pc:+d} == {pd:+d}", pc == pd)

# 27. ТОРМОЗ С ОТПУСКАНИЯ (settle_brake) и ГВОЗДЬ ПО ТАЙМАУТУ (pin_t), полёт 160730
def released(settle_brake, pin_t=0.0):
    vh = make(kp_fwd=40.0, kp_lat=32.0, ki=0.0, brake=5.0, brake_vmax=2.0, brake_t=-1.0,
              settle_brake=settle_brake, pin_t=pin_t)
    vh.seed_trim(0.0, 0.0, st()); t = 100.05
    for i in range(5):                                        # стик вперёд, разгон 3 м/с
        vh.update(st(vx=3.0, x=3.0 * DT * i, t=t), Setpoint(c_fwd=-0.6), DT); t += DT
    return vh, t
vh27a, t27 = released(False)
pa = vh27a.update(st(vx=3.0, x=1.0, t=t27), Setpoint(), DT).pitch - RC_CENTER
vh27b, t27b = released(True)
pb = vh27b.update(st(vx=3.0, x=1.0, t=t27b), Setpoint(), DT).pitch - RC_CENTER
check(f"отпущен на 3 м/с: settle_brake=0 → P·v = 120 ({pa:+d}); =1 → цель −2 (кап), упор 150 ({pb:+d})",
      abs(pa) == 120 and abs(pb) == 150 and pa * pb > 0)
check("settle_brake: фаза set/set (гвоздя нет), станция не задействована",
      vh27b.station_phase()[:2] == ('set', 'set') and vh27b._pinx is None)
pc = vh27b.update(st(vx=0.2, x=2.0, t=t27b + DT), Setpoint(), DT)
check("settle_brake: скорость упала < pin_v → гвоздь по стопу", vh27b._pinx is not None)
# pin_t: под ветром |v| держится 0.5 > pin_v — без таймаута гвоздя нет 10 с, с pin_t 3 — есть
vh27c, t27c = released(False, pin_t=0.0); vh27d, t27d = released(False, pin_t=3.0)
for i in range(int(4.0 / DT)):
    vh27c.update(st(vx=0.5, x=1.0 + 0.5 * DT * i, t=t27c), Setpoint(), DT); t27c += DT
    vh27d.update(st(vx=0.5, x=1.0 + 0.5 * DT * i, t=t27d), Setpoint(), DT); t27d += DT
check("pin_t=0: 4 с на 0.5 м/с — гвоздя нет (унос без тормоза)", vh27c._pinx is None)
check("pin_t=3: через 3 с — гвоздь принудительно, дальше BRAKE станции",
      vh27d._pinx is not None and vh27d._st_fwd.braking)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ DPVINS OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
