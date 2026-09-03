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

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ DPVINS OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
