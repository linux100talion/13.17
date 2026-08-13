#!/usr/bin/env python3
"""Юнит-тесты флоу-стабилизаторов среза 3 (чистый python, без ROS):
- DpRollHold (roll): демпф к цели, velocity-assist (стик=цель → 0 коррекции),
  ПОКАДРОВАЯ интеграция (тот же flow_seq не двигает PID), stale-fade, conf-blend.
- DpYawHold (yaw): чистый rate-демпф (ki=0), velocity-assist.

Запуск:  python3 src/control/test/test_flow_strategies.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.stabilization import DpRollHold, DpYawHold  # noqa: E402
from control_pkg.domain.rc import RC_CENTER                               # noqa: E402
from control_pkg.domain.setpoint import Setpoint                          # noqa: E402
from control_pkg.domain.state import DroneState                           # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def _yh(**kw):
    """DpYawHold БЕЗ ВЗВЕДЕНИЯ. Тесты ниже — про закон управления, и им нужен отклик на
    ПЕРВОМ же кадре. Гейт достоверности (arm_frames, лечение YW1s1) держит ось молчащей
    первые 5 хороших кадров и проверяется отдельно, в конце файла."""
    kw.setdefault('arm_frames', 0)
    return DpYawHold(**kw)


def st(seq, lat=0.0, yaw=0.0, conf=0.5, dt=0.05, now=0.05):
    return DroneState(flow_seq=seq, flow_lateral=lat, flow_yaw=yaw,
                      flow_conf=conf, flow_dt=dt, now_sim=now)


# --- DpRollHold: активный демпф ---
fd = DpRollHold()  # kp8 ki2 kd0 imax120 max150 conf[.05,.2] osign1 cmd_gain10 (c_right=0→цель 0)
fd.enter(st(-1))
rc = fd.update(st(1, lat=5.0, now=0.05), Setpoint(), 0.05)
# blend=1, err=5, i=2*5*0.05=0.5, u=8*5+0.5=40.5 → roll=1540
check("DpRollHold демпфит боковой поток (roll=1540)", rc.roll == 1540)

# ПОКАДРОВО: тот же flow_seq → PID НЕ двигается (выход держится)
rc2 = fd.update(st(1, lat=5.0, now=0.06), Setpoint(), 0.05)
check("DpRollHold покадрово: тот же seq → выход держится", rc2.roll == 1540)

# velocity-assist: стик задаёт цель = поток → ошибка 0 → нет коррекции.
# ⚠️ ЗНАК: было `lat=+5` против `c_right=+5`, то есть снос ВЛЕВО (flow_lat>0 = борт
# идёт влево) гасился стиком ВПРАВО. Тест был написан под реализацию, а не под физику,
# и ровно поэтому пропустил зеркальную команду до самого R2. Правильно: сносит влево —
# отпускает это стик ВЛЕВО.
fd2 = DpRollHold(cmd_gain=1.0)
fd2.enter(st(-1))
rc = fd2.update(st(1, lat=5.0, now=0.05), Setpoint(c_right=-5.0), 0.05)
check("DpRollHold velocity-assist: стик=поток → roll=центр", rc.roll == 1500)

# stale: кадр протух (>0.5с без нового seq) → fade в центр
fd3 = DpRollHold()
fd3.enter(st(-1))
fd3.update(st(1, lat=5.0, now=0.05), Setpoint(), 0.05)
rc = fd3.update(st(1, lat=5.0, now=1.0), Setpoint(), 0.05)   # тот же seq, время ушло
check("DpRollHold stale → fade в центр", rc.roll == 1500)

# confidence ниже порога → авторитет 0
fd4 = DpRollHold()
fd4.enter(st(-1))
rc = fd4.update(st(1, lat=5.0, conf=0.0, now=0.05), Setpoint(), 0.05)
check("DpRollHold conf<min → авторитет 0 (центр)", rc.roll == 1500)

# --- DpYawHold: pos-режим (курс-холд), команда через интегратор уставки ---
# ЭКВИВАЛЕНТНОСТЬ СТАРОМУ ЗАКОНУ. Был чистый rate-демпф yu = kp·flow_yaw при kp=6.
# Стало: D-член pos-режима = kd·(flow_yaw − скорость уставки), kd=6. При kp=0 и нулевой
# команде это ТО ЖЕ ЧИСЛО — победитель свипа [[yaw-hold-tuning]] не переигран.
yh = _yh(leak_sec=0.0)  # kp0 ki0 kd6, утечка выключена → u = 6·flow_yaw РОВНО
yh.enter(st(-1))
rc = yh.update(st(1, yaw=3.0, now=0.05), Setpoint(), 0.05)
check("DpYawHold: старый закон сохранён ТОЧНО (yaw=1518)", rc.yaw == 1518)
check("DpYawHold владеет только yaw", rc.roll == 1500 and rc.pitch == 1500)
# с утечкой по умолчанию (T=8с) D-член слабее на fdt/T = 0.6% за кадр — в PWM не видно
yhl = _yh()
yhl.enter(st(-1))
rc = yhl.update(st(1, yaw=3.0, now=0.05), Setpoint(), 0.05)
check("DpYawHold: утечка T=8с почти не трогает демпфер (1517..1518)",
      rc.yaw in (1517, 1518))

# ЗНАК КОМАНДЫ (замер Y2: без минуса ось отрабатывала токен ЗЕРКАЛЬНО — yaw_l30 давал
# −21…−28°, yaw_r60 давал +70…+76°). `c_yaw>0` = стик ВПРАВО, и открытый контур на нём
# даёт PWM ВЫШЕ центра (см. test_families). Холдер обязан выдавать ТУДА ЖЕ, иначе один
# и тот же токен означает разные стороны с холдером и без него.
yhs = _yh(leak_sec=0.0)
yhs.enter(st(-1))
rc = yhs.update(st(1, yaw=0.0, now=0.05), Setpoint(c_yaw=0.3), 0.05)
check("DpYawHold: c_yaw>0 (вправо) → PWM выше центра, как открытый контур",
      rc.yaw > RC_CENTER)
check("DpYawHold: уставка курса едет в МИНУС (flow_yaw>0 = разворот влево)",
      yhs.hold_dbg()[0] < 0.0)

# КОМАНДА ЕДЕТ ЧЕРЕЗ УСТАВКУ, а не вычитается из сигнала. cmd_gain=1, c_yaw=3 →
# скорость уставки −3 ед/с; за кадр 0.05с уставка уезжает на −0.15, и если борт
# развернулся вправо ровно на столько (flow_yaw=−3) — ошибка ноль, и D-член тоже.
yh2 = _yh(cmd_gain=1.0, leak_sec=0.0)
yh2.enter(st(-1))
rc = yh2.update(st(1, yaw=-3.0, now=0.05), Setpoint(c_yaw=3.0), 0.05)
check("DpYawHold: борт идёт за уставкой → yaw=центр", rc.yaw == 1500)
sp, err, rate = yh2.hold_dbg()
check("DpYawHold отдаёт уставку курса (pos-ось)", abs(sp + 0.15) < 1e-9 and abs(err) < 1e-9)
check("DpYawHold: скорость уставки = −c_yaw·cmd_gain", abs(rate + 3.0) < 1e-9)

# УСТАВКА ОСТАЁТСЯ ПОСЛЕ ОТПУСКАНИЯ СТИКА (в этом вся разница с velocity-assist:
# там борт вставал где угодно, здесь недоехавший угол остаётся долгом контура).
rc = yh2.update(st(2, yaw=0.0, now=0.10), Setpoint(), 0.05)
sp2, err2, rate2 = yh2.hold_dbg()
check("DpYawHold: стик отпущен → уставка стоит, не сбрасывается", abs(sp2 + 0.15) < 1e-9)
check("DpYawHold: скорость уставки обнулилась", rate2 == 0.0)

# УТЕЧКА накопителя: без неё смещение flow_yaw (−0.4°/с, замер Y1s) копится в фантомный
# курс без предела — это тот механизм, которым свип отверг ki.
leaky = _yh(leak_sec=1.0)
leaky.enter(st(-1))
for k in range(1, 41):                       # 40 кадров по 0.05с = 2с постоянного смещения
    leaky.update(st(k, yaw=1.0, now=0.05 * k), Setpoint(), 0.05)
tight = _yh(leak_sec=0.0)
tight.enter(st(-1))
for k in range(1, 41):
    tight.update(st(k, yaw=1.0, now=0.05 * k), Setpoint(), 0.05)
check("DpYawHold: утечка ограничивает накопленный курс", leaky._head < tight._head)
check("DpYawHold: без утечки курс копится линейно (≈2.0)", abs(tight._head - 2.0) < 1e-6)
check("DpYawHold: с утечкой T=1с фантом насыщается (<1.1)", leaky._head < 1.1)

# УТЕЧКА НЕ ЕСТ КОМАНДУ. Течёт ОШИБКА (курс к уставке), а не курс к нулю: борт,
# идущий за уставкой, для утечки неотличим от стоящего. Замер Y2 показал цену
# прежней утечки-к-нулю: на токене 60° накопитель просел с 82° до 46°, и контур
# добирал разницу разворотом — токен переворачивал на +25%.
cmd = _yh(cmd_gain=1.0, leak_sec=1.0)
cmd.enter(st(-1))
for k in range(1, 41):                       # 2с команды, борт идёт РОВНО за уставкой
    cmd.update(st(k, yaw=-3.0, now=0.05 * k), Setpoint(c_yaw=3.0), 0.05)
sp, err, _ = cmd.hold_dbg()
check("DpYawHold: утечка не съедает уставку команды (≈−6.0)", abs(sp + 6.0) < 1e-6)
check("DpYawHold: борт за уставкой → утечке нечего есть (ошибка ≈0)", abs(err) < 1e-6)

# ШАГ ИНТЕГРИРОВАНИЯ = время С ПРОШЛОГО ПРОДВИЖЕНИЯ КОНТУРА, а не flow_dt. Домен тикает
# 20 Гц, камера ~30 кадров/с → часть кадров контур не видит, и по flow_dt он засчитывал
# 2/3 времени (замер Y3: уставка 20.0° за 3.5 с команды вместо 30°).
fdt = _yh(cmd_gain=1.0, leak_sec=0.0)
fdt.enter(st(-1))
fdt.update(st(1, dt=1 / 30, now=0.05), Setpoint(c_yaw=-1.0), 0.05)   # 1-й кадр → flow_dt
sp1 = fdt.hold_dbg()[0]
fdt.update(st(3, dt=1 / 30, now=0.10), Setpoint(c_yaw=-1.0), 0.05)   # seq 1→3: кадр пропущен
sp2 = fdt.hold_dbg()[0]
check("DpYawHold: первый кадр сегмента опирается на flow_dt (≈1/30)",
      abs(sp1 - 1 / 30) < 1e-9)
check("DpYawHold: дальше шаг = зазор домена (0.05), пропущенный кадр не теряется",
      abs((sp2 - sp1) - 0.05) < 1e-9)

# ЗАПИСЬ КОМАНДЫ rate-оси. Без неё командный сегмент ищется по истинной скорости, где
# откат борта после торможения неотличим от новой команды (R1: три «проезда» на две
# команды миссии). Крен обязан отдавать цель, тангаж на этот вопрос молчит.
rr = DpRollHold(kp=8.0, ki=0.0, kd=0.0, cmd_gain=10.0)
rr.enter(st(-1))
rr.update(st(1, lat=0.0), Setpoint(c_right=0.3), 0.05)
dbg = rr.rate_dbg()
check("DpRollHold: rate_dbg отдаёт цель −c_right·cmd_gain (≈−3.0)",
      dbg is not None and abs(dbg[0] + 3.0) < 1e-9)
check("DpRollHold: ошибка = сигнал − цель (≈+3.0)", abs(dbg[1] - 3.0) < 1e-9)
check("DpRollHold: третий слот — PWM выхода", abs(dbg[2] - (rr.update(
    st(2, lat=0.0), Setpoint(c_right=0.3), 0.05).roll - RC_CENTER)) < 1e-6)
check("DpYawHold (pos-ось) на rate_dbg молчит",
      _yh(cmd_gain=1.0).rate_dbg() is None)
check("DpRollHold (rate-ось) на hold_dbg молчит", rr.hold_dbg() is None)

# ЗНАК БОКОВОЙ КОМАНДЫ. Стик ВПРАВО обязан гнать борт вправо, то есть ставить
# ОТРИЦАТЕЛЬНУЮ цель по flow_lateral (мир в кадре уезжает влево) и выдавать PWM ВЫШЕ
# центра (провод крена: PWM>1500 = крен вправо = разгон вправо). R2 показал зеркало.
rs = DpRollHold(kp=8.0, ki=0.0, kd=0.0, cmd_gain=10.0)
rs.enter(st(-1))
rc_r = rs.update(st(1, lat=0.0), Setpoint(c_right=0.3), 0.05)
check("DpRollHold: стик ВПРАВО → цель по сигналу ОТРИЦАТЕЛЬНА",
      rs.rate_dbg()[0] < 0)
check("DpRollHold: стик ВПРАВО → PWM выше центра", rc_r.roll > RC_CENTER)
rs2 = DpRollHold(kp=8.0, ki=0.0, kd=0.0, cmd_gain=10.0)
rs2.enter(st(-1))
check("DpRollHold: стик ВЛЕВО → PWM ниже центра",
      rs2.update(st(1, lat=0.0), Setpoint(c_right=-0.3), 0.05).roll < RC_CENTER)

# --- ЗАЩИТА НАКОПИТЕЛЯ КУРСА ОТ МУСОРНОГО КАДРА (лечение YW1s1) ---
# Событие: на отрыве один кадр дал flow_yaw = −43.8 px/кадр (висенческие ±1) при
# conf=0.18. Накопитель набрал −99° фантома, контур упёрся в потолок и физически
# довернул борт на 95° влево. Проверяем оба гейта по отдельности и вместе.

# 1. ОТСЕВ НЕВОЗМОЖНОГО. Кадр выше потолка не должен попасть в накопитель ВОВСЕ.
rej = DpYawHold(kp=20.0, kd=0.0, leak_sec=0.0, max_step=32.4, arm_frames=0)
rej.enter(st(-1))
rc = rej.update(st(1, yaw=-43.8, conf=1.0, now=0.05), Setpoint(), 0.05)
check("отсев: кадр 43.8 px/кадр выброшен → накопитель пуст, команды нет",
      rc.yaw == RC_CENTER and abs(rej.hold_dbg()[1]) < 1e-9 and rej._rejects == 1)
# ...а правдоподобный кадр проходит: потолок не должен глушить настоящий разворот
ok_frame = DpYawHold(kp=20.0, kd=0.0, leak_sec=0.0, max_step=32.4, arm_frames=0)
ok_frame.enter(st(-1))
rc = ok_frame.update(st(1, yaw=-30.0, conf=1.0, now=0.05), Setpoint(), 0.05)
check("отсев: кадр 30 px/кадр (ниже потолка) проходит и командует",
      rc.yaw != RC_CENTER and ok_frame._rejects == 0)
# ВЫБРАСЫВАЕТСЯ, а не подрезается: подрезка влила бы максимально допустимый фантом
clamped = DpYawHold(kp=20.0, kd=0.0, leak_sec=0.0, max_step=32.4, arm_frames=0)
clamped.enter(st(-1))
clamped.update(st(1, yaw=-1000.0, conf=1.0, now=0.05), Setpoint(), 0.05)
check("отсев: выброс, а не подрезка (накопитель ровно 0, не −32.4·dt)",
      clamped.hold_dbg()[1] == 0.0)

# 2. ВЗВЕДЕНИЕ. Гейт conf_min=0.05 взлётный выброс пропустил (conf там 0.18), поэтому
# накопителю нужен свой порог: conf_full подряд arm_frames кадров.
arm = DpYawHold(kp=20.0, kd=0.0, leak_sec=0.0, conf_full=0.20, arm_frames=5,
                max_step=0.0)
arm.enter(st(-1))
rc = arm.update(st(1, yaw=-43.8, conf=0.18, now=0.05), Setpoint(), 0.05)
check("взведение: conf 0.18 < conf_full → ни команды, ни накопления",
      rc.yaw == RC_CENTER and abs(arm.hold_dbg()[1]) < 1e-9)
# сигнал 5 px/кадр, а не 1: при kp=20 и шаге 0.05с единица даёт ровно 1.0 PWM, и
# округление вниз (int) съело бы отклик — тест провалился бы на арифметике, а не на гейте
for k in range(2, 7):                       # пять хороших кадров подряд — взводимся
    rc = arm.update(st(k, yaw=5.0, conf=1.0, now=0.05 * k), Setpoint(), 0.05)
check("взведение: 5 хороших кадров молчат (ось ещё не взведена)", rc.yaw == RC_CENTER)
rc = arm.update(st(7, yaw=5.0, conf=1.0, now=0.35), Setpoint(), 0.05)
check("взведение: 6-й хороший кадр — ось работает", rc.yaw != RC_CENTER)
# срыв достоверности сбрасывает счётчик: одиночный хороший кадр посреди срыва не взводит
arm.update(st(8, yaw=5.0, conf=0.0, now=0.40), Setpoint(), 0.05)
rc = arm.update(st(9, yaw=5.0, conf=1.0, now=0.45), Setpoint(), 0.05)
check("взведение: срыв conf сбрасывает счётчик (снова молчим)", rc.yaw == RC_CENTER)

# 3. ВМЕСТЕ, НА ДЕФОЛТАХ: сценарий YW1s1 целиком не должен пройти.
yw = DpYawHold(kp=20.0)
yw.enter(st(-1))
rc = yw.update(st(1, yaw=-43.8, conf=0.18, now=0.05), Setpoint(), 0.05)
check("YW1s1 на дефолтах: взлётный кадр не разворачивает борт",
      rc.yaw == RC_CENTER and abs(yw.hold_dbg()[1]) < 1e-9)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ФЛОУ-СТАБИЛИЗАТОРЫ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
