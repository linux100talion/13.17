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

# velocity-assist: стик задаёт цель = поток → ошибка 0 → нет коррекции
fd2 = DpRollHold(cmd_gain=1.0)
fd2.enter(st(-1))
rc = fd2.update(st(1, lat=5.0, now=0.05), Setpoint(c_right=5.0), 0.05)
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
yh = DpYawHold(leak_sec=0.0)  # kp0 ki0 kd6, утечка выключена → u = 6·flow_yaw РОВНО
yh.enter(st(-1))
rc = yh.update(st(1, yaw=3.0, now=0.05), Setpoint(), 0.05)
check("DpYawHold: старый закон сохранён ТОЧНО (yaw=1518)", rc.yaw == 1518)
check("DpYawHold владеет только yaw", rc.roll == 1500 and rc.pitch == 1500)
# с утечкой по умолчанию (T=8с) D-член слабее на fdt/T = 0.6% за кадр — в PWM не видно
yhl = DpYawHold()
yhl.enter(st(-1))
rc = yhl.update(st(1, yaw=3.0, now=0.05), Setpoint(), 0.05)
check("DpYawHold: утечка T=8с почти не трогает демпфер (1517..1518)",
      rc.yaw in (1517, 1518))

# ЗНАК КОМАНДЫ (замер Y2: без минуса ось отрабатывала токен ЗЕРКАЛЬНО — yaw_l30 давал
# −21…−28°, yaw_r60 давал +70…+76°). `c_yaw>0` = стик ВПРАВО, и открытый контур на нём
# даёт PWM ВЫШЕ центра (см. test_families). Холдер обязан выдавать ТУДА ЖЕ, иначе один
# и тот же токен означает разные стороны с холдером и без него.
yhs = DpYawHold(leak_sec=0.0)
yhs.enter(st(-1))
rc = yhs.update(st(1, yaw=0.0, now=0.05), Setpoint(c_yaw=0.3), 0.05)
check("DpYawHold: c_yaw>0 (вправо) → PWM выше центра, как открытый контур",
      rc.yaw > RC_CENTER)
check("DpYawHold: уставка курса едет в МИНУС (flow_yaw>0 = разворот влево)",
      yhs.hold_dbg()[0] < 0.0)

# КОМАНДА ЕДЕТ ЧЕРЕЗ УСТАВКУ, а не вычитается из сигнала. cmd_gain=1, c_yaw=3 →
# скорость уставки −3 ед/с; за кадр 0.05с уставка уезжает на −0.15, и если борт
# развернулся вправо ровно на столько (flow_yaw=−3) — ошибка ноль, и D-член тоже.
yh2 = DpYawHold(cmd_gain=1.0, leak_sec=0.0)
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
leaky = DpYawHold(leak_sec=1.0)
leaky.enter(st(-1))
for k in range(1, 41):                       # 40 кадров по 0.05с = 2с постоянного смещения
    leaky.update(st(k, yaw=1.0, now=0.05 * k), Setpoint(), 0.05)
tight = DpYawHold(leak_sec=0.0)
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
cmd = DpYawHold(cmd_gain=1.0, leak_sec=1.0)
cmd.enter(st(-1))
for k in range(1, 41):                       # 2с команды, борт идёт РОВНО за уставкой
    cmd.update(st(k, yaw=-3.0, now=0.05 * k), Setpoint(c_yaw=3.0), 0.05)
sp, err, _ = cmd.hold_dbg()
check("DpYawHold: утечка не съедает уставку команды (≈−6.0)", abs(sp + 6.0) < 1e-6)
check("DpYawHold: борт за уставкой → утечке нечего есть (ошибка ≈0)", abs(err) < 1e-6)

# ШАГ ИНТЕГРИРОВАНИЯ = время С ПРОШЛОГО ПРОДВИЖЕНИЯ КОНТУРА, а не flow_dt. Домен тикает
# 20 Гц, камера ~30 кадров/с → часть кадров контур не видит, и по flow_dt он засчитывал
# 2/3 времени (замер Y3: уставка 20.0° за 3.5 с команды вместо 30°).
fdt = DpYawHold(cmd_gain=1.0, leak_sec=0.0)
fdt.enter(st(-1))
fdt.update(st(1, dt=1 / 30, now=0.05), Setpoint(c_yaw=-1.0), 0.05)   # 1-й кадр → flow_dt
sp1 = fdt.hold_dbg()[0]
fdt.update(st(3, dt=1 / 30, now=0.10), Setpoint(c_yaw=-1.0), 0.05)   # seq 1→3: кадр пропущен
sp2 = fdt.hold_dbg()[0]
check("DpYawHold: первый кадр сегмента опирается на flow_dt (≈1/30)",
      abs(sp1 - 1 / 30) < 1e-9)
check("DpYawHold: дальше шаг = зазор домена (0.05), пропущенный кадр не теряется",
      abs((sp2 - sp1) - 0.05) < 1e-9)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ ФЛОУ-СТАБИЛИЗАТОРЫ OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
