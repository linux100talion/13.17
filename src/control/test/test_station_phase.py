#!/usr/bin/env python3
"""Юнит-тест поля фазы станции в /mission/status (brk=/ifz=, 2026-09-06). Чистый python.

До него фазу BRAKE в bag было не видно — разбор cmd/3–5 шёл по косвенным признакам
(скорость против гвоздя, константа стрелки ветра). Проверяет: коды фаз DpVins по
стику/стопу/гвоздю/брейку и заморозка трима по осям (latch_axis); демпфер —
маппинг StationKeeper.phase и анти-виндап в _i_frozen; композит DpHold отдаёт
пару осей; hud_status печатает поля только при живом источнике; формат k=v.

Запуск:  python3 src/control/test/test_station_phase.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.hud import hud_status                    # noqa: E402
from control_pkg.domain.control.flow_axes import DpHold                # noqa: E402
from control_pkg.domain.control.station_keeper import StationKeeper    # noqa: E402
from control_pkg.domain.control.vins_axes import DpVins                # noqa: E402
from control_pkg.domain.setpoint import Setpoint                       # noqa: E402
from control_pkg.domain.state import DroneState                        # noqa: E402

DT = 0.05
results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def st(vx=0.0, vy=0.0, x=0.0, y=0.0, t=100.05):
    return DroneState(now_sim=t, vins_x=x, vins_y=y, vins_yaw=0.0,
                      vins_vx=vx, vins_vy=vy, vins_valid=True)


# 1. DpVins: до движения — set/set (гвоздя нет), трим учится (armed ещё нет: ki_trim)
vh = DpVins(kp_fwd=40.0, kp_lat=32.0, ki=8.0, ki_trim=60.0, imax=120.0, max_pwm=150.0,
            cmd_gain=4.0, pos_kp=0.3, pos_vmax=0.3, pos_acc=0.15, vsmooth=0.0, i_latch=True,
            brake=5.0, brake_vmax=2.0, brake_t=-1.0, latch_axis=True)
vh.enter(DroneState(now_sim=100.0))
t = 100.05
vh.update(st(t=t), Setpoint(), DT); t += DT
check("DpVins на входе: set/set, трим учится", vh.station_phase() == ("set", "set", False, False))
# 2. стик тангажа → rel/set; заморожена только продольная (latch_axis)
for i in range(10):
    vh.update(st(vx=1.0, x=1.0 * DT * i, t=t), Setpoint(c_fwd=-0.4), DT); t += DT
check("стик тангажа: rel/set, ifz 1/0 (по-осевая защёлка)",
      vh.station_phase() == ("rel", "set", True, False))
# 3. отпустили, тормозим: set/set, продольная в хвосте до гвоздя (armed после посева)
vh.seed_trim(0.0, 0.0, st(t=t)); vh._trim_armed = True
for i in range(5):
    vh.update(st(vx=0.6, x=1.0 + 0.6 * DT * i, t=t), Setpoint(), DT); t += DT
check("отпущен, тормозим: set/set, хвост защёлки только у продольной (ifz 1/0)",
      vh.station_phase() == ("set", "set", True, False))
# 4. стоп → гвоздь: hold/hold, трим учится
for i in range(5):
    vh.update(st(vx=0.0, x=1.5, t=t), Setpoint(), DT); t += DT
check("стоп → гвоздь: hold/hold, ifz 0/0", vh.station_phase() == ("hold", "hold", False, False))
# 5. унос от гвоздя быстрее brake_v → brk по продольной; brake_t −1 → трим учится
x = 1.5
for i in range(10):
    x += 0.6 * DT
    vh.update(st(vx=0.6, x=x, t=t), Setpoint(), DT); t += DT
ph = vh.station_phase()
check(f"унос 0.6 м/с от гвоздя: brk/hold, ifz 0/0 (brake_t −1) — {ph}",
      ph == ("brk", "hold", False, False))

# 6. демпфер: маппинг фаз StationKeeper и код композита DpHold
sk = StationKeeper(kp=0.3, brake=3.0)
check("StationKeeper.phase: released", sk.phase == "released")
sk.wait_t = 1.0
check("StationKeeper.phase: settling", sk.phase == "settling")
sk.pin = (0.0, 0.0); sk.wait_t = None
check("StationKeeper.phase: hold", sk.phase == "hold")
sk.braking = True
check("StationKeeper.phase: brake", sk.phase == "brake")
dh = DpHold()
ph = dh.station_phase()
check(f"DpHold.station_phase() композита — пара осей ({ph})",
      ph is not None and ph[0] == "rel" and ph[1] == "rel" and ph[2] is False and ph[3] is False)
sub = next(x for x in dh._subs if getattr(x, "_axis", None) == "roll")
sub.station.pin = (0.0, 0.0); sub.station.braking = True; sub._i_frozen = True
check("DpHold: крен в BRAKE с замороженным И → ('rel','brk',False,True)",
      dh.station_phase() == ("rel", "brk", False, True))

# 7. hud_status: полей нет без источника, есть при st_phase
line = hud_status(DroneState(now_sim=5.0), 2.0)
check("hud_status без станции: полей brk=/ifz= нет", "brk=" not in line and "ifz=" not in line)
s = DroneState(now_sim=5.0); s.st_phase = "hold/brk"; s.st_ifz = "0/1"
line = hud_status(s, 2.0)
d = dict(p.split('=', 1) for p in line.split() if '=' in p)
check("hud_status со станцией: brk=hold/brk ifz=0/1 разбирается k=v",
      d.get("brk") == "hold/brk" and d.get("ifz") == "0/1")

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ STATION PHASE OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
