#!/usr/bin/env python3
"""Юнит-тест WindTrim — общего ветрового трима ярусов 0/1 (wind_trim.py). Чистый python.

Проверяет: математику (канал ↔ мир, поворот, по-осевая запись не трогает другую
компоненту, кламп); делегирование StationFrame (пре-osign ↔ канал через sign, reset
рамы не трогает ветер); DpVins на общем триме (учится в него, читает по att_yaw, посев
и сброс — no-op, «выучен» = wind.learned); обмен между ярусами (демпфер записал →
DpVins видит тот же канал под тем же курсом; DpVins выучил → демпфер на входе не
проходит фазу захвата).

Запуск:  python3 src/control/test/test_wind_trim.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.station_frame import StationFrame   # noqa: E402
from control_pkg.domain.control.station_keeper import StationKeeper  # noqa: E402
from control_pkg.domain.control.vins_axes import DpVins              # noqa: E402
from control_pkg.domain.control.wind_trim import WindTrim            # noqa: E402
from control_pkg.domain.setpoint import Setpoint                     # noqa: E402
from control_pkg.domain.state import DroneState                      # noqa: E402

DT = 0.05
results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def st(vx=0.0, vy=0.0, x=0.0, y=0.0, att_yaw=0.0, t=100.05):
    return DroneState(now_sim=t, vins_x=x, vins_y=y, vins_yaw=0.0, att_yaw=att_yaw,
                      vins_vx=vx, vins_vy=vy, vins_valid=True)


# 1. математика
w = WindTrim(imax=120.0)
w.set_channel(0.0, 30.0, -10.0)
check("канал под курсом 0 читается как записан", w.channel(0.0) == (30.0, -10.0))
p, r = w.channel(math.pi / 2)
check("тот же вектор под курсом +90°: pitch_off ← старый roll, roll_off ← −старый pitch",
      abs(p - (-10.0)) < 1e-9 and abs(r - (-30.0)) < 1e-9)
w.set_channel_axis(0.0, "roll", 55.0)
check("по-осевая запись не трогает другую компоненту", w.channel(0.0) == (30.0, 55.0))
w.set_channel(0.0, 500.0, 0.0)
check("кламп imax 120", w.channel(0.0)[0] == 120.0)
w.reset()
check("reset: ноль и не выучен", w.magnitude() == 0.0 and not w.learned)

# 2. StationFrame с общим ветром: пре-osign ↔ канал через sign, reset рамы ветер не трогает
w = WindTrim(150.0)
fr = StationFrame(wind=w)
fr.psi = 0.3
fr.set_trim_body("pitch", 40.0, sign=-1.0)          # ось с osign −1 пишет −40 в канал
check("set_trim_body(sign −1): в канале −40", abs(w.channel(0.3)[0] - (-40.0)) < 1e-9)
check("trim_body(sign −1) читает обратно +40", abs(fr.trim_body("pitch", -1.0) - 40.0) < 1e-9)
fr.reset()
check("reset рамы не трогает общий ветер", abs(w.channel(0.3)[0] - (-40.0)) < 1e-9)
fr_old = StationFrame()
fr_old.set_trim_body("pitch", 40.0)
check("без ветра — старое хранилище в раме (бит в бит)", abs(fr_old.trim_body("pitch") - 40.0) < 1e-9)

# 3. DpVins на общем триме
def make_vins(w):
    vh = DpVins(kp_fwd=40.0, kp_lat=32.0, ki=8.0, ki_trim=60.0, imax=120.0, max_pwm=150.0,
                cmd_gain=4.0, pos_kp=0.3, pos_vmax=0.3, pos_acc=0.15, vsmooth=0.0, i_latch=True)
    vh.wind = w
    vh.enter(DroneState(now_sim=100.0))
    return vh
w = WindTrim(120.0)
vh = make_vins(w)
check("посев — no-op на общем триме", vh.seed_trim(-30.0, 10.0, st()) is False and w.magnitude() == 0.0)
t = 100.05
for i in range(20):                                   # снос вперёд 0.5 м/с, стики центр, гвоздя нет → ki_trim
    vh.update(st(vx=0.5, x=0.5 * DT * i, t=t), Setpoint(), DT); t += DT
p0, r0 = w.channel(0.0)
check(f"DpVins учит трим В ОБЩИЙ объект (pitch_off {p0:+.1f} ≠ 0)", abs(p0) > 5.0 and abs(r0) < 1e-6)
check("trim_pwm DpVins = канал общего трима", vh.trim_pwm() == w.channel(0.0))
vh.reset_trim()
check("reset_trim — no-op на общем триме (ветер физический)", abs(w.channel(0.0)[0] - p0) < 1e-9)
# проекция по att_yaw: борт развернулся на 90° — в теле тот же ветер стал боковым
pf, pr = vh.trim_pwm(yaw=math.pi / 2)
check("под курсом 90° трим в канале крена, тангаж ~0", abs(pf) < 1e-6 and abs(abs(pr) - abs(p0)) < 1e-9)
w.learned = True
check("«выучен» — общий флаг (armed DpVins = wind.learned)", vh._trim_armed)

# 4. обмен между ярусами: демпфер записал → DpVins видит; DpVins выучил → станция без захвата
w = WindTrim(150.0)
fr = StationFrame(wind=w); fr.psi = 0.0
fr.set_trim_body("pitch", 25.0, sign=1.0)            # демпфер (osign +1) выучил 25 PWM «назад»
vh = make_vins(w)
check("DpVins читает трим демпфера тем же каналом", vh.trim_pwm(yaw=0.0)[0] == 25.0)
w.learned = True
sk = StationKeeper(kp=0.3, brake=3.0)
class _Damper:                                        # минимум демпфера для enter-логики
    pass
check("StationKeeper.reset взводит trim_armed (захват) — без ветра", (sk.reset(), sk.trim_armed)[1])
sk.trim_armed = not w.learned                         # то, что делает _FlowDamper1D.enter при wind.learned
check("ветер выучен другим ярусом → станция без фазы захвата", not sk.trim_armed)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ WIND TRIM OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
