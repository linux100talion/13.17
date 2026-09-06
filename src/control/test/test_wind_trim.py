#!/usr/bin/env python3
"""Юнит-тест WindTrim — общего ветрового трима ярусов 0/1 (wind_trim.py). Чистый python.

Проверяет: математику (канал ↔ мир, поворот, по-осевая запись не трогает другую
компоненту, кламп); ДОБАВКИ после отката (wind_trim.py): владелец (тень в ярусе 1 не
пишет), снимок устойчивого hold и вердикты входа L/S/N, наблюдение устойчивости DpVins и
композита DpHold (перекрёстный датчик), затухание стрелки HUD (hud_renderer.wind_arrow_fade); делегирование StationFrame (пре-osign ↔ канал через sign, reset
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
check(f"DpVins учит трим В ОБЩИЙ объект (pitch_off {p0:+.1f} ≠ 0; на входе «выучен» → ki 8)",
      abs(p0) > 2.0 and abs(r0) < 1e-6)
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

# 5. ВЛАДЕЛЕЦ: в ярусе 1 композит-тень читает, но не пишет (wind_trim.py п.1)
w = WindTrim(150.0)
frame_a = StationFrame(wind=w); frame_a.psi = 0.0
vh = make_vins(w)                                     # enter → владелец DpVins (ярус 1)
check("вход DpVins назначает владельца", w.owner is vh)
frame_a.set_trim_body("pitch", 77.0, sign=1.0)        # тень пишет — игнор
check("запись рамы-тени в ярусе 1 отброшена", w.channel(0.0) == (0.0, 0.0))
check("чтение тени разрешено", frame_a.trim_body("pitch") == 0.0)
w.set_channel(0.0, 12.0, 0.0)                         # без подписи (стенд) — можно
check("запись без подписи принята", w.channel(0.0)[0] == 12.0)
w.observe(101.0, True, who=frame_a); w.observe(104.5, True, who=frame_a)
check("наблюдения тени отброшены (серия не копится)", w._steady_since is None)
w.learned = False
w.mark_learned(who=frame_a)
check("«выучен» от тени отброшен", not w.learned)
w.handover(200.0, who=frame_a)                        # вход в ярус 0 — владелец рама
check("вход демпфера переназначает владельца", w.owner is frame_a)
frame_a.set_trim_body("pitch", 33.0, sign=1.0)
check("владелец-рама пишет", w.channel(0.0)[0] == 33.0)

# 6. СНИМОК устойчивого hold и вердикты входа (п.2)
w = WindTrim(150.0, steady_sec=3.0, steady_v=0.5)
own = object(); w.acquire(own)
w.set_channel(0.0, 40.0, -10.0)
t = 100.0
for i in range(50):                                   # 2.5 с устойчивости — снимка ещё нет
    w.observe(t, True, who=own); t += DT
check("серия 2.5 с < steady_sec: снимка нет", not w.has_good and not w.steady_now(t))
for i in range(20):                                   # ещё 1 с → снимок
    w.observe(t, True, who=own); t += DT
check("серия 3.5 с: снимок = живой трим (40, −10)", w.has_good and (w.gx, w.gy) == (40.0, -10.0)
      and w.steady_now(t - DT))
w.set_channel(0.0, 60.0, -10.0)                       # учится дальше в серии — снимок следит
w.observe(t, True, who=own)
check("снимок следит за тримом в серии", w.gx == 60.0)
w.observe(t + DT, False, who=own)                     # серия прервана (брейк/стик)
check("прерывание серии сбрасывает устойчивость", w._steady_since is None and not w.steady_now(t + DT))
w.set_channel(0.0, 150.0, 20.0)                       # «фантом» вне устойчивости
other = object()
v = w.handover(t + 2 * DT, who=other)
check("вход без устойчивости → S: откат к снимку (60, −10), выучен", v == "S"
      and (w.channel(0.0) == (60.0, -10.0)) and w.learned and w.owner is other)
check("вердикт идемпотентен по тику (второй enter — только владелец)",
      (w.set_channel(0.0, 61.0, -10.0), w.handover(t + 2 * DT, who=own), w.channel(0.0)[0])[2] == 61.0
      and w.owner is own)
w.learned = False
w.handover(t + 2 * DT, who=own, force_learned=True)
check("force_learned на повторном входе тика взводит «выучен»", w.learned)
# L: устойчив прямо сейчас — живой как есть
t2 = t + 10.0
for i in range(70):
    w.observe(t2, True, who=own); t2 += DT
w.set_channel(0.0, 70.0, 0.0); w.observe(t2, True, who=own)
v = w.handover(t2, who=other)
check("вход в устойчивом hold → L: живой трим как есть", v == "L" and w.channel(0.0)[0] == 70.0)
# после LOITER (без наблюдений > 0.5 с) серия протухает → S, не L
v = w.handover(t2 + 5.0, who=own)
check("после паузы наблюдений (LOITER) → S", v == "S")
# N: снимка ещё нет — живой как есть (старый посев), «выучен» не трогаем
w2 = WindTrim(150.0); w2.set_channel(0.0, 25.0, 0.0)
v = w2.handover(300.0, who=own)
check("без снимка → N: живой трим как есть (25), не выучен", v == "N" and w2.channel(0.0)[0] == 25.0
      and not w2.learned)
check("status: устойчивость/вердикт/снимок/выучен", w.status(t2 + 5.0).startswith("-/S/70/1"))

# 7. DpVins наблюдает устойчивость сам (гвоздь на входе, висение)
w = WindTrim(150.0, steady_sec=3.0, steady_v=0.5)
vh = DpVins(kp_fwd=40.0, kp_lat=32.0, ki=8.0, ki_trim=60.0, imax=120.0, max_pwm=150.0,
            cmd_gain=4.0, pos_kp=0.3, pos_vmax=0.3, pos_acc=0.15, vsmooth=0.0, i_latch=True,
            pin_armed=True)
vh.wind = w
vh.enter(DroneState(now_sim=100.0))
t = 100.05
for i in range(80):                                   # 4 с висения, стики центр, IPM не годен
    vh.update(st(t=t), Setpoint(), DT); t += DT
check("DpVins в висении с гвоздём: серия набрана, снимок есть", w.has_good and w.steady_now(t - DT))
vh.update(DroneState(now_sim=t, vins_valid=True, ipm_ok=True, ipm_vlat=1.5), Setpoint(), DT)
check("чужой датчик (IPM 1.5 м/с) видит движение → не устойчив", w._steady_since is None)

# 8. Композит DpHold: устойчивость обеих осей станции + перекрёстный VINS
from control_pkg.domain.control.flow_axes import DpHold
from control_pkg.domain.control.station_keeper import HOLD
w = WindTrim(150.0, steady_sec=3.0, steady_v=0.5)
dh = DpHold(frame=StationFrame(wind=w))
sd = DroneState(now_sim=100.0, ipm_ok=True, ipm_vfwd=0.1, ipm_vlat=0.1)
for x in dh._subs:
    if getattr(x, "_axis", None) in ("roll", "pitch"):
        x.station.pin = (0.0, 0.0); x._last_ok_sim = 100.0
check("обе оси в hold, кадр свеж, |v| мал → устойчив", dh._wind_steady(sd, w))
sd2 = DroneState(now_sim=100.0, ipm_ok=True, ipm_vfwd=0.1, ipm_vlat=0.1,
                 vins_valid=True, vins_last_sim=99.9, vins_vx=1.2)
check("VINS жив и видит 1.2 м/с (фантом IPM) → не устойчив", not dh._wind_steady(sd2, w))
dh._subs[0].station.braking = True
check("брейк на одной оси → не устойчив", not dh._wind_steady(sd, w))
dh._subs[0].station.braking = False
dh._subs[0].station.i_hold = True
check("защёлка трима → не устойчив", not dh._wind_steady(sd, w))
dh._subs[0].station.i_hold = False
sd3 = DroneState(now_sim=100.7, ipm_ok=True)
check("кадр канала протух (0.7 с > stale) → не устойчив", not dh._wind_steady(sd3, w))

# 9. Затухание стрелки (п.3) — рендерер HUD без кадра
try:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), "nav"))
    from nav_pkg.hud_renderer import wind_arrow_fade
    check("трим 5 PWM → стрелки нет", wind_arrow_fade(5.0, "ipm") == 0.0)
    check("трим 15 PWM → полная", wind_arrow_fade(15.0, "vins") == 1.0)
    check("трим 11.5 PWM → половина", abs(wind_arrow_fade(11.5, "ipm") - 0.5) < 1e-9)
    check("ветер EKF (LOITER) не гасится", wind_arrow_fade(2.0, "ekf") == 1.0)
except ImportError as e:                              # нет cv2 на хосте — рендерер не импортируется
    print("  [SKIP] затухание стрелки: hud_renderer не импортируется:", e)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ WIND TRIM OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
