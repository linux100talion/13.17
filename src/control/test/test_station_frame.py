#!/usr/bin/env python3
"""Юнит-тест СТАНЦИИ В ОСЯХ КУРСА (StationFrame): гвоздь, мировая позиция и вектор
трима ветра общие для крена и тангажа и повёрнуты курсом; курс — подключаемый вход.

Зачем. Полёт lv2_joy_20260829_153405 (5 м/с): разворот 200° за 4 с — стиков крена/
тангажа нет, цели станции 0, а борт разгоняется с 0.06 до 1.38 м/с. Станция и трим
жили в осях БОРТА: трим ветра (−50 PWM в тангаже) после разворота смотрит в обратную
сторону и толкает ПО ветру (2×50 PWM ≈ 1 м/с за секунду), гвоздь сбрасывался каждые
17° курса, точка терялась. Здесь оси крена и тангажа — те же _FlowDamper1D с теми же
законами (стенд test_station_brake), но через общую раму: ошибка оси = компонента
(гвоздь − позиция) вдоль оси ТЕКУЩЕГО курса, И-член оси = компонента мирового вектора
трима.

Плант — 2D-расширение стенда двух законов: мировая позиция/скорость, курс ψ(t) задан
профилем разворота, ветер — вектор в мировых осях (0.65 м/с² = 52 PWM как 5 м/с),
привод по осям τ_a = 0.2 с, канал видит скорости тела через апериодику τ_s = 0.3 с,
путь IPM = интеграл измеренных скоростей тела (как ipm_fwd/ipm_lat). Без фантома
дерота (он — отдельная тема, ipm_wz_tau).
Что проверяем:
1. разворот на месте 180° в ветер: в осях борта борт уезжает от точки (>1 м), в осях
   курса — стоит (<0.4 м); трим в мировых осях не меняется, в осях борта сам меняет
   знак;
2. регресс без разворота: толчок вперёд и отпускание — рама ведёт себя как оси борта
   (стоп, возврат к точке стопа);
3. толчок + разворот 90° на ходу: линия крена перезахватывается (17° — только пока
   стик жив), после отпускания обе оси у точки стопа, к старому гвоздю не тянет;
4. курс — подключаемый вход: рама с курсом «всегда 0» вырождается в оси борта
   (уезжает так же), рама по умолчанию читает att_yaw;
5. мировая позиция рамы сходится к истине планта (путь = интеграл канала).
Запуск:  python3 src/control/test/test_station_frame.py
"""
import math
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control_pkg.domain.control.stabilization import (                  # noqa: E402
    DpHold, DpPitchRate, DpRollRate, StationFrame)
from control_pkg.domain.rc import RC_CENTER, RcCommand                # noqa: E402
from control_pkg.domain.setpoint import Setpoint                      # noqa: E402
from control_pkg.domain.state import DroneState                       # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


ALPHA, DT = 0.0125, 1.0 / 30.0
FLIGHT = dict(kp=90.0, ki=30.0, ki_trim=60.0, kd=0.0, imax=150.0, cmd_gain=5.0,
              pos_kp=0.3, pos_vmax=0.3, pos_brake=3.0, pos_brake_vmax=1.0, pos_acc=0.15,
              anti_windup=True, pos_brake_v=0.25, max_speed=0.0, alt_band=0.0,
              arm_frames=0)                # дефолт freefly_lv.sh 2026-08-29


class _YawStub:
    axes = frozenset({"yaw"})

    def enter(self, s): pass

    def update(self, s, sp, dt): return RcCommand(throttle=RC_CENTER)


def stack(frame):
    return DpHold(DpRollRate(**FLIGHT), DpPitchRate(**FLIGHT), _YawStub(), frame=frame)


def fly(st, sec=30.0, psi=lambda t: 0.0, wind=(0.65, 0.0), stick_f=None, stick_r=None,
        tau_s=0.3, tau_a=0.2, gain=1.0):
    """Строки: (t, X, Y, ψ, v_f, v_l, pwm_p, pwm_r, I_p, I_r, pin_p, pin_r)."""
    X = Y = Vx = Vy = 0.0
    vm_f = vm_l = 0.0
    path_f = path_l = 0.0
    act_p = act_r = 0.0
    st.enter(DroneState(flow_seq=-1))
    rows, t = [], 0.0
    ax_r, ax_p = st._subs[0], st._subs[1]
    for k in range(int(round(sec / DT))):
        t += DT
        p = psi(t)
        c, si = math.cos(p), math.sin(p)
        v_f = Vx * c + Vy * si
        v_l = -Vx * si + Vy * c
        a = 1.0 - math.exp(-DT / tau_s)
        vm_f += (gain * v_f - vm_f) * a
        vm_l += (gain * v_l - vm_l) * a
        path_f += vm_f * DT
        path_l += vm_l * DT
        sp = Setpoint()
        if stick_f is not None:
            sp.c_fwd = stick_f(t)
        if stick_r is not None:
            sp.c_right = stick_r(t)
        s = DroneState(flow_seq=k + 1, now_sim=t, flow_dt=DT, rel_alt=0.3, ipm_ok=True,
                       flow_conf=0.5, ipm_vlat=vm_l, ipm_lat=path_l, ipm_vfwd=vm_f,
                       ipm_fwd=path_f, att_yaw=p)
        rc = st.update(s, sp, DT)
        pwm_p, pwm_r = rc.pitch - RC_CENTER, rc.roll - RC_CENTER
        b = 1.0 - math.exp(-DT / tau_a)
        act_p += (pwm_p - act_p) * b
        act_r += (pwm_r - act_r) * b
        a_f, a_l = -ALPHA * act_p, -ALPHA * act_r          # тело: вперёд, влево
        Vx += (a_f * c - a_l * si + wind[0]) * DT
        Vy += (a_f * si + a_l * c + wind[1]) * DT
        X += Vx * DT
        Y += Vy * DT
        rows.append((t, X, Y, p, v_f, v_l, pwm_p, pwm_r, ax_p._i, ax_r._i,
                     ax_p._pos_sp is not None, ax_r._pos_sp is not None))
    return rows


def dist(rows, t0, t1, ref):
    return max(math.hypot(r[1] - ref[0], r[2] - ref[1]) for r in rows if t0 <= r[0] <= t1)


def pos_at(rows, t):
    r = next(r for r in rows if r[0] >= t)
    return (r[1], r[2])


def speed_max(rows, t0, t1):
    return max(math.hypot(r[4], r[5]) for r in rows if t0 <= r[0] <= t1)


# --- 1. разворот на месте 180° в ветер ---
print("  разворот на месте 180° в ветер 5 м/с (10–13 с), без стиков:")
turn = lambda t: 0.0 if t < 10.0 else (math.pi * min(1.0, (t - 10.0) / 3.0))
body = fly(stack(None), psi=turn)
fr = StationFrame()
yawf = fly(stack(fr), psi=turn)
pin0 = pos_at(body, 9.9)
d_body = dist(body, 10.0, 30.0, pin0)
pin0f = pos_at(yawf, 9.9)
d_yaw = dist(yawf, 10.0, 30.0, pin0f)
check(f"оси борта: после разворота уезжает от точки ({d_body:.2f} м > 1.0)", d_body > 1.0)
check(f"оси курса: стоит на точке ({d_yaw:.2f} м < 0.4)", d_yaw < 0.4)
check(f"оси курса: пик скорости за разворот {speed_max(yawf, 10.0, 30.0):.2f} < 0.4 м/с "
      f"(борт {speed_max(body, 10.0, 30.0):.2f})", speed_max(yawf, 10.0, 30.0) < 0.4)
i_p_before = next(r[8] for r in yawf if r[0] >= 9.9)
i_p_after = yawf[-1][8]
check(f"оси курса: трим тангажа сменил знак сам ({i_p_before:+.0f} → {i_p_after:+.0f} PWM, "
      f"трим ветра {0.65 / ALPHA:.0f})", abs(i_p_before - 52.0) < 8.0 and abs(i_p_after + 52.0) < 8.0)
tw = fr.trim
check(f"оси курса: вектор трима в мире ≈ (52, 0): ({tw[0]:.0f}, {tw[1]:.0f})",
      abs(tw[0] - 52.0) < 8.0 and abs(tw[1]) < 8.0)
check("оси курса: гвоздь не сброшен разворотом (обе оси в станции всё время после 10 с)",
      all(r[10] and r[11] for r in yawf if r[0] > 10.0))

# --- 2. регресс без разворота: толчок вперёд ---
print("  толчок вперёд 5–8 с без разворота (регресс):")
push = lambda t: 0.5 if 5.0 < t <= 8.0 else 0.0
b2 = fly(stack(None), psi=lambda t: 0.0, stick_f=push)
y2 = fly(stack(StationFrame()), psi=lambda t: 0.0, stick_f=push)


def stop_t(rows, t_rel):
    return next((r[0] - t_rel for r in rows if r[0] > t_rel and r[4] <= 0.0), float('nan'))


def pin_t(rows, t_rel):
    return next((r[0] for r in rows if r[0] > t_rel and r[10]), float('nan'))


zb, zy = stop_t(b2, 8.0), stop_t(y2, 8.0)
pb, py = pin_t(b2, 8.0), pin_t(y2, 8.0)
eb = math.hypot(b2[-1][1] - pos_at(b2, pb)[0], b2[-1][2] - pos_at(b2, pb)[1])
ey = math.hypot(y2[-1][1] - pos_at(y2, py)[0], y2[-1][2] - pos_at(y2, py)[1])
check(f"регресс: стоп {zy:.1f} ≈ {zb:.1f} с, гвоздь {py:.1f} ≈ {pb:.1f} с",
      abs(zy - zb) < 0.3 and abs(py - pb) < 0.3)
check(f"регресс: к 30 с у точки стопа (рама {ey:.2f}, борт {eb:.2f} < 0.15 м)",
      ey < 0.15 and eb < 0.15)
check(f"регресс: конечные позиции совпадают (Δ {math.hypot(y2[-1][1] - b2[-1][1], y2[-1][2] - b2[-1][2]):.2f} < 0.2 м)",
      math.hypot(y2[-1][1] - b2[-1][1], y2[-1][2] - b2[-1][2]) < 0.2)

# --- 3. толчок + разворот 90° на ходу ---
print("  толчок вперёд 5–9 с, разворот 90° на 6–8 с:")
turn90 = lambda t: 0.0 if t < 6.0 else (0.5 * math.pi * min(1.0, (t - 6.0) / 2.0))
push9 = lambda t: 0.5 if 5.0 < t <= 9.0 else 0.0
y3 = fly(stack(StationFrame()), psi=turn90, stick_f=push9)
b3 = fly(stack(None), psi=turn90, stick_f=push9)
p3 = pin_t(y3, 9.0)
stop3 = pos_at(y3, p3)
e3 = math.hypot(y3[-1][1] - stop3[0], y3[-1][2] - stop3[1])
# боковая скорость ТЕЛА на развороте с ходом 2.5 м/с — это инерция (вектор скорости
# мира проецируется на новую ось), а не команда станции: сравниваем с осями борта
lat_y = max(abs(r[5]) for r in y3 if 6.0 <= r[0] <= 9.0)
lat_b = max(abs(r[5]) for r in b3 if 6.0 <= r[0] <= 9.0)
roll_y = max(abs(r[7]) for r in y3 if 6.0 <= r[0] <= 9.0)
roll_b = max(abs(r[7]) for r in b3 if 6.0 <= r[0] <= 9.0)
check(f"толчок+разворот: после отпускания обе оси у точки стопа ({e3:.2f} < 0.3 м), "
      f"к старому гвоздю не тянет (стоп в {math.hypot(*stop3):.1f} м от него)",
      e3 < 0.3 and math.hypot(*stop3) > 1.0)
check(f"толчок+разворот: линия крена перезахватывается — рама не хуже осей борта "
      f"(бок. скорость {lat_y:.2f} ≈ {lat_b:.2f} м/с, PWM крена {roll_y:.0f} ≈ {roll_b:.0f})",
      lat_y <= lat_b + 0.3 and roll_y <= roll_b + 20)
check("толчок+разворот: обе оси в станции к 30 с", y3[-1][10] and y3[-1][11])

# --- 4. курс — подключаемый вход ---
print("  курс — подключаемый вход:")
fz = StationFrame(heading=lambda s: 0.0)
z4 = fly(stack(fz), psi=turn)
d_z = dist(z4, 10.0, 30.0, pos_at(z4, 9.9))
check(f"рама с курсом «всегда 0» вырождается в оси борта: уезжает ({d_z:.2f} м > 1.0)", d_z > 1.0)
check(f"рама по умолчанию читает att_yaw: ψ рамы {math.degrees(fr.psi):.0f}° = 180°",
      abs(fr.psi - math.pi) < 1e-6)

# --- 5. мировая позиция рамы против истины планта ---
# Рама интегрирует ИЗМЕРЕННУЮ скорость тела (лаг τ_s) под ТЕКУЩИМ курсом: на ходу с
# разворотом вектор скорости канала отстаёт от курса на ω·τ_s (45°/с × 0.3 с = 13°) —
# ошибка пути ~v·τ_s·Δψ ≈ 1 м на 8 м. Станции это не мешает: гвоздь берётся заново на
# каждом стопе, а в висении (v ~ 0.1 м/с) ошибка разворота — сантиметры. Резерв: курс
# с задержкой канала (ψ(t − τ_s)) при известном лаге.
f5 = StationFrame()
y5 = fly(stack(f5), psi=lambda t: 0.0, stick_f=push)
e5 = math.hypot(f5.x - y5[-1][1], f5.y - y5[-1][2])
check(f"позиция рамы = истина после толчка без разворота (Δ {e5:.2f} < 0.15 м, "
      f"путь {math.hypot(*y5[-1][1:3]):.1f} м)", e5 < 0.15)
e5b = math.hypot(fr.x - yawf[-1][1], fr.y - yawf[-1][2])
check(f"позиция рамы = истина после разворота на месте в ветер (Δ {e5b:.2f} < 0.1 м)", e5b < 0.1)
f5c = StationFrame()
y5c = fly(stack(f5c), psi=turn90, stick_f=push9)
e5c = math.hypot(f5c.x - y5c[-1][1], f5c.y - y5c[-1][2])
check(f"позиция рамы после хода 2.5 м/с с разворотом 90°: лаг канала даёт Δ {e5c:.2f} м "
      f"(< 1.5; гвоздь перевязывается на стопе)", e5c < 1.5)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ СТАНЦИЯ В ОСЯХ КУРСА OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
