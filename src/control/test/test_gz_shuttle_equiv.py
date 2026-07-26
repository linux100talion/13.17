#!/usr/bin/env python3
"""Числовая эквивалентность среза 1 (GzPositionHold + Shuttle + ControlStack) с
монолитом src/lab/alt_hold_bootstrap.py (S_EXCITE / gz-hold + боковой/продольный челнок).

ORACLE ниже — дословная копия закона монолита (строки gz-hold в _tick_logic +
_shuttle_offset). Прогоняем обе реализации на одной синтетической траектории поз и
требуем ПОБИТОВОГО совпадения roll/pitch (int-truncation одинаков → допускается 0).

Чистый python (stdlib math) — ни ROS, ни Gazebo, ни numpy. Это тот самый выигрыш
«законы тестируются на числах» из architecture.md. Запуск:
    python3 src/control/test/test_gz_shuttle_equiv.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.application.control_stack import ControlStack       # noqa: E402
from control_pkg.domain.control.excitation import NoExcitation       # noqa: E402
from control_pkg.domain.control.stabilization import GzPositionHold  # noqa: E402
from control_pkg.domain.control.trajectory import Shuttle            # noqa: E402
from control_pkg.domain.state import DroneState                      # noqa: E402

RC_CENTER = 1500


# ---------------------------------------------------------------------------
# ORACLE — точная копия закона монолита (gz-hold + shuttle), без ROS.
# Держит своё состояние интегратора (ix, iy, it), как self.gz_* в ноде.
# ---------------------------------------------------------------------------
class MonolithOracle:
    def __init__(self, kp, kd, ki, imax, gz_max, psign, rsign,
                 sh_a, sh_v, sh_pause, sh_fwd):
        self.kp, self.kd, self.ki, self.imax, self.gz_max = kp, kd, ki, imax, gz_max
        self.psign, self.rsign = psign, rsign
        self.sh_a, self.sh_v, self.sh_pause, self.sh_fwd = sh_a, sh_v, sh_pause, sh_fwd
        self.hold_sp = None
        self.hold_yaw0 = 0.0
        self.hold_t0 = None
        self.gz_ix = self.gz_iy = 0.0
        self.gz_it = None

    def _shuttle_offset(self, now):
        V = max(0.1, self.sh_v)
        A = self.sh_a
        P = self.sh_pause
        tleg = A / V
        ts = now - (self.hold_t0 or now)
        if ts < tleg:
            return -V * ts
        if ts < tleg + P:
            return -A
        if ts < 2.0 * tleg + P:
            return -A + V * (ts - (tleg + P))
        return 0.0

    def step(self, gt_x, gt_y, gt_yaw, gt_vx, gt_vy, now):
        if self.hold_sp is None:
            self.hold_sp = (gt_x, gt_y)
            self.hold_yaw0 = gt_yaw
            self.hold_t0 = now
            self.gz_ix = self.gz_iy = 0.0
            self.gz_it = now
        spx, spy = self.hold_sp
        d = self._shuttle_offset(now)
        if self.sh_fwd:
            fx, fy = math.cos(self.hold_yaw0), math.sin(self.hold_yaw0)
            spx += -d * fx
            spy += -d * fy
        else:
            rx, ry = math.sin(self.hold_yaw0), -math.cos(self.hold_yaw0)
            spx += d * rx
            spy += d * ry
        ex = gt_x - spx
        ey = gt_y - spy
        if self.ki > 0 and self.gz_it is not None and now > self.gz_it:
            dt = now - self.gz_it
            self.gz_ix += ex * dt
            self.gz_iy += ey * dt
            cap = self.imax / self.ki
            self.gz_ix = max(-cap, min(cap, self.gz_ix))
            self.gz_iy = max(-cap, min(cap, self.gz_iy))
        self.gz_it = now
        c = math.cos(gt_yaw)
        s = math.sin(gt_yaw)
        e_fwd = ex * c + ey * s
        e_rgt = -ex * s + ey * c
        v_fwd = gt_vx * c + gt_vy * s
        v_rgt = -gt_vx * s + gt_vy * c
        i_fwd = self.gz_ix * c + self.gz_iy * s
        i_rgt = -self.gz_ix * s + self.gz_iy * c
        mx = self.gz_max
        po = self.psign * (self.kp * e_fwd + self.kd * v_fwd + self.ki * i_fwd)
        ro = self.rsign * (self.kp * e_rgt + self.kd * v_rgt + self.ki * i_rgt)
        po = max(-mx, min(mx, po))
        ro = max(-mx, min(mx, ro))
        return RC_CENTER + int(ro), RC_CENTER + int(po)   # (roll, pitch)


def synthetic_track(n=400, dt=0.05):
    """Правдоподобная траектория: дрон дрейфует+качается вокруг старта, курс медленно
    вращается. Ненулевые ex/ey/v/yaw → задействует ВСЕ члены PID и оба проекц. фрейма."""
    t0 = 10.0   # старт не в нуле sim-времени (проверяем базирование t0)
    for k in range(n):
        now = t0 + k * dt
        tau = k * dt
        gt_x = 0.30 * math.sin(0.7 * tau) + 0.02 * tau
        gt_y = 0.25 * math.cos(0.5 * tau) - 0.015 * tau
        gt_yaw = 0.4 * math.sin(0.2 * tau)
        gt_vx = 0.30 * 0.7 * math.cos(0.7 * tau) + 0.02
        gt_vy = -0.25 * 0.5 * math.sin(0.5 * tau) - 0.015
        yield gt_x, gt_y, gt_yaw, gt_vx, gt_vy, now


def run_case(name, gains, shuttle):
    kp, kd, ki, imax, gz_max, psign, rsign = gains
    sh_a, sh_v, sh_pause, sh_fwd = shuttle
    oracle = MonolithOracle(kp, kd, ki, imax, gz_max, psign, rsign,
                            sh_a, sh_v, sh_pause, sh_fwd)
    stack = ControlStack(
        GzPositionHold(kp=kp, kd=kd, ki=ki, imax=imax, max_pwm=gz_max,
                       psign=psign, rsign=rsign),
        Shuttle(amplitude=sh_a, velocity=sh_v, pause=sh_pause, forward=sh_fwd),
        NoExcitation(),
    )
    max_dr = max_dp = 0
    first = True
    for gt_x, gt_y, gt_yaw, gt_vx, gt_vy, now in synthetic_track():
        o_roll, o_pitch = oracle.step(gt_x, gt_y, gt_yaw, gt_vx, gt_vy, now)
        s = DroneState(gt_valid=True, gt_x=gt_x, gt_y=gt_y, gt_yaw=gt_yaw,
                       gt_vx=gt_vx, gt_vy=gt_vy, now_sim=now)
        if first:
            stack.enter(s)   # монолит захватывает origin на 1-м тике — эмулируем явным enter
            first = False
        rc = stack.update(s)
        max_dr = max(max_dr, abs(rc.roll - o_roll))
        max_dp = max(max_dp, abs(rc.pitch - o_pitch))
    ok = (max_dr == 0 and max_dp == 0)
    print(f"  [{'OK ' if ok else 'FAIL'}] {name:22s} Δroll_max={max_dr} Δpitch_max={max_dp}")
    return ok


def main():
    print("Числовая эквивалентность gz-hold + shuttle (новый ControlStack vs монолит):")
    gains_default = (40.0, 120.0, 8.0, 100.0, 150.0, 1.0, 1.0)
    gains_noI = (40.0, 120.0, 0.0, 100.0, 150.0, 1.0, 1.0)
    gains_signs = (55.0, 90.0, 12.0, 80.0, 150.0, -1.0, 1.0)
    cases = [
        ("боковой челнок",     gains_default, (5.0, 1.5, 2.0, False)),
        ("продольный челнок",  gains_default, (5.0, 1.5, 2.0, True)),
        ("без I-члена",        gains_noI,     (4.0, 1.0, 3.0, False)),
        ("иные знаки/гейны",   gains_signs,   (3.0, 2.0, 1.5, True)),
        ("shuttle A=0 (холд)", gains_default, (0.0, 1.5, 2.0, False)),
    ]
    all_ok = all(run_case(n, g, s) for n, g, s in cases)
    print("ИТОГ:", "✅ ЭКВИВАЛЕНТНО" if all_ok else "❌ РАСХОЖДЕНИЕ")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
