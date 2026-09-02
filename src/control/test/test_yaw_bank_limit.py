#!/usr/bin/env python3
"""Оффлайн-тест YawBankLimit (потолок крена виража, чистый python).

Проверяет закон |ω| ≤ g·tan(φ_max)/v и приоритет источников скорости:
- на висении/без хода кап не режет (полный темп прямой передачи);
- на 5 м/с при φ_max=8° кап = 15.8 °/с = 31 PWM (полный стик 130 → 31);
- знак команды сохраняется; малая команда под капом не тронута;
- источник скорости по доступности: IPM → свежий VINS → gt; ни одного →
  капа нет (деградация к прежнему поведению);
- декоратор прозрачен: enter делегируется, чужие атрибуты видны через inner;
- в композиции с DpYawHold: прямая передача капится, отпущенный стик у
  демпфера на висении не трогается.

Запуск:  python3 src/control/test/test_yaw_bank_limit.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.domain.control.bank_limit import YawBankLimit           # noqa: E402
from control_pkg.domain.control.flow_axes import DpYawHold               # noqa: E402
from control_pkg.domain.rc import RC_CENTER, RcCommand                   # noqa: E402
from control_pkg.domain.setpoint import Setpoint                         # noqa: E402
from control_pkg.domain.state import DroneState                          # noqa: E402

results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


class FixedYaw:
    """Мини-стаб: отдаёт фиксированный yaw-офсет (изолируем закон капа)."""
    axes = frozenset({"yaw"})
    name = 'fixed'

    def __init__(self, off):
        self.off = off
        self.enters = 0

    def enter(self, s):
        self.enters += 1

    def update(self, s, sp, dt):
        rc = RcCommand(throttle=RC_CENTER)
        rc.yaw = RC_CENTER + self.off
        return rc


DT = 0.05
SP = Setpoint(0.0, 0.0, 1.0)


def state(**kw):
    d = dict(now_sim=100.0)
    d.update(kw)
    return DroneState(**d)


# --- 1. закон капа: v=5 м/с, φ_max=8° → ω_max=15.8°/с → 31 PWM ---
# g·tan(8°)/5 = 0.2757 рад/с; /(202.5/400 °/с на PWM) = 31.2 → int 31
bl = YawBankLimit(FixedYaw(130), bank_max_deg=8.0)
rc = bl.update(state(ipm_ok=True, ipm_vfwd=5.0, ipm_vlat=0.0), SP, DT)
check("v=5, φ=8°: полный стик 130 → кап 31 PWM", rc.yaw == RC_CENTER + 31)
rc = bl.update(state(ipm_ok=True, ipm_vfwd=3.0, ipm_vlat=4.0), SP, DT)
check("v=hypot(3,4)=5: тот же кап (модуль скорости)", rc.yaw == RC_CENTER + 31)

# --- 2. знак сохраняется; команда под капом не тронута ---
bl = YawBankLimit(FixedYaw(-130), bank_max_deg=8.0)
rc = bl.update(state(ipm_ok=True, ipm_vfwd=5.0), SP, DT)
check("знак: −130 → −31", rc.yaw == RC_CENTER - 31)
bl = YawBankLimit(FixedYaw(20), bank_max_deg=8.0)
rc = bl.update(state(ipm_ok=True, ipm_vfwd=5.0), SP, DT)
check("малая команда 20 < капа 31 — не тронута", rc.yaw == RC_CENTER + 20)

# --- 3. висение/малый ход: капа нет (v ≤ v_floor или кап шире команды) ---
bl = YawBankLimit(FixedYaw(130), bank_max_deg=8.0)
rc = bl.update(state(ipm_ok=True, ipm_vfwd=0.05), SP, DT)
check("висение (v=0.05 ≤ floor): полный темп", rc.yaw == RC_CENTER + 130)
rc = bl.update(state(ipm_ok=True, ipm_vfwd=1.0), SP, DT)
check("v=1: кап 156 PWM шире команды — не режет", rc.yaw == RC_CENTER + 130)

# --- 4. масштаб с φ_max: 20° на 5 м/с → g·tan20°/5 = 40.9°/с = 80 PWM ---
bl = YawBankLimit(FixedYaw(130), bank_max_deg=20.0)
rc = bl.update(state(ipm_ok=True, ipm_vfwd=5.0), SP, DT)
check("φ=20°, v=5: кап 80 PWM", rc.yaw == RC_CENTER + 80)

# --- 5. приоритет источников: IPM → свежий VINS → gt → нет капа ---
bl = YawBankLimit(FixedYaw(130), bank_max_deg=8.0)
s = state(ipm_ok=True, ipm_vfwd=5.0, vins_valid=True, vins_vx=1.0,
          vins_last_sim=100.0, gt_valid=True, gt_vx=0.1)
check("IPM жив: скорость из IPM (кап 31)",
      bl.update(s, SP, DT).yaw == RC_CENTER + 31)
s = state(ipm_ok=False, vins_valid=True, vins_vx=5.0, vins_last_sim=99.5,
          gt_valid=True, gt_vx=0.1)
check("IPM слеп, VINS свеж: скорость из VINS (кап 31)",
      bl.update(s, SP, DT).yaw == RC_CENTER + 31)
s = state(ipm_ok=False, vins_valid=True, vins_vx=5.0, vins_last_sim=90.0,
          gt_valid=True, gt_vx=5.0)
check("VINS протух (10 с > fresh): скорость из gt (кап 31)",
      bl.update(s, SP, DT).yaw == RC_CENTER + 31)
s = state(ipm_ok=False, vins_valid=False, gt_valid=False)
check("ни одного источника: капа нет (полный темп)",
      bl.update(s, SP, DT).yaw == RC_CENTER + 130)

# --- 6. прозрачность декоратора: enter делегируется, атрибуты inner видны ---
inner = FixedYaw(50)
bl = YawBankLimit(inner, bank_max_deg=8.0)
bl.enter(state())
check("enter делегирован inner", inner.enters == 1)
check("чужой атрибут (name) виден через inner", bl.name == 'fixed')
check("оси декоратора = yaw", bl.axes == frozenset({"yaw"}))

# --- 7. с настоящим DpYawHold: прямая передача капится, демпфер на месте ---
dp = DpYawHold(pilot_gain=130.0, leak_sec=0.0, arm_frames=0)
bl = YawBankLimit(dp, bank_max_deg=8.0)
bl.enter(state())
rc = bl.update(state(ipm_ok=True, ipm_vfwd=5.0), Setpoint(0.0, 0.0, 1.0), DT)
check("DpYawHold прямая передача: 130 → кап 31", rc.yaw == RC_CENTER + 31)
rc = bl.update(state(ipm_ok=True, ipm_vfwd=0.0), Setpoint(0.0, 0.0, 0.0), DT)
check("стик отпущен на висении: демпфер молчит, центр", rc.yaw == RC_CENTER)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ YAW BANK LIMIT OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
