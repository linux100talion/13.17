#!/usr/bin/env python3
"""Юнит-тест оценки нуля ω_z деротации канала вида сверху (ipm.py _wz_debias). Чистый python.

КАПКАН (полёт lv2_joy_20260906_195742): гейт сравнивал ω_z с ТЕКУЩЕЙ оценкой нуля — медленно
растущее вращение (круги 11–16°/с) оценка тащила за собой (2 → 16°/с), а после остановки
|ω_z − оценка| > гейта замораживал её НАВСЕГДА: деротация вычитала фантом X·0.27 рад/с ≈ 1.5–3
м/с, боковой канал читал ~0 при истинном сносе 1.5–2.6 м/с, демпфер гнал борт с этой скоростью,
трим намотался до 150 PWM. Проверяет новое правило: гейт по АБСОЛЮТНОЙ |ω_z| + кап |оценка| ≤
ipm_wz_bias_max: на развороте оценка стоит, на висении отпускает за ~τ, кап держит физику
смещения гироскопа; ipm_wz_bias_max=0 — без капа.

Запуск:  python3 src/control/test/test_ipm_wz_bias.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control_pkg.perception.flow_estimator import FlowEstimator     # noqa: E402

FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
results = []


def check(name, ok):
    results.append((name, ok))
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")


def est(gate=0.1, clamp=0.05, tau=2.0):
    return FlowEstimator(480.0, 480.0, 480.0, 270.0, FLOW_R, ipm_model='exact', ipm_derot=1.0,
                         ipm_wz_tau=tau, ipm_wz_gate=gate, ipm_wz_bias_max=clamp)


def run(e, profile, dt=0.033):
    """profile: список (длительность с, ω_z рад/с) — прогон через _wz_debias, вернуть след оценки."""
    t = 100.0; trace = []
    for dur, wz in profile:
        n = int(dur / dt)
        for _ in range(n):
            t += dt
            e._wz_debias(t, wz)
            trace.append((t, wz, e._wz_bias))
    return trace


# 1. сценарий полёта 195742: разгон вращения 0 → 0.25 рад/с за 10 с, круги 60 с, стоп 20 с
ramp = [(1.0, 0.025 * k) for k in range(1, 11)]
prof = [(5.0, 0.0)] + ramp + [(60.0, 0.25), (20.0, 0.0)]
e = est()
tr = run(e, prof)
peak = max(abs(b) for _, _, b in tr)
after = [(t, b) for t, w, b in tr if w == 0.0 and t > tr[0][0] + 5.0 + 10.0 + 60.0]
b5 = next(abs(b) for t, b in after if t >= after[0][0] + 5.0)
check(f"круги 0.25 рад/с 60 с: оценка нуля не превысила капа 0.05 (пик {math.degrees(peak):.1f}°/с)", peak <= 0.05 + 1e-9)
check(f"через 5 с после остановки оценка < 0.01 рад/с ({math.degrees(b5):.2f}°/с) — капкана нет", b5 < 0.01)
# 2. гейт по абсолютной |ω_z|: во время разворота 0.25 оценка не движется
e2 = est(); run(e2, [(5.0, 0.0)]); b_before = e2._wz_bias
run(e2, [(30.0, 0.25)])
check("на развороте 0.25 рад/с (> гейт 0.1) оценка не движется", abs(e2._wz_bias - b_before) < 1e-9)
# 3. настоящее смещение гироскопа 0.02 рад/с — оценка сходится к нему (τ 2 с), фантома нет
e3 = est(); run(e3, [(2.0, 0.0)]); tr3 = run(e3, [(20.0, 0.02)])
check(f"смещение гироскопа 0.02: оценка сошлась ({e3._wz_bias:.4f})", abs(e3._wz_bias - 0.02) < 0.002)
check("после схождения дебиас ≈ 0", abs(e3._wz_debias(tr3[-1][0] + 0.033, 0.02)) < 0.003)
# 4. медленный «разворот» 0.09 (< гейта): без капа оценка уползает к 0.09, с капом — не выше 0.05
e4 = est(clamp=0.0); run(e4, [(2.0, 0.0), (30.0, 0.09)])
e5 = est(clamp=0.05); run(e5, [(2.0, 0.0), (30.0, 0.09)])
check(f"ipm_wz_bias_max=0: медленный разворот 0.09 уходит в оценку ({e4._wz_bias:.3f}) — как раньше",
      abs(e4._wz_bias - 0.09) < 0.005)
check(f"ipm_wz_bias_max=0.05: оценка капнута ({e5._wz_bias:.3f} ≤ 0.05)", e5._wz_bias <= 0.05 + 1e-9)
# 5. первый отсчёт на развороте — тоже под капом
e6 = est(); e6._wz_debias(100.0, 0.3)
check("старт оценки с первого отсчёта на развороте — капнут до 0.05", abs(e6._wz_bias) <= 0.05 + 1e-9)

ok_all = all(ok for _, ok in results)
print("ИТОГ:", "✅ IPM WZ BIAS OK" if ok_all else "❌ СБОЙ")
sys.exit(0 if ok_all else 1)
