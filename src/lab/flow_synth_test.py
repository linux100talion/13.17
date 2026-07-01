#!/usr/bin/env python3
"""СИНТЕТИКА A — детерминированный юнит-тест геометрии/знака FlowEstimator.

Гоняет FlowEstimator (src/lab/flow_estimator.py) на СИНТЕТИЧЕСКИХ парах кадров с
ИЗВЕСТНЫМ сдвигом/поворотом → проверяет, что `lateral` (боковой прокси сноса) и
derotation дают правильный знак и величину — БЕЗ Gazebo/SITL, детерминированно,
за миллисекунды. Пиннит то, что «Шаг 1» вымучивал на реальных бэгах: rsign=+1 и R.

Запуск ВНУТРИ nav-контейнера (нужен cv2):
  docker exec p1317_nav bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/flow_synth_test.py'

Что проверяем (assert, exit 1 при провале):
  1. Чистая боковая трансляция (сдвиг по X) → lateral ≈ сдвиг, ВЕРНЫЙ ЗНАК.
  2. Чистая продольная-в-кадре трансляция (сдвиг по Y) → lateral ≈ 0 (не путаем оси).
  3. Derotation чистого «рыскания» (равномерный гориз. сдвиг ⇔ поворот камеры вокруг
     оси Y): при rsign=+1 поток вычитается (lateral≈0), при rsign=0 «рыскание» течёт
     как ложный боковой снос (lateral≈сдвиг), при rsign=−1 — удваивается.
     Это грунт-проверка связки R (ω_cam=R·ω_imu) + вычитание + знак rsign.

Ограничение теста 3: warp — равномерный гориз. сдвиг, что физически = поворот вокруг
оси Y камеры в ПРИБЛИЖЕНИИ центра кадра (доминирующий член Longuet-Higgins
u_rot≈−fx·wy·dt). Краевые члены (1+xn²) дают малый разброс → допуск ~десятки %.
Тест проверяет СВЯЗКУ/ЗНАК derotation, а не переисследует формулу _rot_flow.
"""
import sys
import numpy as np
import cv2

sys.path.insert(0, '/lab')
sys.path.insert(0, '.')
from flow_estimator import FlowEstimator  # noqa: E402

# --- константы: зеркало alt_hold_bootstrap.py:59-61 (sim.yaml интринсики + экстринсики)
FX = FY = 640.0
CX, CY = 640.0, 360.0
R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
W, H = 1280, 720
DT = 1.0 / 30.0  # интервал кадра, с

_FAILS = []


def _check(name, cond, detail=""):
    tag = "PASS" if cond else "FAIL"
    print(f"  [{tag}] {name}  {detail}")
    if not cond:
        _FAILS.append(name)


def make_texture(seed=1317):
    """Богатый текстурой кадр: goodFeaturesToTrack найдёт много углов по всему полю."""
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 255, size=(H, W), dtype=np.uint8)
    # сглаживаем, чтобы LK-пирамида трекала (не чистый шум-алиасинг)
    img = cv2.GaussianBlur(img, (0, 0), 1.5)
    return img


def translate(img, dx, dy):
    M = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(img, M, (W, H), borderMode=cv2.BORDER_REFLECT)


def run_pair(est, f0, f1, omega, t0=1.0):
    """Два кадра через FlowEstimator → dict второго (первый лишь детектит фичи)."""
    est.process(f0, t0, omega)          # prev := f0 (возвращает None)
    return est.process(f1, t0 + DT, omega)


def main():
    print("СИНТЕТИКА A — геометрия/знак FlowEstimator (детерминированно)")
    Rm = np.array(R, dtype=np.float64).reshape(3, 3)
    base = make_texture()

    # 1) ЧИСТАЯ БОКОВАЯ ТРАНСЛЯЦИЯ: сдвиг по X на +6 px, ω=0 → lateral ≈ +6
    dx = 6.0
    est = FlowEstimator(FX, FY, CX, CY, R, rotflow_sign=1.0)
    res = run_pair(est, base, translate(base, dx, 0.0), np.zeros(3))
    _check("trans_x: lateral≈сдвиг+знак", res is not None and abs(res['lateral'] - dx) < 1.0,
           f"lateral={res['lateral']:+.2f} (ожид {dx:+.1f}), n={res['n']}" if res else "res=None")

    # 1b) отрицательный сдвиг → отрицательный lateral (знак симметричен)
    est = FlowEstimator(FX, FY, CX, CY, R, rotflow_sign=1.0)
    res = run_pair(est, base, translate(base, -dx, 0.0), np.zeros(3))
    _check("trans_x-: знак отрицательный", res is not None and abs(res['lateral'] + dx) < 1.0,
           f"lateral={res['lateral']:+.2f} (ожид {-dx:+.1f})" if res else "res=None")

    # 2) ЧИСТЫЙ СДВИГ ПО Y (продольная в кадре): lateral(=median flow_x) ≈ 0
    est = FlowEstimator(FX, FY, CX, CY, R, rotflow_sign=1.0)
    res = run_pair(est, base, translate(base, 0.0, 6.0), np.zeros(3))
    _check("trans_y: lateral≈0 (оси не путаются)", res is not None and abs(res['lateral']) < 1.0,
           f"lateral={res['lateral']:+.2f} (ожид 0)" if res else "res=None")

    # 3) DEROTATION «рыскания»: равномерный гориз. сдвиг S ⇔ поворот камеры вокруг Y.
    #    wy = -S/(fx·dt) → u_rot≈-fx·wy·dt=S. omega_imu = R^T·(0,wy,0).
    S = 8.0
    wy = -S / (FX * DT)
    w_cam = np.array([0.0, wy, 0.0])
    omega_imu = Rm.T @ w_cam            # ω_cam = R·ω_imu ⇒ ω_imu = R^T·ω_cam
    warped = translate(base, S, 0.0)

    est = FlowEstimator(FX, FY, CX, CY, R, rotflow_sign=1.0)   # derotation ВКЛ
    r_on = run_pair(est, base, warped, omega_imu)
    est = FlowEstimator(FX, FY, CX, CY, R, rotflow_sign=0.0)   # derotation ВЫКЛ
    r_off = run_pair(est, base, warped, omega_imu)
    est = FlowEstimator(FX, FY, CX, CY, R, rotflow_sign=-1.0)  # неверный знак
    r_neg = run_pair(est, base, warped, omega_imu)

    ok = r_on and r_off and r_neg
    lat_on = r_on['lateral'] if r_on else float('nan')
    lat_off = r_off['lateral'] if r_off else float('nan')
    lat_neg = r_neg['lateral'] if r_neg else float('nan')
    print(f"     derot: rsign=0 lateral={lat_off:+.2f} (ложное рыскание≈{S:+.1f}) | "
          f"rsign=+1 lateral={lat_on:+.2f} (гасится→0) | rsign=-1 lateral={lat_neg:+.2f} (×2)")
    _check("derot rsign=0: рыскание течёт как ложный снос",
           ok and abs(lat_off - S) < 0.3 * S, f"|{lat_off:+.2f}-{S}|")
    _check("derot rsign=+1: рыскание ГАСИТСЯ",
           ok and abs(lat_on) < 0.3 * S, f"|{lat_on:+.2f}|<{0.3*S:.1f}")
    # rsign=-1 → tr = flow-(-1)·rot = flow+rot ≈ +2S: удвоение В ПЛЮС (складывает, не гасит)
    _check("derot rsign=-1: рыскание УДВАИВАЕТСЯ (неверный знак)",
           ok and lat_neg > 1.7 * S, f"{lat_neg:+.2f}>{1.7*S:.1f}")

    print()
    if _FAILS:
        print(f"РЕЗУЛЬТАТ: ПРОВАЛ ({len(_FAILS)}): {', '.join(_FAILS)}")
        return 1
    print("РЕЗУЛЬТАТ: все проверки пройдены ✓  (rsign=+1 и R подтверждены синтетикой)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
