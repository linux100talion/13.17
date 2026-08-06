#!/usr/bin/env python3
"""СИНТЕТИКА C — канал курса не должен принимать СНОС за разворот.

Пиннит правку из `ToDo5.md`. Замер `yaw_fidelity.py` показал: пока борт стоит (соседние
оси на оракулах), канал курса видит +0.96 ± 0.12 истинного разворота, а на трёх осях `Dp`,
где борт идёт 1-4 м/с, — −0.09 ± 0.02. Причина в допущении «в дальней сцене трансляция
≈0», записанном прямо над расчётом `yaw_flow`: медиану горизонтального потока заливает
снос, контур нулит снос вместо курса, и курс уходит на 23…360° за висение.

Правка разносит их геометрией: для наклонённой вниз камеры поток от ТРАНСЛЯЦИИ ∝ 1/Z ∝
(y − y_гор), от ВРАЩЕНИЯ — почти не зависит от строки, поэтому подгонка u(y)=a+b·(y−y_гор)
отдаёт вращение свободным членом.

Прошлая синтетика (`flow_synth_test.py`) этого поймать НЕ МОГЛА: там кадры сдвигаются
равномерно, а равномерный сдвиг — это и есть вращение. Нужна ГЛУБИНА, поэтому здесь кадр
переносится гомографией наземной плоскости: H = K (I + T·nᵀ/d) K⁻¹.

Три проверки:
  A. чистый разворот           → и старый, и новый закон видят его (правка не сломала);
  B. чистый боковой снос       → СТАРЫЙ видит ложный разворот, НОВЫЙ ≈ 0;
  C. разворот + снос вместе    → НОВЫЙ ближе к истине, чем СТАРЫЙ.

Запуск (нужны только cv2+numpy):
  docker run --rm -v /root/13.17/src:/src:ro sim-nav:latest \
    bash -lc 'python3 /src/lab/yaw_trans_test.py'
"""
import math
import os
import sys

import cv2
import numpy as np

for _p in (os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'control', 'control_pkg', 'perception'),
           '/root/sim_ws/src/control/control_pkg/perception',
           '/control/control_pkg/perception'):
    if os.path.isfile(os.path.join(_p, 'flow_estimator.py')):
        sys.path.insert(0, _p)
        break
else:                                    # noqa: PLW0120
    sys.exit('боевой flow_estimator.py не найден — проверять нечего')
import flow_estimator as _fe             # noqa: E402
from flow_estimator import FlowEstimator  # noqa: E402
print(f'проверяем оценщик: {_fe.__file__}')

W, H = 1280, 720
FX = FY = 640.0
CX, CY = 640.0, 360.0
R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
DT = 1.0 / 30.0
TILT = 0.26          # наклон камеры вниз, рад (как в боевом конфиге)
ALT = 3.0            # высота, м
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
KI = np.linalg.inv(K)

_FAILS = []


def _check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}  {detail}")
    if not cond:
        _FAILS.append(name)


def base_frame(seed=1317):
    rng = np.random.default_rng(seed)
    return cv2.GaussianBlur(rng.integers(0, 255, size=(H, W), dtype=np.uint8), (0, 0), 1.5)


def ground_H(tx, tz=0.0):
    """Гомография кадра при переносе камеры на (tx, 0, tz) над наземной плоскостью.

    Оси камеры: x вправо, y вниз, z вперёд. Направление «вниз мира» в камере при наклоне
    вниз на TILT: n = (0, cos TILT, sin TILT); плоскость земли: n·P = ALT.
    Точки в камере при переносе камеры на T едут на −T, значит H = K (I + T·nᵀ/d) K⁻¹."""
    n = np.array([0.0, math.cos(TILT), math.sin(TILT)])
    T = np.array([tx, 0.0, tz])
    return K @ (np.eye(3) + np.outer(T, n) / ALT) @ KI


def yaw_H(dpsi):
    """Гомография чистого разворота борта на dpsi (рад): ω_cam = R·(0,0,ω_z)."""
    w_cam = np.asarray(R, dtype=np.float64).reshape(3, 3) @ np.array([0.0, 0.0, dpsi])
    th = np.linalg.norm(w_cam)
    if th < 1e-12:
        return np.eye(3)
    k = w_cam / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    Rot = np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * (Kx @ Kx)
    return K @ Rot.T @ KI          # поворот камеры → картинка едет обратно


def run(Hm, omega_imu, fix):
    est = FlowEstimator(FX, FY, CX, CY, R, rotflow_sign=1.0, cam_tilt=TILT,
                        yaw_trans_fix=fix)
    img = base_frame()
    warped = cv2.warpPerspective(img, Hm, (W, H), borderMode=cv2.BORDER_REFLECT)
    est.process(img, 1.0, omega_imu, pitch=0.0, alt=ALT)
    r = est.process(warped, 1.0 + DT, omega_imu, pitch=0.0, alt=ALT)
    return r['yaw_flow'] if r else float('nan')


print("СИНТЕТИКА C — снос против разворота в канале курса")

# --- A: ЧИСТЫЙ РАЗВОРОТ ------------------------------------------------------
# 20 °/с вправо. Гиро курса контуру НЕ отдаём (ось на то и визуальная), поэтому
# omega_imu = 0: derotation по roll/pitch ничего не снимет, разворот останется в потоке.
wz = math.radians(20.0)
a_old = run(yaw_H(wz * DT), np.zeros(3), fix=False)
a_new = run(yaw_H(wz * DT), np.zeros(3), fix=True)
print(f"     A разворот 20°/с: старый {a_old:+.3f} px/кадр | новый {a_new:+.3f}")
_check("A: старый закон видит разворот", abs(a_old) > 3.0, f"|{a_old:+.3f}|>3")
_check("A: новый закон видит его же (правка не сломала)",
       abs(a_new - a_old) < 0.35 * abs(a_old), f"{a_new:+.3f} против {a_old:+.3f}")

# --- B: ЧИСТЫЙ БОКОВОЙ СНОС --------------------------------------------------
# 2 м/с вбок — типичная скорость ухода в E2. Разворота нет вовсе, значит честный
# канал курса обязан показать ноль.
vx = 2.0
b_old = run(ground_H(vx * DT), np.zeros(3), fix=False)
b_new = run(ground_H(vx * DT), np.zeros(3), fix=True)
print(f"     B снос 2 м/с, разворота НЕТ: старый {b_old:+.3f} px/кадр | новый {b_new:+.3f}")
_check("B: старый закон принимает снос за разворот", abs(b_old) > 1.0, f"|{b_old:+.3f}|>1")
_check("B: новый закон сноса не видит", abs(b_new) < 0.3 * abs(b_old),
       f"|{b_new:+.3f}| < 0.3·|{b_old:+.3f}|")

# --- C: РАЗВОРОТ + СНОС ВМЕСТЕ ----------------------------------------------
Hc = ground_H(vx * DT) @ yaw_H(wz * DT)
c_old = run(Hc, np.zeros(3), fix=False)
c_new = run(Hc, np.zeros(3), fix=True)
print(f"     C разворот+снос: истина ≈{a_old:+.3f} | старый {c_old:+.3f} | новый {c_new:+.3f}")
_check("C: новый закон ближе к истине, чем старый",
       abs(c_new - a_old) < abs(c_old - a_old),
       f"ошибка {abs(c_new - a_old):.3f} против {abs(c_old - a_old):.3f}")

print()
if _FAILS:
    print(f"РЕЗУЛЬТАТ: ПРОВАЛ ({len(_FAILS)}): {', '.join(_FAILS)}")
    sys.exit(1)
print("РЕЗУЛЬТАТ: все проверки пройдены ✓  (канал курса больше не принимает снос за разворот)")
