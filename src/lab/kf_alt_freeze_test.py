#!/usr/bin/env python3
"""СИНТЕТИКА B — затвор опоры по высоте: болтанка НЕ стирает точку удержания.

Пиннит правку из `ToDo5.md`. Раньше `alt_drift > kf_alt_max` пересевал опору и
ВЫБРАСЫВАЛ накопленное смещение (`trust=False`) — на висении это случалось 31-39 раз за
20 с, то есть дом переезжал дважды в секунду, и контур умел только гасить. Теперь блок
опоры на уходе высоты ЗАМИРАЕТ, а пересев идёт по таймауту `kf_alt_hold`.

Проверяем на СИНТЕТИЧЕСКИХ кадрах (зум = продольное движение) с заданной высотой:
  A. Болтанка ALT_HOLD (±8% с периодом 0.6 с, порог 0.06 пересекается многократно)
     → пересевов НЕТ, накопитель растёт, кадры внутри выброса помечены kf_valid=False.
  B. Настоящий набор (3.0 → 4.5 м монотонно) → пересев ЕСТЬ (высота не вернулась), и
     сегмент при нём ЗАСЧИТАН в накопитель, а не выброшен.
  C. Ровная высота → поведение не изменилось (сегменты закрываются штатно).

Запуск (нужен только cv2+numpy, ROS не нужен):
  REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
  docker run --rm -v $REPO/src:/src:ro sim-nav:latest \
    bash -lc 'python3 /src/lab/kf_alt_freeze_test.py'
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
else:                                    # noqa: PLW0120 — не нашли боевую: это ОШИБКА
    sys.exit('боевой flow_estimator.py не найден — проверять нечего')
import flow_estimator as _fe             # noqa: E402
from flow_estimator import FlowEstimator  # noqa: E402
print(f'проверяем оценщик: {_fe.__file__}')

FX = FY = 640.0
CX, CY = 640.0, 360.0
R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
W, H = 1280, 720
FPS = 30.0
DT = 1.0 / FPS
ALT0 = 3.0

_FAILS = []


def _check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}  {detail}")
    if not cond:
        _FAILS.append(name)


def base_frame(seed=1317):
    rng = np.random.default_rng(seed)
    return cv2.GaussianBlur(rng.integers(0, 255, size=(H, W), dtype=np.uint8), (0, 0), 1.5)


def zoom(img, s):
    """Масштаб вокруг центра кадра = продольное движение (дальность меняется)."""
    M = np.float32([[s, 0, (1 - s) * CX], [0, s, (1 - s) * CY]])
    return cv2.warpAffine(img, M, (W, H), borderMode=cv2.BORDER_REFLECT)


def run(alt_fn, n=360, zoom_per_frame=1.0015):
    """Прогон n кадров: зум растёт равномерно, высота — по alt_fn(кадр). → (est, кадры).

    360 кадров = 12 с при 30 Гц. Шесть секунд было мало: на идеальных кадрах точки
    не теряются вовсе, сегмент закрывается только СТРАХОВКОЙ по возрасту
    (kf_seg_cap_sec=10 с), и за 6 с не закрылся бы ни один."""
    est = FlowEstimator(FX, FY, CX, CY, R, rotflow_sign=1.0, pitch_smooth_n=9)
    img = base_frame()
    frames = []
    s = 1.0
    for i in range(n):
        alt = alt_fn(i)
        res = est.process(zoom(img, s), i * DT, np.zeros(3), pitch=0.0, alt=alt)
        frames.append((alt, res))
        s *= zoom_per_frame
    return est, frames


def summary(est, frames):
    seen = [f for _, f in frames if f is not None]
    invalid = sum(1 for f in seen if not f['kf_valid'])
    return dict(segs=est.kf_segs, reseeds=est.kf_reseeds, acc=est.kf_acc,
                invalid=invalid, n=len(seen))


print("СИНТЕТИКА B — затвор опоры по высоте")

# --- C: ровная высота (контроль: штатное поведение не тронуто) ---------------
est, fr = run(lambda i: ALT0)
c = summary(est, fr)
print(f"     C ровная высота: сегментов {c['segs']}, пересевов {c['reseeds']}, "
      f"acc {c['acc']:+.4f}, недостоверных кадров {c['invalid']}/{c['n']}")
_check("C: на ровной высоте сегменты закрываются штатно", c['segs'] >= 2, f"segs={c['segs']}")
_check("C: на ровной высоте пересевов нет", c['reseeds'] == 0, f"reseeds={c['reseeds']}")
_check("C: на ровной высоте все кадры достоверны", c['invalid'] == 0, f"invalid={c['invalid']}")

# --- A: болтанка ALT_HOLD ----------------------------------------------------
# ±8% с периодом 0.6 с: alt_drift пересекает порог 0.06 туда-обратно ~10 раз за прогон,
# но НИ РАЗУ не держится дольше kf_alt_hold (1.5 с) — пересева быть не должно.
wobble = lambda i: ALT0 * (1.0 + 0.08 * math.sin(2 * math.pi * i * DT / 0.6))
est, fr = run(wobble)
a = summary(est, fr)
print(f"     A болтанка ±8%/0.6с: сегментов {a['segs']}, пересевов {a['reseeds']}, "
      f"acc {a['acc']:+.4f}, недостоверных кадров {a['invalid']}/{a['n']}")
_check("A: болтанка НЕ пересевает опору", a['reseeds'] == 0, f"reseeds={a['reseeds']}")
_check("A: накопитель наполняется несмотря на болтанку",
       a['segs'] >= 2 and abs(a['acc']) > 0.02, f"segs={a['segs']}, acc={a['acc']:+.4f}")
_check("A: кадры внутри выброса помечены недостоверными",
       a['invalid'] > 0, f"invalid={a['invalid']}/{a['n']}")
_check("A: смещение не потеряно против ровной высоты",
       abs(a['acc']) > 0.6 * abs(c['acc']), f"{a['acc']:+.4f} против {c['acc']:+.4f}")

# --- B: настоящий набор ------------------------------------------------------
# Сценарий H6_kd: борт стоит на месте и набирает высоту 1.5 м/с (3.0 → 4.8 м за 1.2 с и
# дальше). Зума нет — движения нет, весь масштаб приходит от высоты, и это ровно тот
# случай, где опора обязана протухнуть. Высота НЕ возвращается → после kf_alt_hold опора
# пересевается, но сегмент ЗАСЧИТЫВАЕТСЯ (последнее достоверное значение), а не теряется.
climb = lambda i: ALT0 + 1.5 * i * DT
est, fr = run(climb, zoom_per_frame=1.0)
b = summary(est, fr)
print(f"     B набор 1.5 м/с без движения: сегментов {b['segs']}, пересевов {b['reseeds']}, "
      f"acc {b['acc']:+.4f}, недостоверных кадров {b['invalid']}/{b['n']}")
_check("B: настоящий набор опору пересевает", b['segs'] >= 2,
       f"segs={b['segs']}, reseeds={b['reseeds']}")
_check("B: сегмент при наборе ЗАСЧИТАН, а не выброшен", b['reseeds'] == 0,
       f"reseeds={b['reseeds']} (должно быть 0: значение достоверно)")
_check("B: на наборе контур молчит", b['invalid'] > 0, f"invalid={b['invalid']}/{b['n']}")

print()
if _FAILS:
    print(f"РЕЗУЛЬТАТ: ПРОВАЛ ({len(_FAILS)}): {', '.join(_FAILS)}")
    sys.exit(1)
print("РЕЗУЛЬТАТ: все проверки пройдены ✓  (болтанка не стирает точку удержания)")
