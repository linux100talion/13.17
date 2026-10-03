#!/usr/bin/env python3
"""Юнит-тест camera_mount: бортовой yaml VINS → поза Gazebo / наклон / FLOW_R. Чистый python.

Опоры — значения, которыми сим жил до 2026-10-01 и которые ПОДТВЕРЖДЕНЫ полётами:
наклонная камера (sim.yaml: R с наклоном 0.26, t 0.15/0/0.05) обязана дать ровно
SDF-позу `0.15 0 0.05 0 0.26 0` и лётный FLOW_R демпфера. Плюс текущий бортовой
config.yaml (горизонтальная камера), подмена матриц для sim.yaml и отказ на
повороте по курсу.

Запуск:  python3 src/control/test/test_camera_mount.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_pkg.perception.camera_mount import (CameraMount, read_matrix,   # noqa: E402
                                                 replace_matrix, cam_cfg_path)

results = []


def check(name, ok):
    results.append((name, ok))
    print(('OK   ' if ok else 'FAIL ') + name)


def close(a, b, tol=1e-4):
    return all(abs(x - y) < tol for x, y in zip(a, b))


# 1. наклонная камера сима до 2026-10-01 (sim.yaml + model.sdf + FLOW_R bootstrap_node)
OLD_R = [[0, -0.25708, 0.96639], [-1, 0, 0], [0, -0.96639, -0.25708]]
OLD_FLOW_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]
old = CameraMount(OLD_R, [0.15, 0.0, 0.05], 'old')
check('старая камера: link rpy = (0, 0.26, 0)', close(old.link_rpy, (0, 0.26, 0)))
check('старая камера: tilt = 0.26', abs(old.tilt - 0.26) < 1e-4)
check('старая камера: FLOW_R = лётный', close(old.flow_R, OLD_FLOW_R))
check('старая камера: SDF-поза', old.sdf_pose().split()[:3] == ['0.15', '0', '0.05'])

# 2. горизонтальная камера
hor = CameraMount([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], [0.14, 0, -0.06], 'hor')
check('горизонт: tilt 0', abs(hor.tilt) < 1e-9)
check('горизонт: SDF-поза без «-0»', hor.sdf_pose() == '0.14 0 -0.06 0 0 0')
check('горизонт: FLOW_R', close(hor.flow_R, [0, -1, 0, 0, 0, -1, 1, 0, 0]))

# 3. бортовой config.yaml из репо — он и есть дефолт CAM_CFG
cfg = cam_cfg_path('distro/home/andriy/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/config.yaml')
board = CameraMount.load(cfg)
check('бортовой config.yaml читается', board.source.endswith('config.yaml'))
check('бортовой: t = (0.14, 0, -0.06)', close(board.t, (0.14, 0, -0.06)))

# 4. подмена матриц (sim_nav.launch.py) не трогает остальное и читается обратно
text = open(cfg, encoding='utf-8').read()
new = replace_matrix(text, 'extrinsicRotation', OLD_R)
new = replace_matrix(new, 'extrinsicTranslation', [[0.15], [0.0], [0.05]])
check('replace: R читается обратно', close(sum(read_matrix(new, 'extrinsicRotation'), []),
                                            sum(OLD_R, [])))
check('replace: t читается обратно',
      close([r[0] for r in read_matrix(new, 'extrinsicTranslation')], [0.15, 0, 0.05]))
check('replace: прочее не тронуто',
      new.replace(' ', '').count('max_cnt:') == 1 and len(new.splitlines()) <= len(text.splitlines()))

# 5. отказы: поворот по курсу 10° и не-ортонормированная матрица
c, s = math.cos(math.radians(10)), math.sin(math.radians(10))
Rz = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
R_yaw = [[sum(Rz[i][k] * [[0, 0, 1], [-1, 0, 0], [0, -1, 0]][k][j] for k in range(3))
          for j in range(3)] for i in range(3)]
try:
    CameraMount(R_yaw, [0, 0, 0], 'yaw10')
    check('курс 10° → отказ', False)
except ValueError:
    check('курс 10° → отказ', True)
try:
    CameraMount([[1, 0, 0], [0, 1, 0], [0, 0, 2]], [0, 0, 0], 'bad')
    check('не ортонормирована → отказ', False)
except ValueError:
    check('не ортонормирована → отказ', True)

# 6. интринсики: калибровка под разрешение кадра, идеальная камера сима, отказ на чужих пропорциях
# бортовой конфиг — калибровка Kalibr 2026-10-03 (fx/fy 1124.71/1124.54, cx/cy 630.80/356.42)
check('бортовой: 960×540 → ×0.75 калибровки', close(board.intrinsics_for(960, 540, ideal=False),
                                                   (843.5325, 843.405, 473.1, 267.315)))
check('бортовой: угол обзора Gazebo 2·atan(640/fx) ≈ 59.3°',
      abs(board.sim_hfov - 2 * math.atan(640 / 1124.71)) < 1e-9)
cp = board.camera_params(1280, 720, ideal=False)
check('бортовой: /camera_info — калибровка с дисторсией',
      close([cp[k] for k in ('fx', 'fy', 'cx', 'cy', 'k1', 'k2')], (1124.71, 1124.54, 630.80, 356.42, 0.04214, -0.08968)))
cp = board.camera_params(1280, 720, ideal=True)
check('бортовой: /camera_info сима — фокус конфига, центр, без дисторсии',
      close([cp[k] for k in ('fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2')], (1124.71, 1124.71, 640, 360, 0, 0, 0, 0)))
k = CameraMount(OLD_R, [0, 0, 0], 'k', K=(700, 705, 650, 350, 1280, 720))
check('калибровка 700/705/650/350 @1280 → 960: ×0.75', close(k.intrinsics_for(960, 540, ideal=False),
                                                             (525, 528.75, 487.5, 262.5)))
check('ideal: фокус калибровки, квадратный пиксель, центр посередине',
      close(k.intrinsics_for(960, 540, ideal=True), (525, 525, 480, 270)))
check('угол обзора Gazebo = 2·atan(W/2fx)', abs(k.sim_hfov - 2 * math.atan(1280 / 1400)) < 1e-12)
try:
    k.intrinsics_for(1920, 1200, ideal=False)
    check('чужие пропорции кадра → отказ', False)
except ValueError:
    check('чужие пропорции кадра → отказ', True)

bad = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(bad)}/{len(results)} OK')
sys.exit(1 if bad else 0)
