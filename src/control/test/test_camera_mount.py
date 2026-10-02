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

bad = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(bad)}/{len(results)} OK')
sys.exit(1 if bad else 0)
