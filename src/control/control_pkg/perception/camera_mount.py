"""Положение камеры на корпусе — из ОДНОГО файла: бортового конфига VINS.

Источник правды — `extrinsicRotation`/`extrinsicTranslation` бортового
`distro/home/andriy/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/config.yaml`
(конвенция VINS: imu^R_cam, СТОЛБЦЫ R — оси камеры X вправо / Y вниз / Z вперёд,
выраженные в теле FLU; t — положение камеры в теле, м). Из него ВЫЧИСЛЯЮТСЯ:

  * поза `camera_link` дрона в Gazebo  — `sdf_pose()`   (sim_up.sh, model.sdf);
  * экстринсики VINS в симе            — `R`, `t`        (sim_nav.launch.py, sim.yaml);
  * поворот и наклон канала вида сверху — `flow_R`, `tilt` (bootstrap_node, демпфер).

Ручек «наклон», «сдвиг» нет нарочно — второй источник правды. Другая камера =
другой yaml: env `CAM_CFG` (путь; относительный — от корня репо, см. `cam_cfg_path`).

Модель канала вида сверху (perception/ipm.py, keyframe.py) знает только наклон
камеры ВНИЗ вокруг поперечной оси. Поворот по курсу/крену больше `MAX_SKEW_DEG`
— громкий отказ, а не тихое игнорирование.

Только stdlib: модуль зовётся и в контейнере simulator (там нет numpy/cv2) —
`python3 camera_mount.py <yaml> --sdf-pose`.
"""
import math
import os
import re
import sys

# Путь в бортовом контейнере (distro/home/andriy/vins_ws/src → /root/vins_ws/src).
BOARD_CFG = '/root/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/config.yaml'
# Он же от корня репо — дефолт CAM_CFG в docker/sim/env.default и compose.
REPO_CFG = 'distro/home/andriy/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/config.yaml'
MAX_SKEW_DEG = 2.0

# camera_link Gazebo → оптическая рама: камера смотрит вдоль +X link'а,
# вправо в кадре = −Y link, вниз в кадре = −Z link. Строки — оси оптики в link.
_R_OPT_LINK = ((0.0, -1.0, 0.0),
               (0.0, 0.0, -1.0),
               (1.0, 0.0, 0.0))


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def _transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def read_matrix(text, name):
    """Значения `name: !!opencv-matrix` из текста OpenCV-YAML (rows×cols, построчно)."""
    m = re.search(r'^' + re.escape(name) + r'\s*:\s*!!opencv-matrix(.*?)data\s*:\s*\[(.*?)\]',
                  text, re.M | re.S)
    if not m:
        raise ValueError(f'нет матрицы {name}')
    hdr = m.group(1)
    rows = int(re.search(r'rows\s*:\s*(\d+)', hdr).group(1))
    cols = int(re.search(r'cols\s*:\s*(\d+)', hdr).group(1))
    vals = [float(v) for v in m.group(2).replace('\n', ' ').split(',') if v.strip()]
    if len(vals) != rows * cols:
        raise ValueError(f'{name}: {len(vals)} чисел при {rows}×{cols}')
    return [vals[i * cols:(i + 1) * cols] for i in range(rows)]


def replace_matrix(text, name, rows):
    """Текст с подменёнными data матрицы `name` (rows — список строк)."""
    flat = ', '.join(f'{v:.9g}' for r in rows for v in r)
    pat = re.compile(r'(^' + re.escape(name) + r'\s*:\s*!!opencv-matrix.*?data\s*:\s*\[)(.*?)(\])',
                     re.M | re.S)
    out, n = pat.subn(lambda m: m.group(1) + flat + m.group(3), text, count=1)
    if n != 1:
        raise ValueError(f'нет матрицы {name}')
    return out


def _repo_root():
    env = os.environ.get('REPO_ROOT')
    if env:
        return env
    d = os.path.dirname(os.path.abspath(__file__))
    while d != os.path.dirname(d):
        if os.path.isdir(os.path.join(d, 'distro')) and os.path.isdir(os.path.join(d, 'src')):
            return d
        d = os.path.dirname(d)
    return None


def cam_cfg_path(path=None):
    """Какой yaml читать: аргумент > env CAM_CFG > бортовой путь (если он есть) >
    REPO_CFG. Относительный путь — от корня репо (env REPO_ROOT, в сим-контейнерах
    /root/repo; на хосте — найденный вверх от этого файла)."""
    p = path or os.environ.get('CAM_CFG')
    if not p:
        if os.path.isfile(BOARD_CFG):
            return BOARD_CFG
        p = REPO_CFG
    if not os.path.isabs(p):
        root = _repo_root()
        if root is None:
            raise SystemExit(f'camera_mount: относительный CAM_CFG={p!r}, а корня репо '
                             'не видно (REPO_ROOT не задан)')
        p = os.path.join(root, p)
    return p


class CameraMount:
    """R (imu^R_cam), t (м) и всё, что из них выводится."""

    def __init__(self, R, t, source='?'):
        self.R = tuple(tuple(float(v) for v in r) for r in R)
        self.t = tuple(float(v) for v in t)
        self.source = source
        for i in range(3):
            for j in range(3):
                d = sum(self.R[k][i] * self.R[k][j] for k in range(3)) - (i == j)
                if abs(d) > 1e-3:
                    raise ValueError(f'{source}: extrinsicRotation не ортонормирована')
        roll, pitch, yaw = self.link_rpy
        skew = math.degrees(max(abs(roll), abs(yaw)))
        if skew > MAX_SKEW_DEG:
            raise ValueError(
                f'{source}: камера повёрнута по крену/курсу на {skew:.1f}° '
                f'(roll {math.degrees(roll):.1f}°, yaw {math.degrees(yaw):.1f}°) — модель '
                f'канала вида сверху знает только наклон вниз (порог {MAX_SKEW_DEG}°)')

    @classmethod
    def load(cls, path=None):
        p = cam_cfg_path(path)
        with open(p, encoding='utf-8') as fh:
            text = fh.read()
        R = read_matrix(text, 'extrinsicRotation')
        t = [r[0] for r in read_matrix(text, 'extrinsicTranslation')]
        return cls(R, t, p)

    @property
    def flow_R(self):
        """Тело→камера (строки — оси камеры в теле), плоско 9 чисел — R_cam_imu демпфера."""
        return [v for r in _transpose(self.R) for v in r]

    @property
    def tilt(self):
        """Наклон оптической оси ВНИЗ от горизонта корпуса, рад (cam_tilt канала)."""
        zx, zz = self.R[0][2], self.R[2][2]
        return math.atan2(-zz, zx)

    @property
    def link_rpy(self):
        """Поза camera_link в теле: (roll, pitch, yaw) Gazebo/ROS (Rz·Ry·Rx), рад."""
        M = _matmul(self.R, _R_OPT_LINK)
        pitch = math.asin(max(-1.0, min(1.0, -M[2][0])))
        roll = math.atan2(M[2][1], M[2][2])
        yaw = math.atan2(M[1][0], M[0][0])
        return roll, pitch, yaw

    def sdf_pose(self):
        """Строка `<pose>` camera_link: «x y z roll pitch yaw»."""
        return ' '.join(f'{v + 0.0:.6g}' for v in (*self.t, *self.link_rpy))  # +0.0: без «-0»

    def summary(self):
        r, p, y = (math.degrees(v) + 0.0 for v in self.link_rpy)
        return (f'камера {self.source}: t=({self.t[0]:+.3f}, {self.t[1]:+.3f}, '
                f'{self.t[2]:+.3f}) м, наклон вниз {math.degrees(self.tilt) + 0.0:.2f}° '
                f'(link rpy {r:.2f}/{p:.2f}/{y:.2f}°)')


def main(argv):
    import argparse
    ap = argparse.ArgumentParser(description='положение камеры из бортового yaml VINS')
    ap.add_argument('yaml', nargs='?', help='путь (по умолчанию env CAM_CFG / бортовой)')
    ap.add_argument('--sdf-pose', action='store_true', help='напечатать <pose> camera_link')
    a = ap.parse_args(argv)
    try:
        cm = CameraMount.load(a.yaml)
    except (OSError, ValueError) as e:
        print(f'camera_mount: ОШИБКА: {e}', file=sys.stderr)
        return 1
    print(cm.sdf_pose() if a.sdf_pose else cm.summary())
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
