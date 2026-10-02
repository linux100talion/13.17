"""Положение камеры на корпусе — из ОДНОГО файла: бортового конфига VINS.

Источник правды — `extrinsicRotation`/`extrinsicTranslation` бортового
`distro/home/andriy/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/config.yaml`
(конвенция VINS: imu^R_cam, СТОЛБЦЫ R — оси камеры X вправо / Y вниз / Z вперёд,
выраженные в теле FLU; t — положение камеры в теле, м). Из него ВЫЧИСЛЯЮТСЯ:

  * поза `camera_link` дрона в Gazebo  — `sdf_pose()`   (sim_up.sh, model.sdf);
  * экстринсики VINS в симе            — `R`, `t`        (sim_nav.launch.py, sim.yaml);
  * поворот и наклон канала вида сверху — `flow_R`, `tilt` (bootstrap_node, демпфер);
  * ИНТРИНСИКИ демпфера — `intrinsics_for(w, h)`: fx/fy/cx/cy калибровки (projection_parameters
    при image_width×image_height), пересчитанные под фактический кадр (с 2026-10-02; до того
    демпфер брал идеальную камеру 90° из разрешения). Дисторсию демпфер пока НЕ снимает;
  * угол обзора камеры Gazebo и интринсики VINS сима — `sim_hfov`, `intrinsics_for(…, ideal)`:
    сим рисует идеальную камеру (квадратный пиксель, центр посередине, без дисторсии) с фокусом
    конфига — в симе демпфер берёт ровно её (env CAM_IDEAL=1, compose).

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


def read_scalar(text, name, block=None):
    """Число `name: value` (внутри блока `block:` с отступом, если задан)."""
    if block:
        m = re.search(r'^' + re.escape(block) + r'\s*:\s*\n((?:[ \t]+.*\n?)+)', text, re.M)
        if not m:
            raise ValueError(f'нет блока {block}')
        text = m.group(1)
    m = re.search(r'^\s*' + re.escape(name) + r'\s*:\s*([-+0-9.eE]+)', text, re.M)
    if not m:
        raise ValueError(f'нет {block + "." if block else ""}{name}')
    return float(m.group(1))


def cam_ideal():
    """Камера — идеальная (сим: Gazebo рисует pinhole с центром посередине)? env CAM_IDEAL."""
    return os.environ.get('CAM_IDEAL', '0') not in ('', '0', 'false', 'False')


class CameraMount:
    """R (imu^R_cam), t (м), интринсики калибровки и всё, что из них выводится."""

    def __init__(self, R, t, source='?', K=None):
        self.R = tuple(tuple(float(v) for v in r) for r in R)
        self.t = tuple(float(v) for v in t)
        self.source = source
        # (fx, fy, cx, cy, ширина, высота) калибровки; None — не заданы (тогда идеальная 90°)
        self.K = tuple(float(v) for v in K) if K is not None else None
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
        K = tuple(read_scalar(text, k, 'projection_parameters') for k in ('fx', 'fy', 'cx', 'cy')) \
            + (read_scalar(text, 'image_width'), read_scalar(text, 'image_height'))
        return cls(R, t, p, K)

    def intrinsics_for(self, w, h, ideal=None):
        """(fx, fy, cx, cy) для кадра w×h: калибровка, пересчитанная под разрешение.

        Пропорции кадра обязаны совпадать с калибровкой (другой режим сенсора — обрезка, а не
        масштаб: пересчёт был бы неверен) — иначе ValueError. ideal (None → env CAM_IDEAL):
        камера, которую рисует сим — фокус калибровки, квадратный пиксель, центр посередине."""
        w, h = float(w), float(h)
        if self.K is None:
            return w / 2.0, w / 2.0, w / 2.0, h / 2.0
        fx, fy, cx, cy, W, H = self.K
        if abs(w / h - W / H) > 0.01 * (W / H):
            raise ValueError(f'{self.source}: кадр {w:.0f}×{h:.0f} не в пропорциях калибровки '
                             f'{W:.0f}×{H:.0f} — пересчитать интринсики масштабом нельзя')
        s = w / W
        if ideal is None:
            ideal = cam_ideal()
        if ideal:
            return fx * s, fx * s, w / 2.0, h / 2.0
        return fx * s, fy * s, cx * s, cy * s

    @property
    def sim_hfov(self):
        """Горизонтальный угол обзора, рад, камеры Gazebo с фокусом калибровки."""
        if self.K is None:
            return math.pi / 2.0
        fx, _fy, _cx, _cy, W, _H = self.K
        return 2.0 * math.atan(W / (2.0 * fx))

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
        k = ''
        if self.K is not None:
            fx, fy, cx, cy, W, H = self.K
            k = (f'; fx/fy {fx:g}/{fy:g} cx/cy {cx:g}/{cy:g} @ {W:.0f}×{H:.0f} '
                 f'(обзор {math.degrees(self.sim_hfov):.1f}°)')
        return (f'камера {self.source}: t=({self.t[0]:+.3f}, {self.t[1]:+.3f}, '
                f'{self.t[2]:+.3f}) м, наклон вниз {math.degrees(self.tilt) + 0.0:.2f}° '
                f'(link rpy {r:.2f}/{p:.2f}/{y:.2f}°){k}')


def main(argv):
    import argparse
    ap = argparse.ArgumentParser(description='положение камеры из бортового yaml VINS')
    ap.add_argument('yaml', nargs='?', help='путь (по умолчанию env CAM_CFG / бортовой)')
    ap.add_argument('--sdf-pose', action='store_true', help='напечатать <pose> camera_link')
    ap.add_argument('--sim-hfov', action='store_true', help='напечатать horizontal_fov камеры Gazebo, рад')
    a = ap.parse_args(argv)
    try:
        cm = CameraMount.load(a.yaml)
    except (OSError, ValueError) as e:
        print(f'camera_mount: ОШИБКА: {e}', file=sys.stderr)
        return 1
    if a.sim_hfov:
        print(f'{cm.sim_hfov:.6f}')
    else:
        print(cm.sdf_pose() if a.sdf_pose else cm.summary())
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
