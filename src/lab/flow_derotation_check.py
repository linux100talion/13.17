#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flow_derotation_check — ОФФЛАЙН-валидация знака/направления derotation по rosbag.

НЕ нужен сим-стек (Gazebo/SITL/VINS) — только записанный bag с /image_mono +
/gz_imu/data_flu (такие уже есть с прогонов todo3/todo4, IMU через TOPICS_EXTRA).
Запускать В КОНТЕЙНЕРЕ nav (нужны rosbag2_py + cv2), напр.:
  python3 src/lab/flow_derotation_check.py docker/sim/output/<bag_dir>

Идея (FAQ_vins.md 6-11, спека): при ЧИСТОМ вращении (большое |ω|) правильная
derotation ЗАНУЛЯЕТ трансляционный остаток; неверный знак/направление — УДВАИВАЕТ.
Гоняем bag через FlowEstimator с вариантами {R, Rᵀ}×{знак ±} + baseline(без derotation)
и берём вариант с МИНИМАЛЬНЫМ остатком на кадрах с большим |ω|. Так данные сами
выбирают верную derotation — снимаем TODO[sign] без оракула estimate_extrinsic.

Вывод: ранжировка вариантов по среднему resid_rms (px/кадр) на вращательных кадрах.
Победитель должен быть ЗАМЕТНО ниже baseline. Если нет — модель/extrinsic не сводятся
перестановкой знака (тогда копать формулу rot-flow или extrinsic глубже).
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flow_estimator import FlowEstimator  # noqa: E402

# дефолты intrinsics + extrinsicRotation из sim.yaml
DEF_FX, DEF_FY, DEF_CX, DEF_CY = 640.0, 640.0, 640.0, 360.0
DEF_R = [0.0, -1.0, 0.0, -0.25708, 0.0, -0.96639, 0.96639, 0.0, -0.25708]


def read_bag(bag_dir, image_topic, imu_topic):
    """→ (images: [(stamp, gray)], imus: [(stamp, [wx,wy,wz])]). Требует rosbag2_py."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, Imu

    import cv2

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    images, imus = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == image_topic:
            m = deserialize_message(data, Image)
            buf = np.frombuffer(m.data, dtype=np.uint8)
            # принимаем mono8 и color (bgr8/rgb8 → gray): bag обычно пишет /image_color.
            if m.encoding in ('mono8', '8UC1'):
                gray = buf.reshape(m.height, m.width)
            elif m.encoding in ('bgr8', 'rgb8'):
                img = buf.reshape(m.height, m.width, 3)
                code = cv2.COLOR_BGR2GRAY if m.encoding == 'bgr8' else cv2.COLOR_RGB2GRAY
                gray = cv2.cvtColor(img, code)
            else:
                continue
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            images.append((stamp, gray))
        elif topic == imu_topic:
            m = deserialize_message(data, Imu)
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            imus.append((stamp, np.array([m.angular_velocity.x,
                                          m.angular_velocity.y,
                                          m.angular_velocity.z])))
    images.sort(key=lambda x: x[0])
    imus.sort(key=lambda x: x[0])
    if not images:
        raise RuntimeError(f'нет кадров (mono8/bgr8/rgb8) в {image_topic} '
                           f'(топики bag: {types})')
    if not imus:
        raise RuntimeError(f'нет IMU в {imu_topic}')
    return images, imus


def nearest_omega(imus, stamps, t):
    """ω, ближайший по времени к t (бинарный поиск)."""
    i = np.searchsorted(stamps, t)
    i = min(max(i, 0), len(imus) - 1)
    if i > 0 and abs(imus[i - 1][0] - t) < abs(imus[i][0] - t):
        i -= 1
    return imus[i][1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='путь к rosbag2-каталогу')
    ap.add_argument('--image-topic', default='/image_mono')
    ap.add_argument('--imu-topic', default='/gz_imu/data_flu')
    ap.add_argument('--omega-thresh', type=float, default=0.3,
                    help='|ω| (rad/s) выше которого кадр считается «вращательным»')
    ap.add_argument('--omega-max', type=float, default=float('inf'),
                    help='ВЕРХНЯЯ граница |ω| (rad/s): берём только полосу '
                         'omega_thresh<|ω|≤omega_max. Отсекает слишком быстрые кадры, '
                         'где LK ломается (поток-мусор маскирует derotation). default=inf')
    ap.add_argument('--fx', type=float, default=DEF_FX)
    ap.add_argument('--fy', type=float, default=DEF_FY)
    ap.add_argument('--cx', type=float, default=DEF_CX)
    ap.add_argument('--cy', type=float, default=DEF_CY)
    a = ap.parse_args()

    images, imus = read_bag(a.bag, a.image_topic, a.imu_topic)
    imu_stamps = np.array([s for s, _ in imus])
    print(f'кадров mono8: {len(images)} | IMU-сэмплов: {len(imus)}')

    R = np.array(DEF_R, dtype=np.float64).reshape(3, 3)
    # варианты: baseline (без derotation) + {R, Rᵀ} × {±1}
    variants = {
        'no-derot (baseline)': (R, 0.0),
        'R,  +': (R, +1.0),
        'R,  -': (R, -1.0),
        'R^T, +': (R.T, +1.0),
        'R^T, -': (R.T, -1.0),
    }
    ests = {k: FlowEstimator(a.fx, a.fy, a.cx, a.cy, Rv, sign) for k, (Rv, sign) in variants.items()}
    acc = {k: [] for k in variants}   # resid_rms на вращательных кадрах
    n_rot = 0

    for stamp, gray in images:
        w = nearest_omega(imus, imu_stamps, stamp)
        wn = np.linalg.norm(w)
        rot_frame = a.omega_thresh < wn <= a.omega_max
        for k, est in ests.items():
            res = est.process(gray, stamp, w)
            if res is not None and rot_frame:
                acc[k].append(res['resid_rms'])
        if rot_frame:
            n_rot += 1

    band = f'{a.omega_thresh}<|ω|≤{a.omega_max}' if a.omega_max != float('inf') else f'|ω|>{a.omega_thresh}'
    print(f'\nвращательных кадров ({band}): {n_rot}\n')
    if n_rot < 5:
        print('⚠️  мало вращательных кадров — нужен bag с yaw/раскачкой (excite). '
              'Результат ненадёжен.')
    rows = [(k, float(np.mean(v)) if v else float('nan')) for k, v in acc.items()]
    base = dict(rows).get('no-derot (baseline)', float('nan'))
    rows.sort(key=lambda r: (np.isnan(r[1]), r[1]))
    print(f'{"вариант":>22}  | средний resid_rms (px/кадр) | vs baseline')
    print('-' * 66)
    for k, v in rows:
        ratio = (v / base) if base and not np.isnan(base) and not np.isnan(v) else float('nan')
        mark = '  ← ПОБЕДИТЕЛЬ' if k == rows[0][0] and 'baseline' not in k else ''
        print(f'{k:>22}  | {v:>26.3f} | {ratio:>6.2f}x{mark}')

    winner = next((k for k, _ in rows if 'baseline' not in k), None)
    wval = dict(rows).get(winner, float('nan'))
    print()
    if winner and not np.isnan(wval) and not np.isnan(base) and wval < 0.7 * base:
        Rv, sign = variants[winner]
        print(f'✅ Верная derotation: {winner} (остаток {wval/base:.2f}× от baseline).')
        print(f'   → в flow_estimator/ноду: R={"R^T" if "T" in winner else "R"}, rotflow_sign={sign:+.0f}')
        print('   R (вставить в extrinsic_rotation):')
        print('   ' + np.array2string(Rv.reshape(-1), precision=5, separator=', '))
    else:
        print('⚠️  Ни один вариант не бьёт baseline заметно — derotation не сводится '
              'перестановкой знака/транспонированием. Копать формулу rot-flow или extrinsic '
              '(или мало вращения в bag). См. оракул estimate_extrinsic (todo4).')


if __name__ == '__main__':
    main()
