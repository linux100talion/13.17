#!/usr/bin/env python3
"""strip_bags — выбросить из bag'ов кадры, оставить телеметрию.

Зачем. Прогон пишет `/image_color` (652 кадра ≈ 970 МБ) плюс телеметрию (~2500
сообщений ≈ единицы МБ). Кадры нужны один раз — собрать mp4 и залить на Drive;
после этого 95% объёма bag'а лежит мёртвым грузом, а разбор (`back_check.py`,
`calib_check.py`, `series_check.py`) читает только телеметрию. Двадцать пять
прогонов = 36 ГБ и диск под ноль.

Что делает: для каждого bag'а пишет рядом `<bag>_light` только с топиками KEEP,
сверяет, что число сообщений по каждому совпало, и только ПОСЛЕ этого сносит
оригинал и переименовывает лёгкий на его место. Если что-то не сошлось —
оригинал остаётся нетронутым, лёгкий удаляется.

⚠️ Необратимо для кадров. Видео прогонов лежит на Drive (`13.17/calib/<NAME>.mp4`),
покадровая выборка — в `output/scene_img`; из bag'а кадры больше не достать.

Запуск ВНУТРИ nav (нужен rosbag2_py; /root/sim_ws/output смонтирован на запись):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash; \
    python3 /lab/strip_bags.py /root/sim_ws/output/A1_pitch_sign_bag …'
Без аргументов берёт ВСЕ *_bag в SB_DIR (кроме SB_SKIP).

Env: SB_DIR (/root/sim_ws/output), SB_SKIP (имена через запятую — не трогать),
     SB_DRY (1 = только показать, что было бы сделано).
"""
import os
import shutil
import sys

from rosbag2_py import (ConverterOptions, SequentialReader, SequentialWriter,
                        StorageOptions, TopicMetadata)

# Телеметрия, на которой стоит весь разбор. Всё остальное (кадры) — за борт.
KEEP = {
    '/flow_dbg', '/flow_dbg2', '/flow_dbg3', '/flow_dbg4', '/flow_dbg5',
    '/mavros/global_position/rel_alt', '/gz_imu/data_flu', '/mavros/imu/data_raw',
    '/model/iris_cam/odometry',
    '/mavros/imu/data', '/mavros/imu/data_raw',
    '/mavros/local_position/pose', '/mavros/global_position/rel_alt',
    '/vins_estimator/odometry', '/clock',
}
DIR = os.environ.get('SB_DIR', '/root/sim_ws/output')
SKIP = {s for s in os.environ.get('SB_SKIP', '').split(',') if s}
DRY = os.environ.get('SB_DRY', '0') == '1'


def strip(bag):
    """→ (освобождено байт, сообщение). Оригинал сносится только при сверке 1:1."""
    name = os.path.basename(bag)
    size0 = sum(os.path.getsize(os.path.join(bag, f)) for f in os.listdir(bag))
    light = bag + '_light'
    if os.path.exists(light):
        shutil.rmtree(light)

    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    topics = [t for t in r.get_all_topics_and_types() if t.name in KEEP]
    if not topics:
        return 0, f'{name}: нет ни одного топика телеметрии — ПРОПУСК'
    want = {t.name for t in topics}
    if DRY:
        return 0, f'{name}: {size0/2**20:.0f} МБ → оставил бы {sorted(want)}'

    w = SequentialWriter()
    w.open(StorageOptions(uri=light, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    for t in topics:
        w.create_topic(TopicMetadata(name=t.name, type=t.type,
                                     serialization_format='cdr'))
    n_src, n_dst = {}, {}
    while r.has_next():
        topic, raw, ts = r.read_next()
        n_src[topic] = n_src.get(topic, 0) + 1
        if topic in want:
            w.write(topic, raw, ts)
            n_dst[topic] = n_dst.get(topic, 0) + 1
    del w, r

    bad = [t for t in want if n_src.get(t, 0) != n_dst.get(t, 0)]
    if bad:
        shutil.rmtree(light)
        return 0, f'{name}: РАСХОЖДЕНИЕ по {bad} — оригинал не тронут'

    size1 = sum(os.path.getsize(os.path.join(light, f)) for f in os.listdir(light))
    shutil.rmtree(bag)
    os.rename(light, bag)
    kept = sum(n_dst.values())
    return size0 - size1, (f'{name}: {size0/2**20:.0f} → {size1/2**20:.1f} МБ, '
                           f'телеметрия {kept} сообщений цела')


bags = sys.argv[1:] or sorted(
    os.path.join(DIR, d) for d in os.listdir(DIR)
    if d.endswith('_bag') and d not in SKIP and os.path.isdir(os.path.join(DIR, d)))
# крупные первыми — место освобождается раньше
bags.sort(key=lambda b: -sum(os.path.getsize(os.path.join(b, f)) for f in os.listdir(b)))

freed = 0
for b in bags:
    try:
        got, msg = strip(b)
    except Exception as e:                       # один битый bag не должен рвать проход
        got, msg = 0, f'{os.path.basename(b)}: ОШИБКА {e}'
    freed += got
    print(msg, flush=True)
print(f'\nосвобождено {freed/2**30:.1f} ГБ по {len(bags)} bag(ам)')
