#!/usr/bin/env python3
"""Собрать MP4-видео из rosbag /image_color (ВСЕ кадры, а не выборка по пути).

В отличие от extract_frames.py (диагностические JPEG по пройденному пути), здесь
кодируется ПОЛНЫЙ поток камеры за прогон — «как видела камера дрона». Кадры берём
в порядке записи, FPS видео считаем из sim-штампов сообщений, чтобы длительность
ролика совпадала с длительностью полёта в sim-времени (а не «ускоренно/замедленно»
из-за низкого RTF: bag хранит sim-время, не wall).

Запускается ВНУТРИ nav-контейнера (нужен cv_bridge из /opt/overlay).
Параметры через env:
  SCENE_BAG     путь к rosbag        (default /root/sim_ws/output/scene_bag)
  SCENE_MP4     выходной файл        (default /root/sim_ws/output/scene_img/scene.mp4)
  SCENE_TOPIC   топик изображения     (default /image_color)
  SCENE_FPS     FPS видео; 0 = авто из sim-штампов (default 0)
  SCENE_MAXW    макс. ширина кадра, px; 0 = не масштабировать (default 1280)
"""
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

BAG = os.environ.get("SCENE_BAG", "/root/sim_ws/output/scene_bag")
MP4 = os.environ.get("SCENE_MP4", "/root/sim_ws/output/scene_img/scene.mp4")
TOPIC = os.environ.get("SCENE_TOPIC", "/image_color")
FPS_ENV = float(os.environ.get("SCENE_FPS", "0"))
MAXW = int(os.environ.get("SCENE_MAXW", "1280"))

os.makedirs(os.path.dirname(MP4), exist_ok=True)

bridge = CvBridge()
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
    rosbag2_py.ConverterOptions("", ""),
)

# Собираем кадры + sim-штампы. ВАЖНО: берём msg.header.stamp (sim-время камеры,
# use_sim_time), а НЕ bag-receive-время (3-й элемент read_next) — последнее это
# WALL-время записи, на низком RTF растянутое в ~14× (даст «слайдшоу» 2 fps вместо
# реальных ~30 sim-Гц камеры). По header.stamp длительность ролика = длительности
# полёта в sim-времени.
frames = []   # (t_sim_s, bgr_image)
size = None
while reader.has_next():
    topic, data, _bag_t = reader.read_next()
    if topic != TOPIC:
        continue
    msg = deserialize_message(data, Image)
    t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    if MAXW > 0 and img.shape[1] > MAXW:
        h = int(img.shape[0] * MAXW / img.shape[1])
        img = cv2.resize(img, (MAXW, h), interpolation=cv2.INTER_AREA)
    if size is None:
        size = (img.shape[1], img.shape[0])
    frames.append((t_sim, img))

if not frames:
    raise SystemExit(f"⚠️ в bag нет сообщений {TOPIC} — нечего кодировать")

# FPS: авто из растяжки sim-штампов (длительность ролика = длительность полёта),
# либо фикс из SCENE_FPS.
if FPS_ENV > 0:
    fps = FPS_ENV
else:
    span_s = frames[-1][0] - frames[0][0]
    fps = (len(frames) - 1) / span_s if span_s > 0 else 10.0
    fps = max(1.0, min(fps, 60.0))   # держим в разумных рамках

fourcc = cv2.VideoWriter_fourcc(*"mp4v")
writer = cv2.VideoWriter(MP4, fourcc, fps, size)
if not writer.isOpened():
    raise SystemExit(f"⚠️ VideoWriter не открылся для {MP4} (codec mp4v / size {size})")

for _, img in frames:
    writer.write(img)
writer.release()

dur = len(frames) / fps
print(f"Записано {len(frames)} кадров → {MP4}")
print(f"  размер {size[0]}×{size[1]}, fps={fps:.2f}, длительность ~{dur:.1f}с (sim)")
