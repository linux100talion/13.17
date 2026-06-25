#!/usr/bin/env python3
"""Вытащить кадры из rosbag /image_color ПО ПРОЙДЕННОМУ ПУТИ → JPEG.

Выборка не по времени, а по дистанции: первый кадр сохраняется в стартовой
позиции, далее — каждые SCENE_DIST_M метров пути дрона (по /mavros/local_position
/pose). Если дрон не двигался (не взлетел) или позы в bag нет — останется ТОЛЬКО
первый кадр (на время НЕ откатываемся).

Запускается ВНУТРИ nav-контейнера (нужен cv_bridge из /opt/overlay).
Параметры через env:
  SCENE_BAG     путь к rosbag            (default /root/sim_ws/output/scene_bag)
  SCENE_OUT     куда писать JPEG         (default /root/sim_ws/output/scene_img)
  SCENE_TOPIC   топик изображения        (default /image_color)
  SCENE_POSE    топик позы               (default /mavros/local_position/pose)
  SCENE_DIST_M  шаг выборки по пути, м   (default 0.5)
  SCENE_N       макс. число кадров       (default 30; 0 = без лимита)
"""
import math
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2

BAG = os.environ.get("SCENE_BAG", "/root/sim_ws/output/scene_bag")
OUT = os.environ.get("SCENE_OUT", "/root/sim_ws/output/scene_img")
TOPIC = os.environ.get("SCENE_TOPIC", "/image_color")
POSE_TOPIC = os.environ.get("SCENE_POSE", "/mavros/local_position/pose")
DIST_M = float(os.environ.get("SCENE_DIST_M", "0.5"))
N = int(os.environ.get("SCENE_N", "30"))   # 0 = без лимита

os.makedirs(OUT, exist_ok=True)
for f in os.listdir(OUT):
    if f.endswith(".jpg"):
        os.remove(os.path.join(OUT, f))

bridge = CvBridge()
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
    rosbag2_py.ConverterOptions("", ""),
)

prev_xyz = None       # последняя известная позиция (для накопления пути)
path = 0.0            # суммарный пройденный путь, м
saved_at = None       # значение path в момент последнего сохранения
saved = 0
have_pose = False

while reader.has_next():
    if N > 0 and saved >= N:
        break
    topic, data, t = reader.read_next()

    if topic == POSE_TOPIC:
        have_pose = True
        p = deserialize_message(data, PoseStamped).pose.position
        xyz = (p.x, p.y, p.z)
        if prev_xyz is not None:
            path += math.dist(xyz, prev_xyz)   # 3D-длина дуги
        prev_xyz = xyz
        continue

    if topic != TOPIC:
        continue

    # Кадр сохраняем: первый — всегда; далее — когда с прошлого сохранения
    # набежало >= DIST_M пройденного пути.
    if saved_at is None or (path - saved_at) >= DIST_M:
        msg = deserialize_message(data, Image)
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        fn = os.path.join(OUT, f"frame_{saved:02d}_{path:06.2f}m.jpg")
        cv2.imwrite(fn, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        print(f"{fn}  {img.shape}  path={path:.2f}m  encoding={msg.encoding}")
        saved += 1
        saved_at = path

if not have_pose:
    print(f"⚠️ поза ({POSE_TOPIC}) в bag не найдена — сохранён только первый кадр "
          f"(нет данных о пройденном пути).")
print(f"Сохранено {saved} кадров в {OUT} (шаг {DIST_M}м, путь всего {path:.2f}м)")
