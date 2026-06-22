#!/usr/bin/env python3
"""Вытащить N кадров из rosbag /image_color с заданным шагом → JPEG.

Запускается ВНУТРИ nav-контейнера (нужен cv_bridge из /opt/overlay).
Параметры через env (значения по умолчанию = то, что снимали вручную):
  SCENE_BAG     путь к rosbag           (default /root/sim_ws/output/scene_bag)
  SCENE_OUT     куда писать JPEG        (default /root/sim_ws/output/scene_img)
  SCENE_TOPIC   топик изображения       (default /image_color)
  SCENE_N       сколько кадров          (default 30)
  SCENE_STEP_NS шаг между кадрами, нс   (default 1000000000 = 1с)
"""
import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

BAG = os.environ.get("SCENE_BAG", "/root/sim_ws/output/scene_bag")
OUT = os.environ.get("SCENE_OUT", "/root/sim_ws/output/scene_img")
TOPIC = os.environ.get("SCENE_TOPIC", "/image_color")
N = int(os.environ.get("SCENE_N", "30"))
STEP_NS = int(os.environ.get("SCENE_STEP_NS", "1000000000"))

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

saved = 0
next_t = None
while reader.has_next() and saved < N:
    topic, data, t = reader.read_next()
    if topic != TOPIC:
        continue
    if next_t is None:
        next_t = t
    if t < next_t:
        continue
    msg = deserialize_message(data, Image)
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    fn = os.path.join(OUT, f"frame_{saved:02d}.jpg")
    cv2.imwrite(fn, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    print(f"{fn}  {img.shape}  encoding={msg.encoding}")
    saved += 1
    next_t += STEP_NS

print(f"Сохранено {saved} кадров в {OUT}")
