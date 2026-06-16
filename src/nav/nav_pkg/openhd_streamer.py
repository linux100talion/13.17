#!/usr/bin/env python3
# ============================================================================
# openhd_streamer — сборка даунлинк-видео для OpenHD (вариант 2 архитектуры).
#
# Камера-нода (camera_pkg) БОЛЬШЕ НЕ кодирует поток сама, а публикует
# /image_color (bgr8, полный кадр 1280x720, каждый кадр). Эта нода:
#   - держит видео на ПОЛНОМ fps (детекции нейросетей редкие — NN1 ~1 Гц,
#     NN2 ~раз в 3 с, см. CLAUDE.md), поэтому от инференса fps не зависит;
#   - кэширует ПОСЛЕДНИЕ детекции/семантику от обеих нейросетей;
#   - рисует их на КАЖДОМ кадре (cv2.rectangle/putText) — рамки «залипают»
#     между обновлениями, для FPV-даунлинка это нормально;
#   - кодирует H.264 и шлёт по UDP на host:port (по умолчанию 127.0.0.1:5600).
#
# Нейросети публикуют только геометрию/семантику (килобайты), пиксели не гоняют:
#   /nn1/detections  vision_msgs/Detection2DArray  — якорные ориентиры (NN1)
#   /nn2/scene       std_msgs/String               — метка сцены (NN2)
#
# Рамки от NN1 заданы в координатах ПОЛНОГО кадра; оверлей рисуется на полном
# кадре, и только потом картинка ужимается до out_width x out_height — поэтому
# масштабировать боксы вручную не нужно.
# ============================================================================
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


class OpenHDStreamer(Node):
    def __init__(self):
        super().__init__("openhd_streamer")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5600)
        self.declare_parameter("bitrate", 4000)
        self.declare_parameter("out_width", 640)
        self.declare_parameter("out_height", 360)

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        bitrate = int(self.get_parameter("bitrate").value)
        self.ow = int(self.get_parameter("out_width").value)
        self.oh = int(self.get_parameter("out_height").value)

        self.bridge = CvBridge()
        self.last_detections = None   # vision_msgs/Detection2DArray
        self.last_scene = ""          # str

        pipeline = (
            "appsrc ! videoconvert ! "
            f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate} ! "
            f"rtph264pay ! udpsink host={host} port={port} sync=false"
        )
        self.writer = cv2.VideoWriter(
            pipeline, cv2.CAP_GSTREAMER, 0, 15.0, (self.ow, self.oh), True
        )
        if not self.writer.isOpened():
            self.get_logger().error("Не удалось открыть GStreamer для OpenHD!")
        else:
            self.get_logger().info(f"OpenHD H.264 поток запущен на {host}:{port}")

        self.create_subscription(Image, "/image_color", self.on_image, 10)
        self.create_subscription(Detection2DArray, "/nn1/detections", self.on_nn1, 10)
        self.create_subscription(String, "/nn2/scene", self.on_nn2, 10)

    def on_nn1(self, msg):
        self.last_detections = msg

    def on_nn2(self, msg):
        self.last_scene = msg.data

    def on_image(self, msg):
        if not self.writer.isOpened():
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._draw_overlays(frame)
        out = cv2.resize(frame, (self.ow, self.oh))
        self.writer.write(out)

    def _draw_overlays(self, frame):
        # NN1: рамки якорных ориентиров (зелёные).
        if self.last_detections is not None:
            for det in self.last_detections.detections:
                bb = det.bbox
                cx, cy = bb.center.position.x, bb.center.position.y
                x1, y1 = int(cx - bb.size_x / 2), int(cy - bb.size_y / 2)
                x2, y2 = int(cx + bb.size_x / 2), int(cy + bb.size_y / 2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = det.results[0].hypothesis.class_id if det.results else ""
                if label:
                    cv2.putText(frame, label, (x1, max(0, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # NN2: семантика сцены — баннер сверху (жёлтый).
        if self.last_scene:
            cv2.putText(frame, f"scene: {self.last_scene}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)


def main(args=None):
    rclpy.init(args=args)
    node = OpenHDStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.writer.isOpened():
            node.writer.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
