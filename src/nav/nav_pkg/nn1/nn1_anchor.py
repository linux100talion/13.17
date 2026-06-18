#!/usr/bin/env python3
# ============================================================================
# nn1_anchor — Нейросеть №1 (якорная локализация, замена GPS), Инкремент 1.
#
# SuperPoint+LightGlue матчит /image_color против георефернс-базы облёта
# (см. anchor_matcher.py). Нашёл известный ориентир -> публикует bbox + id в
# vision_msgs/Detection2DArray (сразу видно в оверлее openhd_streamer).
#
# Темп — ТАЙМЕРОМ (NN1 ~1 Гц), подписка держит только последний кадр (depth=1):
# инференс тяжёлый, каждый кадр обрабатывать не нужно.
#
# Инкремент 2 (отдельная нода): ray tracing по bbox+id+координатам ориентира +
# баро/IMU -> абсолютная позиция -> сравнение с VINS -> сброс дрейфа.
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from nav_pkg.nn1.anchor_matcher import AnchorMatcher


class NN1Anchor(Node):
    def __init__(self):
        super().__init__("nn1_anchor")

        default_db = os.path.join(
            get_package_share_directory("nav_pkg"), "reference_db")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("db_path", default_db)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("min_matches", 15)
        self.declare_parameter("max_keypoints", 1024)

        rate = float(self.get_parameter("rate_hz").value)

        self.bridge = CvBridge()
        self.last_image = None

        self.matcher = AnchorMatcher(
            db_path=self.get_parameter("db_path").value,
            device=self.get_parameter("device").value,
            max_keypoints=self.get_parameter("max_keypoints").value,
            min_matches=self.get_parameter("min_matches").value,
            logger=self.get_logger(),
        )

        self.sub = self.create_subscription(Image, "/image_color", self._on_image, 1)
        self.pub = self.create_publisher(Detection2DArray, "/nn1/detections", 10)
        self.create_timer(1.0 / rate, self._on_tick)
        self.get_logger().info(f"NN1 (anchor) SuperPoint+LightGlue, {rate} Гц")

    def _on_image(self, msg):
        self.last_image = msg   # держим только последний кадр (QoS depth=1)

    def _on_tick(self):
        if self.last_image is None:
            return

        frame = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
        match = self.matcher.query(frame)

        out = Detection2DArray()
        out.header = self.last_image.header   # штамп кадра -> синхронизация с ray tracing

        if match is not None:
            x1, y1, x2, y2 = match.bbox
            det = Detection2D()
            det.header = out.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = match.landmark_id      # ray-trace нода ищет координаты по id
            hyp.hypothesis.score = min(1.0, match.num_matches / 100.0)
            det.results.append(hyp)
            out.detections.append(det)

            self.get_logger().info(
                f"NN1: ориентир '{match.landmark_id}' (matches={match.num_matches}, "
                f"эталон {match.ref_name})")

        # Публикуем всегда (в т.ч. пустой) — overlay сбрасывает старую рамку.
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = NN1Anchor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
