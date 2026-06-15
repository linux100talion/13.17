#!/usr/bin/env python3
# ============================================================================
# nn1_anchor — БОЛВАНКА Нейросети №1 (якорная локализация, замена GPS).
#
# Реальная нода: YOLOv8 / SuperPoint+LightGlue находит известные ориентиры →
# Ray Tracing через intrinsics + барометр/IMU → абсолютная позиция → сброс
# дрейфа VINS-Mono. Здесь — заглушка: проверяет сквозной путь до оверлея в
# openhd_streamer.
#
# Темп задаётся ТАЙМЕРОМ (не каждый кадр!) — NN1 ~1 Гц. Подписка держит только
# последний кадр (depth=1), таймер берёт самый свежий и дропает остальные.
# Публикует vision_msgs/Detection2DArray (боксы найденных ориентиров).
# ============================================================================
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class NN1Anchor(Node):
    def __init__(self):
        super().__init__("nn1_anchor")
        self.declare_parameter("rate_hz", 1.0)
        rate = float(self.get_parameter("rate_hz").value)

        self.last_image = None
        self.create_subscription(Image, "/image_color", self._on_image, 1)
        self.pub = self.create_publisher(Detection2DArray, "/nn1/detections", 10)
        self.create_timer(1.0 / rate, self._on_tick)
        self.get_logger().info(f"NN1 (anchor) заглушка, {rate} Гц")

    def _on_image(self, msg):
        self.last_image = msg   # держим только последний кадр (QoS depth=1)

    def _on_tick(self):
        if self.last_image is None:
            return

        # TODO: здесь реальный инференс по self.last_image (YOLOv8 / SuperPoint).
        out = Detection2DArray()
        out.header = self.last_image.header

        det = Detection2D()
        det.header = self.last_image.header
        det.bbox.center.position.x = 640.0
        det.bbox.center.position.y = 360.0
        det.bbox.size_x = 120.0
        det.bbox.size_y = 90.0

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "anchor"
        hyp.hypothesis.score = 0.0
        det.results.append(hyp)

        out.detections.append(det)
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
