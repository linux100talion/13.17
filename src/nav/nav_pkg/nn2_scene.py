#!/usr/bin/env python3
# ============================================================================
# nn2_scene — БОЛВАНКА Нейросети №2 (топологическая карта, семантика сцены).
#
# Реальная нода: DINOv2 + AnyLoc + FAISS сжимает сцену в дескриптор, сравнивает
# с базой облёта → управление «по смыслу». Здесь — заглушка: по таймеру
# (~раз в 3 с) публикует строковую метку сцены, которую openhd_streamer рисует
# баннером поверх видео.
#
# Темп — ТАЙМЕРОМ (NN2 ~раз в 3 с), подписка держит только последний кадр.
# Публикует std_msgs/String (семантика — это не «бокс», поэтому не Detection2D).
# ============================================================================
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class NN2Scene(Node):
    def __init__(self):
        super().__init__("nn2_scene")
        self.declare_parameter("period_s", 3.0)
        period = float(self.get_parameter("period_s").value)

        self.last_image = None
        self.tick = 0
        self.create_subscription(Image, "/image_color", self._on_image, 1)
        self.pub = self.create_publisher(String, "/nn2/scene", 10)
        self.create_timer(period, self._on_tick)
        self.get_logger().info(f"NN2 (scene) заглушка, период {period} с")

    def _on_image(self, msg):
        self.last_image = msg

    def _on_tick(self):
        if self.last_image is None:
            return

        # TODO: здесь реальный инференс (DINOv2/AnyLoc/FAISS) по self.last_image.
        self.tick += 1
        msg = String()
        msg.data = f"sector_{self.tick}"
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NN2Scene()
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
