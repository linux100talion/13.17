#!/usr/bin/env python3
# ============================================================================
# route_heads_node.py — ШАГ 5 (c)-основного: ПРОДЮСЕР /nn2/route_coords (рантайм C).
#
# Закрывает дыру: relocalizer_field ПОТРЕБЛЯЛ /nn2/route_coords, но никто их не
# публиковал (голов C в рантайме не было). Эта нода = route-слой (c)-основного:
#   /image_color -> DINOv2 φ -> головы C (route_heads.RouteHeads) -> s,e ->
#   /nn2/route_coords {s,e,conf}.  БЕЗ FAISS (XXIII: route-рантайм — forward-pass).
#
# Темп — таймером (DINOv2 тяжёлый, подписка держит последний кадр), как nn2_scene.
# conf — базовый параметр; фильтрацию s (s_filter, XXII) и OOD-страж по метрике
# (XXIII) делает ПОТРЕБИТЕЛЬ relocalizer_field (там кэш метрики). Разделение:
# продюсер = инференс голов; потребитель = фильтр/фьюз/страж.
#
# ⚠ КОНЦЕПТ-ветка: φ тут считается ОТДЕЛЬНЫМ DINOv2 (двойной прогон с nn2_scene).
# На борту правильно слить в nn2_scene: один φ -> и метр-блок (FAISS), и головы C
# (infer_feat на том же φ). Здесь раздельно — ради чистоты концепта/декаплинга.
# Не вшит в setup.py/launch (как и relocalizer_field).
# ============================================================================
import json
import sys
from pathlib import Path

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route_heads import RouteHeads                          # noqa: E402


class RouteHeadsNode(Node):
    def __init__(self):
        super().__init__("route_heads")
        self.declare_parameter("ckpt", "")          # train_route_coords.pt (головы C)
        self.declare_parameter("period_s", 1.0)     # темп инференса голов (легче DINOv2)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("base_conf", 0.8)    # conf засечки (потребитель ещё фильтрует)

        ckpt = self.get_parameter("ckpt").value
        self.bridge = CvBridge()
        self.last_image = None
        if not ckpt:
            self.get_logger().warn("параметр ckpt пуст — нода вхолостую "
                                   "(нужен train_route_coords.pt с головами C)")
            self.heads = None
        else:
            self.heads = RouteHeads.load(
                ckpt, device=self.get_parameter("device").value, with_encoder=True)
            self.get_logger().info(f"головы C загружены (L={self.heads.L:.1f} м)")

        self.create_subscription(Image, "/image_color", self._on_image, 1)
        self.pub = self.create_publisher(String, "/nn2/route_coords", 10)
        self.create_timer(float(self.get_parameter("period_s").value), self._on_tick)
        self.get_logger().info("route_heads (рантайм C): /image_color -> s,e -> /nn2/route_coords")

    def _on_image(self, msg):
        self.last_image = msg                       # держим только последний (QoS depth=1)

    def _on_tick(self):
        if self.heads is None or self.last_image is None:
            return
        frame = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
        s, e = self.heads.infer_frame(frame)        # φ -> головы C (один forward-pass)
        payload = {"s": float(s), "e": float(e),
                   "conf": float(self.get_parameter("base_conf").value)}
        self.pub.publish(String(data=json.dumps(payload)))
        self.get_logger().debug(f"route C: s={s:.3f}, e={e:.2f} м -> /nn2/route_coords")


def main(args=None):
    rclpy.init(args=args)
    node = RouteHeadsNode()
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
