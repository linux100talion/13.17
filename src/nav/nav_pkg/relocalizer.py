#!/usr/bin/env python3
# ============================================================================
# relocalizer — ПУСТАЯ нода-заглушка под восстановление VINS-трекинга.
#
# Принимает найденную NN2 позу места (/nn2/relocalization, JSON в String) и пока
# только логирует приём. Сюда позже ляжет логика релокализации: когда VINS
# теряет трекинг, взять эту позу (GPS/ENU + кватернион IMU ближайшего места
# карты облёта) и переинициализировать абсолютную позицию. Инъекцию в полётник
# делает ray_tracer (единственный мост VINS->FCU, см. nn1_anchor_howto.txt) —
# relocalizer будет отдавать поправку ему, а не в /mavros напрямую.
#
# Сейчас допущение: VINS на треке; нода просто подтверждает, что поза доходит.
# ============================================================================
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Relocalizer(Node):
    def __init__(self):
        super().__init__("relocalizer")
        self.create_subscription(String, "/nn2/relocalization", self._on_reloc, 10)
        self.get_logger().info("relocalizer (заглушка): жду /nn2/relocalization")

    def _on_reloc(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn("релокализация: битый JSON, пропускаю")
            return

        # TODO: при потере трекинга VINS применить позу для переинициализации
        # абсолютной позиции (отдать поправку ray_tracer).
        self.get_logger().info(
            f"релокализация: место '{d.get('label')}' (id={d.get('scene_id')}, "
            f"score={d.get('score')}) x={d.get('x')} y={d.get('y')} z={d.get('z')}")


def main(args=None):
    rclpy.init(args=args)
    node = Relocalizer()
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
