#!/usr/bin/env python3
# ============================================================================
# vision_pose_bridge — минимальный мост VINS → ExternalNav, БЕЗ NN1-логики.
#
# Зачем: для тестов ALT_HOLD-bootstrap/handover полётнику нужна горизонтальная
# позиция в EKF (GUIDED — позиционный режим). Её даёт поток
# /mavros/vision_pose/pose. Полноценный ray_tracer (засечки по ориентирам,
# сброс дрейфа, база) пока отложен — а для самого факта «у EKF есть позиция»
# достаточно сырого VINS. Этот узел делает ровно это и ничего больше:
#
#   /vins_estimator/odometry (Odometry) → /mavros/vision_pose/pose (PoseStamped)
#
# Это в точности то, что ray_tracer публикует до первой засечки (offset=0),
# но без подписок на детекции/attitude/rel_alt/camera_info и без базы.
#
# ⚠️ Источник vision_pose должен быть РОВНО ОДИН. Не запускать одновременно с
# ray_tracer (тот тоже публикует в /mavros/vision_pose/pose) — иначе два
# издателя в один топик. Переключение — аргументом vision_pose_source в
# nav.launch.py (ray_tracer | bridge).
#
# Ориентация прокидывается от VINS «как есть» (yaw к NED не выровнен) — тот же
# рывок на handover, что и у ray_tracer; выравнивание появится в ray_tracer.
# ============================================================================
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class VisionPoseBridge(Node):
    def __init__(self):
        super().__init__("vision_pose_bridge")
        # Топики/кадр зеркалят дефолты ray_tracer → поведение идентично его
        # passthrough-режиму (offset=0), переключение прозрачно для EKF.
        self.declare_parameter("vins_odom_topic", "/vins_estimator/odometry")
        self.declare_parameter("vision_pose_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("vision_pose_frame", "map")

        self.frame = self.get_parameter("vision_pose_frame").value
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter("vision_pose_topic").value, 10)
        self.create_subscription(
            Odometry, self.get_parameter("vins_odom_topic").value, self._on_vins, 50)

        self.n = 0
        self.get_logger().info(
            "vision_pose_bridge запущен (сырой VINS → vision_pose, без NN1; "
            "не запускать вместе с ray_tracer)")

    def _on_vins(self, msg):
        vp = PoseStamped()
        vp.header = msg.header
        vp.header.frame_id = self.frame
        vp.pose = msg.pose.pose
        self.pub.publish(vp)
        self.n += 1
        if self.n == 1:
            self.get_logger().info("первая одометрия VINS проброшена в vision_pose")


def main(args=None):
    rclpy.init(args=args)
    node = VisionPoseBridge()
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
