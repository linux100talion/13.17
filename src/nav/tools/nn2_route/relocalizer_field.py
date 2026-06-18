#!/usr/bin/env python3
# ============================================================================
# relocalizer_field.py — НАБРОСОК боевой ноды relocalizer на route_field.
#
# Показывает, как на борту одна пара (s,e) кормит ОБЕ грани (XV):
#   ГРАНЬ 2 (засечка):  p̂ = R(σ)+e·n  -> PoseWithCovarianceStamped -> Калман/
#                       ray_tracer (подтяжка дрейфа VINS);
#   ПОЛЕ (рулёжка):     v = α·T̂ − β·e·n̂ -> (по курсу) body-команда -> guidance.
#
# Лёгкий путь (по умолчанию): (s,e) приходят готовыми из nn2_scene (тяжёлый
# DINOv2+головы крутит он), нода — чистая ГЕОМЕТРИЯ (RouteField, numpy). Под флагом
# self_infer можно крутить головы прямо здесь (RouteCoords) по /image_color.
#
# ⚠ НАБРОСОК на ветке-концепте: не вшит в setup.py/launch; имена топиков и
# ковариации — заготовка под калибровку. Боевая нода позже переедет в nav_pkg.
# Один мост VINS->FCU — ray_tracer (см. NN1): засечку шлём ему/в Калман, НЕ в
# /mavros напрямую.
# ============================================================================
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rclpy                                               # noqa: E402
from rclpy.node import Node                                # noqa: E402
from sensor_msgs.msg import Imu                            # noqa: E402
from std_msgs.msg import String                           # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped  # noqa: E402

from route_field import RouteField                         # noqa: E402
from route_geometry import Centerline                      # noqa: E402


def _yaw(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def _load_centerline(ckpt_path, device="cpu"):
    """Тянем только вершины центрлинии из train_route_coords.pt (torch.load)."""
    import torch
    ckpt = torch.load(str(ckpt_path), map_location=device)
    return Centerline(ckpt["centerline_verts"])


class RelocalizerField(Node):
    def __init__(self):
        super().__init__("relocalizer")
        self.declare_parameter("ckpt", "")          # train_route_coords.pt
        self.declare_parameter("alpha", 1.0)        # тяга «вперёд»
        self.declare_parameter("beta", 0.3)         # тяга «к нити»
        self.declare_parameter("speed", 2.0)        # модуль скорости, м/с
        self.declare_parameter("min_conf", 0.5)     # гейт по уверенности (s,e)
        self.declare_parameter("sigma_along", 5.0)  # ковариация засечки вдоль T, м
        self.declare_parameter("sigma_cross", 2.0)  # вдоль n, м
        self.declare_parameter("world_frame", "map")

        ckpt = self.get_parameter("ckpt").value
        if not ckpt:
            self.get_logger().warn("параметр ckpt пуст — нода вхолостую "
                                   "(нужен train_route_coords.pt)")
            self.field = None
        else:
            cl = _load_centerline(ckpt)
            self.field = RouteField(
                cl,
                alpha=self.get_parameter("alpha").value,
                beta=self.get_parameter("beta").value,
                speed=self.get_parameter("speed").value,
            )
            self.get_logger().info(f"центрлиния: L={cl.L:.1f} м")

        self.yaw = None
        self.create_subscription(Imu, "/mavros/imu/data", self._on_imu, 10)
        # (s,e,conf) от nn2_scene (он крутит DINOv2+головы). TODO: nn2_scene должен
        # начать публиковать /nn2/route_coords (JSON {s,e,conf}).
        self.create_subscription(String, "/nn2/route_coords", self._on_coords, 10)

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/nn2/reloc_pose", 10)   # -> Калман/ray_tracer
        self.pub_cmd = self.create_publisher(
            TwistStamped, "/nn2/guidance", 10)                  # -> offboard/guided
        self.get_logger().info("relocalizer (route_field): жду /nn2/route_coords")

    def _on_imu(self, msg):
        self.yaw = _yaw(msg.orientation)

    def _on_coords(self, msg):
        if self.field is None:
            return
        try:
            d = json.loads(msg.data)
            s, e = float(d["s"]), float(d["e"])
            conf = float(d.get("conf", 1.0))
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn("route_coords: битый JSON, пропускаю")
            return
        if conf < float(self.get_parameter("min_conf").value):
            return                                           # гейт: не уверены — молчим

        now = self.get_clock().now().to_msg()
        # --- ГРАНЬ 2: засечка p̂ = R(σ)+e·n -> в Калман ---
        p = self.field.position(s, e)
        pc = PoseWithCovarianceStamped()
        pc.header.stamp = now
        pc.header.frame_id = self.get_parameter("world_frame").value
        pc.pose.pose.position.x = float(p[0])
        pc.pose.pose.position.y = float(p[1])
        pc.pose.covariance = self._cov(s)
        self.pub_pose.publish(pc)

        # --- ПОЛЕ: рулёжка v = −∇V (нужен курс) ---
        if self.yaw is not None:
            vb = self.field.command_body(s, e, self.yaw)
            tw = TwistStamped()
            tw.header.stamp = now
            tw.header.frame_id = "base_link"
            tw.twist.linear.x = float(vb[0])     # вперёд (body)
            tw.twist.linear.y = float(vb[1])     # вбок (body)
            self.pub_cmd.publish(tw)
        # без курса -> visual servoing (route_field.VisualServo), здесь не собран.

    def _cov(self, s):
        """Анизотропная ковариация засечки: σ вдоль T (из неопр. s) и вдоль n
        (из неопр. e), повёрнутая в рамку мира. НАБРОСОК — диагональ в мире."""
        sa = float(self.get_parameter("sigma_along").value)
        sc = float(self.get_parameter("sigma_cross").value)
        _, T, n = self.field.cl.frame_at(s * self.field.cl.L)
        # cov_world = sa²·TTᵀ + sc²·nnᵀ
        C = sa * sa * np.outer(T, T) + sc * sc * np.outer(n, n)
        cov = [0.0] * 36
        cov[0] = float(C[0, 0]); cov[1] = float(C[0, 1])
        cov[6] = float(C[1, 0]); cov[7] = float(C[1, 1])
        cov[14] = 1.0e3          # z неопределён (баро отдельно)
        cov[21] = cov[28] = cov[35] = 1.0e3   # ориентация не оцениваем
        return cov


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizerField()
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
