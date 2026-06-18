#!/usr/bin/env python3
# ============================================================================
# ray_tracer — Нейросеть №1, Инкремент 2: засечка по ориентиру -> сброс дрейфа.
#
# Берёт детекцию NN1 (/nn1/detections: bbox + id ориентира), по id достаёт из
# базы GPS+высоту ориентира, и «стреляет» лучом из камеры через центр bbox:
#   пиксель -> луч в optical -> разворот в ENU (углы из MAVROS + монтаж камеры)
#   -> пересечение с высотой ориентира при известной высоте дрона (баро)
#   -> абсолютная позиция дрона в ENU (начало = точка взлёта = датум БД).
#
# «Сброс дрейфа» (неинвазивно, без правки VINS): держим поправку-смещение
#   offset = (засечка - поза VINS),
# и публикуем corrected_odom = vins_odom + offset. Каждая засечка обнуляет
# накопленный дрейф. Инъекция corrected_odom в MAVROS vision_pose / обратно в
# VINS — Инкремент 3.
#
# Допущения (см. nn1_anchor_howto.txt): рамка VINS ~ ENU (origin-coupled на
# взлёте + выровнены по рысканью); поправка пока ТРАНСЛЯЦИОННАЯ. Статичные
# развороты/рычаги камеры — ROS-параметры, калибруются на живом запуске.
# ============================================================================
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray

from nav_pkg.nn1 import geo


class RayTracer(Node):
    def __init__(self):
        super().__init__("ray_tracer")

        default_db = os.path.join(
            get_package_share_directory("nav_pkg"), "reference_db")
        self.declare_parameter("db_path", default_db)
        # топики (дефолты под симуляцию)
        self.declare_parameter("detections_topic", "/nn1/detections")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("attitude_topic", "/mavros/imu/data")
        self.declare_parameter("rel_alt_topic", "/mavros/global_position/rel_alt")
        self.declare_parameter("vins_odom_topic", "/vins_estimator/odometry")
        # интринсики-fallback, если /camera_info не пришёл (sim.yaml)
        self.declare_parameter("fx", 640.0)
        self.declare_parameter("fy", 640.0)
        self.declare_parameter("cx", 640.0)
        self.declare_parameter("cy", 360.0)
        # монтаж камеры (model.sdf): rpy camlink относительно body + рычаг (м)
        self.declare_parameter("cam_mount_rpy", [0.0, 0.26, 0.0])
        self.declare_parameter("cam_mount_xyz", [0.15, 0.0, 0.05])
        # сглаживание поправки: 1.0 = жёсткий сброс на каждой засечке
        self.declare_parameter("correction_alpha", 1.0)
        self.declare_parameter("anchor_pos_std", 2.0)   # σ засечки, м
        # Инкремент 3: инъекция скорректированной позы в полётник (ArduPilot
        # EK3 External Nav). ray_tracer = единственный мост VINS->FCU.
        self.declare_parameter("publish_vision_pose", True)
        self.declare_parameter("vision_pose_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("vision_pose_frame", "map")

        self.db_path = self.get_parameter("db_path").value
        self.alpha = float(self.get_parameter("correction_alpha").value)
        self.anchor_std = float(self.get_parameter("anchor_pos_std").value)

        # статичный разворот optical(CV) -> body: R_body_camlink @ R_camlink_opt
        rpy = self.get_parameter("cam_mount_rpy").value
        self.R_body_opt = geo.rpy_to_rotmat(*rpy) @ geo.R_CAMLINK_OPT
        self.lever_body = np.array(self.get_parameter("cam_mount_xyz").value)

        self._load_db()

        # состояние
        self.K = None                 # (fx,fy,cx,cy) из camera_info или params
        self.R_enu_body = None        # из attitude
        self.rel_alt = None           # высота над взлётом, м
        self.vins_pos = None          # последняя поза VINS (ENU)
        self.offset = np.zeros(3)     # поправка-смещение
        self.have_fix = False

        # I/O
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value,
                                 self._on_caminfo, 10)
        self.create_subscription(Imu, self.get_parameter("attitude_topic").value,
                                 self._on_attitude, 50)
        self.create_subscription(Float64, self.get_parameter("rel_alt_topic").value,
                                 self._on_rel_alt, 10)
        self.create_subscription(Odometry, self.get_parameter("vins_odom_topic").value,
                                 self._on_vins, 50)
        self.create_subscription(Detection2DArray, self.get_parameter("detections_topic").value,
                                 self._on_detections, 10)

        self.pub_anchor = self.create_publisher(PoseWithCovarianceStamped, "/nn1/anchor_pose", 10)
        self.pub_corr = self.create_publisher(Odometry, "/nn1/corrected_odom", 10)
        self.pub_drift = self.create_publisher(Vector3Stamped, "/nn1/drift", 10)

        self.publish_vp = bool(self.get_parameter("publish_vision_pose").value)
        self.vp_frame = self.get_parameter("vision_pose_frame").value
        self.pub_vision = None
        if self.publish_vp:
            self.pub_vision = self.create_publisher(
                PoseStamped, self.get_parameter("vision_pose_topic").value, 10)

        self.get_logger().info(
            "ray_tracer запущен (Инкремент 2/3: засечка -> сброс дрейфа -> "
            + ("vision_pose" if self.publish_vp else "vision_pose ВЫКЛ") + ")")

    # --- база (origin + landmarks) -------------------------------------------
    def _load_db(self):
        meta = Path(self.db_path) / "database.json"
        self.origin = None
        self.landmarks = {}
        if not meta.exists():
            self.get_logger().warn(f"database.json не найден ({meta}) — засечки невозможны.")
            return
        db = json.loads(meta.read_text(encoding="utf-8"))
        self.origin = db.get("origin")
        self.landmarks = db.get("landmarks", {})
        if not self.origin:
            self.get_logger().warn("в базе нет origin (датум взлёта) — засечки невозможны.")

    # --- входы ---------------------------------------------------------------
    def _on_caminfo(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])   # fx,fy,cx,cy

    def _on_attitude(self, msg):
        q = msg.orientation
        self.R_enu_body = geo.quat_to_rotmat(q.x, q.y, q.z, q.w)

    def _on_rel_alt(self, msg):
        self.rel_alt = float(msg.data)

    def _on_vins(self, msg):
        p = msg.pose.pose.position
        self.vins_pos = np.array([p.x, p.y, p.z])
        # на каждой одометрии VINS публикуем скорректированную
        corr = Odometry()
        corr.header = msg.header
        corr.child_frame_id = msg.child_frame_id
        corr.pose = msg.pose
        corr.twist = msg.twist
        corr.pose.pose.position.x = p.x + self.offset[0]
        corr.pose.pose.position.y = p.y + self.offset[1]
        corr.pose.pose.position.z = p.z + self.offset[2]
        self.pub_corr.publish(corr)

        # Инкремент 3: та же скорректированная поза -> полётнику (PoseStamped).
        # До первой засечки offset=0 => прокидываем сырой VINS (нужно ArduPilot
        # для GPS-denied); после — с вшитой коррекцией дрейфа.
        # Ориентация: пока от VINS как есть (yaw-коррекция — отдельный шаг).
        if self.pub_vision is not None:
            vp = PoseStamped()
            vp.header = msg.header
            vp.header.frame_id = self.vp_frame
            vp.pose = corr.pose.pose
            self.pub_vision.publish(vp)

    def _intrinsics(self):
        if self.K is not None:
            return self.K
        return (self.get_parameter("fx").value, self.get_parameter("fy").value,
                self.get_parameter("cx").value, self.get_parameter("cy").value)

    # --- засечка -------------------------------------------------------------
    def _on_detections(self, msg):
        if self.origin is None or self.R_enu_body is None or self.rel_alt is None:
            return   # нет датума / углов / высоты — ждём
        if not msg.detections:
            return

        det = msg.detections[0]
        if not det.results:
            return
        lm_id = det.results[0].hypothesis.class_id
        lm = self.landmarks.get(lm_id)
        if lm is None:
            self.get_logger().warn(f"ориентир '{lm_id}' не найден в базе — пропуск.")
            return

        # ориентир в ENU (датум = взлёт)
        P = geo.geodetic_to_enu(lm["lat"], lm["lon"], lm["alt"],
                                self.origin["lat"], self.origin["lon"], self.origin["alt"])

        # луч через центр bbox
        u = det.bbox.center.position.x
        v = det.bbox.center.position.y
        fx, fy, cx, cy = self._intrinsics()
        ray_opt = geo.backproject(u, v, fx, fy, cx, cy)
        ray_world = self.R_enu_body @ (self.R_body_opt @ ray_opt)

        # высота камеры над взлётом = высота body + Z-проекция рычага
        lever_world = self.R_enu_body @ self.lever_body
        cam_z = self.rel_alt + lever_world[2]

        C = geo.solve_camera_position(P, ray_world, cam_z)
        if C is None:
            self.get_logger().warn("засечка отброшена (луч горизонтален / ориентир позади).")
            return

        # позиция КОРПУСА = камера минус рычаг
        drone = C - lever_world

        self._publish_anchor(msg.header, drone)

        # сброс дрейфа: offset так, чтобы vins+offset == засечка
        if self.vins_pos is not None:
            new_off = drone - self.vins_pos
            self.offset = self.alpha * new_off + (1.0 - self.alpha) * self.offset
            self.have_fix = True
            self._publish_drift(msg.header)
            self.get_logger().info(
                f"засечка '{lm_id}': drone ENU=({drone[0]:.1f},{drone[1]:.1f},{drone[2]:.1f}), "
                f"дрейф=({self.offset[0]:.2f},{self.offset[1]:.2f},{self.offset[2]:.2f}) м")
        else:
            self.get_logger().info(
                f"засечка '{lm_id}': drone ENU=({drone[0]:.1f},{drone[1]:.1f},{drone[2]:.1f}); "
                f"одометрии VINS ещё нет — поправка не обновлена.")

    def _publish_anchor(self, header, drone):
        msg = PoseWithCovarianceStamped()
        msg.header = header
        msg.header.frame_id = "enu"
        msg.pose.pose.position.x = float(drone[0])
        msg.pose.pose.position.y = float(drone[1])
        msg.pose.pose.position.z = float(drone[2])
        msg.pose.pose.orientation.w = 1.0
        var = self.anchor_std ** 2
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = var          # x,y,z дисперсии
        msg.pose.covariance = cov
        self.pub_anchor.publish(msg)

    def _publish_drift(self, header):
        d = Vector3Stamped()
        d.header = header
        d.vector.x, d.vector.y, d.vector.z = map(float, self.offset)
        self.pub_drift.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = RayTracer()
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
