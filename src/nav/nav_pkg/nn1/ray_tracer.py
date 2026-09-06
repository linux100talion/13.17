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
# Кадры: мир VINS рождается с курсом ПЕРВОГО кадра камеры (yaw у монокуляра
# ненаблюдаем) — к ENU он повёрнут на курс старта. Выравнивает FrameAnchor
# (frame_anchor.py): Δyaw + трансляция латчатся по паре поз EKF/VINS (разнос
# LOITER при спавне с курсом −169°, прогоны 2026-08-24). Засечка NN1 правит
# только трансляцию; yaw-коррекция ПО ОРИЕНТИРУ — отдельный шаг. Статичные
# развороты/рычаги камеры — ROS-параметры, калибруются на живом запуске.
# ============================================================================
import json
import math
import os
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import Bool, Float64, String
from vision_msgs.msg import Detection2DArray

from nav_pkg.nn1 import geo
from nav_pkg.nn1.bridge_gate import BridgeGate
from nav_pkg.nn1.frame_anchor import FrameAnchor, quat_yaw


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
        # Форк VINS-MONO-ROS2 публикует "odometry" ОТ КОРНЯ (в ROS2 нет
        # приватного пространства ноды как в ROS1) → фактический топик /odometry.
        # Старый дефолт /vins_estimator/odometry не имел издателя — мост молчал.
        self.declare_parameter("vins_odom_topic", "/odometry")
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
        # Якорение кадра VINS на кадр EKF при первой одометрии (см. _on_vins).
        self.declare_parameter("ekf_pose_topic", "/mavros/local_position/pose")
        # Слежение якоря ПОСЛЕ первого латча (полёт 2026-08-21 №7): порог
        # жёсткой подтяжки, м (0 = выкл) и τ мягкого дожима, с (0 = выкл).
        self.declare_parameter("anchor_relatch_m", 1.0)
        self.declare_parameter("anchor_tau_sec", 5.0)
        # ГЕЙТ ЗДОРОВЬЯ МОСТА (bridge_gate.py; полёт 142811 — разнос VINS через
        # 687 подтяжек якоря отравил ориентацию EKF, DpHold унесло): мост закрыт
        # при |twist| > v_max, перерождении потока (дыра/скачок), шторме подтяжек
        # (≥ n за win с) и по вердикту лётной ноды (/vins/sane), hold_sec латч.
        self.declare_parameter("bridge_gate", True)
        self.declare_parameter("bridge_v_max", 12.0)
        self.declare_parameter("bridge_v_jump", 12.0)
        self.declare_parameter("bridge_gap_sec", 1.0)
        self.declare_parameter("bridge_relatch_n", 3)
        self.declare_parameter("bridge_relatch_win", 5.0)
        self.declare_parameter("bridge_hold_sec", 5.0)
        self.declare_parameter("vins_sane_topic", "/vins/sane")
        self.declare_parameter("vins_restart_topic", "/restart")

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
        self.vins_pos = None          # последняя поза VINS (его собственный кадр)
        self.have_fix = False
        self.ekf_pos = None           # последняя поза EKF (local_position)
        self.ekf_yaw = 0.0            # курс EKF из ТОЙ ЖЕ позы (пара к ekf_pos)
        self.ekf_pos_wall = 0.0
        # Якорь кадра VINS→EKF: РЫСКАНЬЕ + трансляция (см. frame_anchor.py:
        # трансляционный якорь при спавне с курсом −169° кормил EKF почти
        # перевёрнутыми смещениями → положительная ОС LOITER, разнос 15 м/с)
        self.anchor = FrameAnchor(
            relatch_m=float(self.get_parameter("anchor_relatch_m").value),
            tau_sec=float(self.get_parameter("anchor_tau_sec").value))
        self.gate = (BridgeGate(
            v_max=float(self.get_parameter("bridge_v_max").value),
            v_jump=float(self.get_parameter("bridge_v_jump").value),
            gap_sec=float(self.get_parameter("bridge_gap_sec").value),
            relatch_n=int(self.get_parameter("bridge_relatch_n").value),
            relatch_win=float(self.get_parameter("bridge_relatch_win").value),
            hold_sec=float(self.get_parameter("bridge_hold_sec").value))
            if bool(self.get_parameter("bridge_gate").value) else None)
        self._gate_open = True          # последнее состояние — для лога переходов
        self._ext_sane = None           # вердикт лётной ноды (/vins/sane)
        self._ext_wall = 0.0

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
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(PoseStamped, self.get_parameter("ekf_pose_topic").value,
                                 self._on_ekf_pose, qos_profile_sensor_data)

        self.pub_anchor = self.create_publisher(PoseWithCovarianceStamped, "/nn1/anchor_pose", 10)
        self.pub_corr = self.create_publisher(Odometry, "/nn1/corrected_odom", 10)
        self.pub_drift = self.create_publisher(Vector3Stamped, "/nn1/drift", 10)
        # состояние моста (open|closed причина подтяжек закрытий перерождений) —
        # лётная нода кладёт в /mission/status (brg=…), пишется в bag
        self.pub_bridge = self.create_publisher(String, "/nn1/bridge", 10)
        if self.gate is not None:
            self.create_subscription(Bool, self.get_parameter("vins_sane_topic").value,
                                     self._on_vins_sane, 10)
            self.create_subscription(Bool, self.get_parameter("vins_restart_topic").value,
                                     self._on_vins_restart, 1)

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

    def _on_vins_sane(self, msg):
        # вердикт гейта здоровья лётной ноды (handover.vins_sane, 20 Гц);
        # протухает за 1 с (см. _on_vins) — без ноды мост живёт своими проверками
        self._ext_sane = bool(msg.data)
        self._ext_wall = time.time()

    def _on_vins_restart(self, msg):
        # наш /restart VINS: поток родится заново — якорь и гейт с чистого листа
        if self.gate is not None and msg.data:
            self.gate.reset()
            self.anchor.reset()
            self.get_logger().info("мост: /restart VINS — якорь кадра заново")

    def _on_ekf_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self.ekf_pos = np.array([p.x, p.y, p.z])
        # курс — из той же позы, что и позиция: пара согласована по времени
        self.ekf_yaw = quat_yaw(q.x, q.y, q.z, q.w)
        self.ekf_pos_wall = time.time()

    def _on_vins(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.vins_pos = np.array([p.x, p.y, p.z])
        vins_yaw = quat_yaw(q.x, q.y, q.z, q.w)
        # ГЕЙТ ЗДОРОВЬЯ МОСТА (bridge_gate.py): по штампу одометрии (одна шкала
        # с потоком), |twist|, вердикту лётной ноды (свежий < 1 с)
        gate_open = True
        if self.gate is not None:
            th = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            tv = msg.twist.twist.linear
            ext = (self._ext_sane if time.time() - self._ext_wall < 1.0 else None)
            gate_open = self.gate.on_odom(th, p.x, p.y, math.hypot(tv.x, tv.y), ext)
            if self.gate.take_relatch():
                self.anchor.reset()
            if gate_open != self._gate_open:
                self._gate_open = gate_open
                if gate_open:
                    self.get_logger().info("мост VINS→EKF ОТКРЫТ (якорь %s)" % (
                        "заново" if not self.anchor.latched else "прежний"))
                else:
                    self.get_logger().warn(
                        f"мост VINS→EKF ЗАКРЫТ: {self.gate.reason} (|v| "
                        f"{math.hypot(tv.x, tv.y):.1f} м/с, закрытий {self.gate.closes}, "
                        f"перерождений {self.gate.rebirths}) — vision_pose не идёт, "
                        f"якорь заморожен")
            self.pub_bridge.publish(String(data=self.gate.state_line(th)))
        # Якорение КАДРА (полёт 2026-08-20 №4): мир VINS рождается в точке его
        # инициализации — в воздухе, куда борт уже улетел от точки арма, а кадр
        # EKF считается от арма. Офсет кадров (9.6 м в том полёте) выносит
        # каждое vision-измерение за инновационный гейт EK3 — фьюжн не
        # начинается вовсе (рантайм-смена EK3_SRC1_POSXY ресет позиции НЕ
        # делает), «EKF variance: position lost». Пока EKF жив (взлёт на GPS) —
        # FrameAnchor защёлкивает Δyaw (курс EKF − курс VINS) + трансляцию тем
        # же механизмом, что у засечки NN1 (та потом уточнит трансляцию). Нет
        # свежей EKF-позы (боевой GPS-denied бут) — якорь не латчится, сырой
        # VINS идёт как есть (поведение прежнее).
        #
        # СЛЕЖЕНИЕ после первого латча (полёт 2026-08-21 №7): разовый латч
        # ловит ХУДШИЙ момент — init VINS, когда юный VIO врёт масштабом
        # (ratio до ×10 первые ~20 с, полёт №3). При активном пилотировании
        # в ветре кадры разошлись на 4 м за 8 с ПОСЛЕ идеального латча
        # (VISP−XKF1: 0.11 м → 4.0 м), своп POSXY→6 через минуту получил
        # vision за гейтом → position lost при живом GPS. Поэтому пока EKF
        # свеж и засечки NN1 нет:
        #   расход > anchor_relatch_m — жёсткая подтяжка (заново Δyaw + t:
        #   расход мог накопиться именно из-за ошибки курса);
        #   меньше — мягкий дожим t с τ=anchor_tau_sec (Δyaw не трогаем).
        # После свапа на extnav EKF сам следует vision → расход ≈ инновация
        # (дециметры) → жёсткая подтяжка не срабатывает, дожим ≈ 0 — обратная
        # связь стабильна. Если EKF умер в const_pos, его поза замирает и
        # слежение поведёт якорь к ней — фьюжн к тому моменту уже потерян
        # (in-flight aiding не рестартует, LV4), хуже не делает.
        if (gate_open and not self.have_fix and self.ekf_pos is not None
                and time.time() - self.ekf_pos_wall < 2.0):
            ev = self.anchor.update(self.vins_pos, vins_yaw,
                                    self.ekf_pos, self.ekf_yaw, time.time())
            if ev == 'relatch' and self.gate is not None and self.gate.on_relatch(
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9):
                # шторм подтяжек = разнос: мост закрыт, якорь заново
                self.anchor.reset()
                gate_open = False
                self._gate_open = False
                self.get_logger().warn(
                    f"мост VINS→EKF ЗАКРЫТ: шторм подтяжек якоря "
                    f"(№{self.anchor.relatch_n}, закрытий {self.gate.closes}) — "
                    f"разнос VINS, якорь заново")
            if ev == 'latch':
                self.get_logger().info(
                    f"кадр VINS заякорен на EKF: Δyaw="
                    f"{math.degrees(self.anchor.yaw_off):+.1f}°, t="
                    f"({self.anchor.t[0]:+.2f},{self.anchor.t[1]:+.2f},"
                    f"{self.anchor.t[2]:+.2f}) м")
            elif ev == 'relatch':
                self.get_logger().info(
                    f"якорь кадра подтянут (№{self.anchor.relatch_n}): Δyaw="
                    f"{math.degrees(self.anchor.yaw_off):+.1f}°, t="
                    f"({self.anchor.t[0]:+.2f},{self.anchor.t[1]:+.2f},"
                    f"{self.anchor.t[2]:+.2f}) м")
        # на каждой одометрии VINS публикуем скорректированную — В КАДРЕ EKF:
        # позиция/ориентация/world-скорость повёрнуты на Δyaw (angular — body,
        # как есть)
        corr = Odometry()
        corr.header = msg.header
        corr.child_frame_id = msg.child_frame_id
        cp = self.anchor.map(self.vins_pos)
        cq = self.anchor.rotate_quat(q.x, q.y, q.z, q.w)
        (corr.pose.pose.position.x, corr.pose.pose.position.y,
         corr.pose.pose.position.z) = map(float, cp)
        (corr.pose.pose.orientation.x, corr.pose.pose.orientation.y,
         corr.pose.pose.orientation.z, corr.pose.pose.orientation.w) = \
            map(float, cq)
        corr.pose.covariance = msg.pose.covariance
        tv = msg.twist.twist.linear
        rv = self.anchor.rotate(np.array([tv.x, tv.y, tv.z]))
        (corr.twist.twist.linear.x, corr.twist.twist.linear.y,
         corr.twist.twist.linear.z) = map(float, rv)
        corr.twist.twist.angular = msg.twist.twist.angular
        corr.twist.covariance = msg.twist.covariance
        self.pub_corr.publish(corr)

        # Инкремент 3: та же скорректированная поза -> полётнику (PoseStamped).
        # До латча якорь тождественен => прокидываем сырой VINS (нужно ArduPilot
        # для GPS-denied); после — в кадре EKF с вшитой коррекцией дрейфа.
        # Yaw-коррекция ПО ЗАСЕЧКЕ NN1 — отдельный шаг (якорь правит кадр).
        if self.pub_vision is not None and gate_open:
            vp = PoseStamped()
            vp.header = msg.header
            vp.header.frame_id = self.vp_frame
            # Штамп — WALL-временем: FCU в SITL живёт по wall (JSON no_time_sync),
            # sim-штамп VINS уехал бы на часы AP_VisualOdom (см. vision-фид
            # бутстрапа). На боевом Orin ROS-время = wall → поведение идентично.
            wall = time.time()
            vp.header.stamp.sec = int(wall)
            vp.header.stamp.nanosec = int((wall % 1.0) * 1e9)
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

        # сброс дрейфа: трансляция якоря так, чтобы map(vins) == засечка
        # (поворот кадра остаётся от латча; yaw по ориентиру — отдельный шаг)
        if self.vins_pos is not None:
            self.anchor.fix_translation(drone, self.vins_pos, self.alpha)
            self.have_fix = True
            self._publish_drift(msg.header)
            t = self.anchor.t
            self.get_logger().info(
                f"засечка '{lm_id}': drone ENU=({drone[0]:.1f},{drone[1]:.1f},{drone[2]:.1f}), "
                f"дрейф=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f}) м")
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
        d.vector.x, d.vector.y, d.vector.z = map(float, self.anchor.t)
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
