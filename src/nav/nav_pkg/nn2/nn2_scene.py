#!/usr/bin/env python3
# ============================================================================
# nn2_scene — Нейросеть №2 (топологическая карта, семантика сцены), Инкремент 1.
#
# DINOv2 сжимает кадр в глобальный дескриптор, FAISS ищет ближайшее место в
# карте облёта (см. scene_descriptor.py). Каждую итерацию найденную позу места
# (GPS/ENU + кватернион IMU) шлём в ноду relocalizer (пока пустая заглушка под
# восстановление VINS-трекинга после потери). Сейчас допущение: VINS на треке,
# мы просто отдаём найденную позу.
#
# Темп — ТАЙМЕРОМ (NN2 ~раз в 3 с), подписка держит только последний кадр:
# DINOv2-инференс тяжёлый, каждый кадр обрабатывать не нужно.
#
# Топики:
#   /nn2/scene          std_msgs/String — метка места (баннер openhd_streamer)
#   /nn2/relocalization std_msgs/String — JSON-поза места для relocalizer; несёт и
#     МЕТР-БЛОК (XVIII): mx,my (позиция места, ENU-метры) + mstd (σ, м) + mconf
#     (уверенность 0..1). Это метрическая опора φ-карты, которую relocalizer_field
#     сливает с route-позой (route_fusion.gaussian_fuse). σ/conf — из режима карты:
#     L2-карта (MLP-топограф) -> score=метры -> σ,conf по дистанции; IP-карта
#     (косинус) -> метров нет -> σ фикс, conf из косинуса над порогом.
#     Тип String с JSON — осознанная заглушка («пока .json», см. идею); потом
#     заменим на типизированный msg (нужен отдельный CMake-интерфейс-пакет).
# ============================================================================
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from nav_pkg.nn2.scene_descriptor import SceneMatcher


class NN2Scene(Node):
    def __init__(self):
        super().__init__("nn2_scene")

        default_map = os.path.join(
            get_package_share_directory("nav_pkg"), "scene_map")
        self.declare_parameter("period_s", 3.0)
        self.declare_parameter("map_path", default_map)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("model_name", "dinov2_vits14")
        self.declare_parameter("min_score", 0.5)   # порог косинуса (карта без MLP)
        self.declare_parameter("max_dist", 10.0)   # порог дистанции, м (карта с MLP)
        # Метр-блок (XVIII): как считать σ/conf метрической опоры из режима карты.
        self.declare_parameter("sigma_metric_floor", 2.0)  # база σ метрики (L2), м
        self.declare_parameter("sigma_metric_ip", 8.0)     # σ метрики (IP, косинус), м
        self.declare_parameter("metric_conf_scale", 5.0)   # затухание conf по дист. (L2), м

        period = float(self.get_parameter("period_s").value)

        self.bridge = CvBridge()
        self.last_image = None

        self.matcher = SceneMatcher(
            map_path=self.get_parameter("map_path").value,
            device=self.get_parameter("device").value,
            model_name=self.get_parameter("model_name").value,
            min_score=self.get_parameter("min_score").value,
            max_dist=self.get_parameter("max_dist").value,
            logger=self.get_logger(),
        )

        self.create_subscription(Image, "/image_color", self._on_image, 1)
        self.pub_scene = self.create_publisher(String, "/nn2/scene", 10)
        self.pub_reloc = self.create_publisher(String, "/nn2/relocalization", 10)
        self.create_timer(period, self._on_tick)
        self.get_logger().info(f"NN2 (scene) DINOv2+FAISS, период {period} с")

    def _on_image(self, msg):
        self.last_image = msg   # держим только последний кадр (QoS depth=1)

    def _on_tick(self):
        if self.last_image is None:
            return

        frame = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
        match = self.matcher.query(frame)

        if match is None:
            # Локализация не уверена — чистим баннер, релокализацию не шлём.
            self.pub_scene.publish(String(data="unknown"))
            return

        # Баннер оператору (контракт openhd_streamer).
        self.pub_scene.publish(String(data=match.label))

        # Найденная поза места -> relocalizer. Штамп берём от кадра.
        e = match.entry
        stamp = self.last_image.header.stamp
        payload = {
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
            "scene_id": match.scene_id,
            "label": match.label,
            "score": match.score,
            "x": e.get("x"), "y": e.get("y"), "z": e.get("z"),     # ENU-метры (VINS-мир)
            "qx": e.get("qx"), "qy": e.get("qy"),
            "qz": e.get("qz"), "qw": e.get("qw"),                  # кватернион IMU
        }
        payload.update(self._metric_block(match))                 # XVIII: mx,my,mstd,mconf
        self.pub_reloc.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"NN2: место '{match.label}' (id={match.scene_id}, "
            f"score={match.score:.3f}, mconf={payload['mconf']:.2f}) -> relocalizer")

    def _metric_block(self, match):
        """МЕТР-БЛОК (XVIII): позиция места (ENU-метры) + σ + conf метрической опоры.
        L2-карта (MLP-топограф): match.score = дистанция в МЕТРАХ -> σ=floor+d,
        conf=exp(-d/scale). IP-карта (косинус): метров нет -> σ фикс, conf —
        насколько косинус выше порога min_score. Кадр в той же VINS-рамке, что и
        route-центрлиния, поэтому позы СЛИВАЕМЫ (route_fusion)."""
        ex, ey = match.entry.get("x"), match.entry.get("y")
        if self.matcher.encoder.metric == "l2":
            d = float(match.score)                                # метры
            floor = float(self.get_parameter("sigma_metric_floor").value)
            scale = float(self.get_parameter("metric_conf_scale").value)
            mstd = floor + d
            mconf = math.exp(-d / scale) if scale > 0 else 0.0
        else:                                                     # ip (косинус)
            cos = float(match.score)
            lo = float(self.get_parameter("min_score").value)
            mstd = float(self.get_parameter("sigma_metric_ip").value)
            mconf = max(0.0, min(1.0, (cos - lo) / (1.0 - lo + 1e-9)))
        return {"mx": float(ex), "my": float(ey),
                "mstd": float(mstd), "mconf": float(mconf)}


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
