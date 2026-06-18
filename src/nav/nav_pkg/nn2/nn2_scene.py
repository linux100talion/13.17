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
#     МЕТР-БЛОК (XVIII): mx,my (метрическая позиция, ENU-метры) + анизотропная
#     ковариация mcxx,mcxy,mcyy (м²) + mstd (скаляр-σ, совместимость) + mconf +
#     msrc. Это метрическая опора φ-карты, которую relocalizer_field сливает с
#     route-позой (route_fusion.gaussian_fuse). Считается kNN-засечкой СО СТРАЖЕМ
#     алиасинга (SceneMatcher.metric_fix), а не top-1: позиция и ковариация — из
#     разброса k соседей; разбежались (алиасинг) -> фолбэк top-1, mconf сбит.
#     Тип String с JSON — осознанная заглушка («пока .json», см. идею); потом
#     заменим на типизированный msg (нужен отдельный CMake-интерфейс-пакет).
# ============================================================================
import json
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
        # Метр-блок (XVIII): kNN-засечка СО СТРАЖЕМ алиасинга вместо top-1.
        self.declare_parameter("knn_k", 6)                 # число соседей kNN
        self.declare_parameter("guard_radius", 8.0)        # порог компактности соседей, м
        self.declare_parameter("sigma_metric_floor", 1.0)  # пол σ ковариации, м
        self.declare_parameter("metric_conf_scale", 5.0)   # затухание conf по дист. (L2), м
        self.declare_parameter("guard_penalty", 0.3)       # множитель conf при срабатывании стража

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
        fix = self.matcher.metric_fix(
            frame,
            k=int(self.get_parameter("knn_k").value),
            guard_radius=float(self.get_parameter("guard_radius").value),
            sigma_floor=float(self.get_parameter("sigma_metric_floor").value),
            conf_scale=float(self.get_parameter("metric_conf_scale").value),
            guard_penalty=float(self.get_parameter("guard_penalty").value),
        )

        if fix is None:
            # Локализация не уверена — чистим баннер, релокализацию не шлём.
            self.pub_scene.publish(String(data="unknown"))
            return

        # Баннер оператору (контракт openhd_streamer) — по top-1.
        self.pub_scene.publish(String(data=fix.top1.label))

        # Найденная поза места -> relocalizer. Штамп берём от кадра. Позиция места и
        # ковариация — kNN-засечка со стражем (mx,my,mstd,mcxx/mcxy/mcyy,mconf).
        t = fix.top1
        e = t.entry
        cov = fix.cov
        stamp = self.last_image.header.stamp
        payload = {
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
            "scene_id": t.scene_id,
            "label": t.label,
            "score": t.score,
            "x": e.get("x"), "y": e.get("y"), "z": e.get("z"),     # top-1 (ENU-метры)
            "qx": e.get("qx"), "qy": e.get("qy"),
            "qz": e.get("qz"), "qw": e.get("qw"),                  # кватернион IMU
            # МЕТР-БЛОК XVIII: kNN-позиция + анизотропная ковариация (м²)
            "mx": float(fix.x), "my": float(fix.y),
            "mstd": float(fix.std), "mconf": float(fix.conf),
            "mcxx": float(cov[0, 0]), "mcxy": float(cov[0, 1]), "mcyy": float(cov[1, 1]),
            "msrc": fix.source,                                    # knn | top1-fallback
        }
        self.pub_reloc.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"NN2: место '{t.label}' (id={t.scene_id}, score={t.score:.3f}) "
            f"-> метрика [{fix.source}] mconf={fix.conf:.2f}, разброс соседей "
            f"{fix.spread:.1f} м -> relocalizer")


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
