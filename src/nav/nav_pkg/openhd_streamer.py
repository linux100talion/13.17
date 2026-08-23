#!/usr/bin/env python3
# ============================================================================
# openhd_streamer — сборка даунлинк-видео для OpenHD (вариант 2 архитектуры).
#
# Камера-нода (camera_pkg) БОЛЬШЕ НЕ кодирует поток сама, а публикует
# /image_color (bgr8, полный кадр 1280x720, каждый кадр). Эта нода:
#   - держит видео на ПОЛНОМ fps (детекции нейросетей редкие — NN1 ~1 Гц,
#     NN2 ~раз в 3 с, см. CLAUDE.md), поэтому от инференса fps не зависит;
#   - кэширует ПОСЛЕДНИЕ детекции/семантику от обеих нейросетей;
#   - рисует их на КАЖДОМ кадре (cv2.rectangle/putText) — рамки «залипают»
#     между обновлениями, для FPV-даунлинка это нормально;
#   - кодирует H.264 и шлёт по UDP на host:port (по умолчанию 127.0.0.1:5600).
#
# Нейросети публикуют только геометрию/семантику (килобайты), пиксели не гоняют:
#   /nn1/detections  vision_msgs/Detection2DArray  — якорные ориентиры (NN1)
#   /nn2/scene       std_msgs/String               — метка сцены (NN2)
#
# Рамки от NN1 заданы в координатах ПОЛНОГО кадра; оверлей рисуется на полном
# кадре, и только потом картинка ужимается до out_width x out_height — поэтому
# масштабировать боксы вручную не нужно.
#
# ── Debug-HUD (параметр hud, default true) ──────────────────────────────────
# Мотив: полёт lv1_joy_20260822_232043 — LOITER не залатчился, потому что VINS
# инициализировался только ПОСЛЕ посадки; пилот щёлкал CH6 вслепую, гейт молча
# держал. HUD показывает пилоту состояние стека прямо в FPV:
#   - баннер гейта (зел/жёлт/крас) — из /mission/status ЛЁТНОЙ НОДЫ. Это ТОТ ЖЕ
#     гейт, которым она пускает штатный LOITER (extnav_ready + свежий VINS + в
#     воздухе), а не параллельная оценка стримера. Нет свежего статуса (нода не
#     запущена — напр. голый стример на Orin) → баннер не рисуется вовсе;
#   - ODO: Гц + возраст /odometry — СОБСТВЕННАЯ оценка живости VINS (публикуется
#     только после init → зелёный ODO = «VINS инициализировался и жив»);
#   - FEAT: счётчик фич feature_tracker (в симе топик remap'нут в /feature, на
#     борту /feature_tracker/feature — подписка на оба, издатель ровно один);
#   - режим+armed из /mavros/state; демпферы (PWM-смещения) из /flow_dbg*;
#   - DRIFT: норма поправки NN1 (/nn1/drift), засечки редкие → показываем возраст.
# Сама отрисовка и пороги — в nav_pkg/hud_renderer.py (БЕЗ ROS): тем же кодом
# hud_video.py (src/lab/) пост-рендерит scene_hud.mp4 из bag. Здесь — только
# подписки, кормящие рендерер часами ноды (sim в симе, wall на борту) →
# возрасты RTF-независимы. Кэш + отрисовка на каждом кадре, fps не гейтится.
# ============================================================================
import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from nav_pkg.hud_renderer import HUD_SCENE, HudRenderer

# mavros_msgs в наших образах есть (MAVROS живёт в том же контейнере), но
# стримеру он не обязателен: без него HUD просто не рисует строку режима.
try:
    from mavros_msgs.msg import State
except ImportError:
    State = None

FONT = cv2.FONT_HERSHEY_SIMPLEX


class OpenHDStreamer(Node):
    def __init__(self):
        super().__init__("openhd_streamer")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5600)
        self.declare_parameter("bitrate", 4000)
        self.declare_parameter("out_width", 640)
        self.declare_parameter("out_height", 360)
        self.declare_parameter("hud", True)     # debug-HUD зрелости VINS/фич/осей
        # точки фич трекера на кадре (зелёные): видно, за что цепляется VINS
        self.declare_parameter("hud_features", True)

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        bitrate = int(self.get_parameter("bitrate").value)
        self.ow = int(self.get_parameter("out_width").value)
        self.oh = int(self.get_parameter("out_height").value)
        self.hud = bool(self.get_parameter("hud").value)
        self.hud_features = bool(self.get_parameter("hud_features").value)

        self.bridge = CvBridge()
        self.last_detections = None   # vision_msgs/Detection2DArray
        self.renderer = HudRenderer()

        pipeline = (
            "appsrc ! videoconvert ! "
            f"openh264enc bitrate={bitrate * 1000} ! "
            # config-interval=1: SPS/PPS каждую секунду, иначе зритель, подключившийся
            # ПОСРЕДИ потока (make fpv, наземка после взлёта), не сможет декодировать.
            f"rtph264pay config-interval=1 ! udpsink host={host} port={port} sync=false"
        )
        self.writer = cv2.VideoWriter(
            pipeline, cv2.CAP_GSTREAMER, 0, 15.0, (self.ow, self.oh), True
        )
        if not self.writer.isOpened():
            self.get_logger().error("Не удалось открыть GStreamer для OpenHD!")
        else:
            self.get_logger().info(f"OpenHD H.264 поток запущен на {host}:{port}")

        self.create_subscription(Image, "/image_color", self.on_image, 10)
        self.create_subscription(Detection2DArray, "/nn1/detections", self.on_nn1, 10)
        self.create_subscription(String, "/nn2/scene", self.on_nn2, 10)
        if self.hud:
            self.create_subscription(String, "/mission/status", self.on_status, 10)
            self.create_subscription(Odometry, "/odometry", self.on_odom, 10)
            self.create_subscription(PointCloud, "/feature", self.on_feat, 10)
            self.create_subscription(PointCloud, "/feature_tracker/feature",
                                     self.on_feat, 10)
            if State is not None:
                self.create_subscription(State, "/mavros/state", self.on_state, 10)
            self.create_subscription(Vector3Stamped, "/flow_dbg",
                                     self.on_dbg_roll, 10)
            self.create_subscription(Vector3Stamped, "/flow_dbg2",
                                     self.on_dbg_pitch, 10)
            self.create_subscription(Vector3Stamped, "/nn1/drift",
                                     self.on_drift, 10)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_nn1(self, msg):
        self.last_detections = msg

    def on_nn2(self, msg):
        self.renderer.set_scene(msg.data)

    def on_status(self, msg):
        self.renderer.set_status(msg.data, self._now())

    def on_odom(self, _msg):
        self.renderer.add_odom(self._now())

    def on_feat(self, msg):
        # каналы feature_tracker: [id, u, v, vx, vy]; u,v — ПИКСЕЛИ кадра
        # камеры (наш кадр /image_color той же величины → координаты 1:1)
        pts = None
        if self.hud_features and len(msg.channels) >= 3:
            pts = list(zip(msg.channels[1].values, msg.channels[2].values))
        self.renderer.set_feat(len(msg.points), self._now(), pts)

    def on_state(self, msg):
        self.renderer.set_state(msg.mode, msg.armed, self._now())

    def on_dbg_roll(self, msg):
        self.renderer.set_cmd_roll(msg.vector.x, self._now())

    def on_dbg_pitch(self, msg):
        self.renderer.set_cmd_pitch(msg.vector.x)

    def on_drift(self, msg):
        v = msg.vector
        self.renderer.set_drift(v.x, v.y, v.z, self._now())

    def on_image(self, msg):
        if not self.writer.isOpened():
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._draw_overlays(frame)
        out = cv2.resize(frame, (self.ow, self.oh))
        self.writer.write(out)

    def _draw_overlays(self, frame):
        # NN1: рамки якорных ориентиров (зелёные).
        if self.last_detections is not None:
            for det in self.last_detections.detections:
                bb = det.bbox
                cx, cy = bb.center.position.x, bb.center.position.y
                x1, y1 = int(cx - bb.size_x / 2), int(cy - bb.size_y / 2)
                x2, y2 = int(cx + bb.size_x / 2), int(cy + bb.size_y / 2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = det.results[0].hypothesis.class_id if det.results else ""
                if label:
                    cv2.putText(frame, label, (x1, max(0, y1 - 6)),
                                FONT, 0.6, (0, 255, 0), 2)
        if self.hud:
            self.renderer.draw(frame, self._now())
        elif self.renderer.scene:
            # HUD выключен — прежний одинокий баннер сцены (жёлтый).
            cv2.putText(frame, f"scene: {self.renderer.scene}", (10, 30),
                        FONT, 0.8, HUD_SCENE, 2)


def main(args=None):
    rclpy.init(args=args)
    node = OpenHDStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.writer.isOpened():
            node.writer.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
