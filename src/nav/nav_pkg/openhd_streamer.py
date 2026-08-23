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
# Всё рисуется по той же схеме, что рамки NN: кэш + putText на каждом кадре,
# fps видео от частоты источников не зависит; протухшие источники гаснут сами.
# Возрасты меряются часами ноды (sim-время в симе, wall на борту) — пороги
# одинаково честны при любом RTF.
# ============================================================================
import collections

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

# mavros_msgs в наших образах есть (MAVROS живёт в том же контейнере), но
# стримеру он не обязателен: без него HUD просто не рисует строку режима.
try:
    from mavros_msgs.msg import State
except ImportError:
    State = None

FONT = cv2.FONT_HERSHEY_SIMPLEX
# BGR-палитра HUD; баннер гейта заливается цветом состояния, текст на нём чёрный
HUD_GREEN = (60, 200, 60)
HUD_YELLOW = (0, 210, 240)
HUD_RED = (50, 50, 230)
HUD_WHITE = (235, 235, 235)
HUD_SCENE = (0, 255, 255)


class OpenHDStreamer(Node):
    def __init__(self):
        super().__init__("openhd_streamer")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5600)
        self.declare_parameter("bitrate", 4000)
        self.declare_parameter("out_width", 640)
        self.declare_parameter("out_height", 360)
        self.declare_parameter("hud", True)     # debug-HUD зрелости VINS/фич/осей

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        bitrate = int(self.get_parameter("bitrate").value)
        self.ow = int(self.get_parameter("out_width").value)
        self.oh = int(self.get_parameter("out_height").value)
        self.hud = bool(self.get_parameter("hud").value)

        self.bridge = CvBridge()
        self.last_detections = None   # vision_msgs/Detection2DArray
        self.last_scene = ""          # str

        # --- кэши HUD: (значение, время прихода по нашим часам) ---
        self.status = {}              # разобранный /mission/status (k=v)
        self.status_t = None
        self.odom_times = collections.deque(maxlen=64)   # приходы /odometry
        self.feat_n, self.feat_t = 0, None
        self.fcu_mode, self.fcu_armed, self.fcu_t = "", False, None
        self.cmd_roll, self.cmd_pitch, self.cmd_t = 0.0, 0.0, None
        self.drift = None             # (норма поправки м, время прихода)

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
        self.last_scene = msg.data

    def on_status(self, msg):
        self.status = dict(p.split("=", 1) for p in msg.data.split() if "=" in p)
        self.status_t = self._now()

    def on_odom(self, _msg):
        self.odom_times.append(self._now())

    def on_feat(self, msg):
        self.feat_n, self.feat_t = len(msg.points), self._now()

    def on_state(self, msg):
        self.fcu_mode, self.fcu_armed, self.fcu_t = msg.mode, msg.armed, self._now()

    def on_dbg_roll(self, msg):
        # /flow_dbg: vector.x = PWM-смещение крена (rc.roll − центр) от стека
        self.cmd_roll, self.cmd_t = msg.vector.x, self._now()

    def on_dbg_pitch(self, msg):
        self.cmd_pitch = msg.vector.x

    def on_drift(self, msg):
        v = msg.vector
        self.drift = ((v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5, self._now())

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
            self._draw_hud(frame)
        elif self.last_scene:
            # HUD выключен — прежний одинокий баннер сцены (жёлтый).
            cv2.putText(frame, f"scene: {self.last_scene}", (10, 30),
                        FONT, 0.8, HUD_SCENE, 2)

    def _hud_line(self, frame, y, text, color, scale=0.8, fill=None):
        """Строка HUD на подложке (читаемость поверх любой сцены); вернёт next y."""
        (tw, th), base = cv2.getTextSize(text, FONT, scale, 2)
        x = 10
        cv2.rectangle(frame, (x - 6, y - th - 6), (x + tw + 6, y + base + 4),
                      (0, 0, 0) if fill is None else fill, -1)
        cv2.putText(frame, text, (x, y), FONT, scale,
                    color if fill is None else (0, 0, 0), 2)
        return y + th + base + 18

    def _draw_hud(self, frame):
        now = self._now()
        y = 34
        # 1) баннер гейта — правда лётной ноды, тухнет за 3 с без /mission/status
        if self.status_t is not None and now - self.status_t < 3.0:
            st = self.status.get("st", "")
            why = self.status.get("why", "-")
            if st == "READY":
                y = self._hud_line(frame, y, "VINS READY", HUD_GREEN,
                                   scale=1.0, fill=HUD_GREEN)
            elif st == "WAIT":
                y = self._hud_line(frame, y, f"VINS WAIT ({why})", HUD_YELLOW,
                                   scale=1.0, fill=HUD_YELLOW)
            else:
                y = self._hud_line(frame, y, f"NO VINS ({why})", HUD_RED,
                                   scale=1.0, fill=HUD_RED)
        # 2) режим FCU + armed
        if self.fcu_t is not None and now - self.fcu_t < 5.0:
            arm = "ARM" if self.fcu_armed else "DISARM"
            y = self._hud_line(frame, y, f"{self.fcu_mode} {arm}", HUD_WHITE)
        # 3) /odometry: Гц (окно 3 с) + возраст; красный ODO -- = VINS без init
        if self.odom_times:
            age = now - self.odom_times[-1]
            win = [t for t in self.odom_times if now - t < 3.0]
            hz = ((len(win) - 1) / (win[-1] - win[0])
                  if len(win) >= 2 and win[-1] > win[0] else 0.0)
            col = (HUD_GREEN if age < 0.5 else
                   HUD_YELLOW if age < 1.5 else HUD_RED)
            y = self._hud_line(frame, y, f"ODO {hz:4.1f}Hz {age:4.1f}s", col)
        else:
            y = self._hud_line(frame, y, "ODO --", HUD_RED)
        # 4) фичи трекера: замолк при живой камере — это ЧП, красним
        if self.feat_t is not None:
            if now - self.feat_t < 3.0:
                y = self._hud_line(frame, y, f"FEAT {self.feat_n}", HUD_WHITE)
            else:
                y = self._hud_line(frame, y, "FEAT --", HUD_RED)
        # 5) PWM-смещения крена/тангажа от стека (/flow_dbg, /flow_dbg2)
        if self.cmd_t is not None and now - self.cmd_t < 2.0:
            y = self._hud_line(frame, y,
                               f"CMD R{self.cmd_roll:+04.0f} "
                               f"P{self.cmd_pitch:+04.0f}", HUD_WHITE)
        # 6) поправка NN1: засечки редкие, старше 10 с — показываем возраст
        if self.drift is not None:
            d, t = self.drift
            age = now - t
            txt = f"DRIFT {d:.2f}m" + (f" ({age:.0f}s)" if age > 10.0 else "")
            y = self._hud_line(frame, y, txt, HUD_WHITE)
        # 7) семантика сцены NN2 (бывший одинокий баннер)
        if self.last_scene:
            self._hud_line(frame, y, f"scene: {self.last_scene}", HUD_SCENE)


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
