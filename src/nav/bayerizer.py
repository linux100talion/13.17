#!/usr/bin/env python3
# ============================================================================
# bayerizer.py — мост Gazebo → виртуальная Bayer-камера.
#
# Берёт RGB-кадр из Gazebo (/camera/image_raw), "портит" его до сырого
# Bayer-мозаика (как делает реальный сенсор), пакует в 16-бит и пишет в
# /dev/rawbayer (устройство v4l2loopback). Камера-нода (camera_pkg) читает
# оттуда штатным v4l2-ctl и дебайеризует обратно на CUDA — то есть работает
# В СИМУЛЯЦИИ "КАК ЕСТЬ", сменив только параметр device:=/dev/rawbayer.
#
#   Gazebo RGB --> [этот скрипт: RGB→Bayer16] --> /dev/rawbayer (v4l2loopback)
#                                              --> camera_node (CUDA debayer)
#
# Запуск (в контейнере nav, после настройки v4l2loopback на хосте — см. README):
#   python3 bayerizer.py --ros-args \
#       -p input_topic:=/camera/image_raw -p device:=/dev/rawbayer -p pattern:=GRBG
#
# ВАЖНО: разрешение Gazebo-камеры должно совпадать с тем, что ждёт camera_node
# (по умолчанию 1280x720). Иначе размеры кадров не сойдутся.
# ============================================================================
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Bayerizer(Node):
    def __init__(self):
        super().__init__("bayerizer")
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("device", "/dev/rawbayer")
        # Паттерн мозаика. GRBG соответствует реальному ArduCam (BA10/SGRBG10),
        # который camera_node дебайеризует через COLOR_BayerGB2RGB.
        # Если в симуляции цвета перепутаны — поменяй на RGGB / BGGR / GBRG.
        self.declare_parameter("pattern", "GRBG")

        self.topic = self.get_parameter("input_topic").value
        self.dev_path = self.get_parameter("device").value
        self.pattern = self.get_parameter("pattern").value.upper()
        self.fd = None
        self.warned_size = False

        self.sub = self.create_subscription(Image, self.topic, self.on_image, 10)
        self.get_logger().info(
            f"bayerizer: {self.topic} (RGB) -> {self.dev_path} (Bayer16, {self.pattern})"
        )

    def _open_device(self):
        # O_WRONLY: пишем кадры как producer в v4l2loopback.
        self.fd = os.open(self.dev_path, os.O_WRONLY)
        self.get_logger().info(f"Открыто устройство вывода: {self.dev_path}")

    def _split_rgb(self, msg):
        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)
        if msg.encoding == "bgr8":
            b, g, r = arr[..., 0], arr[..., 1], arr[..., 2]
        elif msg.encoding in ("rgb8", "rgba8"):
            r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
        else:
            # неизвестная кодировка — считаем rgb
            r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
        return r, g, b

    def _mosaic(self, r, g, b):
        """Собирает однослойный Bayer-мозаик по заданному паттерну.

        Сетка 2x2 (верх-лево, верх-право, низ-лево, низ-право):
          GRBG: G R / B G    RGGB: R G / G B
          BGGR: B G / G R    GBRG: G B / R G
        """
        h, w = g.shape
        bayer = np.empty((h, w), dtype=np.uint8)
        # карта: (позиция в сетке) -> канал
        layout = {
            "GRBG": (g, r, b, g),
            "RGGB": (r, g, g, b),
            "BGGR": (b, g, g, r),
            "GBRG": (g, b, r, g),
        }
        tl, tr, bl, br = layout.get(self.pattern, layout["GRBG"])
        bayer[0::2, 0::2] = tl[0::2, 0::2]
        bayer[0::2, 1::2] = tr[0::2, 1::2]
        bayer[1::2, 0::2] = bl[1::2, 0::2]
        bayer[1::2, 1::2] = br[1::2, 1::2]
        return bayer

    def on_image(self, msg):
        try:
            r, g, b = self._split_rgb(msg)
            bayer8 = self._mosaic(r, g, b)
            # camera_node читает кадр как CV_16UC1 и делает NORM_MINMAX 16->8,
            # поэтому достаточно положить 8-бит значение в младший байт uint16.
            # tobytes() на x86 — little-endian, как и ожидает CV_16UC1.
            bayer16 = bayer8.astype(np.uint16)
            if self.fd is None:
                self._open_device()
            os.write(self.fd, bayer16.tobytes())
        except BrokenPipeError:
            # consumer (camera_node) отвалился — переоткроем при следующем кадре
            self.get_logger().warn("Потребитель отключился, переоткрываю устройство")
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
        except Exception as e:  # noqa: BLE001
            if not self.warned_size:
                self.get_logger().error(f"Ошибка записи кадра: {e}")
                self.warned_size = True

    def destroy_node(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Bayerizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
