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
# v4l2loopback поддерживает только BGR4 (BGRA32, 4 байта/пиксель).
# Мы используем его как транспорт для 16-битных Bayer-данных:
#   - каждый кадр = bayer16 (width*height*2 байт) + нули до sizeimage байт
#   - camera_node проверяет frame_size_ >= w*h*2, берёт первые w*h*2 байт
#     как CV_16UC1 и дальше дебайеризует на CUDA.
# ============================================================================
import ctypes
import fcntl
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


# ---------------------------------------------------------------------------
# v4l2_format / v4l2_pix_format — с правильным выравниванием x86_64.
# sizeof(v4l2_format) = 208 байт: 4 (type) + 4 (padding) + 200 (union).
# Padding возникает потому, что union содержит v4l2_window.clips (pointer),
# который требует 8-байтового выравнивания.
# ---------------------------------------------------------------------------
class _V4L2PixFmt(ctypes.Structure):
    _fields_ = [
        ('width',        ctypes.c_uint32),
        ('height',       ctypes.c_uint32),
        ('pixelformat',  ctypes.c_uint32),
        ('field',        ctypes.c_uint32),
        ('bytesperline', ctypes.c_uint32),
        ('sizeimage',    ctypes.c_uint32),
        ('colorspace',   ctypes.c_uint32),
        ('priv',         ctypes.c_uint32),
        ('flags',        ctypes.c_uint32),
        ('ycbcr_enc',    ctypes.c_uint32),
        ('quantization', ctypes.c_uint32),
        ('xfer_func',    ctypes.c_uint32),
    ]


class _V4L2FmtUnion(ctypes.Union):
    _fields_ = [
        ('pix',      _V4L2PixFmt),
        ('raw_data', ctypes.c_uint8 * 200),
        ('_align',   ctypes.c_uint64),   # принудительное 8-байт выравнивание
    ]


class _V4L2Format(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_uint32),
        ('fmt',  _V4L2FmtUnion),
    ]


_VIDIOC_S_FMT           = 0xC0D05605   # _IOWR('V', 5, 208)
_V4L2_BUF_TYPE_VID_OUT  = 2
_V4L2_FIELD_NONE        = 1
_V4L2_COLORSPACE_SRGB   = 1
_FOURCC_BGR4 = ord('B') | (ord('G') << 8) | (ord('R') << 16) | (ord('4') << 24)


class Bayerizer(Node):
    def __init__(self):
        super().__init__('bayerizer')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('device', '/dev/rawbayer')
        self.declare_parameter('pattern', 'GRBG')
        # Размеры должны совпадать с Gazebo-камерой (и camera_node: 1280×720).
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)

        self.topic    = self.get_parameter('input_topic').value
        self.dev_path = self.get_parameter('device').value
        self.pattern  = self.get_parameter('pattern').value.upper()
        self.width    = self.get_parameter('width').value
        self.height   = self.get_parameter('height').value

        self.fd        = None
        self.sizeimage = 0

        # Открываем устройство НЕМЕДЛЕННО и активируем capture-сторону.
        # v4l2loopback переходит в "ready_for_capture" только после первого
        # write() — до этого camera_node получит EINVAL на G_FMT(CAPTURE).
        # sim_nav.launch.py стартует camera_node с задержкой ~3 с.
        self._open_device()

        self.sub = self.create_subscription(Image, self.topic, self.on_image, 10)
        self.get_logger().info(
            f'bayerizer: {self.topic} → {self.dev_path} (BGR4/{self.pattern})'
        )

    def _open_device(self):
        try:
            self.fd = os.open(self.dev_path, os.O_RDWR)
        except OSError as e:
            self.get_logger().error(f'Не удалось открыть {self.dev_path}: {e}')
            return

        fmt = _V4L2Format()
        fmt.type                  = _V4L2_BUF_TYPE_VID_OUT
        fmt.fmt.pix.width         = self.width
        fmt.fmt.pix.height        = self.height
        fmt.fmt.pix.pixelformat   = _FOURCC_BGR4
        fmt.fmt.pix.field         = _V4L2_FIELD_NONE
        fmt.fmt.pix.bytesperline  = self.width * 4
        fmt.fmt.pix.sizeimage     = self.width * self.height * 4
        fmt.fmt.pix.colorspace    = _V4L2_COLORSPACE_SRGB

        try:
            fcntl.ioctl(self.fd, _VIDIOC_S_FMT, fmt)
        except OSError as e:
            self.get_logger().warn(f'VIDIOC_S_FMT: {e}')

        self.sizeimage = fmt.fmt.pix.sizeimage or (self.width * self.height * 4)
        bayer_size = self.width * self.height * 2

        # Один нулевой кадр: активирует ready_for_capture, camera_node
        # сможет читать формат через G_FMT(CAPTURE).
        try:
            os.write(self.fd, bytes(self.sizeimage))
        except OSError as e:
            self.get_logger().warn(f'Инициализирующий кадр: {e}')

        self.get_logger().info(
            f'Открыто {self.dev_path} ({self.width}x{self.height}) BGR4 '
            f'sizeimage={self.sizeimage} '
            f'(bayer16={bayer_size} + pad={self.sizeimage - bayer_size})'
        )

    def _split_rgb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'bgr8':
            b, g, r = arr[..., 0], arr[..., 1], arr[..., 2]
        elif msg.encoding in ('rgb8', 'rgba8'):
            r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
        else:
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
        layout = {
            'GRBG': (g, r, b, g),
            'RGGB': (r, g, g, b),
            'BGGR': (b, g, g, r),
            'GBRG': (g, b, r, g),
        }
        tl, tr, bl, br = layout.get(self.pattern, layout['GRBG'])
        bayer[0::2, 0::2] = tl[0::2, 0::2]
        bayer[0::2, 1::2] = tr[0::2, 1::2]
        bayer[1::2, 0::2] = bl[1::2, 0::2]
        bayer[1::2, 1::2] = br[1::2, 1::2]
        return bayer

    def on_image(self, msg):
        if self.fd is None:
            return
        try:
            r, g, b = self._split_rgb(msg)
            bayer8  = self._mosaic(r, g, b)
            # camera_node читает кадр как CV_16UC1 и делает NORM_MINMAX 16→8,
            # поэтому достаточно положить 8-bit значение в младший байт uint16.
            # tobytes() на x86 — little-endian, как и ожидает CV_16UC1.
            bayer16 = bayer8.astype(np.uint16)
            data = bayer16.tobytes()                          # width*height*2 байт
            if len(data) < self.sizeimage:
                data = data + bytes(self.sizeimage - len(data))  # zero-pad до sizeimage
            os.write(self.fd, data)
        except BrokenPipeError:
            self.get_logger().warn('Потребитель отключился, переоткрываю устройство')
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
            self._open_device()
        except Exception as e:
            self.get_logger().error(f'Ошибка: {e}', throttle_duration_sec=5.0)

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


if __name__ == '__main__':
    main()
