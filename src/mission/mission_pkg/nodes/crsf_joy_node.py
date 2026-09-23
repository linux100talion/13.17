#!/usr/bin/env python3
"""crsf_joy — мостик CRSF → /joy: живой пульт на борту, МИМО полётника.

Зачем. В симе стики приходят с TX12 по USB (`joy_linux_node` → `/joy`), а на борту
пульт воткнут в полётник — и пока лётная нода пишет `/mavros/rc/override`, в
телеметрии `RC_CHANNELS` лежит ЭХО нашей же команды, а не стики (разбор —
docker/sim/laptop_move.md §5.1–5.2). Поэтому намерение пилота заводится на борт
отдельным проводом: TX приёмника ОТВЕТВЛЁН на RX хедера Orin, а эта нода читает
CRSF и публикует ТОТ ЖЕ `/joy`, что `joy_linux_node`. Ниже `/joy` весь стек
(`JoyPilot` → лесенка → стабилизаторы) не меняется ни на строку.

Контракт `/joy` (зеркало EdgeTX-HID, см. docker/sim/rx.md):
  `axes[i]` = канал i+1 в НОРМИРОВКЕ PWM: v = (µs − 1500)/400, т.е. ±1 = полный ход.
  Публикуются все 16 каналов CRSF, поэтому CH8 на борту ЖИВОЙ — HID-квирк TX12
  (CH7 и CH8 дрались за `axes[6]`) остаётся болезнью симулятора, не борта.
  `buttons` — ПУСТОЙ: у CRSF всё каналы. Значит на борту органы задаются осями:
  `BS_LAND_JOY=a<i>`, `BS_RTH_JOY=a<i>` (`parse_land_src` это умеет, порог 0.5 =
  «µs > 1700»), а не `b<i>` как в симе.

⚠️ Знаки. Зеркальность roll/yaw/throttle в симе — артефакт USB-HID; здесь мы
отдаём каналы КАК ЕСТЬ, поэтому на борту начинать с `BS_JOY_SIGNS=1,1,1,1` и
сверять на земле (`joy_check.sh`). Правило репы прежнее: знак не выверен, пока
ось не отработала в воздухе.

⚠️ ТИШИНА — ЭТО СИГНАЛ. Пропал поток кадров (`stale_sec`) — нода ПЕРЕСТАЁТ
публиковать `/joy`, а не держит последнее значение: так сторож свежести лётной
ноды (`control_pkg/domain/pilot_link.py`, `BS_PILOT_STALE`) увидит тишину и
отпустит override — борт останется на физическом приёмнике. Держать последнее
здесь = молча сломать эту страховку.

Параметры: port (/dev/ttyTHS1), baud (420000), rate (Гц публикации, 50),
stale_sec (0.2), link_log_sec (период лога качества линка, 0 = молчать).
Запуск:  ros2 run mission_pkg crsf_joy --ros-args -p port:=/dev/ttyTHS1
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

from control_pkg.infrastructure.crsf import (CrsfParser, TYPE_LINK_STATS,
                                             TYPE_RC_CHANNELS, to_us,
                                             unpack_channels, unpack_link)

RC_CENTER, RC_FULL = 1500.0, 400.0     # нормировка axes: та же, что у JoyPilot


class CrsfJoy(Node):
    def __init__(self):
        super().__init__('crsf_joy')
        self.port = self.declare_parameter('port', '/dev/ttyTHS1').value
        self.baud = int(self.declare_parameter('baud', 420000).value)
        rate = float(self.declare_parameter('rate', 50.0).value)
        self.stale = float(self.declare_parameter('stale_sec', 0.2).value)
        self.link_log = float(self.declare_parameter('link_log_sec', 10.0).value)

        self.pub = self.create_publisher(Joy, '/joy', qos_profile_sensor_data)
        self.parser = CrsfParser()
        self.ser = None
        self.axes = None
        self.last_frame = 0.0
        self.last_open_try = 0.0
        self.last_link_log = 0.0
        self.link = {}
        self.silent = True              # чтобы «пульт появился» попало в лог один раз

        self.create_timer(0.005, self._read)        # 200 Гц: кадры идут ~197 Гц
        self.create_timer(1.0 / max(1.0, rate), self._publish)
        self.get_logger().info(
            f"crsf_joy: {self.port} @ {self.baud}, публикация {rate:g} Гц, "
            f"тишина > {self.stale:g} с = НЕ публикуем (сторож лётной ноды отпустит override)")

    # --- порт ---
    def _open(self):
        now = time.monotonic()
        if now - self.last_open_try < 2.0:          # не долбить порт каждые 5 мс
            return
        self.last_open_try = now
        try:
            import serial
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info(f"порт {self.port} открыт")
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"порт {self.port} не открылся: {e}", once=False)

    def _read(self):
        if self.ser is None:
            self._open()
            return
        try:
            chunk = self.ser.read(1024)
        except Exception as e:
            self.get_logger().error(f"чтение {self.port}: {e} — переоткрываю")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return
        for ftype, payload in self.parser.feed(chunk):
            if ftype == TYPE_RC_CHANNELS and len(payload) == 22:
                us = [to_us(v) for v in unpack_channels(payload)]
                self.axes = [(u - RC_CENTER) / RC_FULL for u in us]
                self.last_frame = time.monotonic()
            elif ftype == TYPE_LINK_STATS:
                self.link = unpack_link(payload)

    # --- публикация ---
    def _publish(self):
        now = time.monotonic()
        fresh = self.axes is not None and (now - self.last_frame) < self.stale
        if not fresh:
            if not self.silent:
                self.silent = True
                self.get_logger().error(
                    f"ПУЛЬТ ЗАМОЛЧАЛ ({now - self.last_frame:.1f} с без кадров) — "
                    f"/joy не публикуется; кадров ok={self.parser.ok} "
                    f"crc={self.parser.crc_bad} resync={self.parser.resync}")
            return
        if self.silent:
            self.silent = False
            self.get_logger().info(
                f"пульт жив: кадров {self.parser.ok}, битых CRC {self.parser.crc_bad}"
                + (f", LQ {self.link.get('lq')}%, RSSI {self.link.get('rssi')} дБм"
                   if self.link else ""))
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.axes = [float(a) for a in self.axes]
        m.buttons = []                  # у CRSF всё каналы: органы — через a<i>
        self.pub.publish(m)
        if self.link_log > 0 and now - self.last_link_log >= self.link_log:
            self.last_link_log = now
            self.get_logger().info(
                f"линк: LQ {self.link.get('lq', '--')}% RSSI {self.link.get('rssi', '--')} дБм "
                f"| кадров {self.parser.ok} crc {self.parser.crc_bad} resync {self.parser.resync}")


def main():
    rclpy.init()
    node = CrsfJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
