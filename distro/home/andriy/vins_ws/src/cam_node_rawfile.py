import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import numpy as np
import cv2
import os

class V4L2PipeNode(Node):
    def __init__(self):
        super().__init__('raw_camera_node')
        self.publisher_ = self.create_publisher(Image, '/image_mono', 10)
        self.bridge = CvBridge()
        
        #self.width = 1280
        #self.height = 720

        self.width = 1920
        self.height = 1200

        # Автоматически определяем размер кадра по твоему файлу
        try:
            self.frame_size = os.path.getsize('frame.raw')
            self.get_logger().info(f"Размер сырого кадра: {self.frame_size} байт")
        except Exception as e:
            self.get_logger().error("Файл frame.raw не найден! Запусти v4l2-ctl команду для одного кадра.")
            return

        # Запускаем железобетонный захват через консоль напрямую в наш скрипт
        cmd = [
            'v4l2-ctl', '-d', '/dev/video0',
            f'--set-fmt-video=width={self.width},height={self.height},pixelformat=BA10',
            '--stream-mmap', '--stream-to=-'
        ]
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**7)
        self.get_logger().info("Аппаратный поток V4L2 открыт. Публикую в /image_mono...")
        
        # Читаем кадры без остановки
        self.timer = self.create_timer(0.001, self.read_frame)

    def read_frame(self):
        raw_data = self.process.stdout.read(self.frame_size)
        if not raw_data or len(raw_data) != self.frame_size:
            return

        # Декодируем в зависимости от того, как драйвер упаковал BA10
        if self.frame_size == self.width * self.height * 2:
            # 16-bit unpacked (2 байта на пиксель)
            img16 = np.frombuffer(raw_data, dtype=np.uint16).reshape((self.height, self.width))
            img8 = (img16 >> 2).astype(np.uint8)
            
        elif self.frame_size == int(self.width * self.height * 1.25):
            # MIPI packed (5 байт на 4 пикселя)
            # Гениальный хак: берем только первые 4 байта из каждой пятерки (это старшие 8 бит)
            data = np.frombuffer(raw_data, dtype=np.uint8)
            img8 = data.reshape((-1, 5))[:, :4].reshape((self.height, self.width))
            
        else:
            self.get_logger().error(f"Неизвестный размер упаковки: {self.frame_size}")
            return

        # Отправляем в VINS-Mono!
        img_clean = cv2.cvtColor(img8, cv2.COLOR_BayerGR2GRAY)
        # (Или BG2GRAY, если цвета инвертированы)
        msg = self.bridge.cv2_to_imgmsg(img_clean, encoding="mono8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = V4L2PipeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()