import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import numpy as np
import cv2
import re
import os

class V4L2PipeNode(Node):
    def __init__(self):
        super().__init__('raw_camera_node')
        self.publisher_ = self.create_publisher(Image, '/image_mono', 10)
        self.bridge = CvBridge()
        
        # Возвращаем родное разрешение, которое VINS-Mono так отчаянно хочет видеть
        self.width = 1920
        self.height = 1200

        cmd_set = [
            'v4l2-ctl', '-d', '/dev/video0',
            f'--set-fmt-video=width={self.width},height={self.height},pixelformat=BA10',
            '-V'
        ]
        
        try:
            env = os.environ.copy()
            env['LC_ALL'] = 'C'
            
            output = subprocess.check_output(cmd_set, stderr=subprocess.STDOUT, env=env).decode()
            
            match = re.search(r'Size\s*image\s*:\s*(\d+)', output, re.IGNORECASE)
            if not match:
                self.get_logger().error(f"Не удалось определить Sizeimage! Полный вывод:\n{output}")
                return
            
            self.frame_size = int(match.group(1))
            self.get_logger().info(f"Разрешение: {self.width}x{self.height}. Точный размер кадра: {self.frame_size} байт")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка инициализации камеры: {e}")
            return

        cmd_stream = [
            'v4l2-ctl', '-d', '/dev/video0',
            '--stream-mmap', '--stream-to=-'
        ]
        self.process = subprocess.Popen(cmd_stream, stdout=subprocess.PIPE, bufsize=10**7)
        self.get_logger().info("Аппаратный поток V4L2 открыт. Публикую в /image_mono...")
        
        self.timer = self.create_timer(0.001, self.read_frame)

    def read_frame(self):
        raw_data = self.process.stdout.read(self.frame_size)
        if not raw_data or len(raw_data) != self.frame_size:
            return

        try:
            if self.frame_size >= self.width * self.height * 2:
                img16 = np.frombuffer(raw_data, dtype=np.uint16).reshape((-1, self.width))
                img16 = img16[:self.height, :] 
                img8 = (img16 >> 2).astype(np.uint8)
            else:
                data = np.frombuffer(raw_data, dtype=np.uint8)
                img8 = data.reshape((-1, 5))[:, :4].reshape((self.height, self.width))

            img8_contiguous = np.ascontiguousarray(img8)
            
            if img8_contiguous.shape != (self.height, self.width):
                self.get_logger().error(f"Неверная форма: {img8_contiguous.shape}")
                return

            # Тот самый правильный шаблон Байера
            img_clean = cv2.cvtColor(img8_contiguous, cv2.COLOR_BayerGR2GRAY)
            
            msg = self.bridge.cv2_to_imgmsg(img_clean, encoding="mono8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"

            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Сбой OpenCV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = V4L2PipeNode()
    if hasattr(node, 'frame_size'):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()