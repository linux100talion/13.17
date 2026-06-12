import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import subprocess
import numpy as np
import cv2
import re
import os
import time

class V4L2PipeNode(Node):
    def __init__(self):
        super().__init__('raw_camera_node')

        # Устанавливаем целевое разрешение
        self.width = 1280
        self.height = 720
        
        # Публикаторы для изображения и калибровочных данных
        self.publisher_ = self.create_publisher(Image, '/image_mono', 10)
        self.info_publisher_ = self.create_publisher(CameraInfo, '/camera_info', 10)
        
        self.bridge = CvBridge()
        
        # --- ИНИЦИАЛИЗАЦИЯ CAMERA INFO ПОД ТВОЙ КОНФИГ ---

        # Плагин Image в RViz2 просто берет матрицу пикселей и рисует её на экране. 
        # Ему плевать на физику.
        # Плагин Camera работает умнее: он смотрит в TF-дерево, 
        # находит, где физически висит камера на дроне, и проецирует пиксели 
        # в 3D-пространство. Для этого ему критически нужны интринсики объектива 
        # (фокусное расстояние, оптический центр, искажения линзы). 
        # Вся эта геометрия летит именно в сообщениях типа 
        # sensor_msgs/msg/CameraInfo.

        # В VINS-Mono эти данные берутся из твоего dummy_13_7.yaml, 
        # поэтому одометрия работает. 
        # Но чтобы RViz2 тоже "прозрел", нам нужно добавить в наш Python-скрипт 
        # публикатор CameraInfo, 
        # который будет забирать интринсики из этого же yaml-файла 
        # (или просто захардкодить их, раз у нас фикс) и отправлять их синхронно 
        # с каждым кадром.

        # # Предзаполняем статические параметры калибровки
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_frame"
        self.camera_info_msg.width = self.width
        self.camera_info_msg.height = self.height

        # Модель дисторсии (обычно plumb_bob для стандартных линз)
        self.camera_info_msg.distortion_model = "plumb_bob"
        
        # D: Коэффициенты дисторсии (оставляем нулевыми для безопасного старта)
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # K: Матрица камеры (рассчитана математически)
        self.camera_info_msg.k = [
            640.0,   0.0, 640.0,
              0.0, 640.0, 360.0,
              0.0,   0.0,   1.0
        ]
        
        # R: Единичная матрица (без изменений)
        self.camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # P: Матрица проекции (совпадает с K)
        self.camera_info_msg.p = [
            640.0,   0.0, 640.0, 0.0,
              0.0, 640.0, 360.0, 0.0,
              0.0,   0.0,   1.0, 0.0
        ]


        try:
            env = os.environ.copy()
            env['LC_ALL'] = 'C'
            
            # ШАГ 1: Жестко задаем формат и разрешение
            self.get_logger().info("Настройка формата и разрешения...")
            cmd_fmt = [
                'v4l2-ctl', '-d', '/dev/video0',
                f'--set-fmt-video=width={self.width},height={self.height},pixelformat=BA10'
            ]
            subprocess.check_output(cmd_fmt, stderr=subprocess.STDOUT, env=env)
            
            # ШАГ 2: Явно запрашиваем примененный формат, чтобы узнать точный Sizeimage
            self.get_logger().info("Запрос геометрии кадра...")
            cmd_get_size = [
                'v4l2-ctl', '-d', '/dev/video0',
                '--get-fmt-video'
            ]
            output = subprocess.check_output(cmd_get_size, stderr=subprocess.STDOUT, env=env).decode()
            
            match = re.search(r'Size\s*image\s*:\s*(\d+)', output, re.IGNORECASE)
            if not match:
                self.get_logger().error(f"Не удалось определить Sizeimage! Полный вывод:\n{output}")
                return
            
            self.frame_size = int(match.group(1))
            self.get_logger().info(f"Разрешение подготовлено. Размер кадра: {self.frame_size} байт")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка предварительной настройки: {e}")
            return

        # ШАГ 3: ЗАПУСКАЕМ СТРИМ (Открываем устройство)
        cmd_stream = [
            'v4l2-ctl', '-d', '/dev/video0',
            '--stream-mmap', '--stream-to=-'
        ]
        self.process = subprocess.Popen(cmd_stream, stdout=subprocess.PIPE, bufsize=10**7)
        self.get_logger().info("Аппаратный поток V4L2 запущен. Ожидание стабилизации конвейера...")

        # ШАГ 4: Ждем, пока драйвер завершит запуск конвейера STREAMON
        time.sleep(0.5)

        # ШАГ 5: Применяем настройки НАЖИВУЮ в уже запущенный поток
        try:
            self.get_logger().info("Применение настроек экспозиции, FPS и Gain в активный поток...")
            cmd_runtime_ctrls = [
                'v4l2-ctl', '-d', '/dev/video0',
                '-c', 'frame_rate=30',
                '-c', 'analogue_gain=1200',
                '-c', 'exposure=5250'
            ]
            subprocess.check_output(cmd_runtime_ctrls, stderr=subprocess.STDOUT, env=env)
            self.get_logger().info("Настройки успешно зафиксированы в регистрах матрицы!")
        except Exception as e:
            self.get_logger().error(f"Не удалось применить runtime-настройки: {e}")

        # Запускаем таймер чтения кадров
        self.timer = self.create_timer(0.001, self.read_frame)




    def read_frame(self):
        raw_data = self.process.stdout.read(self.frame_size)
        if not raw_data or len(raw_data) != self.frame_size:
            return

        try:
            # Распаковка данных с проверкой размера буфера
            if self.frame_size >= self.width * self.height * 2:
                img16 = np.frombuffer(raw_data, dtype=np.uint16).reshape((-1, self.width))
                img16 = img16[:self.height, :] 
                img8 = cv2.normalize(img16, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            else:
                data = np.frombuffer(raw_data, dtype=np.uint8)
                img8 = data.reshape((-1, 5))[:, :4].reshape((self.height, self.width))

            img8_contiguous = np.ascontiguousarray(img8)
            
            if img8_contiguous.shape != (self.height, self.width):
                self.get_logger().error(f"Неверная форма: {img8_contiguous.shape}")
                return

            # Преобразование Байера в монохром
            img_clean = cv2.cvtColor(img8_contiguous, cv2.COLOR_BayerGR2GRAY)
            
            # Генерация строго единой временной метки для синхронизации топиков
            current_time = self.get_clock().now().to_msg()
            
            # Публикация кадра
            msg = self.bridge.cv2_to_imgmsg(img_clean, encoding="mono8")
            msg.header.stamp = current_time
            msg.header.frame_id = "camera_frame"
            self.publisher_.publish(msg)

            # Публикация калибровочных данных
            self.camera_info_msg.header.stamp = current_time
            self.info_publisher_.publish(self.camera_info_msg)
            
        except Exception as e:
            self.get_logger().error(f"Сбой OpenCV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = V4L2PipeNode()
    if hasattr(node, 'frame_size'):
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()