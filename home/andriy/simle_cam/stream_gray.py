import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import numpy as np
import cv2
import re
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ДЛЯ WEB-СЕРВЕРА ---
current_jpeg = None
jpeg_lock = threading.Lock()

# --- КЛАССЫ WEB-СЕРВЕРА ---
class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            try:
                while True:
                    with jpeg_lock:
                        jpeg = current_jpeg
                    
                    if jpeg:
                        self.wfile.write(b"--jpgboundary\r\n")
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    
                    # Ограничиваем FPS стрима (~30 кадров), чтобы не грузить Wi-Fi
                    time.sleep(0.03) 
            except Exception:
                pass # Клиент отключился
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b"<html><body style='background: #111; display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0;'><img src='/cam.mjpg' style='max-width: 100%;'></body></html>")

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

# --- ROS2 НОДА ---
class V4L2PipeNode(Node):
    def __init__(self):
        super().__init__('raw_camera_node')
        self.publisher_ = self.create_publisher(Image, '/image_mono', 10)
        self.bridge = CvBridge()
        
        self.width = 1280
        self.height = 720

        cmd_set = [
            'v4l2-ctl', '-d', '/dev/video0',
            f'--set-fmt-video=width={self.width},height={self.height},pixelformat=BA10',
            '-c', 'analogue_gain=100',  # Убиваем шум
            '-c', 'frame_rate=20',      # Фиксируем FPS, чтобы не падал до 3
            '-c', 'exposure=8000',      # Замени на то число, которое ты подберешь
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
        global current_jpeg
        
        raw_data = self.process.stdout.read(self.frame_size)
        if not raw_data or len(raw_data) != self.frame_size:
            return

        try:
            # Распаковка
            if self.frame_size >= self.width * self.height * 2:
                img16 = np.frombuffer(raw_data, dtype=np.uint16).reshape((-1, self.width))
                img16 = img16[:self.height, :] 
                # Автоматически и безопасно масштабируем любые 16 бит в 8 бит
                img8 = cv2.normalize(img16, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            else:
                data = np.frombuffer(raw_data, dtype=np.uint8)
                img8 = data.reshape((-1, 5))[:, :4].reshape((self.height, self.width))

            img8_contiguous = np.ascontiguousarray(img8)
            
            if img8_contiguous.shape != (self.height, self.width):
                self.get_logger().error(f"Неверная форма: {img8_contiguous.shape}")
                return

            # Тот самый правильный шаблон Байера
            img_clean = cv2.cvtColor(img8_contiguous, cv2.COLOR_BayerGR2GRAY)
            
            # --- WEB STREAM UPDATE ---
            # Уменьшаем картинку для браузера, чтобы экономить ресурсы
            img_resized = cv2.resize(img_clean, (self.width // 2, self.height // 2))
            ret, jpeg = cv2.imencode('.jpg', img_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ret:
                with jpeg_lock:
                    current_jpeg = jpeg.tobytes()

            # --- ROS2 PUBLISH ---
            msg = self.bridge.cv2_to_imgmsg(img_clean, encoding="mono8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"

            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Сбой OpenCV: {e}")

def main(args=None):
    # 1. Запускаем HTTP сервер в фоновом потоке
    server = ThreadedHTTPServer(('0.0.0.0', 5000), CamHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print("Сервер запущен! Открой в браузере на ноуте: http://192.168.55.1:5000")

    # 2. Запускаем основную ROS2 ноду
    rclpy.init(args=args)
    node = V4L2PipeNode()
    
    if hasattr(node, 'frame_size'):
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
            
    # 3. Чистое завершение
    node.destroy_node()
    rclpy.shutdown()
    server.server_close()

if __name__ == '__main__':
    main()