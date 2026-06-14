import subprocess
import numpy as np
import cv2
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

WIDTH = 1920
HEIGHT = 1200

# Впиши сюда размер кадра, который выдавал твой предыдущий скрипт 
# (скорее всего 4608000 для 16-bit или 2880000 для MIPI packed)
# FRAME_SIZE = 2880000
FRAME_SIZE = 4608000

# Глобальная переменная для хранения последнего готового кадра (в JPEG)
current_jpeg = None
lock = threading.Lock()

def capture_loop():
    global current_jpeg
    cmd = [
        'v4l2-ctl', '-d', '/dev/video0',
        f'--set-fmt-video=width={WIDTH},height={HEIGHT},pixelformat=BA10',
        '--stream-mmap', '--stream-to=-'
    ]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**7)
    print("V4L2 захват запущен...")
    
    while True:
        raw_data = process.stdout.read(FRAME_SIZE)
        if not raw_data or len(raw_data) != FRAME_SIZE:
            continue

        # 1. Возвращаем правильный математический сдвиг для 10-битных данных
        if FRAME_SIZE == WIDTH * HEIGHT * 2:
            img16 = np.frombuffer(raw_data, dtype=np.uint16).reshape((HEIGHT, WIDTH))
            img8 = (img16 >> 2).astype(np.uint8)
            
        elif FRAME_SIZE == int(WIDTH * HEIGHT * 1.25):
            data = np.frombuffer(raw_data, dtype=np.uint8)
            img8 = data.reshape((-1, 5))[:, :4].reshape((HEIGHT, WIDTH))
        else:
            continue

        # 2. Делаем ЧБ (Grayscale), как любит VINS-Mono!
        # Если картинка перевернута или выглядит как сетка - попробуй BayerBG2GRAY
        img_gray = cv2.cvtColor(img8, cv2.COLOR_BayerRG2GRAY) 
        
        # 3. Переводим обратно в 3 канала, чтобы JPEG-кодеру было проще это сжать для браузера
        img_color = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
        
        # 4. Уменьшаем кадр для трансляции
        img_resized = cv2.resize(img_color, (WIDTH // 2, HEIGHT // 2))
        # Пакуем в JPEG
        ret, jpeg = cv2.imencode('.jpg', img_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ret:
            with lock:
                current_jpeg = jpeg.tobytes()

class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            try:
                while True:
                    with lock:
                        jpeg = current_jpeg
                    
                    if jpeg:
                        self.wfile.write(b"--jpgboundary\r\n")
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    
                    # Ограничиваем FPS стрима (~30 кадров), чтобы не грузить сеть
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

if __name__ == '__main__':
    # Запускаем захват с камеры в фоновом потоке
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    
    # Даем камере секунду на инициализацию
    time.sleep(1)
    
    server = ThreadedHTTPServer(('0.0.0.0', 5000), CamHandler)
    print("Сервер запущен! Открой в браузере на ноуте: http://192.168.55.1:5000")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Остановка сервера...")
        server.server_close()