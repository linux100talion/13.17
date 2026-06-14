import subprocess
import numpy as np
import cv2
import re
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import urlparse, parse_qs

# --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ДЛЯ СЕРВЕРОВ ---
current_jpeg_mono = None
jpeg_lock_mono = threading.Lock()

current_jpeg_color = None
jpeg_lock_color = threading.Lock()

# Настройки тюнера (по умолчанию 1.0)
TUNE = {
    'gain': 1.0,
    'r': 1.0,
    'g': 1.0,
    'b': 1.0
}

# --- HTML ИНТЕРФЕЙС ТЮНЕРА ---
HTML_PAGE = """
<html>
<head>
    <title>Jetson Web Tuner</title>
    <style>
        body { background: #111; color: white; font-family: sans-serif; text-align: center; margin: 0; padding: 20px; }
        .container { display: flex; flex-direction: column; align-items: center; }
        img { max-width: 100%; border: 2px solid #444; border-radius: 8px; margin-bottom: 20px; }
        .controls { background: #222; padding: 20px; border-radius: 8px; width: 80%; max-width: 600px; }
        .slider-group { display: flex; align-items: center; justify-content: space-between; margin-bottom: 15px; }
        input[type=range] { width: 60%; }
        span { width: 50px; font-weight: bold; }
    </style>
    <script>
        window.onload = function() {
            document.getElementById('video_stream').src = 'http://' + window.location.hostname + ':5001/cam.mjpg';
        }
        function updateParam(param, val, isHardware=false) {
            document.getElementById(param + '_val').innerText = val;
            fetch('/update?' + param + '=' + val + '&hw=' + (isHardware ? '1' : '0'));
        }
    </script>
</head>
<body>
    <div class="container">
        <h2>Camera Web Tuner (AR0234)</h2>
        <img id="video_stream" src="" />
        
        <div class="controls">
            <div class="slider-group">
                <label>HW FPS:</label>
                <input type="range" min="5" max="60" step="5" value="15" oninput="updateParam('fps', this.value, true)">
                <span id="fps_val">15</span>
            </div>
            <div class="slider-group">
                <label>HW Exposure:</label>
                <input type="range" min="100" max="30000" step="100" value="5250" oninput="updateParam('exposure', this.value, true)">
                <span id="exposure_val">5250</span>
            </div>
            <hr style="border-color: #444;">
            <div class="slider-group">
                <label>Digital Gain:</label>
                <input type="range" min="0.5" max="5.0" step="0.1" value="1.0" oninput="updateParam('gain', this.value)">
                <span id="gain_val">1.0</span>
            </div>
            <div class="slider-group">
                <label style="color: #ff6666;">Red Channel:</label>
                <input type="range" min="0.1" max="3.0" step="0.1" value="1.0" oninput="updateParam('r', this.value)">
                <span id="r_val">1.0</span>
            </div>
            <div class="slider-group">
                <label style="color: #66ff66;">Green Channel:</label>
                <input type="range" min="0.1" max="3.0" step="0.1" value="1.0" oninput="updateParam('g', this.value)">
                <span id="g_val">1.0</span>
            </div>
            <div class="slider-group">
                <label style="color: #6666ff;">Blue Channel:</label>
                <input type="range" min="0.1" max="3.0" step="0.1" value="1.0" oninput="updateParam('b', this.value)">
                <span id="b_val">1.0</span>
            </div>
        </div>
    </div>
</body>
</html>
"""

# --- КЛАССЫ WEB-СЕРВЕРОВ ---
class MonoCamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            try:
                while True:
                    with jpeg_lock_mono:
                        jpeg = current_jpeg_mono
                    if jpeg:
                        self.wfile.write(b"--jpgboundary\r\n")
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.03) 
            except Exception:
                pass 
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b"<html><body style='background: #111; display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0;'><h2 style='color: white; position: absolute; top: 10px;'>VINS MONO (Port 5000)</h2><img src='/cam.mjpg' style='max-width: 100%;'></body></html>")

class ColorCamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            try:
                while True:
                    with jpeg_lock_color:
                        jpeg = current_jpeg_color
                    if jpeg:
                        self.wfile.write(b"--jpgboundary\r\n")
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.03) 
            except Exception:
                pass 
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b"<html><body style='background: #111; display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0;'><h2 style='color: white; position: absolute; top: 10px;'>PILOT COLOR (Port 5001)</h2><img src='/cam.mjpg' style='max-width: 100%;'></body></html>")

class TunerHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed_url = urlparse(self.path)
        
        if parsed_url.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
            
        elif parsed_url.path == '/update':
            qs = parse_qs(parsed_url.query)
            try:
                is_hw = qs.get('hw', ['0'])[0] == '1'
                for key, val_list in qs.items():
                    if key in TUNE:
                        TUNE[key] = float(val_list[0])
                    elif key == 'exposure' and is_hw:
                        exp_val = int(val_list[0])
                        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '-c', f'exposure={exp_val}'])
                    elif key == 'fps' and is_hw:
                        fps_val = int(val_list[0])
                        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '-c', f'frame_rate={fps_val}'])
            except Exception as e:
                print(f"[ERROR] API Update failed: {e}")
            
            self.send_response(200)
            self.end_headers()

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

# --- ОСНОВНОЙ КЛАСС ЗАХВАТА ---
class CameraStreamer:
    def __init__(self):
        self.width = 1280
        self.height = 720
        self.process = None
        self.frame_size = 0

    def setup_and_start(self):
        try:
            env = os.environ.copy()
            env['LC_ALL'] = 'C'
            
            cmd_fmt = [
                'v4l2-ctl', '-d', '/dev/video0',
                f'--set-fmt-video=width={self.width},height={self.height},pixelformat=BA10'
            ]
            subprocess.check_output(cmd_fmt, stderr=subprocess.STDOUT, env=env)
            
            cmd_get_size = ['v4l2-ctl', '-d', '/dev/video0', '--get-fmt-video']
            output = subprocess.check_output(cmd_get_size, stderr=subprocess.STDOUT, env=env).decode()
            match = re.search(r'Size\s*image\s*:\s*(\d+)', output, re.IGNORECASE)
            
            self.frame_size = int(match.group(1))
            print(f"[INFO] Размер кадра: {self.frame_size} байт")
            
        except Exception as e:
            print(f"[ERROR] Ошибка настройки: {e}")
            return False

        cmd_stream = ['v4l2-ctl', '-d', '/dev/video0', '--stream-mmap', '--stream-to=-']
        self.process = subprocess.Popen(cmd_stream, stdout=subprocess.PIPE, bufsize=10**7)
        time.sleep(0.5)

        try:
            cmd_runtime_ctrls = [
                'v4l2-ctl', '-d', '/dev/video0',
                '-c', 'frame_rate=15',
                '-c', 'analogue_gain=1200',
                '-c', 'exposure=5250'
            ]
            subprocess.check_output(cmd_runtime_ctrls, stderr=subprocess.STDOUT, env=env)
        except Exception:
            pass

        return True

    def run_loop(self):
        global current_jpeg_mono, current_jpeg_color
        
        print("[INFO] Начат захват кадров...")
        
        # --- ТАБЛИЦА ГАММЫ ДОБАВЛЕНА СЮДА ---
        gamma = 2.2
        invGamma = 1.0 / gamma
        gamma_table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
        
        frame_counter = 0 
        
        while True:
            raw_data = self.process.stdout.read(self.frame_size)
            if not raw_data or len(raw_data) != self.frame_size:
                continue

            try:
                # 1. Распаковка (Используем твой рабочий метод автоуровней!)
                if self.frame_size >= self.width * self.height * 2:
                    img16 = np.frombuffer(raw_data, dtype=np.uint16).reshape((-1, self.width))
                    img16 = img16[:self.height, :] 
                    # Твоя рабочая строка:
                    img8 = cv2.normalize(img16, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                else:
                    data = np.frombuffer(raw_data, dtype=np.uint8)
                    img8 = data.reshape((-1, 5))[:, :4].reshape((self.height, self.width))

                img8_contiguous = np.ascontiguousarray(img8)
                if img8_contiguous.shape != (self.height, self.width):
                    continue

                # Читаем параметры с веб-тюнера
                c_gain = TUNE['gain']
                c_r = TUNE['r']
                c_g = TUNE['g']
                c_b = TUNE['b']

                # 2. ГАММА
                img8_gamma = cv2.LUT(img8_contiguous, gamma_table)

                # 3. ДЕБАЙЕР (Вернул твой рабочий GR2BGR)
                img_color = cv2.cvtColor(img8_gamma, cv2.COLOR_BayerGR2BGR)
                img_mono = cv2.cvtColor(img8_gamma, cv2.COLOR_BayerGR2GRAY)
                
                # 4. ВЕКТОРНЫЙ БАЛАНС БЕЛОГО И GAIN
                if c_gain != 1.0 or c_r != 1.0 or c_g != 1.0 or c_b != 1.0:
                    # Применяем цвета и общую яркость одним махом
                    color_scales = np.array([c_b * c_gain, c_g * c_gain, c_r * c_gain], dtype=np.float32)
                    img_color = np.clip(img_color * color_scales, 0, 255).astype(np.uint8)
                    
                    if c_gain != 1.0:
                        img_mono = cv2.convertScaleAbs(img_mono, alpha=c_gain)

                # 5. СЖАТИЕ И ОТПРАВКА (каждый 2-й кадр для скорости)
                frame_counter += 1
                if frame_counter % 2 == 0:
                    img_mono_resized = cv2.resize(img_mono, (self.width // 2, self.height // 2))
                    ret_m, jpeg_m = cv2.imencode('.jpg', img_mono_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                    if ret_m:
                        with jpeg_lock_mono:
                            current_jpeg_mono = jpeg_m.tobytes()

                    img_color_resized = cv2.resize(img_color, (self.width // 2, self.height // 2))
                    ret_c, jpeg_c = cv2.imencode('.jpg', img_color_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                    if ret_c:
                        with jpeg_lock_color:
                            current_jpeg_color = jpeg_c.tobytes()

            except Exception as e:
                print(f"[ERROR] Сбой обработки кадра: {e}")

def main():
    server_mono = ThreadedHTTPServer(('0.0.0.0', 5000), MonoCamHandler)
    threading.Thread(target=server_mono.serve_forever, daemon=True).start()
    
    server_color = ThreadedHTTPServer(('0.0.0.0', 5001), ColorCamHandler)
    threading.Thread(target=server_color.serve_forever, daemon=True).start()
    
    server_tuner = ThreadedHTTPServer(('0.0.0.0', 8080), TunerHandler)
    threading.Thread(target=server_tuner.serve_forever, daemon=True).start()

    print("\n=======================================")
    print("ВЕБ-СЕРВЕРЫ УСПЕШНО ЗАПУЩЕНЫ!")
    print("TUNER (GUI):   http://<IP-ДРОНА>:8080")
    print("MONO  (VINS):  http://<IP-ДРОНА>:5000")
    print("COLOR (PILOT): http://<IP-ДРОНА>:5001")
    print("=======================================\n")

    streamer = CameraStreamer()
    if streamer.setup_and_start():
        try:
            streamer.run_loop()
        except KeyboardInterrupt:
            pass
            
    server_mono.server_close()
    server_color.server_close()
    server_tuner.server_close()
    if streamer.process:
        streamer.process.terminate()

if __name__ == '__main__':
    main()