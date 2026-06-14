#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <regex>
#include <opencv2/opencv.hpp>
#include "httplib.h"

// --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ДЛЯ СЕРВЕРОВ ---
std::vector<uchar> current_jpeg_mono;
std::mutex jpeg_lock_mono;

std::vector<uchar> current_jpeg_color;
std::mutex jpeg_lock_color;

// Настройки тюнера
std::atomic<float> tune_gain(1.0f);
std::atomic<float> tune_r(1.0f);
std::atomic<float> tune_g(1.0f);
std::atomic<float> tune_b(1.0f);

const int WIDTH = 1280;
const int HEIGHT = 720;

// --- HTML ИНТЕРФЕЙС ---
const char* HTML_PAGE = R"html(
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
)html";

// Вспомогательная функция выполнения команд системы
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

// Универсальный обработчик MJPEG стрима
void handle_mjpeg_stream(const httplib::Request& req, httplib::Response& res, 
                         std::mutex& lock, std::vector<uchar>& buffer) {
    res.set_content_provider(
        "multipart/x-mixed-replace; boundary=--jpgboundary",
        [&lock, &buffer](size_t offset, httplib::DataSink& sink) {
            while (true) {
                std::vector<uchar> local_buf;
                {
                    std::lock_guard<std::mutex> lk(lock);
                    local_buf = buffer;
                }
                if (!local_buf.empty()) {
                    std::string header = "--jpgboundary\r\nContent-Type: image/jpeg\r\nContent-Length: " + 
                                         std::to_string(local_buf.size()) + "\r\n\r\n";
                    sink.write(header.c_str(), header.size());
                    sink.write((const char*)local_buf.data(), local_buf.size());
                    sink.write("\r\n", 2);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
            return true;
        }
    );
}

void start_mono_server() {
    httplib::Server svr;
    svr.Get("/cam.mjpg", [](const httplib::Request& req, httplib::Response& res) {
        handle_mjpeg_stream(req, res, jpeg_lock_mono, current_jpeg_mono);
    });
    svr.Get("/", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content("<html><body style='background: #111; display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0;'><h2 style='color: white; position: absolute; top: 10px;'>VINS MONO (Port 5000)</h2><img src='/cam.mjpg' style='max-width: 100%;'></body></html>", "text/html");
    });
    svr.listen("0.0.0.0", 5000);
}

void start_color_server() {
    httplib::Server svr;
    svr.Get("/cam.mjpg", [](const httplib::Request& req, httplib::Response& res) {
        handle_mjpeg_stream(req, res, jpeg_lock_color, current_jpeg_color);
    });
    svr.Get("/", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content("<html><body style='background: #111; display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0;'><h2 style='color: white; position: absolute; top: 10px;'>PILOT COLOR (Port 5001)</h2><img src='/cam.mjpg' style='max-width: 100%;'></body></html>", "text/html");
    });
    svr.listen("0.0.0.0", 5001);
}

void start_tuner_server() {
    httplib::Server svr;
    svr.Get("/", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content(HTML_PAGE, "text/html");
    });

    svr.Get("/update", [](const httplib::Request& req, httplib::Response& res) {
        bool is_hw = req.has_param("hw") && req.get_param_value("hw") == "1";
        
        if (req.has_param("gain")) tune_gain = std::stof(req.get_param_value("gain"));
        if (req.has_param("r")) tune_r = std::stof(req.get_param_value("r"));
        if (req.has_param("g")) tune_g = std::stof(req.get_param_value("g"));
        if (req.has_param("b")) tune_b = std::stof(req.get_param_value("b"));

        if (is_hw) {
            if (req.has_param("exposure")) {
                std::string cmd = "v4l2-ctl -d /dev/video0 -c exposure=" + req.get_param_value("exposure");
                system(cmd.c_str());
            }
            if (req.has_param("fps")) {
                std::string cmd = "v4l2-ctl -d /dev/video0 -c frame_rate=" + req.get_param_value("fps");
                system(cmd.c_str());
            }
        }
        res.set_content("OK", "text/plain");
    });
    svr.listen("0.0.0.0", 8080);
}

int main() {
    // 1. Запуск потоков серверов
    std::thread t_mono(start_mono_server);
    std::thread t_color(start_color_server);
    std::thread t_tuner(start_tuner_server);

    std::cout << "\n=======================================\n";
    std::cout << "ВЕБ-СЕРВЕРЫ УСПЕШНО ЗАПУЩЕНЫ!\n";
    std::cout << "TUNER (GUI):   http://<IP-ДРОНА>:8080\n";
    std::cout << "MONO  (VINS):  http://<IP-ДРОНА>:5000\n";
    std::cout << "COLOR (PILOT): http://<IP-ДРОНА>:5001\n";
    std::cout << "=======================================\n\n";

    // 2. Настройка V4L2
    std::string cmd_fmt = "v4l2-ctl -d /dev/video0 --set-fmt-video=width=" + std::to_string(WIDTH) + 
                          ",height=" + std::to_string(HEIGHT) + ",pixelformat=BA10";
    system(cmd_fmt.c_str());

    std::string output = exec("v4l2-ctl -d /dev/video0 --get-fmt-video");
    std::smatch match;
    std::regex rx(R"(Size\s*image\s*:\s*(\d+))", std::regex_constants::icase);
    size_t frame_size = 0;
    
    if (std::regex_search(output, match, rx) && match.size() > 1) {
        frame_size = std::stoull(match.str(1));
        std::cout << "[INFO] Размер кадра: " << frame_size << " байт\n";
    } else {
        std::cerr << "[ERROR] Не удалось определить размер кадра!\n";
        return -1;
    }

    system("v4l2-ctl -d /dev/video0 -c frame_rate=15 -c analogue_gain=1200 -c exposure=5250");

    // 3. Таблица Гаммы (LUT)
    cv::Mat gamma_table(1, 256, CV_8U);
    uchar* p = gamma_table.ptr();
    for (int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(std::pow(i / 255.0, 1.0 / 2.2) * 255.0);
    }

    // 4. Захват потока
    FILE* pipe = popen("v4l2-ctl -d /dev/video0 --stream-mmap --stream-to=-", "r");
    if (!pipe) {
        std::cerr << "[ERROR] Не удалось открыть поток v4l2-ctl!\n";
        return -1;
    }

    std::vector<uint8_t> raw_data(frame_size);
    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 60};
    int frame_counter = 0;

    std::cout << "[INFO] Начат захват кадров...\n";

    while (true) {
        if (fread(raw_data.data(), 1, frame_size, pipe) != frame_size) {
            continue;
        }

        try {
            cv::Mat img8;
            if (frame_size >= WIDTH * HEIGHT * 2) {
                // Создаем 16-битную матрицу прямо поверх буфера
                cv::Mat img16(HEIGHT, WIDTH, CV_16UC1, raw_data.data());
                // Автоуровни -> 8 бит
                cv::normalize(img16, img8, 0, 255, cv::NORM_MINMAX, CV_8U);
            } else {
                continue; // Фолбэк на 8-битный режим, если надо, можно дописать
            }

            // Чтение атомарных переменных
            float c_gain = tune_gain;
            float c_r = tune_r;
            float c_g = tune_g;
            float c_b = tune_b;

            // Гамма-коррекция
            cv::Mat img8_gamma;
            cv::LUT(img8, gamma_table, img8_gamma);

            // Дебайер (GR -> BGR / GRAY)
            cv::Mat img_color, img_mono;
            cv::cvtColor(img8_gamma, img_color, cv::COLOR_BayerGB2BGR);
            cv::cvtColor(img8_gamma, img_mono, cv::COLOR_BayerGB2GRAY);

            // Баланс белого и гейн
            if (c_gain != 1.0f || c_r != 1.0f || c_g != 1.0f || c_b != 1.0f) {
                // Векторное умножение (cv::multiply автоматически ограничивает значения 0-255 для CV_8U)
                cv::multiply(img_color, cv::Scalar(c_b * c_gain, c_g * c_gain, c_r * c_gain), img_color, 1.0, CV_8U);
                
                if (c_gain != 1.0f) {
                    img_mono.convertTo(img_mono, -1, c_gain, 0); // Умножение для ч/б
                }
            }

            // Сжатие и отправка (каждый 2-й кадр)
            frame_counter++;
            if (frame_counter % 2 == 0) {
                cv::Mat img_mono_resized, img_color_resized;
                cv::resize(img_mono, img_mono_resized, cv::Size(WIDTH / 2, HEIGHT / 2));
                cv::resize(img_color, img_color_resized, cv::Size(WIDTH / 2, HEIGHT / 2));

                std::vector<uchar> jpeg_m, jpeg_c;
                cv::imencode(".jpg", img_mono_resized, jpeg_m, encode_params);
                cv::imencode(".jpg", img_color_resized, jpeg_c, encode_params);

                {
                    std::lock_guard<std::mutex> lock(jpeg_lock_mono);
                    current_jpeg_mono = std::move(jpeg_m);
                }
                {
                    std::lock_guard<std::mutex> lock(jpeg_lock_color);
                    current_jpeg_color = std::move(jpeg_c);
                }
            }
        } catch (const cv::Exception& e) {
            std::cerr << "[ERROR] Ошибка OpenCV: " << e.what() << "\n";
        }
    }

    pclose(pipe);
    t_mono.join();
    t_color.join();
    t_tuner.join();

    return 0;
}