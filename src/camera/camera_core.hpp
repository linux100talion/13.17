#pragma once
// ============================================================================
// camera_core.hpp — общий базовый класс V4L2-камеры для VINS-Mono.
//
// Содержит ВСЁ, что не зависит от вычислителя (GPU/CPU):
//   - V4L2-захват (v4l2-ctl, формат BA10) и определение размера кадра,
//   - нормализацию 16→8 бит + гамму,
//   - публикацию /image_mono (вход VINS), /image_color (bgr8, nav-сторона)
//     и /camera_info, штамп через get_clock()->now() (уважает use_sim_time),
//   - опциональный встроенный OpenHD-стример (stream_openhd:=true),
//   - ROS-параметры gain/r/g/b и device на лету.
//
// Дебайеризация + per-channel gain вынесены в чистый виртуальный
// process_frame(), который реализуют наследники:
//   - camera_node.cpp     — CUDA (cv::cuda::*), боевой Orin и штатный GPU-sim;
//   - camera_node_cpu.cpp — CPU  (cv::*), drop-in для машин без GPU/драйвера.
//
// ВАЖНО: поток захвата запускается НЕ в конструкторе, а методом start()
// (вызывать после конструктора наследника) — иначе виртуальный process_frame()
// ещё не задиспатчится в наследника.
// ============================================================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

class V4L2CameraBase : public rclcpp::Node {
public:
    explicit V4L2CameraBase(const std::string& node_name)
        : Node(node_name), width_(1280), height_(720), frame_size_(0), running_(true) {

        // 1. Публикаторы: /image_mono (каждый кадр для VINS), /image_color (nav).
        image_pub_       = this->create_publisher<sensor_msgs::msg::Image>("/image_mono", 10);
        image_color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_color", 10);
        info_pub_        = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);

        // 2. Динамические ROS2-параметры (gain/баланс белого).
        this->declare_parameter("gain", 1.0);
        this->declare_parameter("r", 1.0);
        this->declare_parameter("g", 1.0);
        this->declare_parameter("b", 1.0);

        // Путь к V4L2-устройству. На железе — /dev/video0, в симуляции —
        // /dev/rawbayer (v4l2loopback). Меняется только параметром.
        device_ = this->declare_parameter("device", std::string("/dev/video0"));
        RCLCPP_INFO(this->get_logger(), "V4L2 устройство: %s", device_.c_str());

        // 3. CameraInfo (интринсики из конфига).
        setup_camera_info();

        // 4. V4L2-пайплайн.
        if (!setup_v4l2()) {
            RCLCPP_ERROR(this->get_logger(), "Сбой инициализации V4L2. Нода будет остановлена.");
            rclcpp::shutdown();
            return;
        }

        // 5. Встроенный OpenHD-энкодер — по умолчанию ВЫКЛ (поток собирает
        //    отдельная нода nav_pkg/openhd_streamer). true — standalone-режим.
        stream_openhd_ = this->declare_parameter("stream_openhd", false);
        if (stream_openhd_) {
            std::string gst_pipeline =
                "appsrc ! videoconvert ! "
                "x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 ! "
                "rtph264pay ! udpsink host=127.0.0.1 port=5600 sync=false";

            openhd_writer_.open(gst_pipeline, cv::CAP_GSTREAMER, 0, 15,
                                cv::Size(width_ / 2, height_ / 2), true);
            if (!openhd_writer_.isOpened()) {
                RCLCPP_WARN(this->get_logger(), "Не удалось открыть GStreamer для OpenHD!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Трансляция H.264 для OpenHD запущена на 127.0.0.1:5600");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "OpenHD внутри камеры выключен (stream_openhd:=false) — поток собирает openhd_streamer.");
        }

        init_ok_ = true;
    }

    virtual ~V4L2CameraBase() {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if (pipe_) {
            pclose(pipe_);
        }
    }

    // Запуск потока захвата. Вызывать ПОСЛЕ конструктора наследника, чтобы
    // process_frame() уже диспатчился в реализацию наследника.
    void start() {
        if (!init_ok_) {
            return;  // V4L2 не поднялся — нода уже инициировала shutdown.
        }
        capture_thread_ = std::thread(&V4L2CameraBase::capture_loop, this);
    }

protected:
    // Дебайер + per-channel gain. На вход — Bayer-кадр CV_8UC1 (после гаммы).
    // Заполнить out_mono (mono8, вход VINS) и out_color (bgr8, для nav-стороны).
    // Реализуется наследником (CUDA или CPU).
    virtual void process_frame(const cv::Mat& img8_gamma,
                               double gain, double r, double g, double b,
                               cv::Mat& out_mono, cv::Mat& out_color) = 0;

    int width_, height_;

private:
    std::string device_;
    size_t frame_size_;
    FILE* pipe_ = nullptr;
    std::atomic<bool> running_;
    std::thread capture_thread_;
    bool stream_openhd_ = false;
    bool init_ok_ = false;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    cv::VideoWriter openhd_writer_;

    void setup_camera_info() {
        camera_info_msg_.header.frame_id = "camera_frame";
        camera_info_msg_.width = width_;
        camera_info_msg_.height = height_;
        camera_info_msg_.distortion_model = "plumb_bob";
        camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        camera_info_msg_.k = {640.0, 0.0, 640.0, 0.0, 640.0, 360.0, 0.0, 0.0, 1.0};
        camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        camera_info_msg_.p = {640.0, 0.0, 640.0, 0.0, 0.0, 640.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    }

    // Выполнение системной команды с захватом stdout.
    std::string exec(const char* cmd) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (pipe) {
            while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
                result += buffer.data();
            }
        }
        return result;
    }

    bool setup_v4l2() {
        RCLCPP_INFO(this->get_logger(), "Настройка формата и разрешения V4L2...");
        std::string cmd_fmt = "v4l2-ctl -d " + device_ + " --set-fmt-video=width=" +
                              std::to_string(width_) + ",height=" + std::to_string(height_) + ",pixelformat=BA10";
        system(cmd_fmt.c_str());

        std::string output = exec(("v4l2-ctl -d " + device_ + " --get-fmt-video").c_str());
        std::smatch match;
        std::regex rx(R"(Size\s*image\s*:\s*(\d+))", std::regex_constants::icase);

        if (std::regex_search(output, match, rx) && match.size() > 1) {
            frame_size_ = std::stoull(match.str(1));
            RCLCPP_INFO(this->get_logger(), "Размер кадра: %zu байт", frame_size_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Не удалось определить Sizeimage!");
            return false;
        }

        std::string cmd_stream = "v4l2-ctl -d " + device_ + " --stream-mmap --stream-to=-";
        pipe_ = popen(cmd_stream.c_str(), "r");
        if (!pipe_) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось открыть поток v4l2-ctl!");
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Контролы специфичны для ArduCam; на v4l2loopback (симуляция) дадут
        // ошибку — не фатально.
        RCLCPP_INFO(this->get_logger(), "Применение настроек экспозиции, FPS и Gain...");
        std::string cmd_ctrl = "v4l2-ctl -d " + device_ + " -c frame_rate=15 -c analogue_gain=1200 -c exposure=5250";
        system(cmd_ctrl.c_str());

        return true;
    }

    void capture_loop() {
        std::vector<uint8_t> raw_data(frame_size_);
        int frame_counter = 0;

        // Таблица гаммы.
        cv::Mat gamma_table(1, 256, CV_8U);
        for (int i = 0; i < 256; ++i) {
            gamma_table.at<uchar>(i) = cv::saturate_cast<uchar>(std::pow(i / 255.0, 1.0 / 2.2) * 255.0);
        }

        RCLCPP_INFO(this->get_logger(), "Захват и публикация начаты.");

        while (running_ && rclcpp::ok()) {
            if (fread(raw_data.data(), 1, frame_size_, pipe_) != frame_size_) {
                continue;
            }

            try {
                cv::Mat img8;
                if (frame_size_ >= static_cast<size_t>(width_ * height_ * 2)) {
                    cv::Mat img16(height_, width_, CV_16UC1, raw_data.data());
                    cv::normalize(img16, img8, 0, 255, cv::NORM_MINMAX, CV_8U);
                } else {
                    continue;
                }

                cv::Mat img8_gamma;
                cv::LUT(img8, gamma_table, img8_gamma);

                // ROS2-параметры "на лету".
                double c_gain = this->get_parameter("gain").as_double();
                double c_r = this->get_parameter("r").as_double();
                double c_g = this->get_parameter("g").as_double();
                double c_b = this->get_parameter("b").as_double();

                // Дебайер + gain — в наследнике (GPU или CPU).
                cv::Mat img_color, img_mono;
                process_frame(img8_gamma, c_gain, c_r, c_g, c_b, img_mono, img_color);

                // Единый TimeStamp на все три топика.
                auto current_time = this->get_clock()->now();

                std_msgs::msg::Header header;
                header.stamp = current_time;
                header.frame_id = "camera_frame";

                // Mono8 для VINS-Mono (каждый кадр).
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", img_mono).toImageMsg();
                image_pub_->publish(*msg);

                // Полноразмерный BGR для nav-стороны (нейросети + openhd_streamer).
                sensor_msgs::msg::Image::SharedPtr color_msg = cv_bridge::CvImage(header, "bgr8", img_color).toImageMsg();
                image_color_pub_->publish(*color_msg);

                camera_info_msg_.header.stamp = current_time;
                info_pub_->publish(camera_info_msg_);

                // OpenHD (каждый 2-й кадр), только в standalone-режиме.
                frame_counter++;
                if (frame_counter % 2 == 0 && openhd_writer_.isOpened()) {
                    cv::Mat img_color_resized;
                    cv::resize(img_color, img_color_resized, cv::Size(width_ / 2, height_ / 2));
                    openhd_writer_.write(img_color_resized);
                }

            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Ошибка OpenCV: %s", e.what());
            }
        }
    }
};
