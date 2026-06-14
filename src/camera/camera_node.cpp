#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <cstdio>
#include <regex>

class V4L2CameraNode : public rclcpp::Node {
public:
    V4L2CameraNode() : Node("raw_camera_node"), width_(1280), height_(720), frame_size_(0), running_(true) {
        
        // 1. Инициализация публикаторов
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_mono", 10);
        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);

        // 2. Объявление динамических ROS2 параметров (вместо веб-слайдеров)
        this->declare_parameter("gain", 1.0);
        this->declare_parameter("r", 1.0);
        this->declare_parameter("g", 1.0);
        this->declare_parameter("b", 1.0);

        // Путь к V4L2-устройству. На железе — реальная камера (/dev/video0),
        // в симуляции — виртуальное устройство v4l2loopback (/dev/rawbayer),
        // куда байеризатор пишет кадры из Gazebo. Меняется только параметром.
        device_ = this->declare_parameter("device", std::string("/dev/video0"));
        RCLCPP_INFO(this->get_logger(), "V4L2 устройство: %s", device_.c_str());

        // 3. Заполнение CameraInfo (интринсики из dummy_13_7.yaml)
        setup_camera_info();

        // 4. Инициализация V4L2 пайплайна
        if (!setup_v4l2()) {
            RCLCPP_ERROR(this->get_logger(), "Сбой инициализации V4L2. Нода будет остановлена.");
            rclcpp::shutdown();
            return;
        }

        // 5. Инициализация GStreamer для OpenHD (UDP H.264 поток на порт 5600)
        std::string gst_pipeline = 
            "appsrc ! videoconvert ! "
            "x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 ! "
            "rtph264pay ! udpsink host=127.0.0.1 port=5600 sync=false";
        
        openhd_writer_.open(gst_pipeline, cv::CAP_GSTREAMER, 0, 15, cv::Size(width_ / 2, height_ / 2), true);
        if (!openhd_writer_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Не удалось открыть GStreamer для OpenHD!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Трансляция H.264 для OpenHD запущена на 127.0.0.1:5600");
        }

        // 6. Запуск выделенного потока для захвата и обработки
        capture_thread_ = std::thread(&V4L2CameraNode::capture_loop, this);
    }

    ~V4L2CameraNode() {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if (pipe_) {
            pclose(pipe_);
        }
    }

private:
    int width_, height_;
    std::string device_;
    size_t frame_size_;
    FILE* pipe_ = nullptr;
    std::atomic<bool> running_;
    std::thread capture_thread_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
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

    // Вспомогательная функция выполнения системных команд
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

        // Эти контролы специфичны для ArduCam. На v4l2loopback (симуляция) они
        // не реализованы — v4l2-ctl выдаст ошибку, но это не фатально.
        RCLCPP_INFO(this->get_logger(), "Применение настроек экспозиции, FPS и Gain...");
        std::string cmd_ctrl = "v4l2-ctl -d " + device_ + " -c frame_rate=15 -c analogue_gain=1200 -c exposure=5250";
        system(cmd_ctrl.c_str());
        
        return true;
    }

    void capture_loop() {
        std::vector<uint8_t> raw_data(frame_size_);
        int frame_counter = 0;

        // Предварительное резервирование памяти GPU
        cv::cuda::GpuMat gpu_raw(height_, width_, CV_8UC1);
        cv::cuda::GpuMat gpu_color, gpu_mono;
        std::vector<cv::cuda::GpuMat> gpu_channels(3);

        // Таблица гаммы
        cv::Mat gamma_table(1, 256, CV_8U);
        for (int i = 0; i < 256; ++i) {
            gamma_table.at<uchar>(i) = cv::saturate_cast<uchar>(std::pow(i / 255.0, 1.0 / 2.2) * 255.0);
        }

        RCLCPP_INFO(this->get_logger(), "CUDA захват и публикация начаты.");

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

                // Чтение ROS2 параметров "на лету"
                double c_gain = this->get_parameter("gain").as_double();
                double c_r = this->get_parameter("r").as_double();
                double c_g = this->get_parameter("g").as_double();
                double c_b = this->get_parameter("b").as_double();

                // --- CUDA ПРОЦЕССИНГ ---
                gpu_raw.upload(img8_gamma);
                
                // Дебайер (используем найденный правильный флаг BayerGB2RGB, чтобы компенсировать инверсию)
                cv::cuda::demosaicing(gpu_raw, gpu_color, cv::COLOR_BayerGB2RGB);

                if (c_gain != 1.0 || c_r != 1.0 || c_g != 1.0 || c_b != 1.0) {
                    cv::cuda::split(gpu_color, gpu_channels);
                    gpu_channels[0].convertTo(gpu_channels[0], -1, c_b * c_gain);
                    gpu_channels[1].convertTo(gpu_channels[1], -1, c_g * c_gain);
                    gpu_channels[2].convertTo(gpu_channels[2], -1, c_r * c_gain);
                    cv::cuda::merge(gpu_channels, gpu_color);
                }

                cv::cuda::cvtColor(gpu_color, gpu_mono, cv::COLOR_BGR2GRAY);

                cv::Mat img_color, img_mono;
                gpu_color.download(img_color);
                gpu_mono.download(img_mono);
                // -----------------------

                // Формируем единый TimeStamp
                auto current_time = this->get_clock()->now();

                // Публикация Mono8 в ROS2 (каждый кадр для VINS-Mono)
                std_msgs::msg::Header header;
                header.stamp = current_time;
                header.frame_id = "camera_frame";
                
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", img_mono).toImageMsg();
                image_pub_->publish(*msg);

                camera_info_msg_.header.stamp = current_time;
                info_pub_->publish(camera_info_msg_);

                // Отправка Color в OpenHD (Каждый 2-й кадр)
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<V4L2CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}