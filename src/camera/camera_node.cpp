// ============================================================================
// camera_node.cpp — БОЕВАЯ камера-нода (CUDA). Наследник V4L2CameraBase.
//
// Весь общий V4L2/публикация/гамма/OpenHD-код — в camera_core.hpp. Здесь —
// только GPU-реализация дебайера + gain через cv::cuda::*. Для машин без GPU
// есть drop-in camera_node_cpu.cpp (тот же базовый класс, cv::* на CPU).
//
// Дебайер: COLOR_BayerGB2RGB — CUDA-специфичный код для физического паттерна
// GRBG сенсора (в CUDA-модуле OpenCV сдвиг в именовании, см. CLAUDE.md).
// mono для VINS считается ДО конверсии RGB→BGR (поведение не меняем).
// ============================================================================
#include "camera_core.hpp"

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

class V4L2CameraNodeCuda : public V4L2CameraBase {
public:
    V4L2CameraNodeCuda() : V4L2CameraBase("raw_camera_node") {
        // Предварительное резервирование памяти GPU.
        gpu_raw_.create(height_, width_, CV_8UC1);
        gpu_channels_.resize(3);
        RCLCPP_INFO(this->get_logger(), "CUDA-дебайер активен.");
    }

protected:
    void process_frame(const cv::Mat& img8_gamma,
                       double gain, double r, double g, double b,
                       cv::Mat& out_mono, cv::Mat& out_color) override {
        gpu_raw_.upload(img8_gamma);

        // Дебайер (BayerGB2RGB компенсирует инверсию именования в CUDA).
        cv::cuda::demosaicing(gpu_raw_, gpu_color_, cv::COLOR_BayerGB2RGB);

        if (gain != 1.0 || r != 1.0 || g != 1.0 || b != 1.0) {
            cv::cuda::split(gpu_color_, gpu_channels_);
            // gpu_color_ — RGB (ch0=R, ch2=B); множители подобраны так, что
            // после RGB→BGR swap ниже синий канал получает b, красный — r.
            gpu_channels_[0].convertTo(gpu_channels_[0], -1, b * gain);
            gpu_channels_[1].convertTo(gpu_channels_[1], -1, g * gain);
            gpu_channels_[2].convertTo(gpu_channels_[2], -1, r * gain);
            cv::cuda::merge(gpu_channels_, gpu_color_);
        }

        // mono для VINS считаем ДО конверсии (поведение не меняем).
        cv::cuda::cvtColor(gpu_color_, gpu_mono_, cv::COLOR_BGR2GRAY);
        // gpu_color_ после демозаики — RGB; для /image_color (bgr8) и OpenHD
        // приводим к настоящему BGR.
        cv::cuda::cvtColor(gpu_color_, gpu_bgr_, cv::COLOR_RGB2BGR);

        gpu_bgr_.download(out_color);
        gpu_mono_.download(out_mono);
    }

private:
    cv::cuda::GpuMat gpu_raw_, gpu_color_, gpu_mono_, gpu_bgr_;
    std::vector<cv::cuda::GpuMat> gpu_channels_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<V4L2CameraNodeCuda>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
