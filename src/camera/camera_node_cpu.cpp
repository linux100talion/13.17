// ============================================================================
// camera_node_cpu.cpp — drop-in CPU-версия камера-ноды (без GPU/CUDA).
//
// Тот же базовый класс V4L2CameraBase (camera_core.hpp), что и боевой
// camera_node.cpp, но дебайер + gain считаются на CPU через cv::* —
// для машин без NVIDIA GPU/драйвера (GPU-less прогон gazebo→SITL→VINS,
// ветка nn2_c3_cpu; T4 в дефиците). Боевой Orin и штатный GPU-sim остаются
// на camera_node.cpp — этот executable их не трогает.
//
// ⚠️ Код Байера на CPU ДРУГОЙ: cv::COLOR_BayerGR2BGR (не BayerGB2RGB, как в
//    CUDA-модуле) — см. CLAUDE.md, раздел про камеру. cv::cvtColor на CPU
//    отдаёт сразу BGR, поэтому отдельный RGB→BGR swap не нужен, а gain
//    раскладывается прямо по BGR-каналам (ch0=B, ch1=G, ch2=R).
//
// Линкуется БЕЗ CUDA-модулей OpenCV (см. CMakeLists.txt) — бинарь не тянет
// зависимостей на GPU-рантайм.
// ============================================================================
#include "camera_core.hpp"

class V4L2CameraNodeCpu : public V4L2CameraBase {
public:
    V4L2CameraNodeCpu() : V4L2CameraBase("raw_camera_node") {
        RCLCPP_INFO(this->get_logger(), "CPU-дебайер активен (без CUDA).");
    }

protected:
    void process_frame(const cv::Mat& img8_gamma,
                       double gain, double r, double g, double b,
                       cv::Mat& out_mono, cv::Mat& out_color) override {
        // Дебайер сразу в BGR (CPU-код для физического паттерна GRBG сенсора).
        cv::cvtColor(img8_gamma, out_color, cv::COLOR_BayerGR2BGR);

        if (gain != 1.0 || r != 1.0 || g != 1.0 || b != 1.0) {
            std::vector<cv::Mat> ch(3);
            cv::split(out_color, ch);
            ch[0].convertTo(ch[0], -1, b * gain);  // B
            ch[1].convertTo(ch[1], -1, g * gain);  // G
            ch[2].convertTo(ch[2], -1, r * gain);  // R
            cv::merge(ch, out_color);
        }

        // mono для VINS — стандартный BGR2GRAY уже над корректным BGR.
        cv::cvtColor(out_color, out_mono, cv::COLOR_BGR2GRAY);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<V4L2CameraNodeCpu>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
