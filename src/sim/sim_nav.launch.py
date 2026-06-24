# ============================================================================
# sim_nav.launch.py — запуск nav-стороны в СИМУЛЯЦИИ с use_sim_time:=true.
#
# Все ноды берут время из /clock (его публикует ros_gz_bridge в контейнере
# simulator). Без use_sim_time таймстампы кадров/IMU разойдутся с симуляцией
# и VINS будет молча расходиться.
#
# НЕ включает:
#   - ros_gz_bridge (он ИСТОЧНИК /clock — ему use_sim_time ставить нельзя),
#     запускается в контейнере simulator (см. docker/sim/README.md);
#   - mavros (свой launch; use_sim_time для него — отдельно, см. README).
#
# Запуск (в контейнере nav, после colcon build):
#   ros2 launch /root/sim_ws/src/sim/sim_nav.launch.py
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

CFG = "/root/sim_ws/src/vins/VINS-MONO-ROS2/config_pkg/config/sim.yaml"
DEVICE = "/dev/rawbayer"

# Какой executable камеры запускать:
#   camera_node     — боевой CUDA-дебайер (default, штатный GPU-sim),
#   camera_node_cpu — drop-in CPU-дебайер для машин без GPU (env CAMERA_NODE).
# Переключается через окружение, не правя launch — CPU-оверрайд compose
# выставляет CAMERA_NODE=camera_node_cpu.
CAMERA_EXECUTABLE = os.environ.get("CAMERA_NODE", "camera_node")


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    return LaunchDescription([
        # 1-3. camera_node и VINS стартуют с задержкой 4 с.
        #      Байеризатор запускается ВНЕ этого launch (в nav_up.sh) чтобы его
        #      крах/остановка не убивала весь launch. nav_up.sh ждёт активации
        #      /dev/rawbayer перед вызовом этого launch-файла.
        TimerAction(period=4.0, actions=[

            # Камера-нода: /dev/rawbayer -> /image_mono (VINS) + /image_color.
            # executable выбирается по env CAMERA_NODE (CUDA по умолчанию, CPU в
            # GPU-less прогоне).
            Node(
                package="camera_pkg",
                executable=CAMERA_EXECUTABLE,
                output="screen",
                parameters=[use_sim_time, {"device": DEVICE}],
            ),

            # VINS feature tracker.
            Node(
                package="feature_tracker",
                executable="feature_tracker",
                output="screen",
                parameters=[use_sim_time, {"config_file": CFG}],
            ),

            # VINS estimator.
            Node(
                package="vins_estimator",
                executable="vins_estimator",
                output="screen",
                parameters=[use_sim_time, {"config_file": CFG}],
                remappings=[
                    ("/feature_tracker/feature", "/feature"),
                    ("/feature_tracker/restart", "/restart"),
                ],
            ),

        ]),

        # 5. nav-сторона: nn1_anchor (~1 Гц) + nn2_scene (~3 с) + openhd_streamer
        #    (даунлинк в OpenHD с оверлеем детекций). Подписаны на /image_color.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nav_pkg"), "launch", "nav.launch.py")),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ])
