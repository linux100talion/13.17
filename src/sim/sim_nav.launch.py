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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

CFG = "/root/sim_ws/src/vins/VINS-MONO-ROS2/config_pkg/config/sim.yaml"
DEVICE = "/dev/rawbayer"


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    return LaunchDescription([
        # 1. Байеризатор: Gazebo RGB -> /dev/rawbayer (standalone-скрипт).
        #    use_sim_time здесь не влияет (он не публикует stamped-сообщения),
        #    выставлен для единообразия.
        ExecuteProcess(
            cmd=[
                "python3", "/root/sim_ws/src/sim/bayerizer.py",
                "--ros-args",
                "-p", "input_topic:=/camera/image_raw",
                "-p", f"device:={DEVICE}",
                "-p", "pattern:=GRBG",
                "-p", "use_sim_time:=true",
            ],
            output="screen",
        ),

        # 2-4. camera_node и VINS стартуют с задержкой 4 с.
        #      Байеризатор открывает /dev/rawbayer и пишет инициализирующий кадр
        #      в __init__() — только ПОСЛЕ этого v4l2loopback отвечает на
        #      G_FMT(CAPTURE). 4 с достаточно для ROS-инициализации байеризатора.
        TimerAction(period=4.0, actions=[

            # Камера-нода: /dev/rawbayer -> /image_mono (VINS) + /image_color
            Node(
                package="camera_pkg",
                executable="camera_node",
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
