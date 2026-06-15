# ============================================================================
# nav.launch.py — навигационная сторона «варианта 2»:
#
#   camera_pkg публикует /image_color
#        ├─► nn1_anchor (~1 Гц)  → /nn1/detections (боксы якорей)
#        └─► nn2_scene  (~3 с)   → /nn2/scene      (метка сцены)
#                                       │
#   openhd_streamer ◄── /image_color ──┘  рисует оверлей → H.264 → OpenHD :5600
#
# Камеру/VINS/MAVROS НЕ запускает (см. src/sim/sim_nav.launch.py). use_sim_time
# передаётся аргументом — в симуляции выставляется в true.
#
#   ros2 launch nav_pkg nav.launch.py use_sim_time:=true
# ============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    common = [{"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(package="nav_pkg", executable="openhd_streamer",
             output="screen", parameters=common),
        Node(package="nav_pkg", executable="nn1_anchor",
             output="screen", parameters=common),
        Node(package="nav_pkg", executable="nn2_scene",
             output="screen", parameters=common),
    ])
