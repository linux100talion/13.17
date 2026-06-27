# ============================================================================
# nav.launch.py — навигационная сторона «варианта 2»:
#
#   camera_pkg публикует /image_color
#        ├─► nn1_anchor (~1 Гц)  → /nn1/detections (боксы якорей)
#        │        └─► ray_tracer → /nn1/anchor_pose, /nn1/corrected_odom (сброс дрейфа)
#        └─► nn2_scene  (~3 с)   → /nn2/scene      (метка сцены, баннер)
#                 └─► /nn2/relocalization → relocalizer (заглушка: восст. VINS)
#                                       │
#   openhd_streamer ◄── /image_color ──┘  рисует оверлей → H.264 → OpenHD :5600
#
# Камеру/VINS/MAVROS НЕ запускает (см. src/sim/sim_nav.launch.py). use_sim_time
# передаётся аргументом — в симуляции выставляется в true.
#
#   ros2 launch nav_pkg nav.launch.py use_sim_time:=true
#
# ── Источник /mavros/vision_pose/pose (vision_pose_source) ──────────────────
# GUIDED/ExternalNav нужна позиция в EKF. Её даёт РОВНО ОДИН издатель в
# /mavros/vision_pose/pose — два издателя в один топик ломают фьюжн. Выбор:
#   ray_tracer (default) — полный узел NN1: засечки по ориентирам + сброс дрейфа
#                          (до 1-й засечки = сырой VINS). Боевой путь.
#   bridge               — тонкий vision_pose_bridge: только сырой VINS →
#                          vision_pose, без NN1-логики. Для тестов
#                          ALT_HOLD-bootstrap/handover, пока ray_tracer отложен.
# Узлы взаимоисключающие по этому аргументу (nn1_anchor поднимается всегда, но
# в режиме bridge его детекции никто не слушает — это норма).
#
#   ros2 launch nav_pkg nav.launch.py use_sim_time:=true vision_pose_source:=bridge
# ============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    source = LaunchConfiguration("vision_pose_source")
    common = [{"use_sim_time": use_sim_time}]

    # Взаимоисключающие условия: ровно один издатель vision_pose.
    use_ray_tracer = IfCondition(PythonExpression(["'", source, "' == 'ray_tracer'"]))
    use_bridge = IfCondition(PythonExpression(["'", source, "' == 'bridge'"]))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "vision_pose_source", default_value="ray_tracer",
            description="кто публикует /mavros/vision_pose/pose: ray_tracer | bridge"),

        Node(package="nav_pkg", executable="openhd_streamer",
             output="screen", parameters=common),
        Node(package="nav_pkg", executable="nn1_anchor",
             output="screen", parameters=common),
        # источник vision_pose — взаимоисключающие узлы:
        Node(package="nav_pkg", executable="ray_tracer",
             output="screen", parameters=common, condition=use_ray_tracer),
        Node(package="nav_pkg", executable="vision_pose_bridge",
             output="screen", parameters=common, condition=use_bridge),
        Node(package="nav_pkg", executable="nn2_scene",
             output="screen", parameters=common),
        Node(package="nav_pkg", executable="relocalizer",
             output="screen", parameters=common),
    ])
