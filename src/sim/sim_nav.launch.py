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
import re
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

CFG = "/root/sim_ws/src/vins/VINS-MONO-ROS2/config_pkg/config/sim.yaml"
DEVICE = "/dev/rawbayer"

# Какой executable камеры запускать:
#   camera_node     — лётный CUDA-дебайер (default, штатный GPU-sim),
#   camera_node_cpu — drop-in CPU-дебайер для машин без GPU (env CAMERA_NODE).
# Переключается через окружение, не правя launch — CPU-оверрайд compose
# выставляет CAMERA_NODE=camera_node_cpu.
CAMERA_EXECUTABLE = os.environ.get("CAMERA_NODE", "camera_node")

# Разрешение камеры — единый переключатель по env CAMERA_W/CAMERA_H
# (default 1280×720, как реальный ArduCam). В GPU-less прогоне (llvmpipe слишком
# медленный на 1280×720) CPU-оверрайд compose ставит 320×180 — это в ~16 раз
# меньше пикселей под софтрендер. Значение прокидывается в camera_node И в
# bayerizer (nav_up.sh), плюс пересчитываются интринсики VINS (см. ниже).
CAMERA_W = int(os.environ.get("CAMERA_W", "1280"))
CAMERA_H = int(os.environ.get("CAMERA_H", "720"))

# Источник /mavros/vision_pose/pose (см. nav.launch.py):
#   ray_tracer (default) — полный узел NN1, лётный путь;
#   bridge               — тонкий vision_pose_bridge (сырой VINS), для тестов
#                          ALT_HOLD-bootstrap/handover пока ray_tracer отложен.
# Переключается env VISION_POSE_SOURCE (nav_up.sh), не правя launch.
VISION_POSE_SOURCE = os.environ.get("VISION_POSE_SOURCE", "ray_tracer")


def _camera_mount_module():
    """camera_mount.py: положение камеры из бортового конфига (env CAM_CFG) — тот же
    файл, по которому sim_up.sh поставил camera_link в Gazebo, а bootstrap_node
    считает демпфер."""
    try:
        from control_pkg.perception import camera_mount
    except ImportError:
        sys.path.insert(0, os.path.join(os.environ.get("REPO_ROOT", "/root/repo"), "src/control"))
        from control_pkg.perception import camera_mount
    return camera_mount


def _vins_config(width, height):
    """Конфиг VINS сима: sim.yaml + экстринсики камеры + разрешение, копией в /tmp.

    Экстринсики (extrinsicRotation/Translation) ВСЕГДА берутся из бортового конфига
    (CAM_CFG, camera_mount.py) — значения в sim.yaml лишь заглушка того же вида.
    Сим верит им жёстко (estimate_extrinsic: 0 в sim.yaml): Gazebo ставит камеру
    ровно туда, ошибки крепления в симе нет.

    Интринсики (fx/fy/cx/cy) и разрешение — тоже из конфига камеры (с 2026-10-02): камера,
    которую рисует Gazebo (фокус конфига под текущее разрешение, центр посередине, пиксель
    квадратный; sim_up.sh ставит по тому же фокусу horizontal_fov). Значения в sim.yaml —
    заглушка. Дисторсия sim.yaml нулевая: Gazebo рисует без неё.
    """
    cm = _camera_mount_module()
    cam = cm.CameraMount.load()
    print(f"[sim_nav] {cam.summary()}")
    with open(CFG) as f:
        text = f.read()
    text = cm.replace_matrix(text, "extrinsicRotation", [list(r) for r in cam.R])
    text = cm.replace_matrix(text, "extrinsicTranslation", [[v] for v in cam.t])

    # интринсики VINS сима — камера, которую РИСУЕТ Gazebo: фокус из конфига камеры (sim_up.sh
    # ставит по нему horizontal_fov), квадратный пиксель, центр посередине, без дисторсии
    kfx, kfy, kcx, kcy = cam.intrinsics_for(width, height, ideal=True)
    kv = {"fx": kfx, "fy": kfy, "cx": kcx, "cy": kcy}
    out_lines = []
    for ln in text.splitlines():
        m = re.match(r"^(\s*)(image_width|image_height|fx|fy|cx|cy)(\s*:\s*)([0-9.]+)(.*)$", ln)
        if m:
            indent, key, sep, val, tail = m.groups()
            if key == "image_width":
                nv = str(width)
            elif key == "image_height":
                nv = str(height)
            else:
                nv = f"{kv[key]:.6g}"
            ln = f"{indent}{key}{sep}{nv}{tail}"
        out_lines.append(ln)

    dst = f"/tmp/sim_{width}x{height}.yaml"
    with open(dst, "w") as f:
        f.write("\n".join(out_lines) + "\n")
    return dst


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}
    cfg = _vins_config(CAMERA_W, CAMERA_H)

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
                # stamp_from_frame — честный штамп рендера Gazebo из трейлера
                # bayerizer (иначе при RTF≈1 смещение ~60 мс + джиттер ±30 мс
                # относительно IMU разваливает VINS). Только для симуляции.
                # интринсики /camera_info (их ест ray_tracer) — камера, которую рисует
                # Gazebo: фокус бортового конфига, центр посередине, без дисторсии
                parameters=[use_sim_time, {"device": DEVICE,
                                           "width": CAMERA_W, "height": CAMERA_H,
                                           "stamp_from_frame": True,
                                           **_camera_mount_module().CameraMount.load()
                                           .camera_params(CAMERA_W, CAMERA_H, ideal=True)}],
            ),

            # VINS feature tracker.
            Node(
                package="feature_tracker",
                executable="feature_tracker",
                output="screen",
                parameters=[use_sim_time, {"config_file": cfg}],
            ),

            # VINS estimator.
            Node(
                package="vins_estimator",
                executable="vins_estimator",
                output="screen",
                parameters=[use_sim_time, {"config_file": cfg}],
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
            launch_arguments={"use_sim_time": "true",
                              "vision_pose_source": VISION_POSE_SOURCE}.items(),
        ),
    ])
