#!/usr/bin/env bash
# ============================================================================
# Запуск nav-стороны ВНУТРИ контейнера nav: colcon build + ноды + MAVROS.
# Исполняется так:  docker exec -i p1317_nav bash -s < scripts/nav_up.sh
#                   (или make nav)
#
# Порядок source важен: ROS -> overlay (cv_bridge против CUDA-OpenCV) -> ws.
# ============================================================================
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/overlay/install/setup.bash

LOG=/root/sim_ws/output; mkdir -p "$LOG"
cd /root/sim_ws

# 1a. vins_oss — монтируется из хоста через bind mount (../../src/vins_oss).
#     При fresh-start bind mount переподключается автоматически — клонировать не нужно.
#     Все патчи (Humble, QoS, IMU skip, debug) живут в ветке 1317_debug форка:
#     https://github.com/linux100talion/VINS-MONO-ROS2/tree/1317_debug
if [ ! -d src/vins_oss/vins_estimator ]; then
    echo "  ОШИБКА: src/vins_oss не смонтирован (проверь bind mount в docker-compose.yml)"
    exit 1
fi
echo "  vins_oss: $(git -C src/vins_oss log --oneline -1 2>/dev/null || echo 'не git-репо')"

# 1b. image_transport — нужен feature_tracker/pose_graph из форка linux100talion.
#     Должен быть в Dockerfile; fallback на apt если образ старый (до rebuild).
if ! ros2 pkg list 2>/dev/null | grep -q "^image_transport$"; then
    echo "  image_transport не найден, ставим через apt..."
    apt-get update -q && apt-get install -y -q ros-humble-image-transport
fi

# 1d. numpy<2 — cv_bridge в overlay собран против NumPy 1.x; NumPy 2.x → ABI-краш.
#     Проверяем один раз: если уже <2 — ничего не делаем.
if python3 -c "import numpy; exit(0 if tuple(int(x) for x in numpy.__version__.split('.')[:2]) < (2,0) else 1)" 2>/dev/null; then
    echo "  numpy: $(python3 -c 'import numpy; print(numpy.__version__)') — OK"
else
    echo "  numpy>=2 обнаружен, даунгрейд до <2..."
    pip3 install 'numpy<2' -q
fi

# 1e. Сборка workspace — только если ещё не собран.
#     Для пересборки: make nav-rebuild (или rm -rf install внутри).
if [ ! -f install/setup.bash ]; then
    echo "  colcon build ..."
    colcon build --packages-ignore ar_demo
fi
source install/setup.bash

# 2. Байеризатор: Gazebo RGB → /dev/rawbayer (v4l2loopback).
#    Запускается ВНЕ sim_nav.launch.py: если запустить внутри launch, его крах
#    убивает весь launch (camera_node + VINS). Здесь он изолирован.
#    Eager-init в __init__: открывает /dev/rawbayer, пишет нулевой кадр —
#    только после этого v4l2loopback разрешает G_FMT на стороне capture.
if ! pgrep -f "bayerizer.py" >/dev/null; then
    nohup python3 /root/sim_ws/src/sim/bayerizer.py \
        --ros-args \
        -p input_topic:=/camera/image_raw \
        -p device:=/dev/rawbayer \
        -p pattern:=GRBG \
        -p use_sim_time:=true \
        >"$LOG/bayerizer.log" 2>&1 &
    echo "  bayerizer -> $LOG/bayerizer.log"
    # Ждём пока байеризатор активирует capture-сторону v4l2loopback (eager-init).
    echo -n "  ожидаем /dev/rawbayer..."
    for i in $(seq 1 60); do
        if v4l2-ctl -d /dev/rawbayer --get-fmt-video >/dev/null 2>&1; then
            echo " готово (${i}с)"
            break
        fi
        sleep 1
        echo -n "."
    done
else
    echo "  bayerizer уже запущен"
fi

# 3. Все sim-ноды (camera_node + feature_tracker + vins_estimator),
#    уже с use_sim_time:=true.
if ! pgrep -f "sim_nav.launch" >/dev/null; then
    nohup ros2 launch /root/sim_ws/src/sim/sim_nav.launch.py \
        >"$LOG/sim_nav.log" 2>&1 &
    echo "  sim_nav.launch -> $LOG/sim_nav.log"
else
    echo "  sim_nav.launch уже запущен"
fi

# 4. MAVROS — отдельно, тоже с sim-временем (вход от mavlink_router udp:14540).
#    conn/timesync_mode:=NONE — MAVROS не синхронизирует часы с FCU, а ставит
#    ros_now() (Gazebo sim-время) на каждый пакет. Нужно потому что SITL с
#    JSON-протоколом возвращает no_time_sync и использует wall-time FCU-часы,
#    отличные от Gazebo sim-времени. При дефолтном MAVLINK-режиме offset дрейфует
#    → IMU timestamps уходят назад → VINS получает "imu message in disorder".
if ! pgrep -f "mavros_node" >/dev/null; then
    nohup ros2 run mavros mavros_node --ros-args \
        -p use_sim_time:=true \
        -p fcu_url:="udp://:14540@127.0.0.1" \
        -p conn/timesync_mode:=NONE \
        >"$LOG/mavros.log" 2>&1 &
    echo "  MAVROS   -> $LOG/mavros.log"
    # Дать MAVROS подключиться, затем поднять частоту IMU (RAW_SENSORS stream).
    # По умолчанию ArduPilot шлёт IMU ~7 Гц — VINS нужно >= 100 Гц.
    (sleep 20 && ros2 service call /mavros/set_stream_rate \
        mavros_msgs/srv/StreamRate \
        '{stream_id: 1, message_rate: 200, on_off: true}' \
        >> "$LOG/mavros.log" 2>&1 && echo "  stream_rate IMU 200 Гц установлен") &
else
    echo "  MAVROS   уже запущен"
fi

echo "nav: готово. Логи: docker/sim/output/"
