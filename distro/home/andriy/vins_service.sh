#!/bin/bash

# ==========================================
# 1. Настройки окружения ROS 2 на хосте
# ==========================================
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0

# Если используется CycloneDDS (как в скрипте записи bag), раскомментируйте:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Загружаем базовый ROS 2 на хосте, чтобы утилита `ros2 topic echo` работала
source /opt/ros/humble/setup.bash

# ==========================================
# 2. Настройки VINS-Mono и Docker
# ==========================================
CONTAINER="vins_project_13_7"
PID1=""
PID2=""
PID3=""
PID4=""

# Функция для запуска нод VINS-Mono
start_vins_nodes() {
    echo "Запуск нод VINS-Mono в контейнере $CONTAINER..."
    
    # 1. Camera Node (C++ CUDA)
    docker exec -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=0 $CONTAINER bash -c "source /root/vins_ws/install/setup.bash && exec ros2 run camera_pkg camera_node" &
    PID1=$!

    # 2. Feature Tracker
    docker exec -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=0 $CONTAINER bash -c "source /root/vins_ws/install/setup.bash && exec ros2 run feature_tracker feature_tracker --ros-args -p config_file:=/root/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/dummy_13_7.yaml" &
    PID2=$!

    # 3. VINS Estimator
    docker exec -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=0 $CONTAINER bash -c "source /root/vins_ws/install/setup.bash && exec ros2 run vins_estimator vins_estimator --ros-args -p config_file:=/root/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/dummy_13_7.yaml --remap /feature_tracker/feature:=/feature --remap /feature_tracker/restart:=/restart" &
    PID3=$!

    # 4. OpenHD streamer: /image_color -> H.264 :5600 (с оверлеем детекций NN).
    #    Камера сама OpenHD не гонит (stream_openhd:=false). Боевые ноды NN1/NN2
    #    пока не запускаются — стример отдаёт чистое видео, рамки появятся, когда
    #    нейросети начнут публиковать /nn1/detections, /nn2/scene.
    docker exec -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=0 $CONTAINER bash -c "source /root/vins_ws/install/setup.bash && exec ros2 run nav_pkg openhd_streamer" &
    PID4=$!
}

# Функция для остановки нод VINS-Mono
stop_vins_nodes() {
    if [[ -n "$PID1" ]] || [[ -n "$PID2" ]] || [[ -n "$PID3" ]] || [[ -n "$PID4" ]]; then
        echo "Останавливаем процессы VINS-Mono..."
        kill -INT $PID1 $PID2 $PID3 $PID4 2>/dev/null
        wait $PID1 $PID2 $PID3 $PID4 2>/dev/null
        PID1=""
        PID2=""
        PID3=""
        PID4=""
    fi
}

# Функция очистки при остановке сервиса systemd
cleanup() {
    echo "Сигнал остановки сервиса! Завершаем процессы..."
    stop_vins_nodes
    
    echo "Останавливаем контейнер $CONTAINER..."
    docker stop $CONTAINER
    
    # Жестко прибиваем зависающую утилиту ros2 на хосте
    pkill -9 -f "ros2 topic echo /mavros/state" 2>/dev/null
    
    exit 0
}

# Ловим сигналы от systemd
trap cleanup SIGINT SIGTERM

# ==========================================
# 3. Основная логика работы
# ==========================================

echo "Перезапуск контейнера $CONTAINER..."
docker restart $CONTAINER
sleep 2

echo "Ожидание запуска MAVROS..."
while ! ros2 topic list 2>/dev/null | grep "/mavros/state" >/dev/null 2>&1; do
    sleep 1
done

echo "Ожидание арминга в топике /mavros/state..."

# Читаем топик в основном процессе, 
# чтобы PIDs сохранялись в глобальной области видимости
while read -r line; do
    
    # Если заармились и PID1 пустой (значит процессы еще не запущены)
    if [[ "$line" == *"true"* ]] && [[ -z "$PID1" ]]; then
        echo "Дрон ЗААРМЛЕН! Инициализация VINS-Mono..."
        start_vins_nodes
        
    # Если дизармились и PID1 не пустой (значит процессы работают)
    elif [[ "$line" == *"false"* ]] && [[ -n "$PID1" ]]; then
        echo "Дрон ДИЗАРМЛЕН! Завершение VINS-Mono..."
        stop_vins_nodes
    fi

done < <(stdbuf -oL ros2 topic echo /mavros/state mavros_msgs/msg/State | grep --line-buffered "armed:")