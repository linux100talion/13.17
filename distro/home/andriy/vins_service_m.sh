#!/bin/bash

# Сетевые настройки
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CONTAINER="vins_project_13_7"
PID1=""
PID2=""
PID3=""


#ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 1, message_rate: 200, on_off: true}"
#ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 10, message_rate: 200, on_off: true}"


cleanup() {
    echo "Сигнал остановки! Завершаем процессы VINS-Mono..."
    
    # Мягко останавливаем ноды
    if [[ -n "$PID1" ]] || [[ -n "$PID2" ]] || [[ -n "$PID3" ]]; then
        kill -INT $PID1 $PID2 $PID3 2>/dev/null
        wait $PID1 $PID2 $PID3 2>/dev/null
    fi
    
    # Останавливаем сам контейнер
    docker stop $CONTAINER
    exit 0
}

# Ловим сигналы от systemd
trap cleanup SIGINT SIGTERM

echo "Перезапуск контейнера $CONTAINER..."
docker restart $CONTAINER
sleep 2

echo "Запуск нод..."

# 1. Camera Node (C++ CUDA)
docker exec -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=0 $CONTAINER bash -c "source /root/vins_ws/install/setup.bash && exec ros2 run camera_pkg camera_node" &
PID1=$!

# 2. Feature Tracker
docker exec -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=0 $CONTAINER bash -c "source /root/vins_ws/install/setup.bash && exec ros2 run feature_tracker feature_tracker --ros-args -p config_file:=/root/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/dummy_13_7.yaml" &
PID2=$!

# 3. VINS Estimator
docker exec -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=0 $CONTAINER bash -c "source /root/vins_ws/install/setup.bash && exec ros2 run vins_estimator vins_estimator --ros-args -p config_file:=/root/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/dummy_13_7.yaml --remap /feature_tracker/feature:=/feature --remap /feature_tracker/restart:=/restart" &
PID3=$!

# Удерживаем скрипт активным
wait $PID1 $PID2 $PID3