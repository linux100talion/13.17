#!/bin/bash

# 1. Загружаем базовое окружение ROS2
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Фоновый процесс для гарантированного разгона частоты конкретных сообщений IMU
(
    echo "=== СТАРТ ФОНОВОГО ПРОЦЕССА СИНХРОНИЗАЦИИ ===" > /tmp/mavros_stream.log
    sleep 5
    for i in 1 2 3; do
        echo "[Попытка $i] Запрашиваем HIGHRES_IMU (ID 105) и RAW_IMU (ID 27)..." >> /tmp/mavros_stream.log
        
        # Запрашиваем HIGHRES_IMU (основной источник для MAVROS IMU) на 200 Гц
        ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}" >> /tmp/mavros_stream.log 2>&1
        
        # Запрашиваем RAW_IMU на 200 Гц (на случай, если полетник шлет сырые данные)
        ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}" >> /tmp/mavros_stream.log 2>&1
        
        # Откройте новый терминал на Jetson Orin Nano и проверьте реальную частоту топика,
        # который будет читать VINS-Mono:
        # ros2 topic hz /mavros/imu/data_raw
        sleep 2
    done
    echo "=== ФОНОВЫЙ ПРОЦЕСС ЗАВЕРШЕН ===" >> /tmp/mavros_stream.log
) &

# 2. Запускаем MAVROS
exec ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://127.0.0.1:14555@ \
  -p plugins.ftp.enable:=false