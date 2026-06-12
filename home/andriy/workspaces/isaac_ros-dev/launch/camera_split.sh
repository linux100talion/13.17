#!/bin/bash

# 1. Прописываем пути к аппаратному ускорению CUDA
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# 2. Инициализируем базовый ROS2 и рабочее пространство Isaac ROS
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

# 3. Включаем CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4. Запускаем конвейер по абсолютному пути
ros2 launch /workspaces/isaac_ros-dev/launch/camera_split.launch.py