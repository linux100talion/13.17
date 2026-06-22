#!/usr/bin/env bash
# Посадка дрона (режим LAND).
# Запуск: docker exec p1317_nav bash /lab/land.sh
# Или через make: make land
set -e
source /opt/ros/humble/setup.bash

echo ">>> Режим LAND..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    '{custom_mode: "LAND"}' | grep -o 'mode_sent: [a-z]*'
echo ">>> Садимся."
