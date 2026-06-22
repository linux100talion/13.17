#!/usr/bin/env bash
# Переводит дрон в GUIDED, армирует и взлетает.
# Запуск внутри nav-контейнера:
#   docker exec p1317_nav bash /lab/arm_takeoff.sh [ALTITUDE_M]
# Или через make: make arm
set -e
source /opt/ros/humble/setup.bash

ALT=${1:-3}

echo ">>> Режим GUIDED..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    '{custom_mode: "GUIDED"}' | grep -o 'mode_sent: [a-z]*'

sleep 1

echo ">>> Армирование..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
    '{value: true}' | grep -o 'result: [0-9]*'

sleep 2

echo ">>> Взлёт на ${ALT} м..."
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
    "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: ${ALT}}" \
    | grep -o 'result: [0-9]*'

echo ">>> Ждём набора высоты ${ALT} м (~${ALT}с)..."
sleep $((ALT + 2))
echo ">>> Готово."
