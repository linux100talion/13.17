#!/usr/bin/env bash
# Посадка дрона (режим LAND).
# Запуск: docker exec p1317_nav bash /lab/land.sh
# Или через make: make land
set -e
source /opt/ros/humble/setup.bash

echo ">>> Режим LAND..."
# ros2 печатает "mode_sent=True" (через '=', с большой буквы) — grep без регистра
# и с '|| true', иначе несовпадение паттерна валит set -e (команда LAND при этом
# уже ушла, дрон садится).
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    '{custom_mode: "LAND"}' 2>&1 | grep -oiE 'mode_sent=[a-z]+' || true
echo ">>> Садимся."
