#!/usr/bin/env bash
# ============================================================================
# Запуск симуляции ВНУТРИ контейнера simulator: Gazebo + ArduPilot SITL + мост.
# Исполняется так:  docker exec -i p1317_simulator bash -s < scripts/sim_up.sh
#                   (или make sim)
#
# Все процессы — в фоне (nohup), логи в /root/output (== docker/sim/output).
# bash -s = неинтерактивный шелл: ENV из Dockerfile есть (PATH к SITL),
# но .bashrc НЕ читается — ROS подключаем явно.
# ============================================================================
set -eo pipefail
source /opt/ros/humble/setup.bash

LOG=/root/output; mkdir -p "$LOG"
WORLD="${WORLD:-/root/worlds/mili_fortress.sdf}"

# 1. Gazebo Harmonic — мир + дрон с камерой (ArduPilotPlugin слушает 9002).
if ! pgrep -f "gz sim" >/dev/null; then
    nohup gz sim -s --headless-rendering -v4 -r "$WORLD" >"$LOG/gz_sim.log" 2>&1 &
    echo "  gz sim   -> $LOG/gz_sim.log"
else
    echo "  gz sim   уже запущен"
fi

sleep 5  # дать Gazebo поднять физику/плагины до подключения SITL

# 2. ArduPilot SITL — подключается к ArduPilotPlugin@9002, MAVLink на tcp:5760.
#    --no-mavproxy: телеметрию разводит mavlink_router, консоль MAVProxy не нужна.
if ! pgrep -f "sim_vehicle" >/dev/null; then
    nohup sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
        --add-param-file=/root/ardupilot/Tools/autotest/default_params/gazebo-iris.parm \
        --add-param-file=/root/sitl-extra.parm \
        --no-rebuild --no-mavproxy >"$LOG/sitl.log" 2>&1 &
    echo "  SITL     -> $LOG/sitl.log"
else
    echo "  SITL     уже запущен"
fi

# 3. Мост Gazebo -> ROS2: камера + /clock (источник sim-времени).
if ! pgrep -f "ros_gz_bridge" >/dev/null; then
    nohup ros2 run ros_gz_bridge parameter_bridge \
        "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image" \
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
        >"$LOG/ros_gz_bridge.log" 2>&1 &
    echo "  ros_gz_bridge -> $LOG/ros_gz_bridge.log"
else
    echo "  ros_gz_bridge уже запущен"
fi

echo "simulator: готово. Логи: docker/sim/output/"
