#!/usr/bin/env bash
# Наземная сверка TX12 (стек поднимать НЕ нужно, только контейнер nav):
# поднимает joy_linux_node и печатает живую интерпретацию стиков ядром JoyPilot.
# Лётный стек не трогает (дисциплина прогона не нарушается — это чтение input-
# устройства, без FCU/Gazebo).
#
# Запуск с хоста:  docker exec -it p1317_nav bash /lab/joy_check.sh
# Env: JOY_DEV (/dev/input/js0)
set -e
source /opt/ros/humble/setup.bash
source /root/sim_ws/install/setup.bash 2>/dev/null || true

JOY_DEV="${JOY_DEV:-/dev/input/js0}"
if [ ! -e "$JOY_DEV" ]; then
    echo "ОШИБКА: $JOY_DEV нет. Пульт воткнут и в режиме USB Joystick (HID)?"
    echo "  на хосте:      ls /dev/input/js*"
    echo "  сырой HID:     docker exec -it p1317_nav jstest $JOY_DEV"
    exit 1
fi

ros2 run joy_linux joy_linux_node --ros-args \
    -p dev:="$JOY_DEV" -p autorepeat_rate:=20.0 \
    > /tmp/joy_check_driver.log 2>&1 &
JOY_PID=$!
trap 'kill "$JOY_PID" 2>/dev/null || true' EXIT

python3 /lab/joy_check.py
