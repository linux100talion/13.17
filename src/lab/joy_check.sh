#!/usr/bin/env bash
# Наземная сверка TX12 (стек поднимать НЕ нужно, только контейнер nav):
# поднимает joy_linux_node и печатает живую интерпретацию стиков ядром JoyPilot.
# Лётный стек не трогает (дисциплина прогона не нарушается — это чтение input-
# устройства, без FCU/Gazebo).
#
# Запуск с хоста:  bash src/lab/joy_check.sh   (сам перезапустится внутри nav)
# Env: JOY_DEV (/dev/input/js0), NAV (p1317_nav)
set -e

# НА ХОСТЕ ROS нет — перезапускаемся внутри контейнера nav (скрипт смонтирован
# как /lab/joy_check.sh; /dev/input проброшен каталогом, hotplug работает)
if [ ! -f /opt/ros/humble/setup.bash ]; then
    NAV="${NAV:-p1317_nav}"
    if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$NAV"; then
        echo "ОШИБКА: контейнер $NAV не бежит — поднять: cd docker/sim && make up" >&2
        echo "  (для сверки пульта хватит поднятого контейнера, полёт не нужен)" >&2
        exit 1
    fi
    exec docker exec -it -e JOY_DEV="${JOY_DEV:-/dev/input/js0}" "$NAV" \
        bash /lab/joy_check.sh
fi

source /opt/ros/humble/setup.bash
source /root/sim_ws/install/setup.bash 2>/dev/null || true

JOY_DEV="${JOY_DEV:-/dev/input/js0}"
if [ ! -e "$JOY_DEV" ]; then
    echo "ОШИБКА: $JOY_DEV нет. Пульт воткнут и в режиме USB Joystick (HID)?"
    echo "  на хосте:      ls /dev/input/js*"
    echo "  сырой HID:     docker exec -it p1317_nav jstest $JOY_DEV"
    exit 1
fi

# default_trig_val — публиковать НАЧАЛЬНОЕ состояние осей (JS_EVENT_INIT): без него
# тумблер, выставленный ДО старта ноды, невидим (ось 0.00 до первого щелчка) —
# полёт 2026-08-17: CH6 вверх до взлёта → нода видела «центр» → летели без стабилизатора.
ros2 run joy_linux joy_linux_node --ros-args \
    -p dev:="$JOY_DEV" -p autorepeat_rate:=20.0 -p default_trig_val:=true \
    > /tmp/joy_check_driver.log 2>&1 &
JOY_PID=$!
trap 'kill "$JOY_PID" 2>/dev/null || true' EXIT

python3 /lab/joy_check.py
