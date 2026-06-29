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

# Разрешение камеры: env CAMERA_W/CAMERA_H (default 1280×720). В GPU-less прогоне
# (llvmpipe) CPU-оверрайд compose ставит 320×180 — в ~16 раз меньше пикселей под
# софтрендер. SDF статичен (gz не подставляет env), поэтому при не-дефолтном
# разрешении кладём ПАТЧЕНУЮ копию модели iris_cam в /tmp и выводим её первой в
# GZ_SIM_RESOURCE_PATH — репозиторную модель не трогаем (git чист).
CAM_W="${CAMERA_W:-1280}"; CAM_H="${CAMERA_H:-720}"
if [ "$CAM_W" != "1280" ] || [ "$CAM_H" != "720" ]; then
    PATCH=/tmp/sim_models
    rm -rf "$PATCH"; mkdir -p "$PATCH"
    cp -a /root/worlds/iris_cam "$PATCH/iris_cam"
    sed -i "s|<width>1280</width>|<width>${CAM_W}</width>|; \
            s|<height>720</height>|<height>${CAM_H}</height>|" \
        "$PATCH/iris_cam/model.sdf"
    export GZ_SIM_RESOURCE_PATH="$PATCH:${GZ_SIM_RESOURCE_PATH}"
    echo "  камера: SDF пропатчен до ${CAM_W}x${CAM_H} (модель из $PATCH)"
fi

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
    # SITL пишет eeprom.bin (accel-калибровка + параметры) в cwd. Запускаем из
    # /root/sitl_state — это named volume sitl_eeprom, поэтому eeprom переживает
    # fresh-start (одноразовая калибровка не повторяется на каждом старте).
    # --defaults (--add-param-file) применяется на каждом boot поверх eeprom, так
    # что правки sitl-extra.parm по-прежнему подхватываются, а калибровка (её нет
    # в .parm) сохраняется в eeprom. См. docker/sim/todo.txt.
    mkdir -p /root/sitl_state
    ( cd /root/sitl_state && nohup sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
        --add-param-file=/root/ardupilot/Tools/autotest/default_params/gazebo-iris.parm \
        --add-param-file=/root/sitl-extra.parm \
        --no-rebuild --no-mavproxy >"$LOG/sitl.log" 2>&1 & )
    echo "  SITL     -> $LOG/sitl.log (cwd=/root/sitl_state, eeprom персистентный)"
else
    echo "  SITL     уже запущен"
fi

# 3. Мост Gazebo -> ROS2: камера + /clock (источник sim-времени) + ground-truth
#    одометрия дрона (СИМ-костыль для gz-position-hold в alt_hold_bootstrap).
if ! pgrep -f "ros_gz_bridge" >/dev/null; then
    nohup ros2 run ros_gz_bridge parameter_bridge \
        "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image" \
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
        "/model/iris_cam/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
        >"$LOG/ros_gz_bridge.log" 2>&1 &
    echo "  ros_gz_bridge -> $LOG/ros_gz_bridge.log"
else
    echo "  ros_gz_bridge уже запущен"
fi

echo "simulator: готово. Логи: docker/sim/output/"
