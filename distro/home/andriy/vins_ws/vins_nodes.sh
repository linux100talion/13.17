#!/bin/bash
# vins_nodes.sh — общий запуск/останов нод бортового стека в контейнере. Подключается
# (source) из vins_service.sh (по армингу) и vins_service_m.sh (сразу, ручной режим).
#
# Ноды (контейнер из docker/orin/, код — ~/13.17/src, собран colcon'ом в томе):
#   camera_node      C++ CUDA: V4L2 → /image_mono (VINS) + /image_color (nav)
#   feature_tracker  VINS, бортовой config.yaml
#   vins_estimator   VINS → /odometry
#   openhd_streamer  /image_color → H.264 :5600 (камера сама OpenHD не гонит)
#
# ОСТАНОВ — ИЗНУТРИ контейнера (pkill по шаблону). Прежняя версия слала kill -INT
# PID'у КЛИЕНТА docker exec: до процесса в контейнере сигнал не доходил, ноды не
# умирали и копились на каждом арме (36+36 после 44 армов → кончились участники
# CycloneDDS, auto-bag не стартовал; 2026-09-25). Перед стартом — та же зачистка.

CONTAINER="vins_project_13_7"
CFG=/root/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/config.yaml

# Шаблоны pkill -f: путь исполняемого файла в install/. Скобка [x] — обязательна: у
# контейнера pid: host, pkill/pgrep изнутри видят и ХОСТ, в т.ч. клиента `docker exec …
# pkill -f <шаблон>` — без скобки шаблон совпадал с его же командной строкой (pkill убивал
# свой docker exec, pgrep всегда видел «живую» ноду). Регэксп lib/x[y]z совпадает с
# «lib/xyz», но не с текстом «lib/x[y]z».
NODE_PATTERNS=(
    "lib/camera_pk[g]/camera_node"
    "lib/feature_tracke[r]/feature_tracker"
    "lib/vins_estimato[r]/vins_estimator"
    "lib/nav_pk[g]/openhd_streamer"
)
NODE_PIDS=()

dexec() {   # docker exec с окружением ROS; $1 — команда
    docker exec -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        "$CONTAINER" bash -c "source /root/vins_ws/install/setup.bash && $1"
}

nodes_alive() {   # сколько наших нод живо в контейнере
    local n=0 p
    for p in "${NODE_PATTERNS[@]}"; do
        docker exec "$CONTAINER" pgrep -f "$p" >/dev/null 2>&1 && n=$((n + 1))
    done
    echo $n
}

kill_nodes() {   # SIGINT → до 5 с → SIGKILL остаткам; всё внутри контейнера
    local p i
    for p in "${NODE_PATTERNS[@]}"; do
        docker exec "$CONTAINER" pkill -INT -f "$p" 2>/dev/null
    done
    for i in $(seq 10); do
        [ "$(nodes_alive)" -eq 0 ] && break
        sleep 0.5
    done
    for p in "${NODE_PATTERNS[@]}"; do
        docker exec "$CONTAINER" pkill -KILL -f "$p" 2>/dev/null
    done
}

start_vins_nodes() {
    echo "Запуск нод в контейнере $CONTAINER..."
    kill_nodes   # на случай остатков прошлого запуска
    NODE_PIDS=()
    dexec "exec ros2 run camera_pkg camera_node --ros-args -p stream_openhd:=false" &
    NODE_PIDS+=($!)
    dexec "exec ros2 run feature_tracker feature_tracker --ros-args -p config_file:=$CFG" &
    NODE_PIDS+=($!)
    dexec "exec ros2 run vins_estimator vins_estimator --ros-args -p config_file:=$CFG \
        --remap /feature_tracker/feature:=/feature --remap /feature_tracker/restart:=/restart" &
    NODE_PIDS+=($!)
    dexec "exec ros2 run nav_pkg openhd_streamer" &
    NODE_PIDS+=($!)
}

stop_vins_nodes() {
    [ ${#NODE_PIDS[@]} -eq 0 ] && return
    echo "Останавливаем ноды..."
    kill_nodes
    wait "${NODE_PIDS[@]}" 2>/dev/null   # клиенты docker exec выходят следом за нодами
    NODE_PIDS=()
    echo "Ноды остановлены, живых: $(nodes_alive)"
}
