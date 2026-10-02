#!/bin/bash
# vins_service_m.sh — ручной режим (служба vins_m): ноды стека стартуют сразу, без
# ожидания арминга, и живут до остановки службы. Ноды и их останов — vins_nodes.sh.

export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source "$(dirname "$(readlink -f "$0")")/vins_nodes.sh"

cleanup() {
    echo "Сигнал остановки! Завершаем ноды..."
    stop_vins_nodes
    echo "Останавливаем контейнер $CONTAINER..."
    docker stop $CONTAINER
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "Перезапуск контейнера $CONTAINER..."
docker restart $CONTAINER
sleep 2

start_vins_nodes

# Держим службу, пока живы клиенты docker exec. wait прерывается сигналом → trap.
# Умерла любая нода — гасим остальные и выходим с ошибкой (Restart=on-failure).
wait -n "${NODE_PIDS[@]}"
echo "Одна из нод завершилась — останавливаем остальные, выход с ошибкой для перезапуска"
stop_vins_nodes
exit 1
