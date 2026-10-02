#!/bin/bash

# ==========================================
# 1. Настройки окружения ROS 2 на хосте
# ==========================================
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0

# Если используется CycloneDDS (как в скрипте записи bag), раскомментируйте:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml   # лимит участников DDS (etc/cyclonedds)

# Загружаем базовый ROS 2 на хосте, чтобы утилита `ros2 topic echo` работала
source /opt/ros/humble/setup.bash

# ==========================================
# 2. Ноды стека (общий файл с vins_service_m.sh)
# ==========================================
source "$(dirname "$(readlink -f "$0")")/vins_nodes.sh"

# Функция очистки при остановке сервиса systemd
cleanup() {
    echo "Сигнал остановки сервиса! Завершаем процессы..."
    stop_vins_nodes
    
    echo "Останавливаем контейнер $CONTAINER..."
    docker stop $CONTAINER
    
    # Жестко прибиваем зависающую утилиту ros2 на хосте
    # ТОЛЬКО в своей сессии (у каждой службы systemd своя): без -s этот pkill убивал
    # одноимённый наблюдатель СОСЕДНЕЙ службы (2026-09-25: стоп vins уложил auto-bag)
    pkill -9 -s "$(ps -o sid= -p $$ | tr -d ' ')" -f "ros2 topic echo /mavros/state" 2>/dev/null
    
    exit 0
}

# Ловим сигналы от systemd
trap cleanup SIGINT SIGTERM

# ==========================================
# 3. Основная логика работы
# ==========================================

echo "Перезапуск контейнера $CONTAINER..."
docker restart $CONTAINER
sleep 2

echo "Ожидание запуска MAVROS..."
while ! ros2 topic list 2>/dev/null | grep "/mavros/state" >/dev/null 2>&1; do
    sleep 1
done

echo "Ожидание арминга в топике /mavros/state..."

# Читаем топик в основном процессе, 
# чтобы PIDs сохранялись в глобальной области видимости
while read -r line; do
    
    # Если заармились и ноды ещё не запущены
    if [[ "$line" == *"true"* ]] && [[ ${#NODE_PIDS[@]} -eq 0 ]]; then
        echo "Дрон ЗААРМЛЕН! Инициализация VINS-Mono..."
        start_vins_nodes
        
    # Если дизармились и ноды работают
    elif [[ "$line" == *"false"* ]] && [[ ${#NODE_PIDS[@]} -gt 0 ]]; then
        echo "Дрон ДИЗАРМЛЕН! Завершение VINS-Mono..."
        stop_vins_nodes
    fi

done < <(stdbuf -oL ros2 topic echo /mavros/state mavros_msgs/msg/State | grep --line-buffered "armed:")

# Сюда попадаем, только если наблюдатель /mavros/state умер (убит, упал DDS). Выход с
# ошибкой — чтобы Restart=on-failure поднял службу; с кодом 0 она молча гасла.
stop_vins_nodes
echo "Наблюдатель /mavros/state завершился — выход с ошибкой для перезапуска"
exit 1
