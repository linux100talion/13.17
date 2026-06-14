#!/bin/bash

# sudo systemctl start auto-bag.service
# sudo systemctl stop auto-bag.service
# sudo systemctl enable auto-bag.service
# sudo journalctl -u auto-bag.service -f


# 2. Если ты используешь специфичные сетевые настройки, их нужно явно указать здесь,
# так как systemd игнорирует .bashrc:
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0



# ВАЖНО 1: Если вы используете CycloneDDS в терминале (что стандартно для Jetson + камеры),
# systemd об этом не знает и запускает дефолтный FastRTPS. Раскомментируйте строку ниже, если используете Cyclone!
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Загружаем базовый ROS 2
source /opt/ros/humble/setup.bash

# ВАЖНО 2: Обязательно укажите путь к вашему воркспейсу, где скомпилирован VINS-Mono!
# Без этого ros2 bag может не понимать структуру некоторых сообщений, если они кастомные.
# Замените 'your_workspace' на актуальную папку.
# Дело в том, что VINS-Mono для топика /feature (и всех остальных) использует 
# стандартные типы сообщений ROS 2 — в частности sensor_msgs/msg/PointCloud. 
# Информацию о фичах (ID точек, их координаты, скорости) он хитро запаковывает 
# в стандартные массивы каналов (channels) этого сообщения.
# Утилита ros2 bag record, запущенная на хосте, уже знает структуру всех сообщений, 
# которые будет публиковать VINS-Mono, так как они входят в базовый пакет ROS 2.
# Поэтому это не нужно - 
# source /home/andriy/your_workspace/install/setup.bash








LOG_DIR="/home/andriy/mavlogs"
mkdir -p "$LOG_DIR"

BAG_PID=""

# Функция, которая срабатывает при sudo systemctl stop
cleanup() {
    echo "Сигнал остановки! Закрываем логи..."
    
    # Если запись шла, корректно закрываем bag-файл
    if [[ -n "$BAG_PID" ]]; then
        kill -INT "$BAG_PID" 2>/dev/null
        wait "$BAG_PID" 2>/dev/null
    fi
    
    # Жестко прибиваем зависающую утилиту ros2, чтобы не ждать таймаута
    pkill -9 -f "ros2 topic echo /mavros/state" 2>/dev/null
    
    exit 0
}

# Ловим сигналы остановки от systemd
trap cleanup SIGINT SIGTERM

echo "Ожидание запуска MAVROS..."
while ! ros2 topic list 2>/dev/null | grep "/mavros/state" >/dev/null 2>&1; do
    sleep 1
done

echo "Ожидание арминга в топике /mavros/state..."

# Используем Process Substitution < <(...) вместо pipe `|`
# Теперь цикл работает в основном процессе, и BAG_PID не теряется
while read -r line; do
    
    if [[ "$line" == *"true"* ]] && [[ -z "$BAG_PID" ]]; then
        BAG_NAME="$LOG_DIR/bag_$(date +%Y%m%d_%H%M%S)"
        
        echo "Дрон ЗААРМЛЕН! Начинаю запись в $BAG_NAME..."
        #ros2 bag record -o "$BAG_NAME" /mavros/imu/data_raw /mavros/imu/data /image_mono /odometry /path /feature &
        ros2 bag record -o "$BAG_NAME" /mavros/imu/data_raw /mavros/imu/data /image_mono /camera_info /odometry /path /feature &
BAG_PID=$!
        BAG_PID=$!
        
    elif [[ "$line" == *"false"* ]] && [[ -n "$BAG_PID" ]]; then
        echo "Дрон ДИЗАРМЛЕН! Останавливаю запись..."
        kill -INT "$BAG_PID"
        wait "$BAG_PID" 2>/dev/null
        BAG_PID=""
    fi

done < <(stdbuf -oL ros2 topic echo /mavros/state mavros_msgs/msg/State | grep --line-buffered "armed:")