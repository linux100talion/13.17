#!/bin/bash

# sudo systemctl start auto-bag-m.service
# sudo systemctl stop auto-bag-m.service
# sudo systemctl enable auto-bag-m.service
# sudo journalctl -u auto-bag-m.service -f


# Сетевые настройки
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
# source /home/andriy/your_workspace/install/setup.bash



LOG_DIR="/home/andriy/mavlogs"
mkdir -p "$LOG_DIR"

BAG_PID=""

# Функция, которая срабатывает при остановке скрипта (Ctrl+C или systemctl stop)
cleanup() {
    echo "Сигнал остановки! Закрываем логи..."
    
    if [[ -n "$BAG_PID" ]]; then
        # Корректно завершаем запись (SIGINT = Ctrl+C)
        kill -INT "$BAG_PID" 2>/dev/null
        wait "$BAG_PID" 2>/dev/null
    fi
    
    exit 0
}

# Ловим сигналы остановки
trap cleanup SIGINT SIGTERM

echo "Ожидание запуска MAVROS..."
# Ждем, пока в системе не появится топик MAVROS
while ! ros2 topic list | grep "/mavros/state" > /dev/null; do
    sleep 1
done


# === ПРЯМАЯ ЗАПИСЬ БЕЗ АРМИНГА ===

BAG_NAME="$LOG_DIR/bag_$(date +%Y%m%d_%H%M%S)"
echo "MAVROS запущен! Начинаю немедленную запись в $BAG_NAME..."

# Запускаем процесс записи в фон
#ros2 bag record -o "$BAG_NAME" /mavros/imu/data_raw /mavros/imu/data /image_mono /odometry /path /feature &
ros2 bag record -o "$BAG_NAME" /mavros/imu/data_raw /mavros/imu/data /image_mono /camera_info /odometry /path /feature &
BAG_PID=$!

# Заставляем bash-скрипт висеть и не закрываться, пока работает запись
wait "$BAG_PID"