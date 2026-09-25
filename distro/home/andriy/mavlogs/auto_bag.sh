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

# Остановка записи. Одного SIGINT мало: пришедший, пока ros2 bag record ещё стартует, теряется,
# и wait висит вечно (2026-09-25: ложный арм/дизарм за 1 с при переподключении MAVROS — рекордер
# писал 207 с, служба не видела следующих армов). INT → 5 с → INT → 3 с → TERM → 2 с → KILL.
stop_bag() {
    [[ -z "$BAG_PID" ]] && return
    local sig
    for sig in INT:10 INT:6 TERM:4 KILL:0; do
        kill -"${sig%%:*}" "$BAG_PID" 2>/dev/null || break
        for ((i = 0; i < ${sig##*:}; i++)); do kill -0 "$BAG_PID" 2>/dev/null || break 2; sleep 0.5; done
    done
    wait "$BAG_PID" 2>/dev/null
    BAG_PID=""
}

# Функция, которая срабатывает при sudo systemctl stop
cleanup() {
    echo "Сигнал остановки! Закрываем логи..."
    
    # Если запись шла, корректно закрываем bag-файл
    stop_bag
    
    # Жестко прибиваем зависающую утилиту ros2, чтобы не ждать таймаута
    # ТОЛЬКО в своей сессии (у каждой службы systemd своя): без -s этот pkill убивал
    # одноимённый наблюдатель СОСЕДНЕЙ службы (2026-09-25: стоп vins уложил auto-bag)
    pkill -9 -s "$(ps -o sid= -p $$ | tr -d ' ')" -f "ros2 topic echo /mavros/state" 2>/dev/null
    
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
        # Частоты IMU — ЗАНОВО на каждом арме: start_mavros.sh ставит их один раз при старте
        # MAVROS, а перезагрузка одного полётника их стирает (постоянных MAV1_* нет) — 2026-09-25
        # после ребута FCU 14 bag'ов вышли по 0 сообщений. Запрос — НАПРЯМУЮ через mavlink-router
        # (частоты задаются на USB-канал FCU, MAVROS получит те же), не сервисом MAVROS: 2026-09-25
        # MAVROS упал (exit 245) ровно во время этих вызовов сервиса. В фоне: запись не ждёт.
        python3 - <<'PY' &
import time
from pymavlink import mavutil
m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=250, source_component=194)
m.wait_heartbeat(timeout=5)
for mid in (105, 27, 31):   # HIGHRES_IMU (→ imu/data_raw), RAW_IMU, ATTITUDE_QUATERNION (→ imu/data)
    m.mav.command_long_send(1, 1, 511, 0, mid, 5000, 0, 0, 0, 0, 0)   # 5000 мкс = 200 Гц
    time.sleep(0.1)
print("частоты IMU 200 Гц запрошены у FCU")
PY
        # весь IMU MAVROS + состояние (время арма/режим) + прежние топики VINS/камеры
        ros2 bag record -o "$BAG_NAME" \
            /mavros/imu/data_raw /mavros/imu/data /mavros/imu/mag /mavros/imu/static_pressure \
            /mavros/imu/diff_pressure /mavros/imu/temperature_imu /mavros/imu/temperature_baro \
            /mavros/state \
            /image_mono /camera_info /odometry /path /feature &
        BAG_PID=$!
        
    elif [[ "$line" == *"false"* ]] && [[ -n "$BAG_PID" ]]; then
        echo "Дрон ДИЗАРМЛЕН! Останавливаю запись..."
        stop_bag
        echo "Запись остановлена"
    fi

done < <(stdbuf -oL ros2 topic echo /mavros/state mavros_msgs/msg/State | grep --line-buffered "armed:")

# Сюда попадаем, только если наблюдатель /mavros/state умер (убит, упал DDS). Выход с
# ошибкой — чтобы Restart=on-failure поднял службу; с кодом 0 она молча гасла.
stop_bag
echo "Наблюдатель /mavros/state завершился — выход с ошибкой для перезапуска"
exit 1
