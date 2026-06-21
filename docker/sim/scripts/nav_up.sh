#!/usr/bin/env bash
# ============================================================================
# Запуск nav-стороны ВНУТРИ контейнера nav: colcon build + ноды + MAVROS.
# Исполняется так:  docker exec -i p1317_nav bash -s < scripts/nav_up.sh
#                   (или make nav)
#
# Порядок source важен: ROS -> overlay (cv_bridge против CUDA-OpenCV) -> ws.
# ============================================================================
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/overlay/install/setup.bash

LOG=/root/sim_ws/output; mkdir -p "$LOG"
cd /root/sim_ws

# 1a. vins_oss — клонируется вне монтированного src/, поэтому исчезает при
#     docker compose down. Клонируем и патчим если нет.
if [ ! -d src/vins_oss ]; then
    echo "  клонируем VINS-MONO-ROS2..."
    git clone --depth 1 https://github.com/dongbo19/VINS-MONO-ROS2 src/vins_oss

    # rclcpp::Duration(0) — убран single-int конструктор в Humble
    sed -i 's/rclcpp::Duration(0)/rclcpp::Duration(0, 0)/g' \
        src/vins_oss/vins_estimator/src/utility/visualization.cpp

    # IMU QoS: MAVROS публикует BEST_EFFORT → подписка тоже должна быть BEST_EFFORT
    sed -i '357s/rclcpp::QoS(rclcpp::KeepLast(2000))/rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort()/' \
        src/vins_oss/vins_estimator/src/estimator_node.cpp

    # IMU монотонный патч: /clock ~155 Гц < 250 Гц IMU → несколько сообщений
    # получают одинаковый timestamp → VINS бросает "imu message in disorder".
    # Вместо drop: принудительная монотонность t = last_imu_t + 1e-6;
    # плюс перезапись stamp в сообщении чтобы нижнее обработка видела то же t.
    python3 - <<'PYEOF'
import re, sys
fname = "src/vins_oss/vins_estimator/src/estimator_node.cpp"
with open(fname) as f:
    content = f.read()
if "last_imu_t + 1e-6" in content:
    print("  IMU monotonic patch: already applied")
    sys.exit(0)
# Оригинал: if (t <= last_imu_t) { WARN; return; } last_imu_t = t;
pattern = r'(    if \(t <= last_imu_t\)\s*\{[^}]*\}\s*last_imu_t = t;)'
replacement = (
    "    // Sim-time updates slower than IMU (250Hz > 155Hz /clock):\n"
    "    // multiple msgs share the same timestamp. Enforce monotonicity\n"
    "    // instead of dropping — no IMU measurement is lost.\n"
    "    if (t <= last_imu_t)\n"
    "        t = last_imu_t + 1e-6;\n"
    "    last_imu_t = t;\n"
    "    // Rewrite stamp so downstream processing also sees corrected t.\n"
    "    const_cast<sensor_msgs::msg::Imu*>(imu_msg.get())->header.stamp.sec = (int32_t)t;\n"
    "    const_cast<sensor_msgs::msg::Imu*>(imu_msg.get())->header.stamp.nanosec =\n"
    "        (uint32_t)((t - (int32_t)t) * 1e9);\n"
    "    last_imu_t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);"
)
new_content, n = re.subn(pattern, replacement, content, flags=re.DOTALL)
if n == 0:
    print("  IMU monotonic patch: WARNING pattern not found, skipped", file=sys.stderr)
    sys.exit(0)
with open(fname, "w") as f:
    f.write(new_content)
print(f"  IMU monotonic patch: applied ({n} location)")
PYEOF
fi

# 1b. Сборка workspace — только если ещё не собран.
#     Для пересборки: make nav-rebuild (или rm -rf install внутри).
if [ ! -f install/setup.bash ]; then
    echo "  colcon build ..."
    colcon build
fi
source install/setup.bash

# 2. Байеризатор: Gazebo RGB → /dev/rawbayer (v4l2loopback).
#    Запускается ВНЕ sim_nav.launch.py: если запустить внутри launch, его крах
#    убивает весь launch (camera_node + VINS). Здесь он изолирован.
#    Eager-init в __init__: открывает /dev/rawbayer, пишет нулевой кадр —
#    только после этого v4l2loopback разрешает G_FMT на стороне capture.
if ! pgrep -f "bayerizer.py" >/dev/null; then
    nohup python3 /root/sim_ws/src/sim/bayerizer.py \
        --ros-args \
        -p input_topic:=/camera/image_raw \
        -p device:=/dev/rawbayer \
        -p pattern:=GRBG \
        -p use_sim_time:=true \
        >"$LOG/bayerizer.log" 2>&1 &
    echo "  bayerizer -> $LOG/bayerizer.log"
    # Ждём пока байеризатор активирует capture-сторону v4l2loopback (eager-init).
    echo -n "  ожидаем /dev/rawbayer..."
    for i in $(seq 1 60); do
        if v4l2-ctl -d /dev/rawbayer --get-fmt-video >/dev/null 2>&1; then
            echo " готово (${i}с)"
            break
        fi
        sleep 1
        echo -n "."
    done
else
    echo "  bayerizer уже запущен"
fi

# 3. Все sim-ноды (camera_node + feature_tracker + vins_estimator),
#    уже с use_sim_time:=true.
if ! pgrep -f "sim_nav.launch" >/dev/null; then
    nohup ros2 launch /root/sim_ws/src/sim/sim_nav.launch.py \
        >"$LOG/sim_nav.log" 2>&1 &
    echo "  sim_nav.launch -> $LOG/sim_nav.log"
else
    echo "  sim_nav.launch уже запущен"
fi

# 4. MAVROS — отдельно, тоже с sim-временем (вход от mavlink_router udp:14540).
#    conn/timesync_mode:=NONE — MAVROS не синхронизирует часы с FCU, а ставит
#    ros_now() (Gazebo sim-время) на каждый пакет. Нужно потому что SITL с
#    JSON-протоколом возвращает no_time_sync и использует wall-time FCU-часы,
#    отличные от Gazebo sim-времени. При дефолтном MAVLINK-режиме offset дрейфует
#    → IMU timestamps уходят назад → VINS получает "imu message in disorder".
if ! pgrep -f "mavros_node" >/dev/null; then
    nohup ros2 run mavros mavros_node --ros-args \
        -p use_sim_time:=true \
        -p fcu_url:="udp://:14540@127.0.0.1" \
        -p conn/timesync_mode:=NONE \
        >"$LOG/mavros.log" 2>&1 &
    echo "  MAVROS   -> $LOG/mavros.log"
    # Дать MAVROS подключиться, затем поднять частоту IMU (RAW_SENSORS stream).
    # По умолчанию ArduPilot шлёт IMU ~7 Гц — VINS нужно >= 100 Гц.
    (sleep 20 && ros2 service call /mavros/set_stream_rate \
        mavros_msgs/srv/StreamRate \
        '{stream_id: 1, message_rate: 200, on_off: true}' \
        >> "$LOG/mavros.log" 2>&1 && echo "  stream_rate IMU 200 Гц установлен") &
else
    echo "  MAVROS   уже запущен"
fi

echo "nav: готово. Логи: docker/sim/output/"
