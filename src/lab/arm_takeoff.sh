#!/usr/bin/env bash
# Переводит дрон в GUIDED, армирует и взлетает.
# Запуск внутри nav-контейнера:
#   docker exec p1317_nav bash /lab/arm_takeoff.sh [ALTITUDE_M]
# Или через make: make arm
#
# ВАЖНО: НЕ ждём фиксированными sleep — при низком RTF (симуляция ~13× медленнее,
# ветка nn2_c3_cpu) «sleep 5» = доли sim-секунды, и дрон не успел бы ни сменить
# режим, ни набрать высоту. Поэтому ПОЛЛИМ состояние (mode/armed/высота) — это
# RTF-независимо: ждём по факту, а не по реальным секундам. sleep тут — только
# интервал опроса (wall), не предположение о длительности.
set -e
source /opt/ros/humble/setup.bash

ALT=${1:-3}

# Прочитать одно значение поля топика (пусто при таймауте/отсутствии данных).
get_field() {  # $1=топик  $2=поле
    timeout 10 ros2 topic echo --once --field "$2" "$1" 2>/dev/null | head -1
}

echo ">>> Режим GUIDED..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    '{custom_mode: "GUIDED"}' >/dev/null 2>&1 || true
mode=""
for _ in $(seq 1 30); do
    mode=$(get_field /mavros/state mode)
    [ "$mode" = "GUIDED" ] && break
    sleep 1
done
echo "    mode=$mode"

echo ">>> Армирование..."
for _ in $(seq 1 30); do
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
        '{value: true}' >/dev/null 2>&1 || true
    armed=$(get_field /mavros/state armed)
    [ "$armed" = "true" ] && break
    sleep 1
done
echo "    armed=$armed"

echo ">>> Взлёт на ${ALT} м..."
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
    "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: ${ALT}}" \
    >/dev/null 2>&1 || true

# Ждём набора высоты по факту (z из local_position). Порог 95% от ALT.
# Высота может быть пустой первые секунды (EKF ещё без origin) — пропускаем.
echo ">>> Ждём набора высоты ${ALT} м (поллинг z)..."
z=""
for _ in $(seq 1 180); do
    z=$(get_field /mavros/local_position/pose pose.position.z)
    if [ -n "$z" ] && awk "BEGIN{exit !($z >= $ALT * 0.95)}"; then
        break
    fi
    sleep 1
done
echo ">>> Готово (z=${z:-?} м)."
