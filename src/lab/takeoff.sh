#!/usr/bin/env bash
# Команда `takeoff [ALT]` — взлёт на ALT метров (default 3). Предполагает, что
# дрон уже armed + GUIDED (см. arm.sh).
# Запуск внутри nav-контейнера:  docker exec p1317_nav bash /lab/takeoff.sh 5
# Или через make: make takeoff ALT=5. В секвенсоре: capture_scene.sh ... takeoff 5 ...
#
# ВАЖНО: набор высоты ждём ПО ФАКТУ (поллинг z из local_position), а не sleep —
# RTF-независимо. Высота может быть пустой первые секунды (EKF без origin) —
# пропускаем. sleep тут только интервал опроса (wall).
set -e
source /opt/ros/humble/setup.bash

ALT=${1:-3}

get_field() {  # $1=топик  $2=поле
    timeout 10 ros2 topic echo --once --field "$2" "$1" 2>/dev/null | head -1
}

echo ">>> Взлёт на ${ALT} м..."
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
    "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: ${ALT}}" \
    >/dev/null 2>&1 || true

# Ждём набора высоты по факту (z из local_position). Порог 95% от ALT.
echo ">>> Ждём набора высоты ${ALT} м (поллинг z)..."
z=""
for _ in $(seq 1 180); do
    z=$(get_field /mavros/local_position/pose pose.position.z)
    if [ -n "$z" ] && awk "BEGIN{exit !($z >= $ALT * 0.95)}"; then
        break
    fi
    sleep 1
done
if [ -n "$z" ] && awk "BEGIN{exit !($z >= $ALT * 0.9)}" 2>/dev/null; then
    echo ">>> ВЗЛЁТ OK: z=${z} м (цель ${ALT} м)."
else
    echo ">>> ⚠️ ВЗЛЁТ НЕ ПОДТВЕРЖДЁН: z=${z:-?} м (цель ${ALT} м) — высота не набрана."
fi
