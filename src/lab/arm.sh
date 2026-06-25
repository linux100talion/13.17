#!/usr/bin/env bash
# Команда `arm` — переводит дрон в GUIDED и армирует (БЕЗ взлёта).
# Запуск внутри nav-контейнера:  docker exec p1317_nav bash /lab/arm.sh
# Или через make: make arm. В секвенсоре: capture_scene.sh ... arm ...
#
# ВАЖНО: ждём ПО ФАКТУ (поллинг mode/armed), а не фиксированными sleep — при
# низком RTF (симуляция медленнее реального времени) «sleep 5» = доли sim-секунды.
# sleep тут только интервал опроса (wall), не предположение о длительности.
set -e
source /opt/ros/humble/setup.bash

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
armed=""
for _ in $(seq 1 30); do
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
        '{value: true}' >/dev/null 2>&1 || true
    armed=$(get_field /mavros/state armed)
    [ "$armed" = "true" ] && break
    sleep 1
done
echo ">>> Готово (armed=$armed)."
