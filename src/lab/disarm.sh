#!/usr/bin/env bash
# Команда `disarm` — дизармирует дрон (cmd/arming false), ждёт armed=false по факту.
# Запуск внутри nav-контейнера:  docker exec p1317_nav bash /lab/disarm.sh
# Или через make: make disarm. В секвенсоре: capture_scene.sh ... disarm ...
#
# После LAND ArduCopter обычно дизармится сам (DISARM_DELAY 0); эта команда —
# явный/принудительный дизарм. sleep тут только интервал опроса (wall).
set -e
source /opt/ros/humble/setup.bash

get_field() {  # $1=топик  $2=поле
    timeout 10 ros2 topic echo --once --field "$2" "$1" 2>/dev/null | head -1
}

echo ">>> Дизармирование..."
armed=""
for _ in $(seq 1 30); do
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
        '{value: false}' >/dev/null 2>&1 || true
    armed=$(get_field /mavros/state armed)
    [ "${armed,,}" = "false" ] && break   # ros2 печатает "False" — сравниваем без регистра
    sleep 1
done
echo ">>> Готово (armed=$armed)."
