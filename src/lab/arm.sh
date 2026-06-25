#!/usr/bin/env bash
# Команда `arm` — переводит дрон в GUIDED и армирует (БЕЗ взлёта).
# Запуск внутри nav-контейнера:  docker exec p1317_nav bash /lab/arm.sh
# Или через make: make arm. В секвенсоре: capture_scene.sh ... arm ...
#
# ВАЖНО: ждём ПО ФАКТУ, а БЮДЖЕТ ожидания считаем в SIM-времени (по /clock), а не
# в wall-итерациях. GUIDED латчится только когда EKF дал здоровую горизонтальную
# позицию (GPS-фикс + сходимость, ~15 sim-сек после старта SITL). Раньше цикл
# отводил 30 wall-секунд: на GPU (RTF≈1) это ~30 sim-сек — хватало; на CPU-боксе
# (RTF≈0.07) те же 30 wall-сек = ~2 sim-сек — EKF не успевал, арм отваливался с
# armed=False. Бюджет в sim-секундах переносится между RTF без правок.
set -e
source /opt/ros/humble/setup.bash

# SIM_BUDGET — основной бюджет (прогресс симуляции). WALL_CAP — страховка на
# случай, если /clock не читается (иначе цикл завис бы навсегда при пустом sim).
SIM_BUDGET=${ARM_SIM_BUDGET:-40}     # sim-секунд на готовность EKF + GUIDED/arm
WALL_CAP=${ARM_WALL_CAP:-1200}       # абсолютный потолок wall, сек

get_field() {  # $1=топик  $2=поле
    timeout 10 ros2 topic echo --once --field "$2" "$1" 2>/dev/null | head -1
}
sim_now() { timeout 15 ros2 topic echo --once --field clock.sec /clock 2>/dev/null | head -1; }

# Есть ли ещё бюджет ожидания: $1=sim-старт $2=wall-старт; код 0 = время осталось.
budget_left() {
    local s; s=$(sim_now); [ -n "$s" ] || s="$1"
    [ "$(( s - $1 ))" -lt "$SIM_BUDGET" ] && [ "$(( $(date +%s) - $2 ))" -lt "$WALL_CAP" ]
}

echo ">>> Перевод в GUIDED (бюджет ${SIM_BUDGET} sim-сек; ждём готовности EKF)..."
s0=$(sim_now); [ -n "$s0" ] || s0=0; w0=$(date +%s)
mode=""
while :; do
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
        '{custom_mode: "GUIDED"}' >/dev/null 2>&1 || true
    mode=$(get_field /mavros/state mode)
    [ "$mode" = "GUIDED" ] && break
    budget_left "$s0" "$w0" || { echo "    ⚠️ GUIDED не залатчился в бюджете (mode=$mode)"; break; }
    sleep 2
done
echo "    mode=$mode"

echo ">>> Армирование (бюджет ${SIM_BUDGET} sim-сек)..."
s0=$(sim_now); [ -n "$s0" ] || s0=0; w0=$(date +%s)
armed=""
while :; do
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
        '{value: true}' >/dev/null 2>&1 || true
    armed=$(get_field /mavros/state armed)
    [ "$armed" = "true" ] && break
    budget_left "$s0" "$w0" || { echo "    ⚠️ арм не прошёл в бюджете (armed=$armed)"; break; }
    sleep 2
done
echo ">>> Готово (armed=$armed)."
