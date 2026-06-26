#!/usr/bin/env bash
# Посадка дрона (режим LAND).
# Запуск: docker exec p1317_nav bash /lab/land.sh
# Или через make: make land
#
# ВАЖНО: после команды LAND ждём ФАКТИЧЕСКОГО касания земли (поллинг z из
# local_position падает к нулю), а не возвращаемся сразу. Иначе в секвенсоре
# capture_scene запись bag глушится мгновенно после команды → весь спуск при
# посадке НЕ попадает в bag/видео. Поллинг RTF-независим (sleep — лишь интервал
# опроса), порог касания GROUND_Z, бюджет в итерациях как у takeoff.sh.
set -e
source /opt/ros/humble/setup.bash

GROUND_Z=${LAND_GROUND_Z:-0.3}   # порог «коснулись земли», м (EKF z у земли ≈ 0)
MAXITER=${LAND_MAXITER:-240}     # итераций поллинга (спуск на низком RTF дольше)

get_field() {  # $1=топик  $2=поле — ретраим: --once на низком RTF часто пуст/глючит
    local v=""
    for _ in 1 2 3; do
        v=$(timeout 10 ros2 topic echo --once --field "$2" "$1" 2>/dev/null | head -1)
        [ -n "$v" ] && break
    done
    echo "$v"
}

echo ">>> Режим LAND..."
# ros2 печатает "mode_sent=True" (через '=', с большой буквы) — grep без регистра
# и с '|| true', иначе несовпадение паттерна валит set -e (команда LAND при этом
# уже ушла, дрон садится).
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    '{custom_mode: "LAND"}' 2>&1 | grep -oiE 'mode_sent=[a-z]+' || true

# Ждём касания по факту: z из local_position ниже порога ЛИБО устойчивый дизарм
# после посадки (DISARM_DELAY 0). armed читается глючно (одиночный --once может
# вернуть ложный "false" в полёте) → принимаем дизарм только после 3 ПОДРЯД
# false-чтений, иначе ожидание рвётся досрочно и спуск не попадает в запись.
echo ">>> Ждём касания земли (z < ${GROUND_Z} м; поллинг)..."
z=""
false_streak=0
for _ in $(seq 1 "$MAXITER"); do
    z=$(get_field /mavros/local_position/pose pose.position.z)
    if [ -n "$z" ] && awk "BEGIN{exit !($z <= $GROUND_Z)}"; then
        break
    fi
    armed=$(get_field /mavros/state armed)
    if [ "${armed,,}" = "false" ]; then
        false_streak=$((false_streak + 1))
        [ "$false_streak" -ge 3 ] && { echo "    дизарм после посадки (armed=false x3)"; break; }
    else
        false_streak=0
    fi
    sleep 1
done
if [ -n "$z" ] && awk "BEGIN{exit !($z <= $GROUND_Z)}" 2>/dev/null; then
    echo ">>> ПОСАДКА OK: z=${z} м (порог ${GROUND_Z} м)."
else
    echo ">>> ⚠️ КАСАНИЕ НЕ ПОДТВЕРЖДЕНО: z=${z:-?} м (порог ${GROUND_Z} м)."
fi
echo ">>> Сели."
