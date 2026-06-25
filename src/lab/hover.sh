#!/usr/bin/env bash
# Команда `hover [SIM_SEC]` — удерживать позицию N секунд SIM-времени (default 10).
# Запуск внутри nav-контейнера:  docker exec p1317_nav bash /lab/hover.sh 5
# Или через make: make hover SEC=5. В секвенсоре: capture_scene.sh ... hover 5 ...
#
# В GUIDED коптер ArduPilot сам удерживает точку после takeoff (continuous
# setpoints не нужны, в отличие от PX4 OFFBOARD), поэтому hover = ожидание.
# Ждём по SIM-времени (/clock от ros_gz_bridge), а не wall: при низком RTF фикс.
# wall-секунды были бы мизером sim-времени. sleep тут только интервал опроса.
set -e
source /opt/ros/humble/setup.bash

SEC=${1:-10}

clock_sec() {  # текущее sim-время, целые секунды (пусто при таймауте)
    timeout 10 ros2 topic echo --once --field clock.sec /clock 2>/dev/null | head -1
}

echo ">>> Висение ${SEC} с sim-времени..."
t0=""
for _ in $(seq 1 30); do
    t0=$(clock_sec)
    [ -n "$t0" ] && break
    sleep 1
done
if [ -z "$t0" ]; then
    echo ">>> ⚠️ /clock недоступен — висение пропущено."
    exit 0
fi

# Ждём, пока sim-время не вырастет на SEC. Страховка по wall-итерациям —
# на случай остановки /clock (не виснуть навсегда).
t="$t0"
for _ in $(seq 1 100000); do
    t=$(clock_sec)
    [ -n "$t" ] && awk "BEGIN{exit !($t - $t0 >= $SEC)}" && break
    sleep 1
done
echo ">>> Готово (прошло ~$(awk "BEGIN{print $t - $t0}") с sim-времени)."
