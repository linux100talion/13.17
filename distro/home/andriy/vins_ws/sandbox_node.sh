#!/bin/bash
# sandbox_node.sh — лётная нода на РЕАЛЬНОМ БОРТУ в ПЕСОЧНИЦЕ: всё читает, в полётник НЕ пишет.
#
#   ~/vins_ws/sandbox_node.sh [секунд=120]
#
# Зачем. Первый запуск bootstrap_arch2 на борту (2026-10-02, пропеллеры сняты): сверить вход
# пульта (crsf_joy → /joy → JoyPilot, знаки, SF/SC/SA/SD), /mission/status, мост ray_tracer,
# реверс каналов в логе — НЕ давая ноде армить и вообще трогать полётник.
#
# Как. Все каналы ЗАПИСИ ноды в полётник переименованы (ROS remap) в тупики /blocked/...:
#   арм /mavros/cmd/arming, команда /mavros/cmd/command (force arm/disarm), режим set_mode,
#   override, уставки setpoint_raw/attitude|local, параметры param/set (EK3_SRC*!), дом
#   cmd/set_home, начало координат set_gp_origin, поза/скорость в EKF vision_pose|vision_speed.
#   Сервиса-тупика нет → клиент ноды никогда не «готов» → вызов не уходит вовсе. Что нода
#   ХОТЕЛА бы послать, видно в тупиках: ros2 topic echo /blocked/rc/override и т.п.
#   Оставлены только чтение (param/get_parameters) и частоты телеметрии (set_message_interval).
#   ray_tracer: publish_vision_pose:=false + тот же remap позы (EKF борта на GPS не трогаем).
# После старта скрипт ПРОВЕРЯЕТ `ros2 node info`: любой издатель/клиент в /mavros, кроме
# разрешённых, — немедленный останов всего. Борт заармлен на старте — отказ.
#
# ⚠️ Полётник армится и сам — стиком курса (ARMING_RUDDER 2: газ вниз + курс вправо 2 с).
#    Песочница это НЕ закрывает: жест арма на пульте во время прогона не делать.
#
# Нужны: контейнер vins_project_13_7 и (для VINS/камеры в статусе) служба vins_m.
# Логи нод: ~/sandbox_logs/<дата>/*.log; профиль ноды (BS_*) и env — там же (bs.env, env.txt).

DUR="${1:-120}"
CONTAINER="vins_project_13_7"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/sandbox_logs/$STAMP"
PROFILES="dphold/att_own dpvins/board1 vinshold/baseline vins/scale25 loiter/boot_ipm wind/trim mission/board legacy/baseline"

BLOCK=(
    /mavros/cmd/arming /mavros/cmd/command /mavros/set_mode /mavros/rc/override
    /mavros/setpoint_raw/attitude /mavros/setpoint_raw/local /mavros/param/set
    /mavros/cmd/set_home /mavros/global_position/set_gp_origin
    /mavros/vision_pose/pose /mavros/vision_speed/speed_twist
)
REMAP=""
for t in "${BLOCK[@]}"; do REMAP="$REMAP -r $t:=/blocked${t#/mavros}"; done
ALLOWED='^/mavros/(param/get_parameters|set_message_interval)$'
PATTERNS=("lib/mission_pk[g]/bootstrap_arch2" "lib/mission_pk[g]/crsf_joy" "lib/nav_pk[g]/ray_tracer")

dexec() { docker exec -e USE_SIM_TIME=0 "$CONTAINER" bash -c "source /root/vins_ws/install/setup.bash && $1"; }

stop_all() {
    local p
    for p in "${PATTERNS[@]}"; do docker exec "$CONTAINER" pkill -INT -f "$p" 2>/dev/null; done
    sleep 3
    for p in "${PATTERNS[@]}"; do docker exec "$CONTAINER" pkill -KILL -f "$p" 2>/dev/null; done
    wait 2>/dev/null
}
trap 'echo ">>> останов"; stop_all; echo ">>> логи: $OUT"; exit 0' INT TERM

docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "!! контейнер $CONTAINER не запущен"; exit 1; }
systemctl is-active --quiet vins_m || echo "   (vins_m не запущена — камеры/VINS в статусе не будет)"

ARMED=$(dexec "timeout 8 ros2 topic echo --once /mavros/state 2>/dev/null | grep -m1 '^armed:'")
case "$ARMED" in
    *false*) echo ">>> полётник задизармлен — ок" ;;
    *true*)  echo "!! полётник ЗААРМЛЕН — песочницу не запускаю"; exit 1 ;;
    *)       echo "!! /mavros/state не прочитан (MAVROS жив?) — не запускаю"; exit 1 ;;
esac

stop_all   # остатки прошлого прогона
mkdir -p "$OUT"
dexec "env | grep -E '^(ROS|RMW|USE_SIM)'" > "$OUT/env.txt"
dexec "eval \"\$(python3 /root/vins_ws/src/control/profiles/load.py $PROFILES)\" && env | grep '^BS_' | sort" > "$OUT/bs.env" \
    || { echo "!! профили не собрались"; exit 1; }
echo ">>> профили: $PROFILES ($(grep -c . "$OUT/bs.env") ключей BS_)"

dexec "exec ros2 run mission_pkg crsf_joy" > "$OUT/crsf_joy.log" 2>&1 &
dexec "exec ros2 run nav_pkg ray_tracer --ros-args -p publish_vision_pose:=false \
    -r /mavros/vision_pose/pose:=/blocked/vision_pose/pose" > "$OUT/ray_tracer.log" 2>&1 &
dexec "eval \"\$(python3 /root/vins_ws/src/control/profiles/load.py $PROFILES)\" && \
    exec ros2 run mission_pkg bootstrap_arch2 --ros-args $REMAP" > "$OUT/bootstrap.log" 2>&1 &

# ── проверка песочницы: ни одного писателя в /mavros, кроме разрешённых ─────────
sleep 8
BAD=""
for n in /alt_hold_bootstrap_arch2 /ray_tracer; do
    info=$(dexec "timeout 10 ros2 node info $n 2>/dev/null")
    [ -n "$info" ] || { BAD="$BAD $n:не_видна"; continue; }
    w=$(echo "$info" | awk '/Publishers:|Service Clients:|Action Clients:/{s=1;next} /Subscribers:|Service Servers:|Action Servers:/{s=0} s' \
        | grep -oE '/mavros/[^:]+' | grep -vE "$ALLOWED")
    [ -n "$w" ] && BAD="$BAD $n:$(echo $w | tr ' ' ',')"
done
if [ -n "$BAD" ]; then
    echo "!! ПЕСОЧНИЦА НАРУШЕНА:$BAD — останов"; stop_all; exit 1
fi
echo ">>> песочница цела: у bootstrap_arch2 и ray_tracer нет писателей в /mavros (кроме чтения параметров/частот)"
echo ">>> работает ${DUR} с; команды ноды — в /blocked/* (ros2 topic echo /blocked/setpoint_raw/attitude)"

sleep "$DUR"
echo ">>> время вышло"; stop_all
echo ">>> логи: $OUT"
