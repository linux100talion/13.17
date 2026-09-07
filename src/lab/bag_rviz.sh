#!/bin/bash
# ============================================================================
# bag_rviz.sh — ПРОСМОТР BAG'а ПРОГОНА В RVIZ2: путь, стрелка носа, видео камеры.
#
# Запуск С ХОСТА (ROS jazzy на ноуте; стек сима может оставаться поднятым):
#   bash src/lab/bag_rviz.sh docker/sim/output/joystick/lv2_joy_20260906_231055
#   bash src/lab/bag_rviz.sh <прогон>/bag  [доп. аргументы ros2 bag play]
#   RATE=2 LOOP=1 START=30 bash src/lab/bag_rviz.sh <прогон>
#
# Что поднимает (всё в ОТДЕЛЬНОМ ROS-домене, см. ниже) и гасит по Ctrl+C:
#   1. static_transform_publisher world→map (identity) — /mavros/local_position/pose
#      живёт во frame 'map', истина Gazebo и VINS — в 'world'; TF в bag не пишется.
#   2. bag_path_pub.py — Odometry/PoseStamped → nav_msgs/Path (/truth/path,
#      /vins/path, /ekf/path): в bag'е нет /path, а линию RViz рисует только из Path.
#   3. rviz2 -d src/lab/bag_view.rviz: Image /image_color; Odometry-стрелки (нос =
#      ось x тела) истины (зелёные) и VINS (красные) со шлейфом Keep; Pose EKF
#      (голубая, Best Effort — так публикует MAVROS); три линии Path; Fixed Frame
#      world. Сохранённый вид «Top-down» — в панели Views.
#   4. ros2 bag play (foreground: SPACE пауза, стрелки — шаг/скорость).
#
# ⚠️ ДВА ГРАБЛЯ, из-за которых «Detected jump back in time. Resetting RViz»:
#   • Живой стек сима (network_mode: host, ROS_DOMAIN_ID=0) светит на хост
#     /clock от Gazebo (sim-секунды с запуска) и живые /model/iris_cam/odometry,
#     /odometry. С `ros2 bag play --clock` у RViz ДВА источника /clock (эпоха записи
#     1788725… против ~6300 с Gazebo) вперемешку → скачки назад каждый кадр, плюс
#     живой борт рисуется поверх реплея. Лечение — свой домен: DOMAIN (default 42).
#   • Штампы в bag'е разные: время ЗАПИСИ = wall-эпоха (по нему играет player и
#     считает --clock), заголовки сообщений = sim-время Gazebo (0…120 с). Для
#     просмотра они не нужны вовсе: TF только статический, Image без TF, Fixed
#     Frame = frame сообщений → ни --clock, ни use_sim_time НЕ передаём.
#
# VINS /odometry рисуется в СВОЁМ 'world' (начало — точка init, курс первого
# кадра; FrameAnchor ray_tracer'а выравнивает только то, что уходит в EKF) —
# красная линия смещена/повёрнута относительно зелёной истины по построению.
# /mavros/state в jazzy без mavros_msgs игнорируется player'ом (не нужен RViz);
# при желании: sudo apt install ros-jazzy-mavros-msgs.
# ============================================================================
set -euo pipefail

DISTRO="${DISTRO:-jazzy}"          # ROS на хосте
DOMAIN="${DOMAIN:-42}"             # ROS_DOMAIN_ID реплея — изоляция от живого стека (0)
RATE="${RATE:-1}"                  # скорость реплея
LOOP="${LOOP:-0}"                  # 1 = --loop (Path сбрасывается сам по скачку штампа)
START="${START:-0}"                # --start-offset, с от начала bag'а
RVIZ_CFG="${RVIZ_CFG:-}"           # свой .rviz (default src/lab/bag_view.rviz)
PLAY_EXTRA=()                      # хвост аргументов → ros2 bag play

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -n "$RVIZ_CFG" ] || RVIZ_CFG="$SCRIPT_DIR/bag_view.rviz"

if [ $# -lt 1 ]; then
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//' >&2
    exit 1
fi
ARG="$1"; shift
PLAY_EXTRA=("$@")

# ── где bag: прогон/ → прогон/bag; bag/; *.db3 → его каталог ──────────────────
if [ -f "$ARG" ] && [[ "$ARG" == *.db3 ]]; then
    BAG="$(cd "$(dirname "$ARG")" && pwd)"
elif [ -f "$ARG/metadata.yaml" ]; then
    BAG="$(cd "$ARG" && pwd)"
elif [ -f "$ARG/bag/metadata.yaml" ]; then
    BAG="$(cd "$ARG/bag" && pwd)"
else
    echo "bag не найден: ни $ARG/metadata.yaml, ни $ARG/bag/metadata.yaml" >&2
    exit 2
fi

SETUP="/opt/ros/$DISTRO/setup.bash"
[ -f "$SETUP" ] || { echo "нет $SETUP — ROS $DISTRO на хосте не установлен" >&2; exit 2; }
# setup.bash ROS не переживает set -u (AMENT_TRACE_SETUP_FILES не задана)
set +u
# shellcheck disable=SC1090
source "$SETUP"
set -u
export ROS_DOMAIN_ID="$DOMAIN"
unset ROS_LOCALHOST_ONLY

PIDS=()
cleanup() {
    for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "=== bag_rviz: $BAG (ROS_DOMAIN_ID=$DOMAIN, rate $RATE, start ${START}s${LOOP:+, loop=$LOOP}) ==="
echo "    сим в домене 0 не мешает; выйти — Ctrl+C здесь (гасит rviz и помощников)"

# 1. TF world→map (identity): EKF-поза MAVROS во frame 'map'.
#    Бинарник напрямую, не `ros2 run`: TERM python-обёртке до её ребёнка не
#    доходит, и после Ctrl+C оставался живой static_transform_publisher.
STP="$(ros2 pkg prefix tf2_ros)/lib/tf2_ros/static_transform_publisher"
"$STP" --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id world --child-frame-id map >/dev/null 2>&1 &
PIDS+=($!)

# 2. Odometry/PoseStamped → Path
python3 "$SCRIPT_DIR/bag_path_pub.py" &
PIDS+=($!)

# 3. RViz (stdout/stderr через process substitution — $! остаётся PID rviz2,
#    иначе в пайпе с grep Ctrl+C гасил бы grep, а окно жило дальше)
rviz2 -d "$RVIZ_CFG" > >(grep --line-buffered -vE 'Detected jump back|Clearing TF buffer' || true) 2>&1 &
RVIZ_PID=$!
PIDS+=("$RVIZ_PID")
sleep 3

# 4. реплей (foreground — клавиатура: SPACE пауза, →/↑/↓)
PLAY=(ros2 bag play "$BAG" --rate "$RATE")
[ "$START" != "0" ] && PLAY+=(--start-offset "$START")
[ "$LOOP" = "1" ] && PLAY+=(--loop)
"${PLAY[@]}" "${PLAY_EXTRA[@]}"
echo "=== реплей закончен; rviz открыт — Ctrl+C или закрыть окно ==="
wait "$RVIZ_PID" 2>/dev/null || true
