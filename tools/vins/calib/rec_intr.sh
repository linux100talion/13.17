#!/bin/bash
# rec_intr.sh — bag для калибровки ИНТРИНСИКОВ бортовой камеры (шаг 1 пайплайна, README.md).
# Запускается НА НОУТЕ, запись идёт НА БОРТУ (ros2 bag record хоста Jetson, как auto-bag-m).
#
#   ./rec_intr.sh [сек]          запись, умолч. 120 с → /home/andriy/mavlogs/calib/intr_<дата>
#   ./rec_intr.sh pull <имя>     забрать bag на ноут → docker/sim/output/board/calib/<имя>
#   ./rec_intr.sh ls             что уже записано на борту
#
# Что пишется: ТОЛЬКО /image_mono (вход VINS = ровно то, что видит VINS) + /camera_info.
# 1280×720 mono8 15 Гц ≈ 14 МБ/с → 120 с ≈ 1.7 ГБ. IMU для интринсиков не нужен.
#
# Другие записи тем же путём — env: PREFIX (имя, умолч. intr), TOPICS (список, умолч. выше).
# Прогулка «в руках» для scene_ipm.mp4 + офлайн-реплея VINS (лёгкая, без /image_color):
#   PREFIX=walk ./rec_intr.sh 120     (набор топиков WALK_TOPICS — ниже; 120 с ≈ 1.7 ГБ)
# ⚠️ ноль барометра = первая секунда bag: начинать, когда дрон СТОИТ на земле, поднять через 2–3 с.
#
# Камеру поднимает служба vins_m (camera_node + VINS + стример; лишнее не мешает). Если её
# включил скрипт — он же её и гасит в конце (KEEP_CAM=1 — оставить). Видео для наводки — WFB
# (`make -C docker/sim fpvd`), это тот же кадр, что пишется.
#
# Связь: сначала ssh jetson (Wi-Fi), иначе jetson-wfb (радио, Alfa) — на улице без раздачи.
# JETSON_HOST=jetson-wfb — принудительно. pull по радио медленный и делит канал с видео —
# забирать дома по Wi-Fi.
#
# Запись на борту — отдельный процесс (setsid nohup + timeout -s INT): обрыв Wi-Fi/ssh её НЕ
# прерывает, bag закроется сам через [сек]. Отключились — `./rec_intr.sh ls` после переподключения.
#
# КАК СНИМАТЬ (дрон неподвижен, двигается мишень, 1–2.5 м):
#   - мишень целиком в кадре, все 4 угла видны в большинстве кадров;
#   - пройти мишенью ВСЕ зоны кадра, особенно углы и края (там дисторсия);
#   - близко (мишень на весь кадр) и далеко (на треть кадра);
#   - наклоны 30–45° по обеим осям, повороты в плоскости;
#   - двигать МЕДЛЕННО (смаз портит углы тегов), без бликов на мишени.
# Дальше: rosbags-convert → kalibr_calibrate_cameras (README.md, шаги 2–3).

set -uo pipefail
cd "$(dirname "$(readlink -f "$0")")"

HOSTS=(${JETSON_HOST:-jetson jetson-wfb})   # Wi-Fi (mDNS) → радио WFB-ng (туннель 10.5.0.2)
SUDOPW="${JETSON_SUDO:-ok}"
BOARD_DIR=/home/andriy/mavlogs/calib
LOCAL_DIR="$(git rev-parse --show-toplevel)/docker/sim/output/board/calib"
FPS_MIN=12          # ниже — камера не та/тормозит, не пишем
PREFIX="${PREFIX:-intr}"
# прогулка: камера VINS + IMU (AHRS — углы для ipm_video, сырой — для реплея VINS) + баро
# (высота ipm_video) + что VINS выдал вживую
WALK_TOPICS="/image_mono /camera_info /mavros/imu/data /mavros/imu/data_raw \
/mavros/imu/static_pressure /mavros/state /feature /odometry /path"
if [ "$PREFIX" = walk ]; then TOPICS="${TOPICS:-$WALK_TOPICS}"; else TOPICS="${TOPICS:-/image_mono /camera_info}"; fi
for HOST in "${HOSTS[@]}"; do
    ssh -o ConnectTimeout=4 -o BatchMode=yes "$HOST" true 2>/dev/null && break
    HOST=""
done
[ -n "$HOST" ] || { echo "!! борт не отвечает: ${HOSTS[*]}" >&2; exit 1; }
SSH=(ssh -o ConnectTimeout=8 -o ServerAliveInterval=5 "$HOST")

# окружение ROS хоста борта — то же, что у auto_bag_m.sh
ROSENV='export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml; source /opt/ros/humble/setup.bash'

die() { echo "!! $*" >&2; exit 1; }
board() { "${SSH[@]}" "$ROSENV; $1"; }

case "${1:-}" in
    ls)
        board "cd $BOARD_DIR 2>/dev/null && for b in */; do b=\${b%/}
                   s=закрыт; [ -f \$b/metadata.yaml ] || s='ПИШЕТСЯ/не закрыт'
                   echo \"\$b  \$(du -sh \$b | cut -f1)  \$s\"; done"
        exit ;;
    pull)
        [ -n "${2:-}" ] || die "имя bag: ./rec_intr.sh pull intr_YYYYmmdd_HHMMSS"
        mkdir -p "$LOCAL_DIR"
        rsync -a --info=progress2 "$HOST:$BOARD_DIR/$2" "$LOCAL_DIR/" || die "rsync"
        echo "→ $LOCAL_DIR/$2"
        exit ;;
esac

DUR="${1:-120}"
[[ "$DUR" =~ ^[0-9]+$ ]] || die "длительность — целое число секунд"

echo "== борт на связи через $HOST"

[ "$(board 'systemctl is-active auto-bag-m')" = active ] &&
    echo "   ⚠ auto-bag-m тоже пишет (с /image_color, 55 МБ/с) — для калибровки не нужен: sudo systemctl stop auto-bag-m"

STARTED=0
if [ "$(board 'systemctl is-active vins_m')" != active ]; then
    echo "== камеры нет — поднимаю службу vins_m"
    board "echo $SUDOPW | sudo -S -p '' systemctl start vins_m" || die "systemctl start vins_m"
    STARTED=1
fi

echo "== жду /image_mono"
for _ in $(seq 30); do
    board "timeout 3 ros2 topic echo --once /camera_info >/dev/null 2>&1" && break
    sleep 1
done
HZ=$(board "timeout 6 ros2 topic hz /image_mono 2>/dev/null | awk '/average rate/{r=\$3} END{print r}'")
echo "   /image_mono: ${HZ:-0} Гц"
awk -v h="${HZ:-0}" -v m=$FPS_MIN 'BEGIN{exit !(h>=m)}' || die "камера не даёт кадров (≥$FPS_MIN Гц) — journalctl -u vins_m"

NAME="${PREFIX}_$(date +%Y%m%d_%H%M%S)"
OUT="$BOARD_DIR/$NAME"
board "mkdir -p $BOARD_DIR && setsid nohup bash -c '$ROSENV; exec timeout -s INT $DUR \
       ros2 bag record -o $OUT $TOPICS' > $OUT.log 2>&1 < /dev/null &" ||
    die "не стартовала запись"

echo "== ЗАПИСЬ $NAME, $DUR с — водите мишенью (углы кадра, близко/далеко, наклоны, медленно)"
T0=$(date +%s)
while :; do
    LEFT=$(( DUR - ($(date +%s) - T0) ))
    [ $LEFT -le 0 ] && break
    printf '\r   осталось %3d с ' $LEFT
    (( LEFT % 30 == 0 )) && printf '\a'
    sleep 1
done
echo

echo "== жду закрытия bag"
for _ in $(seq 30); do
    board "test -f $OUT/metadata.yaml" 2>/dev/null && break
    sleep 1
done
INFO=$(board "ros2 bag info $OUT 2>/dev/null")
echo "$INFO" | grep -E 'Duration|Messages|Topic:'
N=$(echo "$INFO" | awk -F'Count: ' '/image_mono/{split($2,a," "); print a[1]}')
EXP=$(( DUR * 15 ))
echo "   кадров ${N:-0} из ~$EXP ожидаемых, размер $(board "du -sh $OUT | cut -f1")"
[ "${N:-0}" -lt $(( EXP * 8 / 10 )) ] && echo "   ⚠ кадров меньше 80 % — смотри $OUT.log"

if [ $STARTED = 1 ] && [ "${KEEP_CAM:-0}" != 1 ]; then
    echo "== гашу vins_m (включал скрипт)"
    board "echo $SUDOPW | sudo -S -p '' systemctl stop vins_m"
fi
echo "Забрать на ноут: $0 pull $NAME"
