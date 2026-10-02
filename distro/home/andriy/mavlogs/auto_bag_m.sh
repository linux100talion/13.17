#!/bin/bash
# auto_bag_m.sh — ЗАПИСЬ BAG НА БОРТУ (служба auto-bag-m): пишет С ПОДЪЁМА MAVROS до остановки
# службы, кусками по 180 с. ВКЛЮЧАЕТСЯ РУКАМИ (в автозапуск не ставим — гигабайты):
#
#   sudo systemctl start auto-bag-m     # начать запись
#   sudo systemctl stop  auto-bag-m     # закончить (SIGINT → bag закрывается корректно)
#   journalctl -u auto-bag-m -f
#
# Решение 2026-10-02 (пилот): ОДИН режим — с подъёма, а не по арму: земля до арма нужна
# (стояние VINS на земле, прогоны «в руках» без арма), арм/дизарм и так видны в /mavros/state.
# Полноценные bag — камера БЕЗ сжатия (/image_mono + /image_color 1280×720 ≈ 55 МБ/с ≈ 10 ГБ
# за кусок); после каждой записи пилот решает, оставлять ли её.
#
# Что пишется (одним регэкспом — подхватывает и ноды, поднятые ПОСЛЕ старта записи):
#   камера   /image_mono /image_color /camera_info
#   VINS     /feature /odometry /path /restart
#   MAVROS   state, imu/*, local_position/pose, global_position/{raw/fix,global,gp_origin},
#            gpsstatus/*, rc/{in,override}, setpoint_raw/*, vision_pose/pose,
#            vision_speed/speed_twist, battery
#   нода     /joy /mission/* /flow_dbg* /nn1/* /vins/* /blocked/* (что нода хотела бы послать
#            в полётник из песочницы sandbox_node.sh)
#   Orin     /orin/sysmon — температуры, частоты, загрузка ядер (sysmon.py, 1 Гц)
# Рядом с bag — meta.txt (время, ядро, режим питания, версии кода/форка).
# Защита диска: свободно < 30 ГБ — не стартуем; в процессе < 15 ГБ — останов записи.

export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml   # лимит участников DDS (etc/cyclonedds)
source /opt/ros/humble/setup.bash

LOG_DIR="/home/andriy/mavlogs"
CHUNK_SEC=180
MIN_FREE_START_GB=30
MIN_FREE_RUN_GB=15
TOPIC_RE='^/(image_mono|image_color|camera_info|feature|odometry|path|restart|joy|mission/.*|flow_dbg.*|nn1/.*|vins/.*|blocked/.*|orin/sysmon|mavros/(state|imu/.*|local_position/pose|global_position/(raw/fix|global|gp_origin)|gpsstatus/.*|rc/(in|override)|setpoint_raw/.*|vision_pose/pose|vision_speed/speed_twist|battery))$'

mkdir -p "$LOG_DIR"
BAG_PID=""
MON_PID=""

free_gb() { df -BG --output=avail "$LOG_DIR" | tail -1 | tr -dc '0-9'; }

cleanup() {
    echo "Сигнал остановки! Закрываем bag..."
    [[ -n "$BAG_PID" ]] && kill -INT "$BAG_PID" 2>/dev/null && wait "$BAG_PID" 2>/dev/null
    [[ -n "$MON_PID" ]] && kill -INT "$MON_PID" 2>/dev/null
    [[ -n "$BAG_NAME" ]] && echo "Записано: $BAG_NAME ($(du -sh "$BAG_NAME" 2>/dev/null | cut -f1))"
    exit 0
}
trap cleanup SIGINT SIGTERM

if [ "$(free_gb)" -lt "$MIN_FREE_START_GB" ]; then
    echo "!! свободно $(free_gb) ГБ < $MIN_FREE_START_GB — запись не начинаю (почисти $LOG_DIR)"
    exit 0   # не 1: Restart=on-failure перезапускал бы службу каждые 5 с
fi

echo "Ожидание запуска MAVROS..."
while ! ros2 topic list 2>/dev/null | grep -q "/mavros/state"; do sleep 1; done

python3 "$(dirname "$(readlink -f "$0")")/sysmon.py" > /tmp/sysmon.log 2>&1 &
MON_PID=$!

BAG_NAME="$LOG_DIR/bag_$(date +%Y%m%d_%H%M%S)"
echo "Запись в $BAG_NAME (куски по $CHUNK_SEC с, свободно $(free_gb) ГБ)"
ros2 bag record -o "$BAG_NAME" --max-bag-duration "$CHUNK_SEC" -e "$TOPIC_RE" &
BAG_PID=$!

# meta.txt — когда ros2 bag создал каталог
for _ in $(seq 20); do [ -d "$BAG_NAME" ] && break; sleep 0.5; done
{
    echo "start:        $(date -Is)"
    echo "host:         $(hostname) $(uname -r)"
    echo "l4t:          $(head -1 /etc/nv_tegra_release 2>/dev/null)"
    echo "power_mode:   $(nvpmodel -q 2>/dev/null | grep -m1 'Power Mode' | cut -d: -f2 | xargs)"
    echo "code(13.17):  $(cat /home/andriy/13.17/DEPLOYED.txt 2>/dev/null)"   # пишет deploy.sh code
    echo "vins_fork:    $(git -C /home/andriy/VINS-MONO-ROS2 log --oneline -1 2>/dev/null)"
    echo "services:     $(for s in mavros vins vins_m; do echo -n "$s=$(systemctl is-active $s) "; done)"
    echo "chunk_sec:    $CHUNK_SEC"
    echo "topics_re:    $TOPIC_RE"
} > "$BAG_NAME/meta.txt" 2>/dev/null

# сторож диска
while kill -0 "$BAG_PID" 2>/dev/null; do
    if [ "$(free_gb)" -lt "$MIN_FREE_RUN_GB" ]; then
        echo "!! свободно $(free_gb) ГБ < $MIN_FREE_RUN_GB — останавливаю запись"
        cleanup
    fi
    sleep 5
done
echo "ros2 bag record завершился сам — выход с ошибкой"
exit 1
