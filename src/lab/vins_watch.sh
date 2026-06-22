#!/usr/bin/env bash
# Мониторинг состояния VINS в реальном времени.
# Показывает: Initialization/NON_LINEAR/ошибки + частоту /odometry.
# Запуск: docker exec p1317_nav bash /lab/vins_watch.sh
# Или через make: make vins-watch
#
# Выход: Ctrl+C

LOG=/root/sim_ws/output/sim_nav.log

echo "=== VINS watch ==="
echo "  Лог:     $LOG"
echo "  Топик:   /vins_estimator/odometry"
echo "  Ctrl+C для выхода"
echo ""

source /opt/ros/humble/setup.bash
source /opt/overlay/install/setup.bash
source /root/sim_ws/install/setup.bash 2>/dev/null || true

# Запускаем мониторинг odometry в фоне
ros2 topic hz /vins_estimator/odometry --window 10 2>&1 | \
    sed 's/^/[odom] /' &
HZ_PID=$!

# Тейлим лог с фильтрацией по ключевым событиям
tail -f "$LOG" 2>/dev/null | grep --line-buffered \
    -E "Initialization|NON_LINEAR|disorder|unstable|reboot|failure|IMU excitation|features|WARN|ERROR"

kill $HZ_PID 2>/dev/null || true
