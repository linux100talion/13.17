#!/usr/bin/env bash
# Команда `square [LOOPS]` — облёт квадрата SQ_SIZE×SQ_SIZE м на высоте SQ_ALT м,
# LOOPS полных кругов (default 1). Обёртка над fly_square.py для секвенсора
# capture_scene (в отличие от `make fly`, который крутит бесконечно до Ctrl+C).
#
# Запуск внутри nav-контейнера:  docker exec p1317_nav bash /lab/square.sh 1
# В секвенсоре: capture_scene.sh ... arm takeoff 5 square 1 land ...
#
# ВАЖНО: нужен предварительный arm + takeoff (fly_square шлёт setpoint'ы в
# local-координатах `map` → требует EKF origin, который появляется после взлёта).
# fly_square на sim-времени (use_sim_time) → круг проходится корректно при любом RTF.
set -e
source /opt/ros/humble/setup.bash

LOOPS="${1:-1}"
SIZE="${SQ_SIZE:-2}"; ALT="${SQ_ALT:-5}"; SIDE="${SQ_SIDE:-6}"

echo ">>> Квадрат ${SIZE}×${SIZE} м, высота ${ALT} м, ${LOOPS} круг(ов), ${SIDE}с/сторона (sim-время)..."
python3 /lab/fly_square.py --size "$SIZE" --alt "$ALT" --side-time "$SIDE" --loops "$LOOPS"
echo ">>> Облёт завершён."
