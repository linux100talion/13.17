#!/usr/bin/env bash
# bootstrap_arch2.sh — запуск лётной ноды `ros2 run mission_pkg bootstrap_arch2`
# (control_pkg + mission_pkg, hexagonal-ядро) ВНУТРИ nav-контейнера.
#
# С 2026-09-07 ручки ноды — ТОЛЬКО из профилей src/control/profiles: этот скрипт
# получает список PROFILES, собирает их загрузчиком (include + дельта, дубль/незнакомый/
# отсутствующий ключ = ошибка со схемой BootstrapConfig) в env BS_<ПОЛЕ> и запускает
# ноду БЕЗ аргументов — она читает env (BootstrapConfig.from_env, нет ключа = SystemExit).
# Проводка BS_FOO → --foo (192 строки) и argparse ноды (196 аргументов) удалены: у ручки
# было четыре места для значения, и любое молча подменяло профиль (свип B3s с ki=0).
#
# Запуск:  docker exec -e PROFILES="dphold/baseline … world/wind2_gust5" p1317_nav \
#              bash /lab/bootstrap_arch2.sh
# В секвенсоре: bash src/lab/capture_scene.sh 960x540 bootstrap_arch2 (PROFILES из env;
#               сам список держит cmd/<имя>/<имя>.sh, см. cmd/README.txt).
#
# Аргументы ПРОГОНА (не параметры, в профилях не живут; проброс capture_scene.sh):
#   BS_REPLAY_SCENARIO — сценарий .json реплея пульта (при BS_PILOT=replay в профиле
#                        mission/replay), BS_REPLAY_RAW — сырой .jsonl, BS_REPLAY_FENCE.
#
# ЖИВОЙ ПУЛЬТ (BS_PILOT=joy): TX12 в режиме USB-джойстика → joy_linux_node → /joy →
# JoyPilot. Мимо FCU — под активным override /mavros/rc/in отдаёт эхо собственной
# команды ноды (петля), поэтому rc/in для живых стиков НЕ используется. Драйвер живёт
# только на время прогона (trap ниже). Устройство — BS_JOY_DEV из профиля mission/
# (default /dev/input/js0; проброшен каталогом, hotplug работает).
#
# ВИРТУАЛЬНЫЙ ПИЛОТ (BS_PILOT=replay): joy_replay.py публикует /joy по сценарию
# (src/lab/joystick/, см. README.md там) — нода видит его как живой пульт (BS_PILOT=joy
# для ноды подменяется здесь), весь стек ниже /joy идентичен ручному полёту. Знаки осей
# BS_JOY_SIGNS уходят и ноде, и реплею — рассинхрон невозможен.
set -e
source /opt/ros/humble/setup.bash
source /root/sim_ws/install/setup.bash 2>/dev/null || true

if ! ros2 pkg list 2>/dev/null | grep -q '^mission_pkg$'; then
    echo "  ОШИБКА: mission_pkg не собран. Проверь mounts в docker-compose.yml и"
    echo "  сборку в nav_up.sh (colcon build --packages-select control_pkg mission_pkg)."
    echo "  Первый запуск после добавления mounts требует: make fresh-start."
    exit 1
fi

# ── ручки: профили → env (строго по схеме) ────────────────────────────────────
if [ -z "${PROFILES:-}" ]; then
    echo "  ОШИБКА: PROFILES не задан — ручки ноды живут ТОЛЬКО в профилях."
    echo "  Пример: PROFILES=\"dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25"
    echo "           loiter/guard wind/trim mission/baseline legacy/baseline world/wind2_gust5\""
    echo "  Обычно список держит cmd/<имя>/<имя>.sh (cmd/README.txt, src/control/profiles/README.md)."
    exit 1
fi
LOADER=/root/sim_ws/src/control/profiles/load.py
# shellcheck disable=SC2086
eval "$(python3 "$LOADER" $PROFILES)" || { echo "  ОШИБКА: профили не собрались (см. выше)"; exit 1; }
echo ">>> профили: $PROFILES → $(env | grep -cE '^BS_') ключей BS_ (pilot=$BS_PILOT mission=$BS_MISSION stab=$BS_STAB)"

# ── живой пульт ───────────────────────────────────────────────────────────────
JOY_PID=""
if [ "${BS_PILOT:-}" = "joy" ]; then
    JOY_DEV="${BS_JOY_DEV:-/dev/input/js0}"
    if [ ! -e "$JOY_DEV" ]; then
        echo "  ОШИБКА: BS_PILOT=joy, но $JOY_DEV нет. Пульт в USB-режиме воткнут?"
        echo "  Проверка на хосте: ls /dev/input/js*; в контейнере: jstest $JOY_DEV"
        exit 1
    fi
    ros2 run joy_linux joy_linux_node --ros-args \
        -p dev:="$JOY_DEV" -p autorepeat_rate:=20.0 -p default_trig_val:=true \
        > /root/sim_ws/output/joy.log 2>&1 &
    JOY_PID=$!
    trap '[ -n "$JOY_PID" ] && kill "$JOY_PID" 2>/dev/null || true' EXIT
    echo ">>> joy_linux_node запущен (dev=$JOY_DEV, pid=$JOY_PID, лог output/joy.log)"
fi

# ── виртуальный пилот (реплей) ────────────────────────────────────────────────
if [ "${BS_PILOT:-}" = "replay" ]; then
    RARGS=()
    [ -n "${BS_REPLAY_SCENARIO:-}" ] && RARGS+=(--scenario "$BS_REPLAY_SCENARIO")
    [ -n "${BS_REPLAY_RAW:-}" ]      && RARGS+=(--raw "$BS_REPLAY_RAW")
    if [ ${#RARGS[@]} -eq 0 ]; then
        echo "  ОШИБКА: BS_PILOT=replay требует BS_REPLAY_SCENARIO (сценарий .json)"
        echo "  или BS_REPLAY_RAW (сырой .jsonl). См. src/lab/joystick/README.md."
        exit 1
    fi
    for f in "${BS_REPLAY_SCENARIO:-}" "${BS_REPLAY_RAW:-}"; do
        if [ -n "$f" ] && [ ! -e "$f" ]; then
            echo "  ОШИБКА: файл реплея не найден: $f (путь — ВНУТРИ контейнера:"
            echo "  /lab/joystick/scenarios/... или /root/sim_ws/output/joystick/...)"
            exit 1
        fi
    done
    [ -n "${BS_JOY_SIGNS:-}" ]    && RARGS+=("--signs=$BS_JOY_SIGNS")
    [ -n "${BS_REPLAY_FENCE:-}" ] && RARGS+=(--fence "$BS_REPLAY_FENCE")
    python3 /lab/joystick/joy_replay.py "${RARGS[@]}" \
        > /root/sim_ws/output/joy_replay.log 2>&1 &
    JOY_PID=$!
    trap '[ -n "$JOY_PID" ] && kill "$JOY_PID" 2>/dev/null || true' EXIT
    export BS_PILOT=joy   # нода видит JoyPilot на /joy, как с TX12
    echo ">>> joy_replay запущен (pid=$JOY_PID, лог output/joy_replay.log)"
fi

echo ">>> ARCH2 bootstrap (control_pkg/mission_pkg): stab=$BS_STAB mission=$BS_MISSION pilot=$BS_PILOT"
ros2 run mission_pkg bootstrap_arch2
echo ">>> bootstrap_arch2 завершён."
