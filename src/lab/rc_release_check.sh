#!/usr/bin/env bash
# rc_release_check.sh — АТОМАРНЫЙ наземный прогон: семантика нуля в
# RC_CHANNELS_OVERRIDE (на чём стоит сторож свежести пульта, см. gates.md).
#
# Дисциплина прогона: стек поднимается ЦЕЛИКОМ (restart-all → wait), дальше всё
# делает один скрипт. Полёта нет — борт стоит дизармированным, лётная нода не
# запускается (иначе на override было бы два писателя).
#
# Замер идёт ПРЯМЫМ MAVLink мимо MAVROS (pymavlink → эндпоинт mavlink-router
# 14541 «кастомный узел»): меряем полётник, лишний слой только добавил бы
# подозреваемых. Тот же скрипт годится на борту — там роутер тоже раздаёт UDP.
#
# Запуск:  bash src/lab/rc_release_check.sh
# Env: SKIP_RESTART=1 — не трогать стек; PY — интерпретатор с pymavlink;
#      URL — эндпоинт (дефолт udpin:127.0.0.1:14541)
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SIM="$HERE/../../docker/sim"

# интерпретатор с pymavlink: системный, иначе venv ardupilot
PY="${PY:-}"
if [ -z "$PY" ]; then
    if python3 -c 'import pymavlink' 2>/dev/null; then PY=python3
    elif [ -x "$HOME/venv-ardupilot/bin/python3" ]; then PY="$HOME/venv-ardupilot/bin/python3"
    else echo "ОШИБКА: нет python с pymavlink (PY=<путь> или pip install pymavlink)" >&2; exit 2; fi
fi

if [ "${SKIP_RESTART:-0}" != "1" ]; then
    echo ">>> стек целиком: restart-all → wait"
    make -C "$SIM" restart-all 2>&1 | tail -3
    # для ЭТОГО замера нужен только SITL: nav может не дойти до «готово»
    # (сборка/камера) — предупреждаем, но меряем
    make -C "$SIM" wait 2>&1 | tail -3 || echo "!!! nav не дописал «готово» — для замера хватит SITL"
fi

echo ">>> замер (борт на земле, дизармирован, лётной ноды нет); python: $PY"
"$PY" "$HERE/rc_release_check.py" ${URL:+--url "$URL"}
