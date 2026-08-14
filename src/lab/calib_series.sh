#!/usr/bin/env bash
#
# calib_series.sh — СЕРИЯ одинаковых прогонов для набора статистики.
#
# Зачем: одиночный прогон говорит, что дрейф есть; серия говорит, ПОВТОРЯЕМ ли он.
# Разброс между прогонами — это и есть то, что демпфер обязан вытягивать сверх среднего
# (см. src/control/ToDo.md, критерий п.3: разброс < 30%).
#
# Каждый повтор — полноценный атомарный прогон через calib_run.sh: fresh-start стека,
# полёт, свой bag `<NAME><i>_bag`, своя мета `<NAME><i>.env`, своё видео на Drive
# `13.17/calib/<NAME><i>.mp4`. Ничего не перезатирается.
#
# Запуск (с хоста, отвязанно — серия идёт часами):
#   REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
#   cd $REPO && setsid nohup env NAME=lift_stat N=5 CMD=liftland \
#     bash src/lab/calib_series.sh > /root/run_series.log 2>&1 < /dev/null & disown
#
# Env: NAME (база имени), N (сколько повторов, 5), CMD (лётная команда, liftland),
#      START (с какого индекса начинать, 1) — чтобы дозаливать серию, не затирая старое.
#      Остальное (BS_*, TOPICS_EXTRA, RES, CPU) прокидывается в calib_run.sh как есть.
#
set -uo pipefail        # без -e: падение одного повтора не должно рвать серию

NAME="${NAME:-lift_stat}"
N="${N:-5}"
CMD="${CMD:-liftland}"
START="${START:-1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN="$SCRIPT_DIR/calib_run.sh"

echo "############ СЕРИЯ: $N × '$CMD', имена ${NAME}${START}..${NAME}$((START + N - 1)) ############"
date

ok=0
fail=0
for i in $(seq "$START" $((START + N - 1))); do
    echo
    echo "############ ПОВТОР $i/$((START + N - 1)) — ${NAME}${i} ############"
    if NAME="${NAME}${i}" CMD="$CMD" bash "$RUN"; then
        ok=$((ok + 1))
    else
        fail=$((fail + 1))
        echo "⚠️ повтор ${NAME}${i} завершился с ошибкой — продолжаю серию"
    fi
done

echo
echo "############ СЕРИЯ ЗАВЕРШЕНА: успешно $ok, с ошибкой $fail ############"
date
ls -d "$SCRIPT_DIR/../../docker/sim/output/${NAME}"*_bag 2>/dev/null
