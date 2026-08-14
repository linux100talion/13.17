#!/usr/bin/env bash
# yaw_kp_sweep.sh — СВИП одной ручки по списку значений (по умолчанию yaw_kp).
#
# Отличие от hover_series.sh: тот гоняет N ОДИНАКОВЫХ прогонов (снимает разброс
# стенда), этот — по одному прогону на каждое значение ручки (ищет порог дрожи).
# Методика — tune.md Фаза 3: свип одиночными, порог дрожи, рабочее значение вдвое
# ниже порога, и только ПОБЕДИТЕЛЬ проверяется серией n=3 против базы.
#
# Зачем именно yaw_kp. После перевода рыскания в pos-режим (см. control.md) ось
# получила курс-холд: kp — П-член по НАКОПЛЕННОМУ визуальному курсу. Единицы:
# 1 ед. сигнала = 1/S градусов, S = 0.253 px/кадр на °/с (замер Y1s), то есть
# 3.95°. Значит ошибка 10° даёт kp·2.53 PWM: kp=10 → 25 PWM, kp=40 → 101 PWM.
#
# ⚠️ kp=0 — ОБЯЗАТЕЛЬНАЯ первая точка, это контроль рефактора: при нём выход
# совпадает с прежним законом побитово (проверено юнит-тестом), значит прогон
# обязан воспроизвести доpos-режимное поведение. Не воспроизвёл — сломан порт,
# и остальные точки свипа смысла не имеют.
#
# Каждый прогон атомарен (calib_run.sh сам делает fresh-start + wait), свип просто
# ставит их в очередь. Видео каждого уходит на Drive под своим именем.
#
# Запуск (отвязанно, чтобы переживал обрыв связи):
#   REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
#   cd $REPO/docker/sim && setsid nohup env PREFIX=Y2 KNOB=BS_YAW_KP \
#     VALUES="0 10 20 40" BS_STAB="GzRollHold+GzPitchHold+DpYawHold" \
#     BS_MISSION="climb3,hover5,yaw_l30,hover8,yaw_r60,hover8,land" ... \
#     bash $REPO/src/lab/yaw_kp_sweep.sh > output/Y2_sweep.log 2>&1 < /dev/null &
#
# Env: PREFIX (имя-основа), KNOB (имя BS_*-переменной), VALUES (значения через
#      пробел), STRIP (1 = потрошить бэг), остальное — как у calib_run.sh.
set -u

PREFIX="${PREFIX:-sweep}"
KNOB="${KNOB:-BS_YAW_KP}"
VALUES="${VALUES:-0 10 20 40}"
STRIP="${STRIP:-1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"   # корень репы — от места скрипта, не зашит
OUT="$REPO_ROOT/docker/sim/output"

i=0
for v in $VALUES; do
    i=$((i + 1))
    # имя несёт значение ручки: kp=0 → Y2_kp0, kp=10 → Y2_kp10 (точка → 'p')
    NAME="${PREFIX}_$(echo "$KNOB" | sed 's/^BS_YAW_//; s/^BS_//' | tr 'A-Z' 'a-z')${v//./p}"
    echo "############ СВИП ${PREFIX}: точка $i → ${KNOB}=${v} → $NAME ############"
    NAME="$NAME" env "${KNOB}=${v}" bash "$SCRIPT_DIR/calib_run.sh"
    rc=$?
    echo "############ $NAME завершён (код $rc) ############"
    if [ "$STRIP" = "1" ] && [ -d "$OUT/${NAME}_bag" ]; then
        docker exec p1317_nav bash -lc \
            "source /opt/ros/humble/setup.bash; python3 /lab/strip_bags.py /root/sim_ws/output/${NAME}_bag" \
            2>&1 | tail -2
    fi
    df -h / | tail -1
done
echo "############ СВИП ${PREFIX} ЗАВЕРШЁН ($i точек) ############"
