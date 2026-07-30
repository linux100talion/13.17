#!/usr/bin/env bash
# hover_series.sh — СЕРИЯ одинаковых калибровочных прогонов (база: среднее и разброс).
#
# Зачем. Серия J показала, что одиночным прогоном настройки различать НЕЛЬЗЯ: при
# полностью одинаковом конфиге уход за 40 с висения вышел 13.3 и 7.4 м (J1c, два
# полёта в одном бэге), а J4/J5 дали 7.7 и 13 м на разнице в одной проводке ω. То
# есть разброс стенда сопоставим с самим эффектом. База из N повторов даёт среднее и
# разброс, без которых следующее «стало лучше» — самообман.
#
# Что делает: N раз зовёт calib_run.sh с ОДНИМИ И ТЕМИ ЖЕ BS_*, именами <PREFIX><i>,
# и ПОСЛЕ каждого прогона потрошит его бэг (strip_bags.py: выкидывает /image_color,
# телеметрию оставляет). Потрошение обязательно: три бэга по 2.6 ГБ на диск не влезают,
# а видео к этому моменту уже собрано и залито.
#
# Каждый прогон — атомарный (calib_run.sh сам делает fresh-start + wait), серия просто
# ставит их в очередь. Видео каждого уходит на Drive под своим именем.
#
# Запуск (отвязанно, чтобы переживал обрыв связи):
#   cd /root/13.17/docker/sim && setsid nohup env N=3 PREFIX=K1s \
#     BS_STAB="DpRollHold+DpPitchHold" BS_MISSION="climb3,hover40,land" ... \
#     bash /root/13.17/src/lab/hover_series.sh > output/K1_series.log 2>&1 < /dev/null &
#
# Env: N (сколько прогонов, 3), PREFIX (имя-основа, series), STRIP (1 = потрошить),
#      остальное — как у calib_run.sh (BS_*, GDRIVE_UP, CPU).
set -u

N="${N:-3}"
PREFIX="${PREFIX:-series}"
STRIP="${STRIP:-1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="/root/13.17/docker/sim/output"

for i in $(seq 1 "$N"); do
    NAME="${PREFIX}${i}"
    echo "############ СЕРИЯ ${PREFIX}: прогон $i из $N → $NAME ############"
    NAME="$NAME" bash "$SCRIPT_DIR/calib_run.sh"
    rc=$?
    echo "############ $NAME завершён (код $rc) ############"
    if [ "$STRIP" = "1" ] && [ -d "$OUT/${NAME}_bag" ]; then
        # Потрошим ВНУТРИ nav: rosbag2_py и overlay живут там, а не на хосте.
        docker exec p1317_nav bash -lc \
            "source /opt/ros/humble/setup.bash; python3 /lab/strip_bags.py /root/sim_ws/output/${NAME}_bag" \
            2>&1 | tail -2
    fi
    df -h / | tail -1
done
echo "############ СЕРИЯ ${PREFIX} ЗАВЕРШЕНА ($N прогонов) ############"
