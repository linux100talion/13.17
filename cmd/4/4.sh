#!/usr/bin/env bash
# cmd/4/4.sh — A/B-полёт кандидата DpVins BRAKE 5 с хвостом брейка как у демпфера
# (трим в брейке не заморожен, brake_t −1; ki 8) под порывами (серия dphold_vs_dpvins).
# Профили: dphold/baseline + dpvins/brake5_tail (кандидат) + vins/baseline + loiter/baseline,
# ветер 1 м/с с порывами 8 м/с каждые 20 с. Пути — от корня репы, запускать откуда угодно:
#   bash cmd/4/4.sh
# Зачем и что меняет — README.txt рядом.
# WIND_SPD / WIND_GUST снаружи перекрывают дефолты строки (env сильнее профиля —
# см. src/control/profiles/README.md). Доп. аргументы уходят в freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

set -a
. src/control/profiles/dphold/baseline.txt
. src/control/profiles/dpvins/brake5_tail.txt
. src/control/profiles/vins/baseline.txt
. src/control/profiles/loiter/baseline.txt
set +a

export WIND_SPD="${WIND_SPD:-1}"
export WIND_GUST="${WIND_GUST:-spd=8 at=30 rise=2 hold=5 fall=4 every=20}"
echo ">>> cmd/4/4.sh: профили dphold/baseline + dpvins/brake5_tail + vins/baseline + loiter/baseline;" \
     "BS_DPVINS_KI=$BS_DPVINS_KI BRAKE=$BS_DPVINS_POS_BRAKE/$BS_DPVINS_POS_BRAKE_VMAX BRAKE_T=$BS_DPVINS_POS_BRAKE_T WIND_SPD=$WIND_SPD WIND_GUST=\"$WIND_GUST\""
exec bash src/lab/freefly_lv.sh "$@"
