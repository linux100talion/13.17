#!/usr/bin/env bash
# cmd/8/8.sh — A/B-полёт: DpVins brake5_line + СТОП ПРИ ОТПУСКАНИИ (settle_brake, pin_t 3), twist.
# Профили: dphold/baseline + dpvins/brake5_stop (кандидат) + vins/twist + loiter/baseline,
# ветер 1 м/с с порывами 8 м/с каждые 20 с. Запускать откуда угодно:
#   bash cmd/8/8.sh
# Зачем и что меняет — README.txt рядом. WIND_SPD / WIND_GUST снаружи перекрывают дефолты
# строки (env сильнее профиля — см. src/control/profiles/README.md). Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

set -a
. src/control/profiles/dphold/baseline.txt
. src/control/profiles/dpvins/brake5_stop.txt
. src/control/profiles/vins/twist.txt
. src/control/profiles/loiter/baseline.txt
set +a

export WIND_SPD="${WIND_SPD:-1}"
export WIND_GUST="${WIND_GUST:-spd=8 at=30 rise=2 hold=5 fall=4 every=20}"
echo ">>> cmd/8/8.sh: профили dphold/baseline + dpvins/brake5_stop + vins/twist + loiter/baseline;" \
     "SETTLE_BRAKE=$BS_DPVINS_SETTLE_BRAKE PIN_T=$BS_DPVINS_PIN_T FF=$BS_DPVINS_FF LINE_HOLD=$BS_DPVINS_LINE_HOLD WIND_SPD=$WIND_SPD WIND_GUST=\"$WIND_GUST\""
exec bash src/lab/freefly_lv.sh "$@"
