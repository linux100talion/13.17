#!/usr/bin/env bash
# cmd/10/10.sh — A/B-полёт: чек занижения VINS против IPM до 25 м (vins/scale25: alt_max 25,
# ipm_min 1.2) поверх лучшего DpVins (brake5_stop) и LOITER с гейтами (loiter/guard).
# Профили: dphold/baseline + dpvins/brake5_stop + vins/scale25 (кандидат) + loiter/guard,
# ветер 1 м/с с порывами 8 м/с каждые 20 с. Запускать откуда угодно:
#   bash cmd/10/10.sh
# Зачем и что меняет — README.txt рядом. WIND_SPD / WIND_GUST снаружи перекрывают дефолты
# строки (env сильнее профиля — см. src/control/profiles/README.md). Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

set -a
. src/control/profiles/dphold/baseline.txt
. src/control/profiles/dpvins/brake5_stop.txt
. src/control/profiles/vins/scale25.txt
. src/control/profiles/loiter/guard.txt
set +a

export WIND_SPD="${WIND_SPD:-1}"
export WIND_GUST="${WIND_GUST:-spd=8 at=30 rise=2 hold=5 fall=4 every=20}"
echo ">>> cmd/10/10.sh: профили dphold/baseline + dpvins/brake5_stop + vins/scale25 + loiter/guard;" \
     "SCALE_ALT_MAX=$BS_VINS_SCALE_ALT_MAX SCALE_IPM_MIN=$BS_VINS_SCALE_IPM_MIN VEL_SRC=$BS_VINS_VEL_SRC LOITER_GUARD=$BS_LOITER_GUARD WIND_SPD=$WIND_SPD WIND_GUST=\"$WIND_GUST\""
exec bash src/lab/freefly_lv.sh "$@"
