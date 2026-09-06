#!/usr/bin/env bash
# cmd/9/9.sh — A/B-полёт: ярус LOITER с гейтами яруса 1 (loiter/guard: зрелость потока, удержание
# 5 с после выхода, закрытый мост = выход) поверх лучшего DpVins (brake5_stop) и twist.
# Профили: dphold/baseline + dpvins/brake5_stop + vins/twist + loiter/guard (кандидат),
# ветер 1 м/с с порывами 8 м/с каждые 20 с. Запускать откуда угодно:
#   bash cmd/9/9.sh
# Зачем и что меняет — README.txt рядом. WIND_SPD / WIND_GUST снаружи перекрывают дефолты
# строки (env сильнее профиля — см. src/control/profiles/README.md). Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

set -a
. src/control/profiles/dphold/baseline.txt
. src/control/profiles/dpvins/brake5_stop.txt
. src/control/profiles/vins/twist.txt
. src/control/profiles/loiter/guard.txt
set +a

export WIND_SPD="${WIND_SPD:-1}"
export WIND_GUST="${WIND_GUST:-spd=8 at=30 rise=2 hold=5 fall=4 every=20}"
echo ">>> cmd/9/9.sh: профили dphold/baseline + dpvins/brake5_stop + vins/twist + loiter/guard;" \
     "LOITER_GUARD=$BS_LOITER_GUARD VEL_SRC=$BS_VINS_VEL_SRC SETTLE_BRAKE=$BS_DPVINS_SETTLE_BRAKE WIND_SPD=$WIND_SPD WIND_GUST=\"$WIND_GUST\""
exec bash src/lab/freefly_lv.sh "$@"
