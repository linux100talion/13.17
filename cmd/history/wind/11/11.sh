#!/usr/bin/env bash
# cmd/11/11.sh — A/B-полёт: общий ветровой трим ярусов 0/1 (WindTrim, wind/trim.txt) поверх cmd/10
# (dphold/baseline + dpvins/brake5_stop + vins/scale25 + loiter/guard), ветер 2 м/с с порывами
# 5 м/с каждые 20 с. Два плеча одним скриптом:
#   bash cmd/11/11.sh            # B: WindTrim ВКЛ (wind/trim.txt)
#   WT=0 bash cmd/11/11.sh       # A: старое — свой трим у каждого яруса + посев (wind/baseline.txt)
# Зачем и что меняет — README.txt рядом. WIND_SPD / WIND_GUST снаружи перекрывают дефолты строки
# (env сильнее профиля — см. src/control/profiles/README.md). Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

WT="${WT:-1}"
set -a
. src/control/profiles/dphold/baseline.txt
. src/control/profiles/dpvins/brake5_stop.txt
. src/control/profiles/vins/scale25.txt
. src/control/profiles/loiter/guard.txt
if [ "$WT" = "0" ]; then
  . src/control/profiles/wind/baseline.txt
else
  . src/control/profiles/wind/trim.txt
fi
set +a

export WIND_SPD="${WIND_SPD:-2}"
export WIND_GUST="${WIND_GUST:-spd=5 at=30 rise=2 hold=5 fall=4 every=20}"
echo ">>> cmd/11/11.sh: плечо $([ "$WT" = "0" ] && echo 'A (wind/baseline: свой трим + посев)' || echo 'B (wind/trim: WindTrim)');" \
     "WIND_TRIM=$BS_WIND_TRIM STEADY_SEC=$BS_WIND_STEADY_SEC STEADY_V=$BS_WIND_STEADY_V" \
     "VEL_SRC=$BS_VINS_VEL_SRC LOITER_GUARD=$BS_LOITER_GUARD WIND_SPD=$WIND_SPD WIND_GUST=\"$WIND_GUST\""
exec bash src/lab/freefly_lv.sh "$@"
