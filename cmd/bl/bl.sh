#!/usr/bin/env bash
# cmd/bl/bl.sh — BASELINE с 2026-09-06 (бывш. cmd/11, история — cmd/history/wind/): A/B-полёт: общий ветровой трим ярусов 0/1 (WindTrim, wind/trim.txt) поверх cmd/10
# (dphold/baseline + dpvins/brake5_stop + vins/scale25 + loiter/guard), ветер 2 м/с с порывами
# 5 м/с каждые 20 с. Два плеча одним скриптом:
#   bash cmd/bl/bl.sh            # B: WindTrim ВКЛ (wind/trim.txt)
#   WT=0 bash cmd/bl/bl.sh       # A: старое — свой трим у каждого яруса + посев (wind/baseline.txt)
# Зачем и что меняет — README.txt рядом. WIND_SPD / WIND_GUST снаружи перекрывают дефолты строки
# (они через ${X:-…}); ключи BS_* из профилей, наоборот, перекрывают внешний env и .env —
# профили собирает src/control/profiles/load.py (include + дельта, дубль ключа между профилями =
# ошибка; см. src/control/profiles/README.md, docker/sim/env.md). Реплей пульта:
#   BS_PILOT=replay BS_REPLAY_SCENARIO=<…>.json bash cmd/bl/bl.sh   (профиль mission/replay)
# Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

WT="${WT:-1}"
P=(dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 loiter/guard)
if [ "$WT" = "0" ]; then P+=(wind/baseline); else P+=(wind/trim); fi
# миссия/пилот и легаси-поля ноды — тоже из профилей (с 2026-09-07 у каждого поля ноды есть
# место в профилях; сценарий реплея — аргумент прогона, не параметр)
if [ "${BS_PILOT:-}" = "replay" ]; then P+=(mission/replay); else P+=(mission/baseline); fi
P+=(legacy/baseline)
set -a
eval "$(python3 src/control/profiles/load.py "${P[@]}")"
set +a

export WIND_SPD="${WIND_SPD:-2}"
export WIND_GUST="${WIND_GUST:-spd=5 at=30 rise=2 hold=5 fall=4 every=20}"
echo ">>> cmd/bl/bl.sh: плечо $([ "$WT" = "0" ] && echo 'A (wind/baseline: свой трим + посев)' || echo 'B (wind/trim: WindTrim)');" \
     "WIND_TRIM=$BS_WIND_TRIM STEADY_SEC=$BS_WIND_STEADY_SEC STEADY_V=$BS_WIND_STEADY_V" \
     "VEL_SRC=$BS_VINS_VEL_SRC LOITER_GUARD=$BS_LOITER_GUARD PILOT=$BS_PILOT WIND_SPD=$WIND_SPD WIND_GUST=\"$WIND_GUST\"" \
     "(профили: ${P[*]})"
exec bash src/lab/freefly_lv.sh "$@"
