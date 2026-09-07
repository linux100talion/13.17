#!/usr/bin/env bash
# cmd/7/7.sh — A/B-полёт: DpVins brake5_pin + скорость стику (cmd_gain 5, ff 10) + линия свободной
# оси на плече (line_hold), twist. Профили: dphold/baseline + dpvins/brake5_line (кандидат) +
# vins/twist + loiter/baseline, ветер 1 м/с с порывами 8 м/с каждые 20 с. Запускать откуда угодно:
#   bash cmd/7/7.sh
# Зачем и что меняет — README.txt рядом. WIND_SPD / WIND_GUST снаружи перекрывают дефолты
# строки (env сильнее профиля — см. src/control/profiles/README.md). Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"   # cmd/history/<кампания>/<n>/ → корень репы
cd "$REPO"

P=()
P+=(dphold/baseline)
P+=(dpvins/brake5_line)
P+=(vins/twist)
P+=(loiter/baseline)
# профили собирает load.py (с 2026-09-07: include + дельта, дубль = ошибка); mission/ и legacy/ —
# поля ноды вне стабилизаторов теми же значениями, что летали (дефолты ноды/env) — поведение то же
set -a
eval "$(python3 src/control/profiles/load.py "${P[@]}" vinshold/baseline mission/baseline legacy/baseline)"
set +a

export WIND_SPD="${WIND_SPD:-1}"
export WIND_GUST="${WIND_GUST:-spd=8 at=30 rise=2 hold=5 fall=4 every=20}"
echo ">>> cmd/7/7.sh: профили dphold/baseline + dpvins/brake5_line + vins/twist + loiter/baseline;" \
     "CMD_GAIN=$BS_DPVINS_CMD_GAIN FF=$BS_DPVINS_FF LINE_HOLD=$BS_DPVINS_LINE_HOLD VEL_SRC=$BS_VINS_VEL_SRC WIND_SPD=$WIND_SPD WIND_GUST=\"$WIND_GUST\""
exec bash src/lab/freefly_lv.sh "$@"
