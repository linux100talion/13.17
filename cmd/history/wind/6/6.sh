#!/usr/bin/env bash
# cmd/6/6.sh — A/B-полёт: скорость VINS из TWIST (vins/twist) + гвоздь сразу на входе при
# посеянном триме (dpvins/brake5_pin = brake5_axis + BS_DPVINS_PIN_ARMED=1).
# Профили: dphold/baseline + dpvins/brake5_pin + vins/twist + loiter/baseline,
# ветер 1 м/с с порывами 8 м/с каждые 20 с. Пути — от корня репы, запускать откуда угодно:
#   bash cmd/6/6.sh
# Зачем и что меняет — README.txt рядом.
# WIND_SPD / WIND_GUST снаружи перекрывают дефолты строки (env сильнее профиля —
# см. src/control/profiles/README.md). Доп. аргументы уходят в freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"   # cmd/history/<кампания>/<n>/ → корень репы
cd "$REPO"

P=()
P+=(dphold/baseline)
P+=(dpvins/brake5_pin)
P+=(vins/twist)
P+=(loiter/baseline)
# профили собирает load.py (с 2026-09-07: include + дельта, дубль = ошибка); mission/ и legacy/ —
# поля ноды вне стабилизаторов теми же значениями, что летали (дефолты ноды/env) — поведение то же
set -a
eval "$(python3 src/control/profiles/load.py "${P[@]}" vinshold/baseline mission/baseline legacy/baseline)"
set +a

export WIND_SPD="${WIND_SPD:-1}"
export WIND_GUST="${WIND_GUST:-spd=8 at=30 rise=2 hold=5 fall=4 every=20}"
echo ">>> cmd/6/6.sh: профили dphold/baseline + dpvins/brake5_pin + vins/twist + loiter/baseline;" \
     "VEL_SRC=$BS_VINS_VEL_SRC PIN_ARMED=$BS_DPVINS_PIN_ARMED KI=$BS_DPVINS_KI BRAKE=$BS_DPVINS_POS_BRAKE/$BS_DPVINS_POS_BRAKE_VMAX WIND_SPD=$WIND_SPD WIND_GUST=\"$WIND_GUST\""
exec bash src/lab/freefly_lv.sh "$@"
