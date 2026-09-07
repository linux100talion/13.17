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
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"   # cmd/history/<кампания>/<n>/ → корень репы
cd "$REPO"

# 2026-09-07: ручки — только профили; список едет в freefly_lv/контейнер как PROFILES
# (mission/ legacy/ vinshold/ — поля ноды вне стека, теми же значениями, что летали;
# ветер — world/wind1_gust8). Архив: экспорты BS_/WIND_ и eval загрузчика удалены.
export PROFILES="dphold/baseline dpvins/brake5_tail vins/baseline loiter/baseline vinshold/baseline mission/baseline legacy/baseline world/wind1_gust8"
echo ">>> $0: профили [$PROFILES]"
exec bash src/lab/freefly_lv.sh "$@"
