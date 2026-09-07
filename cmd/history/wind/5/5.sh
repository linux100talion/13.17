#!/usr/bin/env bash
# cmd/5/5.sh — A/B-полёт кандидата DpVins brake5_tail + ПО-ОСЕВАЯ ЗАЩЁЛКА ТРИМА
# (BS_DPVINS_LATCH_AXIS=1: на стике морозится только движимая ось, свободная учит трим).
# Профили: dphold/baseline + dpvins/brake5_axis (кандидат) + vins/baseline + loiter/baseline,
# ветер 1 м/с с порывами 8 м/с каждые 20 с. Пути — от корня репы, запускать откуда угодно:
#   bash cmd/5/5.sh
# Зачем и что меняет — README.txt рядом.
# WIND_SPD / WIND_GUST снаружи перекрывают дефолты строки (env сильнее профиля —
# см. src/control/profiles/README.md). Доп. аргументы уходят в freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"   # cmd/history/<кампания>/<n>/ → корень репы
cd "$REPO"

# 2026-09-07: ручки — только профили; список едет в freefly_lv/контейнер как PROFILES
# (mission/ legacy/ vinshold/ — поля ноды вне стека, теми же значениями, что летали;
# ветер — world/wind1_gust8). Архив: экспорты BS_/WIND_ и eval загрузчика удалены.
export PROFILES="dphold/baseline dpvins/brake5_axis vins/baseline loiter/baseline vinshold/baseline mission/baseline legacy/baseline world/wind1_gust8"
echo ">>> $0: профили [$PROFILES]"
exec bash src/lab/freefly_lv.sh "$@"
