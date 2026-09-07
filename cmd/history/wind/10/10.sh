#!/usr/bin/env bash
# cmd/10/10.sh — A/B-полёт: чек занижения VINS против IPM до 25 м (vins/scale25: alt_max 25,
# ipm_min 1.2) поверх лучшего DpVins (brake5_stop) и LOITER с гейтами (loiter/guard).
# Профили: dphold/baseline + dpvins/brake5_stop + vins/scale25 (кандидат) + loiter/guard,
# ветер 1 м/с с порывами 8 м/с каждые 20 с. Запускать откуда угодно:
#   bash cmd/10/10.sh
# Зачем и что меняет — README.txt рядом. WIND_SPD / WIND_GUST снаружи перекрывают дефолты
# строки (env сильнее профиля — см. src/control/profiles/README.md). Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"   # cmd/history/<кампания>/<n>/ → корень репы
cd "$REPO"

# 2026-09-07: ручки — только профили; список едет в freefly_lv/контейнер как PROFILES
# (mission/ legacy/ vinshold/ — поля ноды вне стека, теми же значениями, что летали;
# ветер — world/wind2_gust5). Архив: экспорты BS_/WIND_ и eval загрузчика удалены.
export PROFILES="dphold/baseline dpvins/brake5_stop vins/scale25 loiter/guard vinshold/baseline mission/baseline legacy/baseline world/wind2_gust5"
echo ">>> $0: профили [$PROFILES]"
exec bash src/lab/freefly_lv.sh "$@"
