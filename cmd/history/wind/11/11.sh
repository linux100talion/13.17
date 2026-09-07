#!/usr/bin/env bash
# cmd/11/11.sh — A/B-полёт: общий ветровой трим ярусов 0/1 (WindTrim, wind/trim.txt) поверх cmd/10
# (dphold/baseline + dpvins/brake5_stop + vins/scale25 + loiter/guard), ветер 2 м/с с порывами
# 5 м/с каждые 20 с. Два плеча одним скриптом:
#   bash cmd/11/11.sh            # B: WindTrim ВКЛ (wind/trim.txt)
#   WT=0 bash cmd/11/11.sh       # A: старое — свой трим у каждого яруса + посев (wind/baseline.txt)
# Зачем и что меняет — README.txt рядом. WIND_SPD / WIND_GUST снаружи перекрывают дефолты строки
# (env сильнее профиля — см. src/control/profiles/README.md). Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"   # cmd/history/<кампания>/<n>/ → корень репы
cd "$REPO"

WT="${WT:-1}"
# 2026-09-07: ручки — только профили; список едет в freefly_lv/контейнер как PROFILES
# (mission/ legacy/ vinshold/ — поля ноды вне стека, теми же значениями, что летали;
# ветер — world/wind2_gust5). Архив: экспорты BS_/WIND_ и eval загрузчика удалены.
P="dphold/baseline dpvins/brake5_stop vins/scale25 loiter/guard"
if [ "$WT" = "0" ]; then P="$P wind/baseline"; else P="$P wind/trim"; fi
export PROFILES="$P vinshold/baseline mission/baseline legacy/baseline world/wind2_gust5"
echo ">>> $0: профили [$PROFILES]"
exec bash src/lab/freefly_lv.sh "$@"
