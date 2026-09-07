#!/usr/bin/env bash
# cmd/9/9.sh — A/B-полёт: ярус LOITER с гейтами яруса 1 (loiter/guard: зрелость потока, удержание
# 5 с после выхода, закрытый мост = выход) поверх лучшего DpVins (brake5_stop) и twist.
# Профили: dphold/baseline + dpvins/brake5_stop + vins/twist + loiter/guard (кандидат),
# ветер 1 м/с с порывами 8 м/с каждые 20 с. Запускать откуда угодно:
#   bash cmd/9/9.sh
# Зачем и что меняет — README.txt рядом. WIND_SPD / WIND_GUST снаружи перекрывают дефолты
# строки (env сильнее профиля — см. src/control/profiles/README.md). Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"   # cmd/history/<кампания>/<n>/ → корень репы
cd "$REPO"

# 2026-09-07: ручки — только профили; список едет в freefly_lv/контейнер как PROFILES
# (mission/ legacy/ vinshold/ — поля ноды вне стека, теми же значениями, что летали;
# ветер — world/wind1_gust8). Архив: экспорты BS_/WIND_ и eval загрузчика удалены.
export PROFILES="dphold/baseline dpvins/brake5_stop vins/twist loiter/guard vinshold/baseline mission/baseline legacy/baseline world/wind1_gust8"
echo ">>> $0: профили [$PROFILES]"
exec bash src/lab/freefly_lv.sh "$@"
