#!/usr/bin/env bash
# check.sh — расхождения профилей с метой прогона <RUN>.env (пишет freefly_lv.sh на каждый
# прогон). Тонкая обёртка над load.py --diff: профили собираются загрузчиком (include +
# дельта, дубли между профилями = ошибка), сравниваются ключи, которые в мете есть; ключи
# только в профилях (в мету не попали — до этапа «нода без дефолтов» это дефолт ноды)
# считаются отдельно.  Использование:
#   bash src/control/profiles/check.sh docker/sim/output/joystick/<RUN>/<RUN>.env [профиль...]
# Без списка профилей — активный стек cmd/bl (ATT=1 WT=1, с 2026-10-01): dphold/att_own
# dpvins/board1 vinshold/baseline vins/scale25 loiter/boot_ipm wind/trim mission/att_out legacy/baseline.
set -euo pipefail
ENVF=${1:?путь к <RUN>.env}; shift || true
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROFILES=("$@")
[ ${#PROFILES[@]} -eq 0 ] && PROFILES=(dphold/att_own dpvins/board1 vinshold/baseline vins/scale25 loiter/boot_ipm wind/trim mission/att_out legacy/baseline)
exec python3 "$HERE/load.py" --diff "$ENVF" "${PROFILES[@]}"
