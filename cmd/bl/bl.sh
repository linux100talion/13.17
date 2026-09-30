#!/usr/bin/env bash
# cmd/bl/bl.sh — ТЕКУЩИЙ BASELINE с 2026-09-30: ВЫХОД В УГЛАХ (GUIDED_NOGPS, SET_ATTITUDE_TARGET,
# арм нодой) + DpVins под отклик борта (dpvins/board1), поверх прежнего стека: dphold/baseline,
# vinshold/baseline, vins/scale25, loiter/guard, WindTrim; ветер 2 м/с + порывы 5 каждые 20 с.
# Прежний baseline (override + brake5_stop) — cmd/history/bl/1/. Зачем и что меняет — README.txt.
#
#   bash cmd/bl/bl.sh             # выход в углах (mission/att_out)
#   ATT=0 bash cmd/bl/bl.sh       # вариант: выход override, ALT_HOLD — как летали до 2026-09-30
#   WT=0 bash cmd/bl/bl.sh        # вариант: свой трим у каждого яруса + посев (wind/baseline)
#   BS_PILOT=replay BS_REPLAY_SCENARIO=/lab/joystick/scenarios/<имя>.json bash cmd/bl/bl.sh
#
# Скрипт ДЕРЖИТ ТОЛЬКО СПИСОК профилей (договорённость 2026-09-07, cmd/README.txt): ручки — в
# src/control/profiles, собирает их load.py строго по схеме BootstrapConfig. Список = load.BASELINE_STACK
# (тесты BootstrapConfig.baseline(), check.sh без аргументов) при ATT=1 WT=1 без реплея — менять вместе.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

ATT="${ATT:-1}"
WT="${WT:-1}"
P="dphold/baseline dpvins/board1 vinshold/baseline vins/scale25 loiter/guard"
if [ "$WT" = "0" ]; then P="$P wind/baseline"; else P="$P wind/trim"; fi
if [ "${BS_PILOT:-}" = "replay" ]; then
    if [ "$ATT" = "0" ]; then P="$P mission/replay"; else P="$P mission/att_out_replay"; fi
else
    if [ "$ATT" = "0" ]; then P="$P mission/baseline"; else P="$P mission/att_out"; fi
fi
export PROFILES="$P legacy/baseline world/wind2_gust5"
echo ">>> cmd/bl/bl.sh: выход $([ "$ATT" = "0" ] && echo 'override (ALT_HOLD)' || echo 'в углах (GUIDED_NOGPS)'), трим $([ "$WT" = "0" ] && echo 'свой + посев' || echo 'WindTrim'); профили [$PROFILES]"
exec bash src/lab/freefly_lv.sh "$@"
