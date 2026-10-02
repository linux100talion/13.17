#!/usr/bin/env bash
# cmd/ipm_lever/ipm_lever.sh — A/B ВЫНОСА КАМЕРЫ в канале вида сверху (2026-10-02): cmd/bl, но
# сторона задаётся аргументом — 1 = с учётом выноса (dphold/att_own_lever), 0 = без
# (dphold/att_own_nolever, BS_IPM_LEVER=0 — штатный). Зачем — README.txt.
#
#   bash cmd/ipm_lever/ipm_lever.sh 1      # реплей сценария ipm_lever.json, с учётом выноса
#   bash cmd/ipm_lever/ipm_lever.sh 0      # то же без учёта
#   bash cmd/ipm_lever/ipm_lever.sh w      # ФВЧ ускорения в осях курса (BS_IPM_ACC_WORLD=1)
#   bash cmd/ipm_lever/ipm_lever.sh wl     # то же + учёт выноса
#   WORLD_PROF=world/baseline bash cmd/ipm_lever/ipm_lever.sh w   # ветер 5 м/с без порывов (по умолч. wind2_gust5)
#
# Скрипт ДЕРЖИТ ТОЛЬКО СПИСОК профилей (cmd/README.txt). Реплей по умолчанию; живой пилот —
# BS_PILOT=joy (тогда mission/att_out вместо mission/att_out_replay).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

SIDE="${1:-1}"
case "$SIDE" in
    1) DP=dphold/att_own_lever ;;
    0) DP=dphold/att_own_nolever ;;
    w) DP=dphold/att_own_accw ;;          # ФВЧ ускорения в осях курса, вынос не учтён
    wl) DP=dphold/att_own_accw_lever ;;   # то же + учёт выноса
    w60) DP=dphold/yaw60_w ;;             # штат + yaw-стик 60 °/с (BS_YAW_PILOT_GAIN 333)
    wl60) DP=dphold/yaw60_wl ;;           # то же + учёт выноса
    *) echo "ipm_lever.sh: сторона 1|0|w|wl|w60|wl60, дано '$SIDE'" >&2; exit 2 ;;
esac
export BS_PILOT="${BS_PILOT:-replay}"
if [ "$BS_PILOT" = "replay" ]; then
    export BS_REPLAY_SCENARIO="${BS_REPLAY_SCENARIO:-/lab/joystick/scenarios/ipm_lever.json}"
    M=mission/att_out_replay
else
    M=mission/att_out
fi
export PROFILES="$DP dpvins/board1 vinshold/baseline vins/scale25 loiter/boot_ipm wind/trim $M legacy/baseline ${WORLD_PROF:-world/wind2_gust5}"
W_TAG=""; [ -n "${WORLD_PROF:-}" ] && W_TAG="_$(basename "$WORLD_PROF")"
export NAME="${NAME:-lever${SIDE}${W_TAG}_$(date +%Y%m%d_%H%M%S)}"
echo ">>> cmd/ipm_lever: сторона $SIDE ($DP), пилот $BS_PILOT; профили [$PROFILES]"
exec bash src/lab/freefly_lv.sh
