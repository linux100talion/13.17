#!/usr/bin/env bash
# cmd/vz_phantom/vz_phantom.sh — A/B ФАНТОМА НАБОРА канала вида сверху (2026-10-02): cmd/bl под
# реплеем сценария vz_phantom.json; сторона 0 = прежний учёт (att_own_oldscale), 1 = штат (scale_exact).
# Зачем — README.txt.
#
#   bash cmd/vz_phantom/vz_phantom.sh 0
#   bash cmd/vz_phantom/vz_phantom.sh 1
#   bash cmd/vz_phantom/vz_phantom.sh c      # + клиренс корпуса (BS_IPM_GROUND_CLEAR 0.195)
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
SIDE="${1:-1}"
case "$SIDE" in
    0) DP=dphold/att_own_oldscale ;;   # прежний учёт (BS_IPM_SCALE_EXACT=0)
    1) DP=dphold/scale_exact ;;          # = штат att_own (клиренс 0)
    c) DP=dphold/ground_clear ;;         # штат + клиренс корпуса 0.195 в геометрии
    *) echo "vz_phantom.sh: сторона 0|1|c, дано '$SIDE'" >&2; exit 2 ;;
esac
export BS_PILOT=replay
export BS_REPLAY_SCENARIO="${BS_REPLAY_SCENARIO:-/lab/joystick/scenarios/vz_phantom.json}"
export PROFILES="$DP dpvins/board1 vinshold/baseline vins/scale25 loiter/boot_ipm wind/trim mission/att_out_replay legacy/baseline world/wind2_gust5"
export NAME="${NAME:-vzph_s${SIDE}_$(date +%Y%m%d_%H%M%S)}"
echo ">>> cmd/vz_phantom: сторона $SIDE ($DP); профили [$PROFILES]"
exec bash src/lab/freefly_lv.sh
