#!/usr/bin/env bash
# cmd/att_out/att_out.sh — A/B ВЫХОДА ДОМЕНА: override (PWM) против углов (SET_ATTITUDE_TARGET).
# Стек = cmd/bl бит в бит; между сторонами меняется РОВНО один ключ BS_ATT_OUT
# (mission/replay → mission/att_out_replay). Маршрут — реплей (по умолчанию vins_init.json).
#
#   bash cmd/att_out/att_out.sh rc     # override, как всегда
#   bash cmd/att_out/att_out.sh att    # углы: ALT_HOLD — пилот по RC-входу, в воздухе GUIDED_NOGPS
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

SIDE="${1:-}"
[ $# -ge 1 ] && shift
case "$SIDE" in
    rc)  M=mission/replay ;;
    att) M=mission/att_out_replay ;;
    *) echo "ОШИБКА: сторона — rc или att (дано '${SIDE:-пусто}')" >&2; exit 2 ;;
esac
export BS_PILOT=replay
export BS_REPLAY_SCENARIO="${BS_REPLAY_SCENARIO:-/lab/joystick/scenarios/vins_init.json}"
export NAME="${NAME:-attout_${SIDE}_$(date +%Y%m%d_%H%M%S)}"
export PROFILES="dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 loiter/guard wind/trim $M legacy/baseline world/wind2_gust5"
echo ">>> cmd/att_out: сторона $SIDE ($M), прогон $NAME, сценарий $BS_REPLAY_SCENARIO"
exec bash src/lab/freefly_lv.sh "$@"
