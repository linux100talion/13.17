#!/usr/bin/env bash
# cmd/att_oracle/att_oracle.sh — ОПЫТ (сим): крен/тангаж канала вида сверху из ИСТИНЫ Gazebo.
# Стек = cmd/bl (выход в углах + dpvins/board1) бит в бит, отличие — dphold/att_oracle
# (BS_PERC_ATT_SRC=truth). Сторона ekf — тот же список с dphold/baseline, для пары.
#   bash cmd/att_oracle/att_oracle.sh truth     # оракул
#   bash cmd/att_oracle/att_oracle.sh ekf       # как летали (контроль)
# Сценарий по умолчанию — vins_init_5.json (взлёт сразу на 5 м). Зачем — README.txt рядом.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
SIDE="${1:-}"
[ $# -ge 1 ] && shift
case "$SIDE" in
    truth) D=dphold/att_oracle ;;
    ekf)   D=dphold/baseline ;;
    *) echo "ОШИБКА: сторона — truth или ekf (дано '${SIDE:-пусто}')" >&2; exit 2 ;;
esac
export BS_PILOT=replay
export BS_REPLAY_SCENARIO="${BS_REPLAY_SCENARIO:-/lab/joystick/scenarios/vins_init_5.json}"
export NAME="${NAME:-attoracle_${SIDE}_$(date +%Y%m%d_%H%M%S)}"
export PROFILES="$D dpvins/board1 vinshold/baseline vins/scale25 loiter/guard wind/trim mission/att_out_replay legacy/baseline world/wind2_gust5"
echo ">>> cmd/att_oracle: сторона $SIDE ($D), прогон $NAME, сценарий $BS_REPLAY_SCENARIO"
exec bash src/lab/freefly_lv.sh "$@"
