#!/usr/bin/env bash
# cmd/dpv_board/dpv_board.sh — A/B DpVins ПОД ОТКЛИК БОРТА на канале углов (BS_ATT_OUT=1).
# Стек = cmd/att_out att бит в бит; между сторонами один профиль dpvins/ (brake5_stop → board1).
#   bash cmd/dpv_board/dpv_board.sh <base|board> [gust|nogust]
# Маршрут — реплей, по умолчанию roll_big.json (BS_REPLAY_SCENARIO переопределяет). README.txt рядом.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
SIDE="${1:-}"; WIND="${2:-gust}"
[ $# -ge 1 ] && shift; [ $# -ge 1 ] && shift
case "$SIDE" in base) D=dpvins/brake5_stop ;; board) D=dpvins/board1 ;;
    *) echo "ОШИБКА: сторона — base или board" >&2; exit 2 ;; esac
case "$WIND" in gust) W=world/wind2_gust5 ;; nogust) W=world/wind2 ;;
    *) echo "ОШИБКА: ветер — gust или nogust" >&2; exit 2 ;; esac
export BS_PILOT=replay
export BS_REPLAY_SCENARIO="${BS_REPLAY_SCENARIO:-/lab/joystick/scenarios/roll_big.json}"
export NAME="${NAME:-dpvb_${SIDE}_${WIND}_$(date +%Y%m%d_%H%M%S)}"
export PROFILES="dphold/baseline $D vinshold/baseline vins/scale25 loiter/guard wind/trim mission/att_out_replay legacy/baseline $W"
echo ">>> cmd/dpv_board: $SIDE ($D), ветер $WIND ($W), прогон $NAME, сценарий $BS_REPLAY_SCENARIO"
exec bash src/lab/freefly_lv.sh "$@"
