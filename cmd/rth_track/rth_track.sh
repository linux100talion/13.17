#!/usr/bin/env bash
# cmd/rth_track/rth_track.sh — ВОЗВРАТ ПО СВОЕМУ ТРЕКУ: GUIDED + поток уставок,
# курс по треку, дома — сразу мягкая посадка. Ручки стека — БИТ В БИТ cmd/bl;
# отличия: профиль loiter/rth_track (WPNAV_* под наш масштаб) и BS_RTH_MODE=guided
# из mission/baseline. Зачем и чем судить — README.txt рядом.
#
#   bash cmd/rth_track/rth_track.sh          # живой пилот (TX12)
#
# В полёте: дождаться ЗЕЛЁНОГО «RTH READY» в левом нижнем углу FPV (латч дома),
# уйти ЛОМАНОЙ 20–30 м, нажать SD. Борт сам размотает трек назад и сядет дома.
# Второе нажатие SD / SF вниз — отмена, борт возвращается пилоту на демпфере.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

P="dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 loiter/rth_track wind/trim"
if [ "${BS_PILOT:-}" = "replay" ]; then P="$P mission/replay"; else P="$P mission/baseline"; fi
export PROFILES="$P legacy/baseline world/wind2_gust5"
echo ">>> cmd/rth_track: возврат ПО СВОЕМУ ТРЕКУ (GUIDED); профили [$PROFILES]"
echo ">>> в полёте: дождаться зелёного RTH READY → уйти ломаной → SD"
exec bash src/lab/freefly_lv.sh "$@"
