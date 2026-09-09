#!/usr/bin/env bash
# cmd/yaw_vins/yaw_vins.sh — КУРС EKF ОТ VINS (EK3_SRC1_YAW=6) поверх возврата по треку.
# Ручки стека — БИТ В БИТ cmd/rth_track; отличие ровно одно: профиль mission/yaw_vins
# (BS_EKF_YAW_SRC=vins) вместо mission/baseline. Зачем и чем судить — README.txt рядом.
#
#   bash cmd/yaw_vins/yaw_vins.sh          # живой пилот (TX12)
#
# В полёте: повисеть, пока в левом нижнем углу FPV не загорится зелёное RTH READY —
# в этот же момент нода переключает курс (в статусе yawsrc=vins). Дальше как обычно:
# уйти ломаной, вернуться кнопкой SD. Чтобы измерить рывок отката — спровоцировать
# перерождение VINS в полёте (ros2 topic pub --once /restart std_msgs/msg/Bool "data: true").
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

P="dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 loiter/rth_track wind/trim"
if [ "${BS_PILOT:-}" = "replay" ]; then P="$P mission/yaw_vins_replay"; else P="$P mission/yaw_vins"; fi
export PROFILES="$P legacy/baseline world/wind2_gust5"
echo ">>> cmd/yaw_vins: курс EKF от VINS (EK3_SRC1_YAW=6); профили [$PROFILES]"
echo ">>> смотреть в статусе: yawsrc=, brd= (должен СТОЯТЬ), brdn= (уход курса)"
exec bash src/lab/freefly_lv.sh "$@"
