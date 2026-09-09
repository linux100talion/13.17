#!/usr/bin/env bash
# cmd/yaw_ab/yaw_ab.sh — A/B «ЧЕЙ КУРС ДЕРЖИТ EKF»: компас против VINS, на двух точках
# спавна. Стек БИТ В БИТ = cmd/yaw_vins; между сторонами меняется РОВНО ОДИН ключ
# (mission/baseline → mission/yaw_vins, то есть BS_EKF_YAW_SRC=compass|vins), между
# точками — только SPAWN_POSE. Зачем и чем судить — README.txt рядом.
#
#   bash cmd/yaw_ab/yaw_ab.sh compass east
#   bash cmd/yaw_ab/yaw_ab.sh vins    east
#   bash cmd/yaw_ab/yaw_ab.sh compass diagonal
#   bash cmd/yaw_ab/yaw_ab.sh vins    diagonal
#
# Четыре прогона, ОДНА И ТА ЖЕ схема полёта (иначе сравнивать нечего — см. README).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

SIDE="${1:-}"
SPOT="${2:-east}"
# свои аргументы снимаем — дальше в freefly_lv.sh уезжает только ХВОСТ (его флаги)
[ $# -ge 1 ] && shift
[ $# -ge 1 ] && shift
case "$SIDE" in
    compass|vins) ;;
    *) echo "ОШИБКА: сторона — compass или vins (дано '${SIDE:-пусто}')" >&2
       echo "  bash cmd/yaw_ab/yaw_ab.sh <compass|vins> [east|diagonal]" >&2; exit 2 ;;
esac
case "$SPOT" in
    east)     ;;                       # штатный спавн в центре площадки, курс на восток
    diagonal) export SPAWN_POSE=diagonal ;;   # пресет docker/sim/output/spawn/diagonal, курс 117.7°
    *) echo "ОШИБКА: точка — east или diagonal (дано '$SPOT')" >&2; exit 2 ;;
esac

# Ручки стека — БИТ В БИТ cmd/yaw_vins; отличие сторон ровно одно, в mission-профиле.
P="dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 loiter/rth_track wind/trim"
if [ "${BS_PILOT:-}" = "replay" ]; then
    [ "$SIDE" = "vins" ] && P="$P mission/yaw_vins_replay" || P="$P mission/replay"
else
    [ "$SIDE" = "vins" ] && P="$P mission/yaw_vins" || P="$P mission/baseline"
fi
export PROFILES="$P legacy/baseline world/wind2_gust5"

echo ">>> cmd/yaw_ab: сторона $SIDE, точка спавна $SPOT"
echo ">>> профили [$PROFILES]"
[ "$SPOT" = "diagonal" ] && echo ">>> SPAWN_POSE=diagonal (курс 117.7° — Δyaw якоря будет ≈ +100°)"
echo ">>> схема полёта ОДНА для всех четырёх: висеть в круге до зелёного RTH READY,"
echo ">>>   уйти ломаной на 50-70 м с разворотами, вернуться кнопкой SD, дать сесть."
echo ">>> смотреть в статусе: yawsrc= (compass или vins), brd= (должен СТОЯТЬ), brdn="
exec bash src/lab/freefly_lv.sh "$@"
