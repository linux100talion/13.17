#!/usr/bin/env bash
# cmd/rc_rev/rc_rev.sh — ПРОВЕРКА КОМПЕНСАЦИИ РЕВЕРСА КАНАЛОВ FCU в живом SITL (5780d48).
# Стек БИТ В БИТ = cmd/bl (плечо WindTrim); между сторонами меняется РОВНО ОДНО —
# реверс каналов полётника в eeprom SITL (loiter/guard → loiter/rc_rev_pitch|rc_rev_all,
# ключ BS_FCU_PARAMS). Маршрут — один сценарий реплея на все стороны. README.txt рядом.
#
#   bash cmd/rc_rev/rc_rev.sh none     # RC1..4_REVERSED 0 — эталон (как всегда в симе)
#   bash cmd/rc_rev/rc_rev.sh pitch    # RC2_REVERSED 1 — как на реальном борту
#   bash cmd/rc_rev/rc_rev.sh all      # RC1/2/4_REVERSED 1 — общность компенсации
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

SIDE="${1:-}"
[ $# -ge 1 ] && shift
case "$SIDE" in
    none)  LOITER=loiter/guard ;;
    pitch) LOITER=loiter/rc_rev_pitch ;;
    all)   LOITER=loiter/rc_rev_all ;;
    *) echo "ОШИБКА: сторона — none, pitch или all (дано '${SIDE:-пусто}')" >&2
       echo "  bash cmd/rc_rev/rc_rev.sh <none|pitch|all>" >&2; exit 2 ;;
esac

# маршрут ОДИН на все стороны — реплей, не живой пилот (разброс рук больше разницы)
export BS_PILOT="${BS_PILOT:-replay}"
export BS_REPLAY_SCENARIO="${BS_REPLAY_SCENARIO:-/lab/joystick/scenarios/old/rc_rev.json}"
export NAME="${NAME:-rcrev_${SIDE}_$(date +%Y%m%d_%H%M%S)}"

P="dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 $LOITER wind/trim"
if [ "$BS_PILOT" = "replay" ]; then P="$P mission/replay"; else P="$P mission/baseline"; fi
export PROFILES="$P legacy/baseline world/wind2_gust5"

echo ">>> cmd/rc_rev: сторона $SIDE ($LOITER), прогон $NAME"
echo ">>> профили [$PROFILES]"
[ "$BS_PILOT" = "replay" ] && echo ">>> РЕПЛЕЙ ПУЛЬТА, сценарий $BS_REPLAY_SCENARIO"
echo ">>> в логе ноды ждать: «реверс каналов FCU: RC1=… RC2=… — зеркалю: …»"
exec bash src/lab/freefly_lv.sh "$@"
