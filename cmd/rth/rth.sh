#!/usr/bin/env bash
# cmd/rth/rth.sh — ВОЗВРАТ ДОМОЙ: штатный RTL полётника к точке взлёта на EKF-от-VINS,
# без единой секунды GPS (LV=2). Ручки стека — БИТ В БИТ cmd/bl (текущий baseline):
# прогон меряет не стабилизатор, а промах возврата. Зачем и чем судить — README.txt рядом.
#
#   bash cmd/rth/rth.sh                     # живой пилот (TX12), возврат — `make rth`
#   BS_PILOT=replay BS_REPLAY_SCENARIO=<…>.json bash cmd/rth/rth.sh
#
# В полёте (второй терминал, из docker/sim): `make rth` = импульс /mission/rth →
# шаг Rth плана freefly шлёт RTL и отдаёт борт полётнику. Повторный `make rth`
# ОТМЕНЯЕТ возврат; SF не вверх (MANUAL) — забирает борт пилоту.
# Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

# loiter/rth = loiter/guard + параметры возврата в eeprom FCU (BS_FCU_PARAMS):
#   RTL_ALT_M 7 (было 15), RTL_LOIT_TIME 1000 (было 5000), LAND_ALT_LOW_M 3 (было 10)
P="dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 loiter/rth wind/trim"
if [ "${BS_PILOT:-}" = "replay" ]; then P="$P mission/replay"; else P="$P mission/baseline"; fi
export PROFILES="$P legacy/baseline world/wind2_gust5"
echo ">>> cmd/rth/rth.sh: возврат домой по RTL (профили cmd/bl бит в бит) [$PROFILES]"
echo ">>> в полёте: cd docker/sim && make rth   (повторно — отмена)"
exec bash src/lab/freefly_lv.sh "$@"
