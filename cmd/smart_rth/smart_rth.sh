#!/usr/bin/env bash
# cmd/smart_rth/smart_rth.sh — ВОЗВРАТ ПО СЛЕДУ: SMART_RTL полётника разматывает
# пройденный путь по хлебным крошкам (EKF-от-VINS, без GPS, LV=2). Ручки стека —
# БИТ В БИТ cmd/bl; отличие от cmd/rth одно: профиль loiter/smart_rth (SRTL_ACCURACY 1)
# и триггер `make smart-rth`. Зачем и чем судить — README.txt рядом.
#
#   bash cmd/smart_rth/smart_rth.sh          # живой пилот (TX12)
#   BS_PILOT=replay BS_REPLAY_SCENARIO=<…>.json bash cmd/smart_rth/smart_rth.sh
#
# В полёте (второй терминал, из docker/sim): `make smart-rth` = импульс
# /mission/smart_rth → шаг Rth плана freefly шлёт SMART_RTL и отдаёт борт полётнику.
# Повторный вызов (любой из двух) ОТМЕНЯЕТ возврат; SF не вверх (MANUAL) — забирает
# борт пилоту. `make rth` рядом — тот же шаг, но прямая на home (cmd/rth).
# Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

# loiter/smart_rth = loiter/guard + параметры возврата в eeprom FCU (BS_FCU_PARAMS):
#   SRTL_ACCURACY 1 (было 2) — вдвое честнее след; RTL_ALT_M 7, RTL_LOIT_TIME 1000,
#   LAND_ALT_LOW_M 3 — хвост посадки как в cmd/rth
P="dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 loiter/smart_rth wind/trim"
if [ "${BS_PILOT:-}" = "replay" ]; then P="$P mission/replay"; else P="$P mission/baseline"; fi
export PROFILES="$P legacy/baseline world/wind2_gust5"
echo ">>> cmd/smart_rth: возврат ПО СЛЕДУ (SMART_RTL); профили [$PROFILES]"
echo ">>> в полёте: cd docker/sim && make smart-rth   (повторно — отмена)"
exec bash src/lab/freefly_lv.sh "$@"
