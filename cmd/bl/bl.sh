#!/usr/bin/env bash
# cmd/bl/bl.sh — BASELINE с 2026-09-06 (бывш. cmd/11, история — cmd/history/wind/): A/B-полёт: общий ветровой трим ярусов 0/1 (WindTrim, wind/trim.txt) поверх cmd/10
# (dphold/baseline + dpvins/brake5_stop + vins/scale25 + loiter/guard), ветер 2 м/с с порывами
# 5 м/с каждые 20 с (world/wind2_gust5). Два плеча одним скриптом:
#   bash cmd/bl/bl.sh            # B: WindTrim ВКЛ (wind/trim.txt)
#   WT=0 bash cmd/bl/bl.sh       # A: старое — свой трим у каждого яруса + посев (wind/baseline.txt)
# Зачем и что меняет — README.txt рядом.
#
# С 2026-09-07 этот скрипт ДЕРЖИТ ТОЛЬКО СПИСОК профилей (PROFILES): ни eval, ни export
# BS_*/WIND_*. Список собирает freefly_lv.sh на хосте (мета, eeprom, ветер compose) и
# bootstrap_arch2.sh в контейнере (env ноды) — один и тот же load.py, строго по схеме
# BootstrapConfig: дубль/незнакомый/отсутствующий ключ = ошибка. Переопределить ручку
# через env нельзя по построению — только файл-кандидат + копия этого скрипта в
# cmd/<имя>/ (см. cmd/README.txt, src/control/profiles/README.md). Реплей пульта:
#   BS_PILOT=replay BS_REPLAY_SCENARIO=<…>.json bash cmd/bl/bl.sh   (профиль mission/replay;
#   сценарий — аргумент прогона, едет в контейнер env'ом)
# Доп. аргументы → freefly_lv.sh.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

WT="${WT:-1}"
P="dphold/baseline dpvins/brake5_stop vinshold/baseline vins/scale25 loiter/guard"
if [ "$WT" = "0" ]; then P="$P wind/baseline"; else P="$P wind/trim"; fi
if [ "${BS_PILOT:-}" = "replay" ]; then P="$P mission/replay"; else P="$P mission/baseline"; fi
export PROFILES="$P legacy/baseline world/wind2_gust5"
echo ">>> cmd/bl/bl.sh: плечо $([ "$WT" = "0" ] && echo 'A (wind/baseline: свой трим + посев)' || echo 'B (wind/trim: WindTrim)'); профили [$PROFILES]"
exec bash src/lab/freefly_lv.sh "$@"
