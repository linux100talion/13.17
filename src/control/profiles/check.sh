#!/usr/bin/env bash
# check.sh — расхождения профилей с env прогона: какие KEY заданы в профилях иначе,
# чем зафиксировано в <RUN>.env (freefly_lv.sh пишет его на каждый прогон).
# Ключи, которых в env прогона нет (слой «нода»), не сравниваются — печатаются
# отдельно счётчиком.  Использование:
#   bash src/control/profiles/check.sh docker/sim/output/joystick/<RUN>/<RUN>.env [профиль...]
# Без списка профилей — baseline.txt активного стека (dphold, dpvins, vins, loiter).
set -euo pipefail
ENVF=${1:?путь к <RUN>.env}; shift || true
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# дефолт — АКТИВНЫЙ стек; vinshold/ (откат, BS_VINS_STAB=vinshold) — только явным аргументом
PROFILES=("$@"); [ ${#PROFILES[@]} -eq 0 ] && PROFILES=("$HERE"/{dphold,dpvins,vins,loiter}/baseline.txt)
diff_n=0; skip_n=0
for p in "${PROFILES[@]}"; do
    while IFS= read -r line; do
        [[ "$line" =~ ^([A-Z][A-Z0-9_]*)=(.*)$ ]] || continue
        k=${BASH_REMATCH[1]}; v=${BASH_REMATCH[2]}
        e=$(grep -E "^$k=" "$ENVF" | tail -1 | cut -d= -f2- || true)
        if [ -z "$e" ]; then skip_n=$((skip_n+1)); continue; fi
        if [ "$e" != "$v" ]; then
            echo "≠ $k: профиль=$v  env=$e   ($(basename "$(dirname "$p")")/$(basename "$p"))"
            diff_n=$((diff_n+1))
        fi
    done < "$p"
done
echo "расхождений: $diff_n; ключей только в профиле (слой «нода», в env прогона нет): $skip_n"
[ "$diff_n" -eq 0 ]
