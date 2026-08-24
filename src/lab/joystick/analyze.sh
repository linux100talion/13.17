#!/usr/bin/env bash
# analyze.sh — запустить joy_timeline.py внутри p1317_nav по bag прогона.
# Стек не трогает (только чтение bag) — дисциплина прогона не нарушается.
#
# С хоста:
#   bash src/lab/joystick/analyze.sh                 # свежий output/scene_bag,
#                                                    #   результаты → output/joystick/
#   RUN=lv1_joy_20260822_153000 bash src/lab/joystick/analyze.sh
#       # bag и результаты — в архиве прогона output/joystick/<RUN>/
#       # (архив делает freefly_lv.sh, KEEP_BAG=1)
#   BAG=/root/sim_ws/output/my_bag bash src/lab/joystick/analyze.sh  # явный bag
# Доп. аргументы уходят в joy_timeline.py:  ... analyze.sh --eps 0.05
set -euo pipefail
NAV="${NAV:-p1317_nav}"
RUN="${RUN:-}"
if [ -n "$RUN" ]; then
    BAG="${BAG:-/root/sim_ws/output/joystick/$RUN/bag}"
    OUT="/root/sim_ws/output/joystick/$RUN"
else
    BAG="${BAG:-/root/sim_ws/output/scene_bag}"
    OUT="/root/sim_ws/output/joystick"
fi
# Схема SF-мастер: от неё зависят подписи CH6 в отчёте, а эвристика «ось 6
# живая» врёт, когда SF уже сидит в миксере, но полёт легаси (прогон 213830).
# Истину берём из меты прогона (.env пишет freefly_lv.sh); явный --sf-master
# в доп. аргументах перекрывает (argparse: последнее вхождение выигрывает).
SF_ARG=""
if [ -n "$RUN" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    ENVF="$SCRIPT_DIR/../../../docker/sim/output/joystick/$RUN/$RUN.env"
    if [ -f "$ENVF" ]; then
        if grep -q '^BS_SF_MASTER=1' "$ENVF"; then SF_ARG="--sf-master 1"
        else SF_ARG="--sf-master 0"; fi
    fi
fi
docker exec "$NAV" bash -lc "
    source /opt/ros/humble/setup.bash
    source /opt/overlay/install/setup.bash 2>/dev/null || true
    source /root/sim_ws/install/setup.bash 2>/dev/null || true
    python3 /lab/joystick/joy_timeline.py '$BAG' --out '$OUT' $SF_ARG $*"
echo "→ на хосте: docker/sim/${OUT#/root/sim_ws/}/{report.txt,scenario_draft.json,raw.jsonl}"
