#!/bin/bash
# Запустить CPU-only бокс dev-workspace-1317-cpu (без GPU) в europe-west4-a.
#
# Отдельный «верстак» под GPU-less прогон gazebo→SITL→VINS (ветка nn2_c3_cpu),
# поднят пока T4 в дефиците. Это НЕ GPU-инстанс dev-workspace-1317 — у него своё
# имя, свой диск; GPU-инстанс и снапшоты он не трогает.
#
# ТОНКИЙ старт существующего инстанса (gcloud instances start). CPU-ёмкость
# дефицитом обычно не страдает. Создание — ../01_create_workspace.sh с тогглами
# (GPU=0 MACHINE_TYPE=c2d-standard-8 INSTANCE_NAME=dev-workspace-1317-cpu).

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317-cpu"
ZONE="${ZONE:-europe-west4-a}"

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

if ! g compute instances describe "$INSTANCE_NAME" --zone="$ZONE" >/dev/null 2>&1; then
    echo "ℹ️  Инстанса '$INSTANCE_NAME' нет в $ZONE."
    echo "   Создай: GPU=0 MACHINE_TYPE=c2d-standard-8 INSTANCE_NAME=$INSTANCE_NAME \\"
    echo "           ../01_create_workspace.sh"
    exit 1
fi

STATUS=$(g compute instances describe "$INSTANCE_NAME" --zone="$ZONE" --format="value(status)")
if [ "$STATUS" = "RUNNING" ]; then
    echo "✅ $INSTANCE_NAME в $ZONE уже работает — ничего не делаем."
    echo "   SSH: ./ssh.sh"
    exit 0
fi

echo "🔌 Запускаем $INSTANCE_NAME в $ZONE (статус $STATUS)..."
set +e
OUT=$(g compute instances start "$INSTANCE_NAME" --zone="$ZONE" 2>&1); RC=$?
set -e
echo "$OUT"
if [ $RC -eq 0 ]; then
    echo "✅ Запущен. Счётчик биллинга пошёл (CPU-инстанс тарифицируется, пока RUNNING)."
    echo "   SSH: ./ssh.sh   •   Погасить: ./stop.sh"
else
    echo "❌ Запуск не удался (код $RC), см. вывод выше."
    exit 1
fi
