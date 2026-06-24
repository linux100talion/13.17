#!/bin/bash
# Остановить CPU-only бокс dev-workspace-1317-cpu в europe-west4-a.
#
# Парный к start.sh: гасит инстанс (boot-диск остаётся в зоне, платишь только за
# storage). CPU-инстанс тарифицируется, пока RUNNING, — гаси, когда не нужен.

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317-cpu"
ZONE="${ZONE:-europe-west4-a}"

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

if ! g compute instances describe "$INSTANCE_NAME" --zone="$ZONE" >/dev/null 2>&1; then
    echo "ℹ️  Инстанса '$INSTANCE_NAME' нет в $ZONE — гасить нечего."
    exit 1
fi

STATUS=$(g compute instances describe "$INSTANCE_NAME" --zone="$ZONE" --format="value(status)")
if [ "$STATUS" = "TERMINATED" ]; then
    echo "💤 $INSTANCE_NAME в $ZONE уже остановлен — ничего не делаем."
    exit 0
fi

echo "🛑 Останавливаем $INSTANCE_NAME в $ZONE (статус $STATUS)..."
if g compute instances stop "$INSTANCE_NAME" --zone="$ZONE"; then
    echo "💤 Остановлен. Платишь только за хранение диска (он остаётся в $ZONE)."
    echo "   Поднять обратно: ./start.sh"
else
    echo "❌ Остановка не удалась. Статус: gcloud compute instances describe $INSTANCE_NAME --zone=$ZONE"
    exit 1
fi
