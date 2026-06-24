#!/bin/bash
# Остановить on-demand GPU (Tesla T4) инстанс в зоне europe-west4-c.
#
# Парный к c_start.sh: гасит инстанс в своей зоне (boot-диск остаётся там же,
# платишь только за storage). on-demand не вытесняется — stop это обычный
# способ «выключить на ночь». Поднять обратно — c_start.sh.
#
# Три зоны — три скрипта (a/b/c_stop.sh), отличаются только TARGET_ZONE.

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"

TARGET_ZONE="europe-west4-c"     # ← единственное, чем отличаются a/b/c_stop.sh

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

# Есть ли инстанс в целевой зоне?
if ! g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        >/dev/null 2>&1; then
    INST_ZONE=$(g compute instances list --filter="name=$INSTANCE_NAME" \
        --format="value(zone.basename())" | head -n1)
    if [ -z "$INST_ZONE" ]; then
        echo "ℹ️  Инстанса '$INSTANCE_NAME' нет ни в одной зоне — гасить нечего."
    else
        echo "⚠️  Инстанс не в $TARGET_ZONE, а в $INST_ZONE."
        echo "   Останови его скриптом своей зоны (a/b/c_stop.sh)"
        echo "   или: ZONE=$INST_ZONE ../02_power_manager.sh stop"
    fi
    exit 1
fi

STATUS=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --format="value(status)")
if [ "$STATUS" = "TERMINATED" ]; then
    echo "💤 Инстанс в $TARGET_ZONE уже остановлен (статус TERMINATED) — ничего не делаем."
    exit 0
fi

echo "🛑 Останавливаем $INSTANCE_NAME в $TARGET_ZONE (статус $STATUS)..."
if g compute instances stop "$INSTANCE_NAME" --zone="$TARGET_ZONE"; then
    echo "💤 Остановлен. Платишь только за хранение диска (он остаётся в $TARGET_ZONE)."
    echo "   Поднять обратно: ./c_start.sh"
else
    echo "❌ Остановка не удалась. Статус: ZONE=$TARGET_ZONE ../04_check_money_leak.sh"
    exit 1
fi
