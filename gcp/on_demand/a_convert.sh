#!/bin/bash
# Конвертировать инстанс dev-workspace-1317 в зоне europe-west4-a из SPOT в
# on-demand (STANDARD) БЕЗ пересоздания — `gcloud instances set-scheduling`.
#
# В отличие от ../08_add_gpu.sh (пересоздание: delete + create), это конвертация
# IN-PLACE: инстанс и его boot-диск остаются на месте, меняется только
# provisioning model. Provisioning model нельзя сменить через start — поэтому
# это разовая операция «перевести SPOT-инстанс в on-demand», дальше обычные
# a_start/a_stop/a_ssh.
#
# set-scheduling требует ОСТАНОВЛЕННОГО инстанса — если работает, скрипт его
# сначала гасит. provisioning-model=STANDARD + --no-preemptible снимают и
# Spot-специфичный instance-termination-action. Maintenance policy (TERMINATE,
# обязательна для GPU) не трогается.
#
# Три зоны — три скрипта (a/b/c_convert.sh), отличаются только TARGET_ZONE.

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"

TARGET_ZONE="europe-west4-a"     # ← единственное, чем отличаются a/b/c_convert.sh

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

# Есть ли инстанс в целевой зоне?
if ! g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        >/dev/null 2>&1; then
    INST_ZONE=$(g compute instances list --filter="name=$INSTANCE_NAME" \
        --format="value(zone.basename())" | head -n1)
    if [ -z "$INST_ZONE" ]; then
        echo "ℹ️  Инстанса '$INSTANCE_NAME' нет ни в одной зоне."
        echo "   Создать on-demand на существующем диске: ../08_add_gpu.sh."
    else
        echo "⚠️  Инстанс не в $TARGET_ZONE, а в $INST_ZONE."
        echo "   Конвертируй скриптом своей зоны (a/b/c_convert.sh)."
    fi
    exit 1
fi

# Уже on-demand?
MODEL=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --format="value(scheduling.provisioningModel)")
if [ "$MODEL" = "STANDARD" ]; then
    echo "✅ Инстанс в $TARGET_ZONE уже on-demand (STANDARD) — конвертировать нечего."
    exit 0
fi
echo "ℹ️  Текущий provisioning model: $MODEL → переводим в STANDARD (on-demand)."

# set-scheduling требует остановленного инстанса.
STATUS=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --format="value(status)")
if [ "$STATUS" != "TERMINATED" ]; then
    echo "⏸️  Инстанс $STATUS — останавливаем (set-scheduling требует остановки)..."
    g compute instances stop "$INSTANCE_NAME" --zone="$TARGET_ZONE"
fi

echo "🔄 Конвертируем $INSTANCE_NAME ($TARGET_ZONE) в on-demand..."
g compute instances set-scheduling "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --provisioning-model=STANDARD --no-preemptible

NEW_MODEL=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --format="value(scheduling.provisioningModel)")
echo "✅ Готово. Provisioning model теперь: $NEW_MODEL."
echo "   Поднять: ./a_start.sh"
