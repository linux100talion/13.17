#!/bin/bash
# Конвертировать инстанс dev-workspace-1317 в зоне europe-west4-c из on-demand
# (STANDARD) в SPOT БЕЗ пересоздания — `gcloud instances set-scheduling`.
#
# Обратное к ../on_demand/c_convert.sh. In-place: инстанс и его boot-диск
# остаются на месте, меняется только provisioning model. Provisioning model
# нельзя сменить через start — поэтому это разовая операция «перевести инстанс
# в SPOT», дальше обычные a_start/a_stop/a_ssh.
#
# set-scheduling требует ОСТАНОВЛЕННОГО инстанса — если работает, скрипт его
# сначала гасит. provisioning-model=SPOT + instance-termination-action=STOP:
# при вытеснении инстанс ОСТАНАВЛИВАЕТСЯ (boot-диск цел), а не удаляется
# (как и при создании в *_start.sh). Maintenance policy (TERMINATE, обязательна
# для GPU) не трогается.
#
# ⚠️ SPOT дешевле, но ВЫТЕСНЯЕМ — Google может остановить инстанс в любой момент.
#
# Три зоны — три скрипта (a/b/c_convert.sh), отличаются только TARGET_ZONE.

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"

TARGET_ZONE="europe-west4-c"     # ← единственное, чем отличаются a/b/c_convert.sh

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

# Есть ли инстанс в целевой зоне?
if ! g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        >/dev/null 2>&1; then
    INST_ZONE=$(g compute instances list --filter="name=$INSTANCE_NAME" \
        --format="value(zone.basename())" | head -n1)
    if [ -z "$INST_ZONE" ]; then
        echo "ℹ️  Инстанса '$INSTANCE_NAME' нет ни в одной зоне."
        echo "   Создать SPOT на существующем диске: SPOT=1 ../08_add_gpu.sh (или ./c_start.sh)."
    else
        echo "⚠️  Инстанс не в $TARGET_ZONE, а в $INST_ZONE."
        echo "   Конвертируй скриптом своей зоны (a/b/c_convert.sh)."
    fi
    exit 1
fi

# Уже SPOT?
MODEL=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --format="value(scheduling.provisioningModel)")
if [ "$MODEL" = "SPOT" ]; then
    echo "✅ Инстанс в $TARGET_ZONE уже SPOT — конвертировать нечего."
    exit 0
fi
echo "ℹ️  Текущий provisioning model: $MODEL → переводим в SPOT (вытесняемый)."

# set-scheduling требует остановленного инстанса.
STATUS=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --format="value(status)")
if [ "$STATUS" != "TERMINATED" ]; then
    echo "⏸️  Инстанс $STATUS — останавливаем (set-scheduling требует остановки)..."
    g compute instances stop "$INSTANCE_NAME" --zone="$TARGET_ZONE"
fi

echo "🔄 Конвертируем $INSTANCE_NAME ($TARGET_ZONE) в SPOT..."
g compute instances set-scheduling "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --provisioning-model=SPOT --instance-termination-action=STOP

NEW_MODEL=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --format="value(scheduling.provisioningModel)")
echo "✅ Готово. Provisioning model теперь: $NEW_MODEL."
echo "   Поднять: ./c_start.sh"
