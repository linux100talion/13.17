#!/bin/bash
# Подключиться по SSH к SPOT GPU (Tesla T4) инстансу в зоне europe-west4-b.
#
# Парный к b_start.sh / b_stop.sh: коннектится к инстансу в СВОЕЙ зоне.
# Тонкая обёртка над `gcloud compute ssh` (то же, что ../02_power_manager.sh ssh,
# но с зоной, прибитой к europe-west4-b).
#
# Три зоны — три скрипта (a/b/c_ssh.sh), отличаются только TARGET_ZONE.
# Любые доп. аргументы пробрасываются в gcloud (например: ./b_ssh.sh --command='nvidia-smi').

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"

TARGET_ZONE="europe-west4-b"     # ← единственное, чем отличаются a/b/c_ssh.sh

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

# Есть ли инстанс в целевой зоне?
if ! g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        >/dev/null 2>&1; then
    INST_ZONE=$(g compute instances list --filter="name=$INSTANCE_NAME" \
        --format="value(zone.basename())" | head -n1)
    if [ -z "$INST_ZONE" ]; then
        echo "ℹ️  Инстанса '$INSTANCE_NAME' нет ни в одной зоне — поймай его: ./b_start.sh"
    else
        echo "⚠️  Инстанс не в $TARGET_ZONE, а в $INST_ZONE."
        echo "   Подключайся скриптом своей зоны (a/b/c_ssh.sh)"
        echo "   или: ZONE=$INST_ZONE ../02_power_manager.sh ssh"
    fi
    exit 1
fi

STATUS=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --format="value(status)")
if [ "$STATUS" != "RUNNING" ]; then
    echo "⚠️  Инстанс в $TARGET_ZONE не запущен (статус $STATUS). Подними: ./b_start.sh"
    exit 1
fi

echo "💻 Подключаемся по SSH к $INSTANCE_NAME ($TARGET_ZONE)..."
exec g compute ssh "$INSTANCE_NAME" --zone="$TARGET_ZONE" "$@"
