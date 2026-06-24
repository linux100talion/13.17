#!/bin/bash
# Подключиться по SSH к SPOT GPU (Tesla T4) инстансу в зоне europe-west4-c.
#
# Парный к c_start.sh / c_stop.sh: коннектится к инстансу в СВОЕЙ зоне.
# Тонкая обёртка над `gcloud compute ssh` (то же, что ../02_power_manager.sh ssh,
# но с зоной, прибитой к europe-west4-c).
#
# Три зоны — три скрипта (a/b/c_ssh.sh), отличаются только TARGET_ZONE.
# Любые доп. аргументы пробрасываются в gcloud (например: ./c_ssh.sh --command='nvidia-smi').

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"

TARGET_ZONE="europe-west4-c"     # ← единственное, чем отличаются a/b/c_ssh.sh

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

# Есть ли инстанс в целевой зоне?
if ! g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        >/dev/null 2>&1; then
    INST_ZONE=$(g compute instances list --filter="name=$INSTANCE_NAME" \
        --format="value(zone.basename())" | head -n1)
    if [ -z "$INST_ZONE" ]; then
        echo "ℹ️  Инстанса '$INSTANCE_NAME' нет ни в одной зоне — поймай его: ./c_start.sh"
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
    echo "⚠️  Инстанс в $TARGET_ZONE не запущен (статус $STATUS). Подними: ./c_start.sh"
    exit 1
fi

echo "💻 Подключаемся по SSH к $INSTANCE_NAME ($TARGET_ZONE)..."
if [ "$#" -eq 0 ]; then
    # Без доп. аргументов — открываем интерактивную сессию сразу в ~/13.17.
    # -t форсит выделение TTY; cd до exec сохраняет каталог в логин-шелле.
    exec gcloud compute ssh "$INSTANCE_NAME" --zone="$TARGET_ZONE" --project="$PROJECT" \
        -- -t 'cd ~/13.17 2>/dev/null; exec bash -l'
else
    # Есть аргументы (напр. --command=...) — пробрасываем как есть, без cd.
    exec gcloud compute ssh "$INSTANCE_NAME" --zone="$TARGET_ZONE" --project="$PROJECT" "$@"
fi
