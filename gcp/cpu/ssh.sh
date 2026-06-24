#!/bin/bash
# SSH в CPU-only бокс dev-workspace-1317-cpu (europe-west4-a). Требует RUNNING.
#
# Без аргументов — заходит сразу в ~/13.17. С аргументами (напр.
# ./ssh.sh --command='docker ps') — пробрасывает их в gcloud compute ssh как есть.

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317-cpu"
ZONE="${ZONE:-europe-west4-a}"

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

if ! g compute instances describe "$INSTANCE_NAME" --zone="$ZONE" >/dev/null 2>&1; then
    echo "ℹ️  Инстанса '$INSTANCE_NAME' нет в $ZONE — подними: ./start.sh"
    exit 1
fi

STATUS=$(g compute instances describe "$INSTANCE_NAME" --zone="$ZONE" --format="value(status)")
if [ "$STATUS" != "RUNNING" ]; then
    echo "⚠️  $INSTANCE_NAME в $ZONE не запущен (статус $STATUS). Подними: ./start.sh"
    exit 1
fi

echo "💻 Подключаемся по SSH к $INSTANCE_NAME ($ZONE)..."
if [ "$#" -eq 0 ]; then
    # Без доп. аргументов — открываем интерактивную сессию сразу в ~/13.17.
    # -t форсит выделение TTY; cd до exec сохраняет каталог в логин-шелле.
    exec gcloud compute ssh "$INSTANCE_NAME" --zone="$ZONE" --project="$PROJECT" \
        -- -t 'cd ~/13.17 2>/dev/null; exec bash -l'
else
    # Есть аргументы (напр. --command=...) — пробрасываем как есть, без cd.
    exec gcloud compute ssh "$INSTANCE_NAME" --zone="$ZONE" --project="$PROJECT" "$@"
fi
