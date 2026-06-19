#!/bin/bash
# Проверка статуса: не забыл ли я выключить сервер?

ZONE="us-central1-a"
INSTANCE_NAME="dev-workspace-137"

echo "🔍 Проверяем статус инстанса $INSTANCE_NAME..."
STATUS=$(gcloud compute instances describe $INSTANCE_NAME --zone=$ZONE --format="value(status)" 2>/dev/null)

if [ "$STATUS" == "RUNNING" ]; then
    echo "⚠️ ВНИМАНИЕ: Сервер РАБОТАЕТ! Счетчик биллинга крутится."
elif [ "$STATUS" == "TERMINATED" ]; then
    echo "🛡️ Все спокойно. Сервер остановлен. Деньги за CPU/GPU не списываются."
else
    echo "Текущий статус сервера: $STATUS (возможно, инстанс удален или в процессе запуска)"
fi
