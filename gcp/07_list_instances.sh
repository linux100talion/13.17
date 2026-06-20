#!/bin/bash
# Показать все инстансы в проекте: что есть, где, какого типа и в каком статусе.
# Тонкая обёртка над `gcloud compute instances list`. Только чтение.

echo "📋 Инстансы в проекте $(gcloud config get-value project 2>/dev/null):"
echo

gcloud compute instances list \
    --format="table(
        name,
        zone.basename():label=ZONE,
        machineType.basename():label=TYPE,
        status,
        scheduling.preemptible.yesno(yes='SPOT',no='-'):label=SPOT,
        networkInterfaces[0].accessConfigs[0].natIP:label=EXTERNAL_IP
    )"

# Пустой список — это нормальный результат (инстансов нет), а не ошибка.
echo
echo "ℹ️ Пусто? Создать рабочую машину: ./01_create_workspace.sh [DISK_GB]"
