#!/bin/bash
# Полное удаление инстанса (ВНИМАНИЕ: удаляется и загрузочный диск!)

PROJECT="drone-13-17-workspace-2026"
ZONE="${ZONE:-europe-west4-a}"
INSTANCE_NAME="dev-workspace-1317"

echo "⚠️ Ты собираешься УДАЛИТЬ инстанс $INSTANCE_NAME в зоне $ZONE."
echo "   Загрузочный диск будет СОХРАНЕН (--keep-disks=boot)."
read -p "Точно удалить? Введи 'yes' для подтверждения: " CONFIRM

if [ "$CONFIRM" != "yes" ]; then
    echo "❌ Отменено. Инстанс не тронут."
    exit 1
fi

echo "🗑️ Удаляем инстанс $INSTANCE_NAME..."
gcloud compute instances delete $INSTANCE_NAME \
    --project=$PROJECT \
    --zone=$ZONE \
    --keep-disks=boot \
    --quiet

echo "✅ Готово! Инстанс удален, загрузочный диск сохранен."
echo "   Биллинг за CPU/GPU остановлен, но за хранение диска ты продолжаешь платить."
