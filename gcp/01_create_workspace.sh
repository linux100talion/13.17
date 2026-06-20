#!/bin/bash
# Скрипт для первичного создания воркспейса

PROJECT="drone-13-17-workspace-2026"
ZONE="${ZONE:-europe-west4-a}"
INSTANCE_NAME="dev-workspace-1317"
DISK_SIZE="${1:-120}"   # размер загрузочного диска в GB (по умолчанию 120)

echo "🚀 Создаем инстанс $INSTANCE_NAME в зоне $ZONE (диск ${DISK_SIZE}GB)..."

# Ловим вывод и КОД ВОЗВРАТА gcloud — иначе печатали бы "✅" даже при провале.
OUT=$(gcloud compute instances create $INSTANCE_NAME \
    --project=$PROJECT \
    --zone=$ZONE \
    --machine-type=n1-standard-8 \
    --maintenance-policy=TERMINATE \
    --accelerator=type=nvidia-tesla-t4,count=1 \
    --image-family=ubuntu-2404-lts-amd64 \
    --image-project=ubuntu-os-cloud \
    --boot-disk-size=${DISK_SIZE}GB \
    --boot-disk-type=pd-balanced 2>&1)
RC=$?
echo "$OUT"

if [ $RC -eq 0 ]; then
    echo "✅ Готово! Подключайся командой: gcloud compute ssh $INSTANCE_NAME --zone=$ZONE"
elif echo "$OUT" | grep -q "RESOURCE_POOL_EXHAUSTED"; then
    echo "🛑 В зоне $ZONE сейчас нет свободных T4 (RESOURCE_POOL_EXHAUSTED)."
    echo "   Инстанс НЕ создан. Повтори позже или смени ZONE во всех скриптах"
    echo "   (соседние: us-central1-b/c/f; другие: us-east1-*, us-west1-*)."
    exit 1
else
    echo "❌ Создание не удалось (код $RC). Подробности в выводе выше."
    exit 1
fi
