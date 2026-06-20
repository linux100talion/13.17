#!/bin/bash
# Скрипт для первичного создания воркспейса

PROJECT="drone-13-17-workspace-2026"
ZONE="${ZONE:-europe-west4-a}"
INSTANCE_NAME="dev-workspace-1317"
DISK_SIZE="${1:-120}"   # размер загрузочного диска в GB (по умолчанию 120)
GPU="${GPU:-1}"         # 1 = с T4 (рантайм CUDA/Gazebo), 0 = CPU-only build-box

# GPU нужен только чтобы ИСПОЛНЯТЬ CUDA. Для проверки кода (compile/colcon/линт/
# сборка образов) GPU не нужен — `nvcc` собирает CUDA и без видеокарты. CPU-only
# вариант доступен сразу (нет дефицита T4) и не привязывает к арх GPU.
if [ "$GPU" = "1" ]; then
    GPU_ARGS=(--accelerator=type=nvidia-tesla-t4,count=1 --maintenance-policy=TERMINATE)
    echo "🚀 Создаём $INSTANCE_NAME в $ZONE (диск ${DISK_SIZE}GB) — режим GPU (T4)..."
else
    GPU_ARGS=()   # без --accelerator: обычная CPU-машина, без TERMINATE-политики
    echo "🚀 Создаём $INSTANCE_NAME в $ZONE (диск ${DISK_SIZE}GB) — режим CPU-only..."
fi

# Ловим вывод и КОД ВОЗВРАТА gcloud — иначе печатали бы "✅" даже при провале.
OUT=$(gcloud compute instances create $INSTANCE_NAME \
    --project=$PROJECT \
    --zone=$ZONE \
    --machine-type=n1-standard-8 \
    "${GPU_ARGS[@]}" \
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
    echo "   Инстанс НЕ создан. Варианты:"
    echo "     • другая зона:  ZONE=europe-west4-b ./01_create_workspace.sh"
    echo "     • без GPU:      GPU=0 ./01_create_workspace.sh  (CPU-only, доступен сразу)"
    exit 1
else
    echo "❌ Создание не удалось (код $RC). Подробности в выводе выше."
    exit 1
fi
