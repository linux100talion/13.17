#!/bin/bash
# Добавить GPU (T4) к существующему CPU-only инстансу.
# Инстанс останавливается, апгрейдится, запускается.
# После старта — вручную поставить драйвер + nvidia-container-toolkit (см. ниже).
#
# Использование:
#   ./08_add_gpu.sh
#   ZONE=europe-west4-b ./08_add_gpu.sh   # если T4 нет в дефолтной зоне

set -e

PROJECT="drone-13-17-workspace-2026"
ZONE="${ZONE:-europe-west4-a}"
INSTANCE_NAME="dev-workspace-1317"

echo "⏳ Шаг 1: Останавливаем $INSTANCE_NAME (нужно для смены железа)..."
OUT=$(gcloud compute instances stop $INSTANCE_NAME --zone=$ZONE --project=$PROJECT 2>&1)
RC=$?
echo "$OUT"
if [ $RC -ne 0 ]; then
    echo "❌ Не удалось остановить инстанс (код $RC)."
    exit 1
fi

echo "⚙️  Шаг 2: Добавляем ускоритель T4..."
OUT=$(gcloud compute instances set-accelerator $INSTANCE_NAME \
    --accelerator=type=nvidia-tesla-t4,count=1 \
    --zone=$ZONE --project=$PROJECT 2>&1)
RC=$?
echo "$OUT"
if [ $RC -ne 0 ]; then
    echo "❌ set-accelerator не удался (код $RC)."
    exit 1
fi

echo "⚙️  Шаг 3: Политика обслуживания → TERMINATE (обязательно для GPU)..."
OUT=$(gcloud compute instances set-scheduling $INSTANCE_NAME \
    --maintenance-policy=TERMINATE \
    --zone=$ZONE --project=$PROJECT 2>&1)
RC=$?
echo "$OUT"
if [ $RC -ne 0 ]; then
    echo "❌ set-scheduling не удался (код $RC)."
    exit 1
fi

echo "🔌 Шаг 4: Запускаем инстанс с GPU..."
OUT=$(gcloud compute instances start $INSTANCE_NAME --zone=$ZONE --project=$PROJECT 2>&1)
RC=$?
echo "$OUT"

if [ $RC -eq 0 ]; then
    echo ""
    echo "✅ $INSTANCE_NAME запущен с T4."
    echo ""
    echo "─── Следующий шаг: поставить NVIDIA-драйвер + nvidia-container-toolkit ───"
    echo "    (нужно один раз; без этого 'runtime: nvidia' в docker-compose не работает)"
    echo ""
    echo "    gcloud compute ssh $INSTANCE_NAME --zone=$ZONE --project=$PROJECT"
    echo ""
    echo "    Внутри инстанса:"
    echo "      # 1. Драйвер"
    echo "      sudo apt-get install -y nvidia-driver-535"
    echo "      sudo reboot"
    echo "      # (после ребута — переподключиться)"
    echo "      nvidia-smi   # проверка"
    echo ""
    echo "      # 2. nvidia-container-toolkit"
    echo "      curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \\"
    echo "        | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-ctk-keyring.gpg"
    echo "      curl -sL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \\"
    echo "        | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-ctk-keyring.gpg] https://#g' \\"
    echo "        | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list"
    echo "      sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit"
    echo "      sudo nvidia-ctk runtime configure --runtime=docker"
    echo "      sudo systemctl restart docker"
    echo "      docker run --rm --gpus all nvidia/cuda:12.2.2-base-ubuntu22.04 nvidia-smi  # проверка"
elif echo "$OUT" | grep -q "RESOURCE_POOL_EXHAUSTED"; then
    echo ""
    echo "🛑 В зоне $ZONE сейчас нет свободных T4 (RESOURCE_POOL_EXHAUSTED)."
    echo "   Инстанс остановлен, GPU добавлен в конфиг, но не запустился."
    echo "   Варианты:"
    echo "     • подождать и запустить вручную: ./02_power_manager.sh start"
    echo "     • зону поменять нельзя у существующего инстанса"
    echo "       (нужен снапшот диска + пересоздание — отдельная история)"
    exit 1
else
    echo "❌ Запуск не удался (код $RC). Подробности в выводе выше."
    exit 1
fi
