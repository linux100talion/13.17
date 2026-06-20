#!/bin/bash
# Апгрейд CPU-only build-box до GPU-машины (T4) БЕЗ потери данных.
#
# Прикрепить GPU к существующему инстансу gcloud-командой НЕЛЬЗЯ (нет такого
# subcommand; REST `instances.update` с guestAccelerators отдаёт 503). Поэтому
# единственный надёжный путь — ПЕРЕСОЗДАНИЕ инстанса с сохранением boot-диска:
#   стоп → delete --keep-disks=boot → create с тем же диском + --accelerator.
# Загрузочный диск (вся работа, образы, репа) переживает пересоздание.
#
# Идемпотентно: если инстанса уже нет (прошлый прогон упал на create после
# удаления), скрипт просто пересоздаёт его из сохранённого диска.
#
# Использование:
#   ./08_add_gpu.sh
#   ZONE=europe-west4-b ./08_add_gpu.sh        # если T4 нет в дефолтной зоне
#   MACHINE_TYPE=n1-standard-4 ./08_add_gpu.sh # меньше host'ов → реже EXHAUSTED
#   SPOT=1 ./08_add_gpu.sh                      # Spot-пул: чаще доступен и дешевле
#   SPOT=1 MACHINE_TYPE=n1-standard-4 ./08_add_gpu.sh  # комбинируется
#
# При RESOURCE_POOL_EXHAUSTED тогглы НЕ переносят диск (зона та же) — пробуют
# другой тип машины и/или Spot-пул в той же europe-west4-a, диск остаётся цел.

set -e

PROJECT="drone-13-17-workspace-2026"
ZONE="${ZONE:-europe-west4-a}"
INSTANCE_NAME="dev-workspace-1317"
MACHINE_TYPE="${MACHINE_TYPE:-n1-standard-8}"   # env-override: n1-standard-4 и т.п.
SPOT="${SPOT:-0}"                                # 1 = Spot (вытесняемый, дешевле)
BOOT_DISK="$INSTANCE_NAME"   # boot-диск назван так же, как инстанс

# Spot-инстанс: отдельный пул мощностей (часто доступен, когда on-demand пуст) +
# дешевле. Минус — Google может вытеснить в любой момент; ставим termination=STOP,
# чтобы при вытеснении инстанс ОСТАНОВИЛСЯ (boot-диск цел), а не удалился.
if [ "$SPOT" = "1" ]; then
    SPOT_ARGS=(--provisioning-model=SPOT --instance-termination-action=STOP)
    echo "💸 Режим Spot (вытесняемый): дешевле и чаще доступен, но без гарантий."
else
    SPOT_ARGS=()
fi

# ── Шаг 0: существует ли инстанс? ────────────────────────────────────────────
if gcloud compute instances describe "$INSTANCE_NAME" \
      --zone="$ZONE" --project="$PROJECT" >/dev/null 2>&1; then
    # На всякий случай уточняем реальное имя boot-диска (вдруг отличается).
    DISK=$(gcloud compute instances describe "$INSTANCE_NAME" \
        --zone="$ZONE" --project="$PROJECT" \
        --format="value(disks[0].source.basename())" 2>/dev/null)
    [ -n "$DISK" ] && BOOT_DISK="$DISK"

    echo "⏳ Шаг 1: Останавливаем $INSTANCE_NAME..."
    gcloud compute instances stop "$INSTANCE_NAME" \
        --zone="$ZONE" --project="$PROJECT"

    echo "🗑️  Шаг 2: Удаляем инстанс (boot-диск '$BOOT_DISK' СОХРАНЯЕТСЯ)..."
    gcloud compute instances delete "$INSTANCE_NAME" \
        --zone="$ZONE" --project="$PROJECT" \
        --keep-disks=boot --quiet
else
    echo "ℹ️  Инстанса $INSTANCE_NAME нет — пересоздаём из сохранённого диска '$BOOT_DISK'."
fi

# ── Шаг 3: пересоздаём инстанс с тем же диском + T4 ──────────────────────────
echo "🚀 Шаг 3: Создаём $INSTANCE_NAME ($MACHINE_TYPE + T4) на диске '$BOOT_DISK'..."
set +e
OUT=$(gcloud compute instances create "$INSTANCE_NAME" \
    --project="$PROJECT" \
    --zone="$ZONE" \
    --machine-type="$MACHINE_TYPE" \
    --accelerator=type=nvidia-tesla-t4,count=1 \
    --maintenance-policy=TERMINATE \
    "${SPOT_ARGS[@]}" \
    --disk=name="$BOOT_DISK",boot=yes,mode=rw,auto-delete=yes 2>&1)
RC=$?
set -e
echo "$OUT"

if [ $RC -eq 0 ]; then
    echo ""
    echo "✅ $INSTANCE_NAME пересоздан с T4 (данные на диске целы)."
    echo ""
    echo "─── Следующий шаг: поставить NVIDIA-драйвер + nvidia-container-toolkit ───"
    echo "    (если диск уже build-box без драйвера — нужно один раз)"
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
    echo "   Инстанс НЕ создан, но boot-диск '$BOOT_DISK' СОХРАНЁН."
    echo "   Варианты (диск остаётся в $ZONE, перенос не нужен):"
    echo "     • повторить позже:   ./08_add_gpu.sh"
    echo "     • Spot-пул:          SPOT=1 ./08_add_gpu.sh"
    echo "     • меньший тип:       MACHINE_TYPE=n1-standard-4 ./08_add_gpu.sh"
    echo "     • комбo:             SPOT=1 MACHINE_TYPE=n1-standard-4 ./08_add_gpu.sh"
    echo "     • другая зона — НЕЛЬЗЯ: диск привязан к зоне $ZONE"
    echo "       (нужен снапшот диска в другую зону — отдельная история)"
    exit 1
else
    echo "❌ Создание не удалось (код $RC). boot-диск '$BOOT_DISK' сохранён."
    echo "   Повторить: ./08_add_gpu.sh"
    exit 1
fi
