#!/bin/bash
# Поймать SPOT GPU (Tesla T4) инстанс в зоне europe-west4-a.
#
# Boot-диск проекта (dev-workspace-1317) ЗОНАЛЕН и лежит в той зоне, где инстанс
# работал последним. Прицепить зональный диск к инстансу в ЧУЖОЙ зоне нельзя —
# поэтому, чтобы поймать спот в другой зоне, диск туда переносится снапшотом:
#   снапшот свежего диска → зональный диск в целевой зоне → spot-инстанс на нём.
#
# Три зоны — три скрипта (10–12_spot_gpu_a/b/c.sh), отличаются только TARGET_ZONE.
# Стратегия «ловли»: гоняем по очереди a → b → c, пока зона не отдаст T4.
#
# Тогглы (env):
#   MACHINE_TYPE=n1-standard-4 ./10_spot_gpu_a.sh  # меньше host'ов → реже EXHAUSTED
#   SPOT=0 ./10_spot_gpu_a.sh                       # on-demand вместо spot (дороже, стабильнее)
#   REFRESH=1 ./10_spot_gpu_a.sh                    # перезалить устаревший диск целевой зоны из свежего снапшота
#
# ВНИМАНИЕ: скрипт НЕ удаляет старые зональные диски сам (кроме REFRESH=1).
# Канон — диск инстанса, работавшего последним; диски в других зонах могут быть
# устаревшими (за них идёт плата за storage). Чистить вручную:
#   gcloud compute disks delete <name> --zone=<zone> --project=drone-13-17-workspace-2026

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"
MACHINE_TYPE="${MACHINE_TYPE:-n1-standard-8}"
SPOT="${SPOT:-1}"
REFRESH="${REFRESH:-0}"

TARGET_ZONE="europe-west4-a"     # ← единственное, чем отличаются 10–12_spot_gpu_{a,b,c}.sh
FALLBACK_ZONE="europe-west4-a"   # где исторически лежит исходный диск (для холодного старта)
SNAPSHOT="${INSTANCE_NAME}-snap" # рабочий снапшот переноса (пересоздаётся при каждом переезде)

if [ "$SPOT" = "1" ]; then
    PROV_ARGS=(--provisioning-model=SPOT --instance-termination-action=STOP)
    echo "💸 Режим SPOT (вытесняемый): дешевле и чаще доступен, termination=STOP."
else
    PROV_ARGS=()
    echo "💰 Режим on-demand (не вытесняемый, дороже)."
fi

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

# ── 1. Где сейчас инстанс? (агрегированный список по всем зонам) ──────────────
INST_ZONE=$(g compute instances list --filter="name=$INSTANCE_NAME" \
    --format="value(zone.basename())" | head -n1)

if [ "$INST_ZONE" = "$TARGET_ZONE" ]; then
    STATUS=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        --format="value(status)")
    if [ "$STATUS" = "RUNNING" ]; then
        echo "✅ Инстанс уже работает в $TARGET_ZONE — ловить нечего."
        exit 0
    fi
    echo "▶️  Инстанс есть в $TARGET_ZONE (статус $STATUS) — пробуем start (спот возвращается из preemption)..."
    set +e
    OUT=$(g compute instances start "$INSTANCE_NAME" --zone="$TARGET_ZONE" 2>&1); RC=$?
    set -e
    echo "$OUT"
    if [ $RC -eq 0 ]; then
        echo "✅ Поймали: $INSTANCE_NAME запущен в $TARGET_ZONE."
        echo "   SSH:  ZONE=$TARGET_ZONE ./02_power_manager.sh ssh"
        exit 0
    elif echo "$OUT" | grep -qiE "EXHAUSTED|ZONE_RESOURCE"; then
        echo "🛑 В $TARGET_ZONE сейчас нет свободных T4. Пробуй соседнюю зону (10–12_spot_gpu_*.sh)."
        exit 1
    else
        echo "❌ start не удался (код $RC), см. вывод выше."
        exit 1
    fi
fi

# ── 2. Источник состояния для снапшота (самый свежий диск) ───────────────────
# Инстанс в ДРУГОЙ зоне → его boot-диск самый свежий. Иначе — любой диск с этим
# именем (предпочитая FALLBACK_ZONE).
if [ -n "$INST_ZONE" ]; then
    SRC_ZONE="$INST_ZONE"
    SRC_DISK=$(g compute instances describe "$INSTANCE_NAME" --zone="$INST_ZONE" \
        --format="value(disks[0].source.basename())")
    echo "ℹ️  Инстанс найден в $INST_ZONE — источник состояния: диск '$SRC_DISK' там."
else
    SRC_ZONE=$(g compute disks list --filter="name=$INSTANCE_NAME" \
        --format="value(zone.basename())" | grep -x "$FALLBACK_ZONE" || true)
    [ -z "$SRC_ZONE" ] && SRC_ZONE=$(g compute disks list --filter="name=$INSTANCE_NAME" \
        --format="value(zone.basename())" | head -n1)
    SRC_DISK="$INSTANCE_NAME"
    if [ -z "$SRC_ZONE" ]; then
        echo "❌ Нет ни инстанса, ни boot-диска '$INSTANCE_NAME' ни в одной зоне — грузиться не с чего."
        echo "   Сначала создай рабочую машину: ./01_create_workspace.sh"
        exit 1
    fi
    echo "ℹ️  Инстанса нет; источник состояния: диск '$SRC_DISK' в $SRC_ZONE."
fi

# ── 3. Обеспечить boot-диск в ЦЕЛЕВОЙ зоне ───────────────────────────────────
TARGET_DISK_EXISTS=0
if g compute disks describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" >/dev/null 2>&1; then
    TARGET_DISK_EXISTS=1
fi

if [ "$SRC_ZONE" = "$TARGET_ZONE" ]; then
    echo "✅ Свежий диск уже в целевой зоне $TARGET_ZONE — снапшот не нужен."
elif [ "$TARGET_DISK_EXISTS" = "1" ] && [ "$REFRESH" != "1" ]; then
    echo "⚠️  В $TARGET_ZONE уже есть диск '$INSTANCE_NAME', но самый свежий — в $SRC_ZONE."
    echo "   Диск в $TARGET_ZONE может быть УСТАРЕВШИМ. Варианты:"
    echo "     • перезалить из свежего снапшота:  REFRESH=1 $0"
    echo "     • ловить в зоне-источнике $SRC_ZONE."
    exit 1
else
    echo "📸 Снимаем снапшот '$SNAPSHOT' с диска '$SRC_DISK' ($SRC_ZONE)..."
    g compute snapshots delete "$SNAPSHOT" --quiet >/dev/null 2>&1 || true
    g compute snapshots create "$SNAPSHOT" \
        --source-disk="$SRC_DISK" --source-disk-zone="$SRC_ZONE"
    if [ "$TARGET_DISK_EXISTS" = "1" ]; then
        echo "🗑️  Удаляем устаревший диск '$INSTANCE_NAME' в $TARGET_ZONE (REFRESH=1)..."
        g compute disks delete "$INSTANCE_NAME" --zone="$TARGET_ZONE" --quiet
    fi
    echo "💽 Создаём диск '$INSTANCE_NAME' в $TARGET_ZONE из снапшота..."
    g compute disks create "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        --source-snapshot="$SNAPSHOT" --type=pd-balanced
fi

# ── 4. Освободить имя инстанса (если он висит в другой зоне) ──────────────────
if [ -n "$INST_ZONE" ] && [ "$INST_ZONE" != "$TARGET_ZONE" ]; then
    echo "🧹 Удаляем инстанс в $INST_ZONE (boot-диск сохраняется), чтобы освободить имя..."
    g compute instances stop "$INSTANCE_NAME" --zone="$INST_ZONE" >/dev/null 2>&1 || true
    g compute instances delete "$INSTANCE_NAME" --zone="$INST_ZONE" --keep-disks=boot --quiet
fi

# ── 5. Создать spot GPU инстанс в целевой зоне ───────────────────────────────
echo "🚀 Создаём $INSTANCE_NAME ($MACHINE_TYPE + T4) в $TARGET_ZONE..."
set +e
OUT=$(g compute instances create "$INSTANCE_NAME" \
    --zone="$TARGET_ZONE" \
    --machine-type="$MACHINE_TYPE" \
    --accelerator=type=nvidia-tesla-t4,count=1 \
    --maintenance-policy=TERMINATE \
    "${PROV_ARGS[@]}" \
    --disk=name="$INSTANCE_NAME",boot=yes,mode=rw,auto-delete=yes 2>&1); RC=$?
set -e
echo "$OUT"

if [ $RC -eq 0 ]; then
    echo ""
    echo "✅ Поймали T4 в $TARGET_ZONE! Инстанс $INSTANCE_NAME запущен."
    echo "   SSH:           ZONE=$TARGET_ZONE ./02_power_manager.sh ssh"
    echo "   Статус/деньги: ZONE=$TARGET_ZONE ./04_check_money_leak.sh"
elif echo "$OUT" | grep -qiE "EXHAUSTED|ZONE_RESOURCE"; then
    echo ""
    echo "🛑 В $TARGET_ZONE нет свободных T4 (RESOURCE_POOL_EXHAUSTED)."
    echo "   Диск '$INSTANCE_NAME' в $TARGET_ZONE готов — повтори позже эту зону"
    echo "   или попробуй соседнюю: 10–12_spot_gpu_*.sh."
    exit 1
else
    echo "❌ Создание не удалось (код $RC), см. вывод выше."
    exit 1
fi
