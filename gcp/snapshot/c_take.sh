#!/bin/bash
# Снять снапшот boot-диска dev-workspace-1317 в зоне europe-west4-c.
#
# ТОНКИЙ бэкап-снапшот: имя датированное —
#   <instance>-snap-YYYYMMDD  (напр. dev-workspace-1317-snap-20260624).
# Буквы зоны в имени НЕТ: инстанс/диск живёт в одной зоне за раз, снапшотим
# единственный существующий диск — коллизии имён нет. Снапшот глобальный, его
# зона роли не играет (take в зоне a спокойно restore-ится в b/c).
# Существующие снапшоты НЕ трогаются (это НЕ транзитный '<instance>-snap', который
# пересоздают ../spot/*_start.sh при переезде между зонами). Каждый запуск
# добавляет новый снимок текущего состояния диска зоны.
#
# Снапшот можно снять и на работающем инстансе. Storage location — региональный
# (= регион зоны диска): глобально доступен для create в любой зоне, дешевле
# мультирегиона. Снапшоты инкрементальные (дельта к предыдущему).
#
# Идемпотентно: если снапшот с таким именем за сегодня уже есть — выходим.
# Тег даты переопределяется:  TAG=manual ./c_take.sh
#
# Три зоны — три скрипта (a/b/c_take.sh), отличаются только TARGET_ZONE.
# Чистка старых снапшотов — вручную:
#   gcloud compute snapshots delete <name> --project=drone-13-17-workspace-2026

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"

TARGET_ZONE="europe-west4-c"     # ← единственное, чем отличаются a/b/c_take.sh
TAG="${TAG:-$(date +%Y%m%d)}"
REGION="${TARGET_ZONE%-*}"               # europe-west4-c → europe-west4
SNAPSHOT="${INSTANCE_NAME}-snap-${TAG}"  # …-snap-YYYYMMDD (без буквы зоны)

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

# Есть ли диск в целевой зоне?
if ! g compute disks describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        >/dev/null 2>&1; then
    echo "ℹ️  Диска '$INSTANCE_NAME' в $TARGET_ZONE нет — снимать нечего."
    DISK_ZONE=$(g compute disks list --filter="name=$INSTANCE_NAME" \
        --format="value(zone.basename())" | head -n1)
    [ -n "$DISK_ZONE" ] && echo "   Диск сейчас в $DISK_ZONE — снимай скриптом той зоны (a/b/c_take.sh)."
    exit 1
fi

# Снапшот за сегодня уже есть?
if g compute snapshots describe "$SNAPSHOT" >/dev/null 2>&1; then
    echo "✅ Снапшот '$SNAPSHOT' уже существует — ничего не делаем (идемпотентно)."
    exit 0
fi

echo "📸 Снимаем снапшот '$SNAPSHOT' с диска '$INSTANCE_NAME' ($TARGET_ZONE)..."
g compute snapshots create "$SNAPSHOT" \
    --source-disk="$INSTANCE_NAME" --source-disk-zone="$TARGET_ZONE" \
    --storage-location="$REGION"

echo "✅ Готово. Снапшоты этого диска:"
g compute snapshots list --filter="sourceDisk:$INSTANCE_NAME" \
    --format="table(name,diskSizeGb,storageLocations.list(),creationTimestamp)"
