#!/bin/bash
# Показать снапшоты диска dev-workspace-1317 + где сейчас сами диски по зонам.
#
# Снапшоты ГЛОБАЛЬНЫ (не зональны) — поэтому скрипт ОДИН (не a/b/c): per-zone
# списки были бы идентичны. Зональны только сами диски — их сводка отдельным
# блоком (видно, в какой зоне живёт диск и занят ли инстансом).
# Только чтение.

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

echo "== Снапшоты диска '$INSTANCE_NAME' (глобальные) =="
g compute snapshots list --filter="sourceDisk:$INSTANCE_NAME" \
    --sort-by=~creationTimestamp \
    --format="table(name,diskSizeGb,storageLocations.list(),status,creationTimestamp)"

echo ""
echo "== Диски '$INSTANCE_NAME' по зонам =="
g compute disks list --filter="name=$INSTANCE_NAME" \
    --format="table(name,zone.basename(),sizeGb,status,users.basename())"
