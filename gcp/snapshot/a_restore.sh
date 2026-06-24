#!/bin/bash
# Восстановить boot-диск dev-workspace-1317 в зоне europe-west4-a из снапшота.
#
# Создаёт диск '<instance>' в СВОЕЙ зоне из бэкап-снапшота (см. *_take.sh).
# Снапшоты глобальны → восстановить можно в любой зоне; зона-источник снапшота
# роли не играет.
#
# Какой снапшот: по умолчанию САМЫЙ СВЕЖИЙ снапшот этого диска (по времени
# создания). Переопределить конкретным:  SNAPSHOT=<name> ./a_restore.sh
#
# Существующий диск в целевой зоне молча НЕ затирается:
#   • занят инстансом      → отказ (сначала освободить, см. подсказку);
#   • свободен (отвязан)   → перезатереть только с REPLACE=1 (delete + create).
#
# Три зоны — три скрипта (a/b/c_restore.sh), отличаются только TARGET_ZONE.

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"

TARGET_ZONE="europe-west4-a"     # ← единственное, чем отличаются a/b/c_restore.sh
REPLACE="${REPLACE:-0}"

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

# ── 1. Выбрать снапшот ───────────────────────────────────────────────────────
if [ -n "$SNAPSHOT" ]; then
    if ! g compute snapshots describe "$SNAPSHOT" >/dev/null 2>&1; then
        echo "❌ Снапшот '$SNAPSHOT' не найден."
        exit 1
    fi
else
    SNAPSHOT=$(g compute snapshots list --filter="sourceDisk:$INSTANCE_NAME" \
        --sort-by=~creationTimestamp --format="value(name)" | head -n1)
    if [ -z "$SNAPSHOT" ]; then
        echo "❌ Нет ни одного снапшота диска '$INSTANCE_NAME' — восстанавливать не из чего."
        echo "   Снять снапшот: ./a_take.sh"
        exit 1
    fi
    echo "ℹ️  Самый свежий снапшот: '$SNAPSHOT' (переопределить — SNAPSHOT=…)."
fi

# ── 2. Обработать существующий диск в целевой зоне ───────────────────────────
if g compute disks describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" >/dev/null 2>&1; then
    USERS=$(g compute disks describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        --format="value(users.basename())")
    if [ -n "$USERS" ]; then
        echo "🛑 Диск '$INSTANCE_NAME' в $TARGET_ZONE привязан к инстансу ($USERS) — заменить нельзя."
        echo "   Освободи диск: ZONE=$TARGET_ZONE ../05_delete_instance.sh (отвяжет, boot сохранит),"
        echo "   потом перезалей: REPLACE=1 $0"
        exit 1
    fi
    if [ "$REPLACE" != "1" ]; then
        echo "⚠️  Диск '$INSTANCE_NAME' в $TARGET_ZONE уже есть (свободен)."
        echo "   Восстановление ПЕРЕЗАТРЁТ его. Подтверди: REPLACE=1 $0"
        exit 1
    fi
    echo "🗑️  Удаляем существующий диск '$INSTANCE_NAME' в $TARGET_ZONE (REPLACE=1)..."
    g compute disks delete "$INSTANCE_NAME" --zone="$TARGET_ZONE" --quiet
fi

# ── 3. Создать диск из снапшота ──────────────────────────────────────────────
echo "💽 Создаём диск '$INSTANCE_NAME' в $TARGET_ZONE из '$SNAPSHOT'..."
g compute disks create "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --source-snapshot="$SNAPSHOT" --type=pd-balanced

echo "✅ Готово. Диск восстановлен в $TARGET_ZONE."
echo "   Поднять инстанс на нём: ../spot/a_start.sh (создаст инстанс в этой зоне)."
