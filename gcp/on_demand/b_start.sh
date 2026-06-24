#!/bin/bash
# Запустить on-demand GPU (Tesla T4) инстанс в зоне europe-west4-b.
#
# ТОНКИЙ старт существующего инстанса: только `gcloud instances start` в своей
# зоне. Без снапшотов / переезда между зонами / создания — это НЕ забота
# on_demand (переезд диска по зонам делает spot/, создание —
# ../01_create_workspace.sh или ../08_add_gpu.sh). on-demand не вытесняется,
# инстанс живёт постоянно — поэтому достаточно start.
#
# Если инстанса нет в этой зоне — скрипт подсказывает, но сам ничего не
# переносит и не создаёт.
#
# Три зоны — три скрипта (a/b/c_start.sh), отличаются только TARGET_ZONE.

set -e

PROJECT="drone-13-17-workspace-2026"
INSTANCE_NAME="dev-workspace-1317"

TARGET_ZONE="europe-west4-b"     # ← единственное, чем отличаются a/b/c_start.sh

# Сахар: проект подставляется в каждый вызов gcloud сам.
g() { gcloud "$@" --project="$PROJECT"; }

# Есть ли инстанс в целевой зоне?
if ! g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
        >/dev/null 2>&1; then
    INST_ZONE=$(g compute instances list --filter="name=$INSTANCE_NAME" \
        --format="value(zone.basename())" | head -n1)
    if [ -z "$INST_ZONE" ]; then
        echo "ℹ️  Инстанса '$INSTANCE_NAME' нет ни в одной зоне."
        echo "   Создай: ../01_create_workspace.sh (или ../08_add_gpu.sh под GPU)."
    else
        echo "⚠️  Инстанс не в $TARGET_ZONE, а в $INST_ZONE."
        echo "   Запусти скриптом своей зоны (a/b/c_start.sh)"
        echo "   или перенеси зону снапшотом через ../spot/."
    fi
    exit 1
fi

STATUS=$(g compute instances describe "$INSTANCE_NAME" --zone="$TARGET_ZONE" \
    --format="value(status)")
if [ "$STATUS" = "RUNNING" ]; then
    echo "✅ Инстанс в $TARGET_ZONE уже работает — ничего не делаем."
    exit 0
fi

echo "🔌 Запускаем $INSTANCE_NAME в $TARGET_ZONE (статус $STATUS)..."
set +e
OUT=$(g compute instances start "$INSTANCE_NAME" --zone="$TARGET_ZONE" 2>&1); RC=$?
set -e
echo "$OUT"
if [ $RC -eq 0 ]; then
    echo "✅ Запущен. Счётчик биллинга пошёл."
    echo "   SSH: ./b_ssh.sh"
elif echo "$OUT" | grep -qiE "EXHAUSTED|ZONE_RESOURCE"; then
    echo "🛑 В $TARGET_ZONE нет свободных T4 (RESOURCE_POOL_EXHAUSTED)."
    echo "   on-demand ёмкость тоже плавает — повтори позже или переезжай в"
    echo "   другую зону через ../spot/."
    exit 1
else
    echo "❌ Запуск не удался (код $RC), см. вывод выше."
    exit 1
fi
