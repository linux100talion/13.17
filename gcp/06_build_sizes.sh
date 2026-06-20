#!/bin/bash
# Сколько места реально съела сборка проекта на инстансе.
# Тонкая обёртка над `gcloud ... ssh --command "docker ..."`: показывает размеры
# докер-образов, build-кэша и общий расклад по диску. Запускать на RUNNING-инстансе.

ZONE="${ZONE:-europe-west4-a}"
INSTANCE_NAME="dev-workspace-1317"

echo "🔍 Проверяем, что инстанс $INSTANCE_NAME запущен..."
STATUS=$(gcloud compute instances describe $INSTANCE_NAME --zone=$ZONE --format="value(status)" 2>/dev/null)
if [ "$STATUS" != "RUNNING" ]; then
    echo "🛑 Инстанс не RUNNING (статус: ${STATUS:-неизвестен})."
    echo "   Подними его: ./02_power_manager.sh start"
    exit 1
fi

echo "📊 Считаем размеры на инстансе (docker + диск)..."
echo

# Всё считается УДАЛЁННО одной ssh-сессией. Образы проекта фильтруем по шаблону;
# полагаемся на готовые колонки `docker images` (без хрупких табов в awk/sort).
gcloud compute ssh $INSTANCE_NAME --zone=$ZONE --command='
PAT="p1317|sim[-_]nav|sim[-_]simulator|opencv|cuda|ardupilot|ros-|/ros|nav|simulator|osrf|dustynv"

echo "=== 1. Сводка docker (images / containers / build cache) ==="
docker system df
echo

echo "=== 2. Образы проекта (SIZE — последняя колонка) ==="
docker images | head -n 1
docker images | tail -n +2 | grep -Ei "$PAT"
echo

echo "=== 3. Build cache подробно (чистится: docker builder prune -f) ==="
docker system df -v | sed -n "/Build cache usage/,\$p" | head -n 40
echo

echo "=== 4. Диск инстанса целиком ==="
df -h / /var/lib/docker 2>/dev/null | sort -u
'

echo
echo "✅ Готово. Чтобы освободить место от промежуточных слоёв сборки:"
echo "   ./02_power_manager.sh ssh   →   docker builder prune -f   (и docker image prune -f)"
