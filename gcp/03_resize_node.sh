#!/bin/bash
# Изменение мощности инстанса "на лету"

ZONE="us-central1-a"
INSTANCE_NAME="dev-workspace-1317"

if [ -z "$1" ]; then
    echo "Использование: $0 {light|heavy}"
    echo "  light - 4 ядра (n1-standard-4) для правки кода и логов"
    echo "  heavy - 8 ядер (n1-standard-8) для тяжелой компиляции C++/ROS2"
    exit 1
fi

echo "⏳ Шаг 1: Останавливаем инстанс (это необходимо для смены железа)..."
gcloud compute instances stop $INSTANCE_NAME --zone=$ZONE

if [ "$1" == "light" ]; then
    echo "⚙️ Шаг 2: Включаем эконом-режим (n1-standard-4)..."
    gcloud compute instances set-machine-type $INSTANCE_NAME --zone=$ZONE --machine-type=n1-standard-4
elif [ "$1" == "heavy" ]; then
    echo "🚀 Шаг 2: Включаем максимальную мощность (n1-standard-8)..."
    gcloud compute instances set-machine-type $INSTANCE_NAME --zone=$ZONE --machine-type=n1-standard-8
else
    echo "❌ Ошибка: Неизвестный профиль. Выбери light или heavy."
    exit 1
fi

echo "🔌 Шаг 3: Запускаем инстанс с новыми параметрами..."
gcloud compute instances start $INSTANCE_NAME --zone=$ZONE
echo "✅ Готово! Можно работать."
