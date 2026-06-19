#!/bin/bash
# Повседневный менеджер питания для экономии бюджета

ZONE="us-central1-a"
INSTANCE_NAME="dev-workspace-137"

case "$1" in
    start)
        echo "🔌 Запускаем сервер..."
        gcloud compute instances start $INSTANCE_NAME --zone=$ZONE
        echo "✅ Сервер запущен. Счетчик биллинга пошел."
        ;;
    stop)
        echo "🛑 Останавливаем сервер..."
        gcloud compute instances stop $INSTANCE_NAME --zone=$ZONE
        echo "💤 Сервер остановлен. Ты платишь только за хранение диска."
        ;;
    ssh)
        echo "💻 Подключаемся по SSH..."
        gcloud compute ssh $INSTANCE_NAME --zone=$ZONE
        ;;
    *)
        echo "Использование: $0 {start|stop|ssh}"
        echo "Пример: ./02_power_manager.sh start"
        exit 1
        ;;
esac
