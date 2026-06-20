#!/bin/bash
# Повседневный менеджер питания для экономии бюджета

ZONE="${ZONE:-europe-west4-a}"
INSTANCE_NAME="dev-workspace-1317"

case "$1" in
    start)
        echo "🔌 Запускаем сервер..."
        # При stop GPU уходит в общий пул, поэтому start заново запрашивает T4
        # и может упасть с RESOURCE_POOL_EXHAUSTED. Проверяем код возврата,
        # а не печатаем "✅" вслепую.
        OUT=$(gcloud compute instances start $INSTANCE_NAME --zone=$ZONE 2>&1)
        RC=$?
        echo "$OUT"
        if [ $RC -eq 0 ]; then
            echo "✅ Сервер запущен. Счетчик биллинга пошел."
        elif echo "$OUT" | grep -q "RESOURCE_POOL_EXHAUSTED"; then
            echo "🛑 В зоне $ZONE сейчас нет свободных T4 (RESOURCE_POOL_EXHAUSTED)."
            echo "   Сервер НЕ запущен (диск цел). Попробуй позже — ёмкость плавает."
            echo "   Зону существующего инстанса сменить нельзя: только ждать или"
            echo "   пересоздавать в другой зоне через снапшот диска."
            exit 1
        else
            echo "❌ Запуск не удался (код $RC). Подробности в выводе выше."
            exit 1
        fi
        ;;
    stop)
        echo "🛑 Останавливаем сервер..."
        if gcloud compute instances stop $INSTANCE_NAME --zone=$ZONE; then
            echo "💤 Сервер остановлен. Ты платишь только за хранение диска."
        else
            echo "❌ Остановка не удалась. Проверь статус: ./04_check_money_leak.sh"
            exit 1
        fi
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
