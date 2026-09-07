#!/bin/bash



# Проверяем, передан ли первый параметр
if [ -z "$1" ]; then
    echo "Ошибка: Не указан IP-адрес назначения."
    echo "Использование: $0 <IP-адрес>"
    echo "Пример: $0 192.168.1.50"
    exit 1
fi

JETSON_IP="$1"
JETSON_USER="andriy"


# ssh andriy@192.168.x.x "sudo cat /etc/NetworkManager/system-connections/Home34_5G.nmconnection" > ./Home34_5G.nmconnection

rsync -av --rsync-path="sudo rsync" ${JETSON_USER}@${JETSON_IP}:/etc/NetworkManager/system-connections/ ./etc/NetworkManager/system-connections/





















