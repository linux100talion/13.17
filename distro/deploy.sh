#!/bin/bash


#-------------------------------------------------------------------


# Выполнение конкретных команд, требующих прав администратора 
# (например, rsync или перезапуск определенных сервисов systemd), 
# разрешить без пароля.
# Для этого на Jetson нужно выполнить sudo visudo и добавить исключения, например:

# andriy ALL=(ALL) NOPASSWD: /usr/bin/rsync, /bin/systemctl restart vins.service


#---------------------------------------------------------------------

# ---
# Настройка ключей
# ---

# ssh-keygen -t ed25519 -C "laptop-to-jetson"
# потребуется ввести пароль один, последний раз
#ssh-copy-id -i ./jetson.pub andriy@192.168.0.133
#Now try logging into the machine, with:   "ssh 'andriy@192.168.0.133'"
#and check to make sure that only the key(s) you wanted were added.

# на ноуте
# ~/.ssh/config

#Host 192.168.0.133
#    User andriy
#    IdentityFile /usr/local/DATA/Ardupilot/nvidia_skd_doc/distro/ssh-keys/jetson

#На Jetson
#чтобы жестко зафиксировать правильные права:
#chmod 700 ~/.ssh
#chmod 600 ~/.ssh/authorized_keys
#-------------------------------------------------------------------


# Проверяем, передан ли первый параметр
if [ -z "$1" ]; then
    echo "Ошибка: Не указан IP-адрес назначения."
    echo "Использование: $0 <IP-адрес>"
    echo "Пример: $0 192.168.1.50"
    exit 1
fi

JETSON_IP="$1"
JETSON_USER="andriy"



echo "Начинаем деплой конфигурации на Jetson ($JETSON_IP)..."

# 1. Копируем домашнюю директорию (с сохранением прав пользователя)
echo "--- Копируем /home/andriy/ ---"
rsync -avz ./home/andriy/ ${JETSON_USER}@${JETSON_IP}:/home/andriy/

# 2. Копируем настройки в /etc (принудительно задаем владельца root:root)
echo "--- Копируем /etc/ ---"
rsync -avz --rsync-path="sudo rsync" --chown=root:root ./etc/ ${JETSON_USER}@${JETSON_IP}:/etc/

# 3. Копируем исполняемые файлы в /usr (принудительно задаем владельца root:root)
echo "--- Копируем /usr/ ---"
rsync -avz --rsync-path="sudo rsync" --chown=root:root ./usr/ ${JETSON_USER}@${JETSON_IP}:/usr/

echo "Деплой завершен!"















