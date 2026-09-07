#!/bin/bash

# Проверка, запущен ли скрипт от имени root (sudo)
if [ "$EUID" -ne 0 ]; then
  echo "Ошибка: Пожалуйста, запустите скрипт с правами sudo (sudo ./create_wifi.sh)"
  exit 1
fi

# Настройки по умолчанию (можно поменять под себя)
CON_ID="Starlink"
INTERFACE="wlP1p1s0"
PRIORITY=100

# Запрос данных у пользователя
read -p "Введите SSID (Имя сети) [$CON_ID]: " input_ssid
SSID=${input_ssid:-$CON_ID}

read -p "Введите пароль для $SSID: " PASSWORD

if [ -z "$PASSWORD" ]; then
  echo "Ошибка: Пароль не может быть пустым."
  exit 1
fi

# Генерация уникального UUID
UUID=$(uuidgen)
FILE_PATH="/etc/NetworkManager/system-connections/${SSID}.nmconnection"

echo "Создание конфигурации для $SSID..."

# Запись конфигурации в файл
cat <<EOF > "$FILE_PATH"
[connection]
id=$SSID
uuid=$UUID
type=wifi
interface-name=$INTERFACE
autoconnect=true
autoconnect-priority=$PRIORITY

[wifi]
mode=infrastructure
ssid=$SSID

[wifi-security]
auth-alg=open
key-mgmt=wpa-psk
psk=$PASSWORD

[ipv4]
method=auto

[ipv6]
addr-gen-mode=stable-privacy
method=auto

[proxy]
EOF

# Критически важный шаг: установка правильных прав доступа
chown root:root "$FILE_PATH"
chmod 600 "$FILE_PATH"

# Применение изменений
nmcli connection reload

echo "----------------------------------------"
echo "Успех! Файл $FILE_PATH создан."
echo "Сгенерированный UUID: $UUID"
echo "Сетевой интерфейс: $INTERFACE (Приоритет: $PRIORITY)"
echo "----------------------------------------"
echo "Для принудительного подключения выполните:"
echo "sudo nmcli connection up \"$SSID\""