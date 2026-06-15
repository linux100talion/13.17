#!/usr/bin/env bash
# ============================================================================
# Хостовая подготовка ПЕРЕД запуском стека. Нужен sudo (модуль ядра + X11).
# Запуск: make host-setup  (или bash scripts/host_setup.sh)
# ============================================================================
set -euo pipefail

# 1. X11 доступ для GUI Gazebo (контейнер simulator рендерит в дисплей хоста).
xhost +local:root || true

# 2. v4l2loopback — виртуальная камера /dev/rawbayer.
#    Модуль ядра, в docker НЕ ставится; создаётся на хосте, пробрасывается в nav.
if ! lsmod | grep -q '^v4l2loopback'; then
    sudo modprobe v4l2loopback devices=1 video_nr=9 \
        card_label="rawbayer" exclusive_caps=0
fi
# Фиксированное имя: docker-compose пробрасывает именно /dev/rawbayer.
sudo ln -sf /dev/video9 /dev/rawbayer

echo "host: DISPLAY=${DISPLAY:-<не задан>}, /dev/rawbayer -> /dev/video9 готовы"
