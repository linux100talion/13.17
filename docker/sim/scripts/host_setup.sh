#!/usr/bin/env bash
# ============================================================================
# Хостовая подготовка ПЕРЕД запуском стека. Нужен root/sudo (модуль ядра + X11).
# Запуск: make host-setup  (или bash scripts/host_setup.sh)
#
# Делает три вещи:
#   1. X11-доступ для GUI Gazebo (только если есть xhost/DISPLAY; headless —
#      пропускается, gz sim идёт --headless-rendering).
#   2. Гарантирует ядро V4L2 (videodev) — на свежих/облачных образах (GCE и др.)
#      его нет: он в пакете linux-modules-extra-$(uname -r). Без videodev
#      v4l2loopback не грузится ("Unknown symbol v4l2_*").
#   3. v4l2loopback — виртуальная камера /dev/rawbayer (video_nr=9), куда
#      байеризатор пишет кадры из Gazebo; пробрасывается в контейнер nav.
#
# Идемпотентно: повторный запуск ничего не ломает.
# ============================================================================
set -euo pipefail

# --- 1. X11 для GUI Gazebo (опционально; headless-бокс — пропуск) ---------
if command -v xhost >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    xhost +local:root || true
else
    echo "host: xhost/DISPLAY нет — пропускаю X11 (gz sim --headless-rendering)"
fi

# --- 2. videodev (ядро V4L2) — зависимость v4l2loopback -------------------
# v4l2loopback зависит от videodev; если его нет в /lib/modules, modprobe
# v4l2loopback падает на неразрешённых символах. videodev приходит с
# linux-modules-extra-<kernel> (на GCE/минимальных образах не предустановлен).
if ! modinfo videodev >/dev/null 2>&1; then
    echo "host: videodev отсутствует — ставлю linux-modules-extra-$(uname -r)..."
    sudo apt-get update -qq
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
        "linux-modules-extra-$(uname -r)"
fi
sudo modprobe videodev

# --- 3. v4l2loopback — /dev/rawbayer --------------------------------------
# ВНИМАНИЕ: параметры width=/height= тут НЕ передаём — в v4l2loopback >= 0.13
# их нет (есть только max_width/max_height, дефолт 8192). Реальный формат/
# разрешение задаёт camera_node через VIDIOC_S_FMT (1280×720). Передача
# width=/height= ломает загрузку ("unknown parameter").
if ! lsmod | grep -q '^v4l2loopback'; then
    sudo modprobe v4l2loopback devices=1 video_nr=9 \
        card_label="rawbayer" exclusive_caps=0
fi

# Фиксированное имя: docker-compose пробрасывает именно /dev/rawbayer.
sudo ln -sf /dev/video9 /dev/rawbayer
# udev переустанавливает права на video-устройствах асинхронно после modprobe —
# дождёмся, иначе chmod может не «прилипнуть».
command -v udevadm >/dev/null 2>&1 && sudo udevadm settle || true
sudo chmod 666 /dev/video9 || true

echo "host: DISPLAY=${DISPLAY:-<не задан>}, /dev/rawbayer -> /dev/video9 готовы"
