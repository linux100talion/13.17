#!/usr/bin/env bash
# ============================================================================
# host_persist.sh — ОДИН РАЗ НА МАШИНУ (нужен root): сделать /dev/rawbayer
# постоянным, чтобы после каждого ребута не гонять `make host-setup` с паролем.
#
#   sudo bash docker/sim/scripts/host_persist.sh
#
# Почему не ~/.bashrc: модуль ядра грузится один раз на СИСТЕМУ, а не на шелл;
# в bashrc это дёргало бы sudo в каждом терминале (и просило пароль), да ещё и
# гонка с docker-compose, который требует /dev/rawbayer уже на СОЗДАНИИ nav.
#
# Ставит три файла (идемпотентно, повторный запуск ничего не ломает):
#   /etc/modprobe.d/v4l2loopback-rawbayer.conf — параметры модуля (video_nr=9)
#   /etc/modules-load.d/v4l2loopback.conf      — грузить при старте системы
#   /etc/udev/rules.d/99-rawbayer.rules        — симлинк /dev/rawbayer + права
# и применяет их немедленно, без ребута.
#
# После этого `make host-setup` больше не нужен: он остаётся только для xhost
# (GUI Gazebo) и работает БЕЗ sudo, потому что все root-шаги уже выполнены.
# ============================================================================
set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "ОШИБКА: запускать под root — sudo bash $0" >&2; exit 1; }

# videodev — зависимость v4l2loopback (на минимальных образах его нет)
if ! modinfo videodev >/dev/null 2>&1; then
    echo "host_persist: videodev отсутствует — ставлю linux-modules-extra-$(uname -r)"
    apt-get update -qq
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "linux-modules-extra-$(uname -r)"
fi
modinfo v4l2loopback >/dev/null 2>&1 || {
    echo "ОШИБКА: модуля v4l2loopback нет в системе." >&2
    echo "  поставить: apt-get install v4l2loopback-dkms" >&2
    exit 1
}

# 1. Параметры модуля. width=/height= НЕ передаём — в v4l2loopback >= 0.13 их нет
#    (формат задаёт camera_node через VIDIOC_S_FMT), передача ломает загрузку.
cat > /etc/modprobe.d/v4l2loopback-rawbayer.conf <<'EOF'
# Проект 13.17, симуляция: виртуальная камера /dev/video9 (→ симлинк /dev/rawbayer),
# в неё пишет байеризатор кадры из Gazebo, из неё читает camera_node.
options v4l2loopback devices=1 video_nr=9 card_label=rawbayer exclusive_caps=0
EOF

# 2. Грузить при старте системы (systemd-modules-load подхватит параметры выше)
echo v4l2loopback > /etc/modules-load.d/v4l2loopback.conf

# 3. Симлинк и права — udev'ом, а не ln+chmod после modprobe: udev переустанавливает
#    права асинхронно и затирал ручной chmod (гонка из host_setup.sh).
cat > /etc/udev/rules.d/99-rawbayer.rules <<'EOF'
# /dev/video9 (v4l2loopback, video_nr=9 из modprobe.d) → стабильное имя /dev/rawbayer,
# которое пробрасывается в контейнер nav (docker-compose devices:). Права 0666 —
# контейнер пишет/читает не от root.
KERNEL=="video9", SUBSYSTEM=="video4linux", SYMLINK+="rawbayer", MODE="0666"
EOF

# --- применить немедленно, без ребута ---
udevadm control --reload-rules
if ! lsmod | grep -q '^v4l2loopback'; then
    modprobe v4l2loopback
else
    echo "host_persist: модуль уже загружен — перезагружаю с новыми параметрами"
    rmmod v4l2loopback 2>/dev/null || true
    modprobe v4l2loopback
fi
udevadm trigger --subsystem-match=video4linux
command -v udevadm >/dev/null 2>&1 && udevadm settle || true

if [ -e /dev/rawbayer ]; then
    echo "host_persist: ГОТОВО — $(ls -l /dev/rawbayer)"
    echo "  переживёт ребут; make host-setup больше не требует sudo"
else
    echo "ОШИБКА: /dev/rawbayer не появился. Проверить: lsmod | grep v4l2loopback; ls -l /dev/video*" >&2
    exit 1
fi
