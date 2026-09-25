#!/bin/bash
# setup_gs.sh — WFB-ng НАЗЕМНАЯ СТАНЦИЯ на ноуте (Ubuntu 24.04, наземный Alfa AWUS036ACH,
# штатный драйвер rtw88_8812au ядра). Пара к борту: distro/etc/wifibroadcast.cfg +
# distro/usr/local/sbin/setup-wfb-ng.sh (тот же тег wfb-ng, тот же канал/ldpc/stbc).
#
# Запуск:  sudo bash tools/wfb/setup_gs.sh
# Нужен tools/wfb/gs.key (пара к /etc/drone.key борта; секрет, в .gitignore).
# Ставит wfb-ng из исходников, отдаёт Alfa из-под NetworkManager, пишет /etc/wifibroadcast.cfg,
# /etc/default/wifibroadcast, /etc/gs.key, ЗАПУСКАЕТ wifibroadcast@gs и включает автозапуск
# (с 2026-09-25; без воткнутого Alfa служба просто перезапускается раз в 5 с, Restart=on-failure).
# Дальше:  wfb-cli gs   (статистика линка), distro/home/andriy/hw_check.sh wfb (проверка),
# ssh andriy@10.5.0.2 (туннель), телеметрия — UDP 127.0.0.1:14550.
# Откат: systemctl disable --now wifibroadcast@gs wifibroadcast.service; rm /etc/NetworkManager/conf.d/90-wfb-gs-unmanaged.conf;
#        systemctl reload NetworkManager
set -euo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
REPO=https://github.com/svpcom/wfb-ng.git
COMMIT=ec936d5153461b4b1c57ca90f732373170ecb37c   # тег wfb-ng-25.01.2, как на борту
SRC=/opt/src/wfb-ng
MAC=00:c0:ca:ba:ca:b9                            # наземный Alfa (doc/HW/alfa.md §4)
export DEBIAN_FRONTEND=noninteractive
APT=(apt-get -y -q -o Dpkg::Options::=--force-confold -o Dpkg::Options::=--force-confdef)

[ "$(id -u)" = 0 ] || { echo "нужен root: sudo bash $0"; exit 1; }
[ -f "$HERE/gs.key" ] || { echo "нет $HERE/gs.key (пара к drone.key борта)"; exit 1; }
IF=$(for n in /sys/class/net/*; do [ "$(cat "$n/address" 2>/dev/null)" = "$MAC" ] && basename "$n"; done)
[ -n "$IF" ] || { echo "наземный Alfa $MAC не найден — воткнуть в USB"; exit 1; }
echo "== наземный Alfa: $IF, драйвер $(basename "$(readlink "/sys/class/net/$IF/device/driver")")"

echo "== зависимости сборки"
dpkg --configure -a --force-confold --force-confdef || true
apt-get update -q || true
"${APT[@]}" install python3-all python3-all-dev libpcap-dev libsodium-dev libevent-dev \
    python3-pip python3-pyroute2 python3-msgpack python3-twisted python3-serial python3-jinja2 \
    iw virtualenv debhelper dh-python fakeroot build-essential libgstrtspserver-1.0-dev socat git

echo "== сборка wfb-ng @ ${COMMIT:0:7}"
mkdir -p "$(dirname "$SRC")"
[ -d "$SRC/.git" ] || git clone -q "$REPO" "$SRC"
git -C "$SRC" fetch -q --tags origin
git -C "$SRC" checkout -q "$COMMIT"
git -C "$SRC" clean -fdxq
(cd "$SRC" && make deb)
"${APT[@]}" install "$SRC"/deb_dist/*.deb
dpkg -l wfb-ng | tail -1

echo "== NetworkManager не трогает наземный Alfa (скан — тоже передача)"
cat > /etc/NetworkManager/conf.d/90-wfb-gs-unmanaged.conf <<EOF
# WFB-ng: наземный Alfa $MAC в мониторе, NM его не трогает (tools/wfb/setup_gs.sh)
[keyfile]
unmanaged-devices=mac:$MAC
EOF
systemctl reload NetworkManager
sleep 2

echo "== конфиг, интерфейс, ключ"
install -m 644 "$HERE/gs.cfg" /etc/wifibroadcast.cfg
echo "WFB_NICS=\"$IF\"" > /etc/default/wifibroadcast
install -m 600 "$HERE/gs.key" /etc/gs.key

systemctl daemon-reload
# экземпляр WantedBy=wifibroadcast.service: без enable родителя при загрузке его никто не
# запускает (так на борту 2026-09-25 радио молчало после ребута при «enabled» экземпляре)
systemctl enable wifibroadcast.service wifibroadcast@gs
systemctl restart wifibroadcast@gs
sleep 6
systemctl --no-pager status wifibroadcast@gs | sed -n 1,14p
iw dev "$IF" info | grep -E 'type|channel|txpower' || true
ip -br addr show gs-wfb || echo "(туннель gs-wfb ещё не поднялся)"
echo
echo "Дальше:  wfb-cli gs      — пакеты/RSSI с борта;   ping 10.5.0.2 / ssh andriy@10.5.0.2 — туннель"
