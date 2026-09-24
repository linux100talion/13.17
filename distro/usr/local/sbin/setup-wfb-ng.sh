#!/bin/bash
# setup-wfb-ng.sh — WFB-ng на борту: сборка deb из исходников и установка (apt.wfb-ng.org
# 2026-09-24 не отвечал). Радиолинк ноут↔борт на инжекции: MAVLink (mavlink-router →
# 127.0.0.1:14560) и IP-туннель drone-wfb 10.5.0.2 ↔ gs-wfb 10.5.0.1 (ssh поверх радио).
#
# Источник: https://github.com/svpcom/wfb-ng тег wfb-ng-25.01.2 @ ec936d5.
# Нужны (deploy etc, ДО запуска): /etc/wifibroadcast.cfg, /etc/default/wifibroadcast,
# драйвер 88XXau_wfb (setup-rtl8812au-wfb.sh) с малым rtw_tx_pwr_idx_override.
# Ключ /etc/drone.key (пара к gs.key ноута; секрет, в git нет) — если его нет, служба
# ставится, но НЕ включается (без ключа борт в эфир не выходит).
#
# Запуск с ноута:  cd distro && ./deploy.sh -X /usr/local/sbin/setup-wfb-ng.sh usr etc
#   ключи впервые:  WFB_KEYGEN=1 → сгенерит пару в /root/wfb-keys (забрать оба на ноут:
#                   drone.key → distro/etc/drone.key, gs.key → tools/wfb/gs.key)
set -euo pipefail
REPO=https://github.com/svpcom/wfb-ng.git
TAG=wfb-ng-25.01.2
COMMIT=ec936d5153461b4b1c57ca90f732373170ecb37c
SRC=/opt/src/wfb-ng
export DEBIAN_FRONTEND=noninteractive
# наши /etc/default/wifibroadcast и /etc/wifibroadcast.cfg (из distro) — dpkg их не трогает
APT=(apt-get -y -q -o Dpkg::Options::=--force-confold -o Dpkg::Options::=--force-confdef)

echo "== предохранители"
[ -f /etc/wifibroadcast.cfg ] && [ -f /etc/default/wifibroadcast ] \
    || { echo "нет /etc/wifibroadcast.cfg или /etc/default/wifibroadcast — сначала deploy etc"; exit 1; }
grep -q '^wifi_txpower = None' /etc/wifibroadcast.cfg \
    || { echo "wifi_txpower в /etc/wifibroadcast.cfg не None — мощность должен держать модуль (бустер!)"; exit 1; }
grep -q '^88XXau_wfb ' /proc/modules || { echo "драйвер 88XXau_wfb не загружен — setup-rtl8812au-wfb.sh"; exit 1; }
echo "мощность модуля: rtw_tx_pwr_idx_override=$(cat /sys/module/88XXau_wfb/parameters/rtw_tx_pwr_idx_override)"

echo "== зависимости сборки"
dpkg --configure -a --force-confold --force-confdef || true
apt-get update -q
"${APT[@]}" install python3-all python3-all-dev libpcap-dev libsodium-dev libevent-dev \
    python3-pip python3-pyroute2 python3-msgpack python3-twisted python3-serial python3-jinja2 \
    iw virtualenv debhelper dh-python fakeroot build-essential libgstrtspserver-1.0-dev socat git

echo "== исходники $TAG @ ${COMMIT:0:7}"
mkdir -p "$(dirname "$SRC")"
[ -d "$SRC/.git" ] || git clone -q "$REPO" "$SRC"
git -C "$SRC" fetch -q --tags origin
git -C "$SRC" checkout -q "$COMMIT"
cd "$SRC"
git clean -fdxq

echo "== сборка deb"
make deb
"${APT[@]}" install ./deb_dist/*.deb
dpkg -l wfb-ng | tail -1

if [ "${WFB_KEYGEN:-0}" = 1 ]; then
    echo "== ключи: новая пара в /root/wfb-keys"
    mkdir -p /root/wfb-keys && chmod 700 /root/wfb-keys
    (cd /root/wfb-keys && wfb_keygen)
    ls -l /root/wfb-keys
fi

systemctl daemon-reload
if [ -f /etc/drone.key ]; then
    chmod 600 /etc/drone.key
    systemctl enable wifibroadcast@drone
    systemctl restart wifibroadcast@drone
    sleep 5
    systemctl --no-pager status wifibroadcast@drone | head -15
    iw dev "$(sed -n 's/^WFB_NICS="\([^" ]*\).*/\1/p' /etc/default/wifibroadcast)" info | grep -E 'type|channel|txpower'
else
    echo "== /etc/drone.key НЕТ — wifibroadcast@drone не включён (в эфир не выходим)"
fi
