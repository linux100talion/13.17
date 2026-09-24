#!/bin/bash
# setup-rtl8812au-wfb.sh — драйвер Alfa AWUS036ACH (RTL8812AU, usb 0bda:8812) под WFB-ng:
# svpcom/rtl8812au (модуль 88XXau_wfb) — монитор + инжекция + мощность индексом 1..63.
# ЗАМЕНЯЕТ morrownr/8812au (setup-rtl8812au.sh, режим клиента роутера): тот монитора не даёт.
#
# Источник: https://github.com/svpcom/rtl8812au ветка v5.2.20 @ 6e75916 (2026-08-20) —
# её рекомендует wfb-ng Setup HOWTO («NVIDIA Jetson has stock rtl8812au installed. You need
# to remove it!»). DKMS-пакет realtek-rtl88xxau/5.2.20.2~20190429.
#
# ПОРЯДОК БЕЗОПАСНОСТИ (за Alfa бустер, предел на входе 20 dBm — doc/HW/alfa.md §5):
#   до запуска на борту уже должны лежать (deploy etc):
#   - /etc/modprobe.d/88XXau_wfb.conf — rtw_tx_pwr_idx_override=10 (старт на малой мощности);
#   - /etc/NetworkManager/conf.d/90-alfa-unmanaged.conf — NM не сканирует через Alfa.
#   Скрипт проверяет оба и без них не работает. Интерфейс НЕ поднимает и ничего не передаёт.
#
# Запуск с ноута:  cd distro && ./deploy.sh -X /usr/local/sbin/setup-rtl8812au-wfb.sh usr etc
# Сборка на Orin Nano ~3–5 мин, нужен интернет (встроенный Wi-Fi). После обновления ядра —
# DKMS пересоберёт сам (AUTOINSTALL=yes); если нет — запустить снова.
# Откат на morrownr: dkms remove realtek-rtl88xxau/5.2.20.2~20190429 --all && setup-rtl8812au.sh
set -euo pipefail
REPO=https://github.com/svpcom/rtl8812au.git
COMMIT=6e75916416de1dce5ecd37f824896bebf96aaf8f
SRC=/opt/src/rtl8812au-wfb
PKG=realtek-rtl88xxau
VER=5.2.20.2~20190429
MAC=00:c0:ca:b9:55:0c
export DEBIAN_FRONTEND=noninteractive

echo "== предохранители"
grep -q '^options 88XXau_wfb .*rtw_tx_pwr_idx_override=' /etc/modprobe.d/88XXau_wfb.conf \
    || { echo "НЕТ /etc/modprobe.d/88XXau_wfb.conf с rtw_tx_pwr_idx_override — сначала deploy etc"; exit 1; }
grep -q "unmanaged-devices=.*$MAC" /etc/NetworkManager/conf.d/90-alfa-unmanaged.conf \
    || { echo "НЕТ 90-alfa-unmanaged.conf — NM будет сканировать через Alfa; сначала deploy etc"; exit 1; }
grep -h '^options 88XXau_wfb' /etc/modprobe.d/88XXau_wfb.conf

echo "== ядро $(uname -r), заголовки: $(ls -d /lib/modules/$(uname -r)/build 2>/dev/null || echo НЕТ)"
apt-get update -q
apt-get install -y -q --no-install-recommends dkms build-essential bc git

echo "== снять morrownr (8812au)"
modprobe -r 8812au 2>/dev/null || true
for v in $(dkms status 2>/dev/null | sed -n 's#^rtl8812au/\([^,]*\),.*#\1#p' | sort -u); do
    dkms remove "rtl8812au/$v" --all || true
done

echo "== исходники svpcom @ ${COMMIT:0:7}"
mkdir -p "$(dirname "$SRC")"
[ -d "$SRC/.git" ] || git clone -q "$REPO" "$SRC"
git -C "$SRC" fetch -q origin "$COMMIT" || git -C "$SRC" fetch -q origin
git -C "$SRC" checkout -q "$COMMIT"
grep -q "PACKAGE_VERSION=\"$VER\"" "$SRC/dkms.conf" || { echo "dkms.conf: версия не $VER"; exit 1; }

echo "== DKMS $PKG/$VER"
dkms remove "$PKG/$VER" --all 2>/dev/null || true
rm -rf "/usr/src/$PKG-$VER"
cp -a "$SRC" "/usr/src/$PKG-$VER"
dkms add "$PKG/$VER"
dkms build "$PKG/$VER"
dkms install "$PKG/$VER"

echo "== загрузка модуля (интерфейс не поднимаем)"
modprobe 88XXau_wfb
sleep 3
IF=$(for n in /sys/class/net/*; do [ "$(cat $n/address 2>/dev/null)" = "$MAC" ] && basename $n; done)
echo "интерфейс: ${IF:-НЕТ}"
[ -n "$IF" ] || { echo "Alfa $MAC не появился: lsusb | grep 8812"; exit 1; }
DRV=$(basename "$(readlink /sys/class/net/$IF/device/driver)")
echo "драйвер: $DRV (патченый svpcom = rtl88xxau_wfb; ethtool version у него = версия ядра, это норма)"
[ "$DRV" = rtl88xxau_wfb ] || { echo "Alfa взял НЕ тот драйвер: $DRV"; exit 1; }
echo "rtw_tx_pwr_idx_override = $(cat /sys/module/88XXau_wfb/parameters/rtw_tx_pwr_idx_override)"
echo "состояние: $(ip -br link show "$IF")"
P=phy$(cat /sys/class/net/$IF/phy80211/index)
iw phy "$P" info | sed -n '/Supported interface modes/,/Band/p' | grep -E '\*'
dkms status | grep -E "$PKG|8812" || true
