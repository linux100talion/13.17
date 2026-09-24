#!/bin/bash
# setup_gs_driver.sh — драйвер svpcom/rtl8812au (88XXau_wfb) для НАЗЕМНОГО Alfa на ноуте.
# Зачем: штатный rtw88_8812au ядра ловит монитор, но инжектированные кадры НЕ передаёт
# (2026-09-24: wfb_tx без ошибок, а у интерфейса TX packets 0) — линк борт→земля есть,
# земля→борт (туннель, команды MAVLink) нет.
#
# Запуск:  sudo bash tools/wfb/setup_gs_driver.sh
# Порядок: сначала ТОЛЬКО сборка DKMS; штатный rtw88 блокируется и модули меняются лишь
# если сборка под текущее ядро прошла. Ядро ноута заморожено (apt-mark hold), DKMS
# пересоберёт сам при смене ядра (AUTOINSTALL). Мощность: rtw_tx_pwr_idx_override=10 —
# с земли уходят только команды MAVLink/ssh, главное — приём (от мощности не зависит); было 20,
# но на батарее ноута Alfa (800 мА + пики TX) отваливался с USB 2026-09-24 → снижено.
# Откат: dkms remove realtek-rtl88xxau/5.2.20.2~20190429 --all;
#        rm /etc/modprobe.d/wfb-gs.conf; modprobe rtw88_8812au
set -euo pipefail
REPO=https://github.com/svpcom/rtl8812au.git
COMMIT=6e75916416de1dce5ecd37f824896bebf96aaf8f   # v5.2.20, как на борту
SRC=/opt/src/rtl8812au-wfb
PKG=realtek-rtl88xxau
VER=5.2.20.2~20190429
MAC=00:c0:ca:ba:ca:b9
export DEBIAN_FRONTEND=noninteractive
[ "$(id -u)" = 0 ] || { echo "нужен root: sudo bash $0"; exit 1; }

echo "== ядро $(uname -r)"
apt-get install -y -q dkms build-essential bc git "linux-headers-$(uname -r)" >/dev/null

mkdir -p "$(dirname "$SRC")"
[ -d "$SRC/.git" ] || git clone -q "$REPO" "$SRC"
git -C "$SRC" fetch -q origin "$COMMIT" || git -C "$SRC" fetch -q origin
git -C "$SRC" checkout -q "$COMMIT"

echo "== DKMS $PKG/$VER — только сборка"
dkms remove "$PKG/$VER" --all 2>/dev/null || true
rm -rf "/usr/src/$PKG-$VER"
cp -a "$SRC" "/usr/src/$PKG-$VER"
dkms add "$PKG/$VER"
if ! dkms build "$PKG/$VER" -k "$(uname -r)"; then
    echo "!! сборка под $(uname -r) НЕ прошла — штатный rtw88 оставлен, ничего не меняли"
    echo "   лог: /var/lib/dkms/$PKG/$VER/build/make.log"
    tail -20 "/var/lib/dkms/$PKG/$VER/build/make.log" || true
    dkms remove "$PKG/$VER" --all || true
    exit 1
fi
dkms install "$PKG/$VER" -k "$(uname -r)"

echo "== замена rtw88 → 88XXau_wfb"
cat > /etc/modprobe.d/wfb-gs.conf <<EOF
# tools/wfb/setup_gs_driver.sh: наземный Alfa под WFB-ng — патченый svpcom вместо штатного rtw88
blacklist rtw88_8812au
blacklist rtw_8812au
options 88XXau_wfb rtw_tx_pwr_idx_override=10
EOF
systemctl stop wifibroadcast@gs 2>/dev/null || true
modprobe -r rtw88_8812au 2>/dev/null || modprobe -r rtw_8812au 2>/dev/null || true
modprobe 88XXau_wfb
sleep 3
IF=$(for n in /sys/class/net/*; do [ "$(cat "$n/address" 2>/dev/null)" = "$MAC" ] && basename "$n"; done)
echo "интерфейс: ${IF:-НЕТ}, драйвер: $(basename "$(readlink "/sys/class/net/$IF/device/driver" 2>/dev/null)" 2>/dev/null)"
[ -n "$IF" ] || { echo "наземный Alfa не появился — переткнуть USB"; exit 1; }
echo "WFB_NICS=\"$IF\"" > /etc/default/wifibroadcast
systemctl restart wifibroadcast@gs
sleep 6
systemctl --no-pager status wifibroadcast@gs | sed -n 1,4p
iw dev "$IF" info | grep -E 'type|channel|txpower'
