#!/bin/bash
# setup-rtl8812au.sh — драйвер Wi-Fi Alfa AWUS036ACH (Realtek RTL8812AU, usb 0bda:8812)
# для ядра борта через DKMS. В ядре 5.15.185-tegra драйвера нет (in-tree rtw88_8812au
# появился только в 6.13), устройство энумерируется, но интерфейса wlx… не даёт.
#
# Источник: https://github.com/morrownr/8812au-20210820 @ 4722250 (2026-08-20),
# модуль 8812au, dkms rtl8812au/5.13.6-23. Идемпотентен: install-driver.sh сам снимает
# прежнюю dkms-версию и ставит заново. Опции модуля — /etc/modprobe.d/8812au.conf
# (installer кладёт свой; тот же файл лежит в distro/etc/modprobe.d/ — источник правды).
#
# Запуск с ноута:  cd distro && ./deploy.sh -X /usr/local/sbin/setup-rtl8812au.sh usr
# Сборка на Orin Nano ~3–5 мин. После — modprobe 8812au, интерфейс wlx<mac> появляется сам.
set -euo pipefail
REPO=https://github.com/morrownr/8812au-20210820.git
COMMIT=4722250273e4316daf9d8688e9916ea94c36a5ff
SRC=/opt/src/8812au-20210820
export DEBIAN_FRONTEND=noninteractive

echo "== ядро $(uname -r), заголовки: $(ls -d /lib/modules/$(uname -r)/build 2>/dev/null || echo НЕТ)"
apt-get update -q
apt-get install -y -q --no-install-recommends dkms build-essential bc git

mkdir -p "$(dirname "$SRC")"
if [ ! -d "$SRC/.git" ]; then
    git clone -q "$REPO" "$SRC"
fi
git -C "$SRC" fetch -q origin "$COMMIT" || git -C "$SRC" fetch -q origin
git -C "$SRC" checkout -q "$COMMIT"
echo "== исходники: $SRC @ $(git -C "$SRC" rev-parse --short HEAD)"

cd "$SRC"
./install-driver.sh NoPrompt

echo "== dkms:"; dkms status | grep -i 8812 || true
echo "== модуль:"; modinfo -F version 8812au; modinfo -F filename 8812au
modprobe 8812au || true
sleep 3
echo "== интерфейсы:"; ip -br link | grep -E "^wl" || echo "(wlx… пока нет — адаптер воткнут?)"
lsusb | grep -i 8812 || echo "(адаптер 0bda:8812 на USB не найден)"
