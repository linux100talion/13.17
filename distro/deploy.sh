#!/bin/bash
# deploy.sh — деплой каталога distro/ на Jetson по rsync.
#
#   ./deploy.sh [-H host] [-n] [-r] [-x 'cmd'] [-X 'cmd'] [секция ...]
#
#   секции   home | etc | usr        по умолчанию — все три
#   -H host  адрес Jetson            умолч. $JETSON_HOST, иначе 192.168.55.1 (USB)
#   -n       dry-run: показать, что изменится, ничего не писать
#   -r       после деплоя: `nmcli connection reload` + `systemctl daemon-reload`
#            на борту (root)
#   -x cmd   выполнить cmd на борту после деплоя от andriy (можно несколько)
#   -X cmd   то же от root (sudo с паролем $JETSON_SUDO, умолч. см. CLAUDE.md)
#   Опции — ДО секций (getopts).
#
# Что едет куда (без --delete: лишнее на борту НЕ трогается, удаление — руками):
#   home/andriy/ → /home/andriy/   от пользователя (права/владелец andriy)
#   etc/         → /etc/           через `sudo rsync`, root:root
#   usr/         → /usr/           через `sudo rsync`, root:root
# doc/ (заметки, ssh-ключ, wifi.txt) — НЕ деплоится.
#
# Вывод rsync — itemize (-i): печатаются ТОЛЬКО изменённые файлы
# (`>f.st...` — содержимое/время, `>f+++++++` — новый, `cd+++++++` — новый каталог).
#
# ---- Однократная подготовка Jetson --------------------------------------
# 1. ssh-ключ: пара doc/ssh-keys/jetson{,.pub}; на ноуте в ~/.ssh/config
#      Host 192.168.55.1
#          User andriy
#          IdentityFile /usr/local/DATA/Calude/13.17/distro/doc/ssh-keys/jetson
#    на борту: ssh-copy-id -i doc/ssh-keys/jetson.pub andriy@<host>,
#    chmod 700 ~/.ssh; chmod 600 ~/.ssh/authorized_keys
# 2. sudo без пароля на rsync (уже стоит в /etc/sudoers борта):
#      andriy ALL=(ALL) NOPASSWD: /usr/bin/rsync
#    — нужен секциям etc/usr (`--rsync-path="sudo rsync"`). Остальное root'ом
#    (-r, -X) идёт через `sudo -S`, пароль подаётся в stdin ssh (в ps не светится).
# --------------------------------------------------------------------------

set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

HOST="${JETSON_HOST:-192.168.55.1}"
USER_="andriy"
SUDOPW="${JETSON_SUDO:-ok}"
DRY=""
RELOAD=0
POST=()      # элементы "u:cmd" (andriy) / "r:cmd" (root)

usage() { sed -n '2,17p' "$0"; exit "${1:-0}"; }

while getopts "H:nrx:X:h" o; do
    case $o in
        H) HOST="$OPTARG" ;;
        n) DRY="-n" ;;
        r) RELOAD=1 ;;
        x) POST+=("u:$OPTARG") ;;
        X) POST+=("r:$OPTARG") ;;
        h) usage 0 ;;
        *) usage 1 ;;
    esac
done
shift $((OPTIND - 1))
SECTIONS=("$@")
[ ${#SECTIONS[@]} -eq 0 ] && SECTIONS=(home etc usr)

for s in "${SECTIONS[@]}"; do
    case $s in home|etc|usr) ;; *) echo "неизвестная секция: $s" >&2; usage 1 ;; esac
done

T="${USER_}@${HOST}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5)
as_user() { "${SSH[@]}" "$T" "$1"; }
as_root() { "${SSH[@]}" "$T" "sudo -S -p '' bash -c $(printf %q "$1")" <<< "$SUDOPW"; }

echo "== Jetson $T $( [ -n "$DRY" ] && echo '(DRY-RUN)' )"
"${SSH[@]}" "$T" 'echo "   $(hostname) $(uname -m), up $(uptime -p)"' \
    || { echo "!! $T недоступен по ssh" >&2; exit 2; }

# -O: не трогать mtime каталогов (шум в itemize). Для etc/usr права СУЩЕСТВУЮЩИХ
# файлов/каталогов на борту не трогаем (--no-perms), новым — по --chmod (git хранит
# только бит x, каталоги рабочей копии 775). Секреты NM: новые файлы строго 600,
# иначе NetworkManager молча игнорирует профиль.
RS=(rsync -a -z -i -O $DRY -e "${SSH[*]}")
SUDO=(--rsync-path="sudo rsync" --chown=root:root --no-perms --chmod=Dgo-w)
NMC=etc/NetworkManager/system-connections
changed=0

sync_section() {   # <секция> <src/> <dst/> [доп. опции rsync...]
    local name=$1 src=$2 dst=$3; shift 3
    echo "-- $name: $src → $T:$dst"
    local out
    out=$("${RS[@]}" "$@" "$src" "$T:$dst")
    if [ -n "$out" ]; then
        echo "$out" | sed 's/^/   /'
        changed=$((changed + $(echo "$out" | grep -c '^[<>c]' || true)))
    else
        echo "   без изменений"
    fi
}

for s in "${SECTIONS[@]}"; do
    case $s in
        home) sync_section home ./home/andriy/ /home/andriy/ ;;
        etc)  sync_section etc  ./etc/ /etc/ "${SUDO[@]}" --exclude="/${NMC#etc/}/"
              sync_section etc/nm ./$NMC/ /$NMC/ "${SUDO[@]}" --chmod=D700,F600 ;;
        usr)  sync_section usr  ./usr/ /usr/ "${SUDO[@]}" ;;
    esac
done

echo "== изменённых файлов/каталогов: $changed$( [ -n "$DRY" ] && echo ' (dry-run, не записано)' )"
[ -n "$DRY" ] && exit 0

if [ $RELOAD -eq 1 ]; then
    echo "-- reload на борту (nmcli connection reload, systemctl daemon-reload)"
    as_root 'nmcli connection reload && systemctl daemon-reload && echo "   ok"' \
        || echo "!! reload не удался" >&2
fi

for item in "${POST[@]}"; do
    cmd=${item#?:}
    case $item in
        u:*) echo "-- на борту (andriy): $cmd"; as_user "$cmd" | sed 's/^/   /' ;;
        r:*) echo "-- на борту (root): $cmd";   as_root "$cmd" | sed 's/^/   /' ;;
    esac
done

echo "== деплой завершён"
