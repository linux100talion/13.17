#!/usr/bin/env bash
#
# capture_scene.sh — единый АТОМАРНЫЙ прогон симуляции с хоста: СЕКВЕНСОР команд.
#
# Синтаксис:
#   capture_scene.sh [WxH] <команда> [арг] <команда> [арг] ...
#
#   WxH        — (опц., 1-й позиц. аргумент) разрешение камеры, напр. 640x480.
#                Если задано → стек ПЕРЕСОЗДАЁТСЯ (fresh-start), т.к. env
#                применяется при создании контейнера. Если не задано → restart-all.
#   команды    — 5 чистых лётных команд (каждая = свой скрипт в /lab/):
#                  arm            GUIDED + арм (без взлёта)
#                  takeoff [ALT]  взлёт на ALT м (default 3)
#                  hover [SIM_SEC] висение SIM_SEC секунд sim-времени (default 10)
#                  land           посадка (режим LAND)
#                  disarm         дизарм
#
# Запись rosbag + извлечение кадров по пути + сборка mp4 (полный поток камеры) +
# заливка на Google Drive идут АВТОМАТИЧЕСКИ вокруг всей последовательности
# (управляются env, см. ниже). mp4 можно выключить: MP4=0.
#
# В НАЧАЛЕ прогона чистятся артефакты прошлого: на хосте — bag + кадры + scene.mp4
# ($OUTPUT_DIR/scene_bag*, scene_img/); на Google Drive — корневая папка проекта
# (GDRIVE_ROOT, только она). На Drive заливается ТОЛЬКО scene.mp4 (кадры — нет).
#
# Запускать С ХОСТА из любого места.  Примеры:
#   bash src/lab/capture_scene.sh 640x480 arm takeoff 5 hover 2 land
#   GDRIVE_UP=0 bash src/lab/capture_scene.sh arm takeoff 3 hover 5 land disarm
#   CPU=1 bash src/lab/capture_scene.sh 320x180 arm takeoff 3 hover 5 land
#
set -euo pipefail

# ── параметры записи/извлечения/заливки (env; полётные параметры — позиционные) ─
RESTART="${RESTART:-1}"         # 1 = перезапуск стека; 0 = на живом стеке (⚠️ рассинхрон)
RECORD="${RECORD:-1}"           # 1 = писать rosbag (/image_color + поза)
MP4="${MP4:-1}"                 # 1 = собрать mp4 из ВСЕХ кадров /image_color и залить с кадрами
MP4_MAXW="${MP4_MAXW:-1280}"    # макс. ширина кадра в mp4, px (0 = не масштабировать)
N_FRAMES="${N_FRAMES:-30}"      # макс. число кадров (0 = без лимита)
DIST_M="${DIST_M:-0.5}"         # шаг выборки кадров по пройденному пути, м
FRAMES="${FRAMES:-1}"           # 0 = совсем НЕ извлекать JPEG-кадры (mp4 не затронут)
TOPIC="${TOPIC:-/image_color}"  # топик камеры
POSE_TOPIC="${POSE_TOPIC:-/mavros/local_position/pose}" # поза для расчёта пути
TOPICS_EXTRA="${TOPICS_EXTRA:-}" # доп. топики в bag (через пробел), напр. диагностика IMU
SKIP_CAM="${SKIP_CAM:-0}"       # 1 = НЕ писать/не обрабатывать /image_color: лёгкий bag
                                #     (мегабайты) для анализа только по IMU/позе, напр. FFT
NAV="${NAV:-p1317_nav}"         # имя nav-контейнера
SIM="${SIM:-p1317_simulator}"   # имя simulator-контейнера (порывы ветра)
CPU="${CPU:-}"                  # CPU=1 → GPU-less режим (docker-compose.cpu.yml)
GDRIVE_UP="${GDRIVE_UP:-1}"            # 1 = заливать на Google Drive (ТОЛЬКО scene.mp4); 0 = нет
GDRIVE_REMOTE="${GDRIVE_REMOTE:-gdrive}"      # имя rclone-remote (из rclone.conf)
GDRIVE_DIR="${GDRIVE_DIR:-13.17/scene_img}"   # папка на Drive (куда кладём scene.mp4)
GDRIVE_ROOT="${GDRIVE_ROOT:-${GDRIVE_DIR%%/*}}" # корневая папка проекта на Drive — чистится в
                                # НАЧАЛЕ прогона (только она); по умолчанию 1-й сегмент GDRIVE_DIR (13.17)

# SKIP_CAM=1: выкидываем камеру из всего пайплайна. Нельзя сделать просто
# TOPIC="" снаружи — выше стоит ${TOPIC:-/image_color}, а ':-' подставляет дефолт
# и на ПУСТУЮ строку. Поэтому отдельный флаг: гасит запись /image_color, сборку
# mp4, извлечение кадров и заливку (заливать нечего — bag без картинки).
if [ "$SKIP_CAM" = "1" ]; then
    TOPIC=""        # из ros2 bag record выпадает -> пишется только поза + TOPICS_EXTRA
    MP4=0           # нет кадров -> нет видео
    GDRIVE_UP=0     # заливать нечего
fi

# ── разбор позиционных аргументов: [WxH] + последовательность команд ───────────
CAMERA_W="" ; CAMERA_H=""
ARGS=("$@")
if [ "${#ARGS[@]}" -gt 0 ] && [[ "${ARGS[0]}" =~ ^[0-9]+x[0-9]+$ ]]; then
    CAMERA_W="${ARGS[0]%x*}"
    CAMERA_H="${ARGS[0]#*x}"
    export CAMERA_W CAMERA_H
    ARGS=("${ARGS[@]:1}")   # сдвиг: дальше только команды
fi
SEQ=("${ARGS[@]}")          # последовательность команд (могут идти с числ. аргументом)

if [ "${#SEQ[@]}" -eq 0 ]; then
    echo "ОШИБКА: не задана последовательность команд." >&2
    echo "Пример: bash src/lab/capture_scene.sh 640x480 arm takeoff 5 hover 2 land" >&2
    exit 2
fi

# Валидация команд заранее (до рестарта стека) — чтобы не поднимать стек впустую.
# takeoff/hover принимают необязательный числовой аргумент; arm/land/disarm — нет.
i=0
while [ "$i" -lt "${#SEQ[@]}" ]; do
    cmd="${SEQ[$i]}"
    case "$cmd" in
        arm|land|disarm|bootstrap|liftland|bootstrap_arch2) ;;
        takeoff|hover|square)
            # опциональный числовой аргумент: если следующий токен — число, он наш
            # (takeoff=ALT, hover=SIM_SEC, square=число кругов)
            nxt="${SEQ[$((i+1))]:-}"
            [[ "$nxt" =~ ^[0-9]+(\.[0-9]+)?$ ]] && i=$((i+1))
            ;;
        *)
            echo "ОШИБКА: неизвестная команда '$cmd'." >&2
            echo "Допустимо: arm, bootstrap, bootstrap_arch2, liftland, takeoff [ALT], hover [SIM_SEC], square [LOOPS], land, disarm." >&2
            exit 2
            ;;
    esac
    i=$((i+1))
done

# ── пути ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SIM_DIR="$REPO_ROOT/docker/sim"
OUTPUT_DIR="$SIM_DIR/output"          # смонтирован в nav как /root/sim_ws/output
BAG_HOST="$OUTPUT_DIR/scene_bag"
IMG_HOST="$OUTPUT_DIR/scene_img"      # кадры извлекаются сюда, отсюда же грузим на Drive

# CPU=1 прокидывается в каждую make-цель (иначе часть пойдёт по базовому compose).
# CPU_NOTE берётся из ТОЙ ЖЕ проверки, что и флаг: раньше лог печатал "(CPU=1)" по
# ${CPU:+…}, то есть на любое непустое значение — включая CPU=0, когда прогон шёл
# на GPU. В разборе кампании такой лог уводит в сторону.
MK=(make -C "$SIM_DIR")
CPU_NOTE=""
[ "$CPU" = "1" ] && { MK+=(CPU=1); CPU_NOTE=" (CPU=1)"; }

# окружение ROS внутри контейнера (overlay нужен для cv_bridge)
SRC='source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash'

log() { echo -e "\n=== $* ==="; }

# ── владелец артефактов ───────────────────────────────────────────────────────
# Всё в output/ пишут root-процессы ВНУТРИ контейнеров (bag, кадры, mp4, логи) —
# bind mount сохраняет UID писателя, на хосте файлы выходят root'овыми, и cp/rm
# под обычным юзером (ноут) упираются в права. Возвращаем владельца юзеру хоста
# изнутри nav-контейнера (там мы root → sudo на хосте не нужен). trap EXIT →
# срабатывает на ЛЮБОМ выходе: успех, ошибка, ранние exit (SKIP_CAM/RECORD=0).
HOST_UG="$(id -u):$(id -g)"
chown_output() {
    docker exec "$NAV" chown -R "$HOST_UG" /root/sim_ws/output >/dev/null 2>&1 || true
}
# Публикатор порывов (WIND_GUST) гасим в том же trap: он живёт в контейнере
# simulator и переживает ЛЮБОЙ выход секвенсора (успех/ошибка/ранний exit) —
# без этого бесконечный профиль (every>0) качал бы ветер и после прогона.
# SIGTERM для него вежливый: восстанавливает базовый ветер и выходит.
cleanup() {
    if [ -n "${WIND_GUST:-}" ]; then
        docker exec "$SIM" pkill -f wind_gust >/dev/null 2>&1 || true
    fi
    chown_output
}
trap cleanup EXIT

# ── 0. подготовка хоста + очистка артефактов прошлого прогона ─────────────────
# fresh-start/restart монтируют /dev/rawbayer (v4l2loopback) в nav. Модуль ядра
# может выгрузиться (ребут бокса) → docker не найдёт устройство и fresh-start
# упадёт ("error gathering device information ... /dev/rawbayer"). Поднимаем
# ЛЕНИВО — только если устройства нет (host_setup.sh идемпотентен, требует sudo).
if [ ! -e /dev/rawbayer ]; then
    log "host_setup: /dev/rawbayer нет — поднимаю v4l2loopback"
    bash "$SIM_DIR/scripts/host_setup.sh" || {
        echo "ОШИБКА: host_setup.sh не смог поднять /dev/rawbayer (нужен sudo)." >&2
        exit 1
    }
fi

# В НАЧАЛЕ прогона: с хоста удаляем старый bag, кадры и видео; на Google Drive
# чистим корневую папку проекта (ТОЛЬКО её, GDRIVE_ROOT). Свежие артефакты этого
# прогона создаются ниже.
# ⚠️ ЗДЕСЬ УМИРАЕТ BAG ПРОШЛОГО ПРОГОНА. С 2026-09-07 freefly_lv кладёт в архив
# КОПИЮ, а настоящий каталог оставляет в output/scene_bag — чтобы сразу после
# записи из него играл `ros2 bag play` и работали инструменты с дефолтным
# SCENE_BAG. Живёт он ровно до этой строки, то есть до старта следующего прогона;
# архив joystick/<NAME>/bag не трогаем. Если прогон шёл с KEEP_BAG=0 или архив не
# получился — здесь пропадает единственная копия (freefly_lv предупреждает вслух).
log "очистка артефактов прошлого прогона"
echo "  хост: чищу СОДЕРЖИМОЕ $OUTPUT_DIR/scene_bag (сама папка остаётся)" \
     "+ rm $IMG_HOST (кадры + scene.mp4)"
# САМ КАТАЛОГ scene_bag НЕ УДАЛЯЕТСЯ НИКОГДА (решение 2026-09-07) — чистим только
# содержимое. Удалить и создать заново нельзя даже «на миг»: это ДРУГОЙ inode, и
# всё, что держало каталог (терминал с cd, открытый дескриптор, наблюдатель),
# оказывается в удалённой директории. Поэтому rosbag2 пишет в свой временный
# каталог output/.rec (он отказывается писать в существующий: «Output folder
# 'scene_bag' already exists», флага overwrite в Humble нет — проверено), а на
# стопе записи файлы ПЕРЕЕЗЖАЮТ в эту же папку (шаги 2 и 4). Соседей вида
# scene_bag_* (старые имена, оборванные прогоны) и остатки .rec сносим целиком.
# Бэг/кадры пишет root ВНУТРИ контейнера; на хосте под обычным юзером (ноут, в
# отличие от root-бокса GCE) rm упирается в права. Остатки добиваем через
# контейнер (root); output/ — общий bind mount, путь в контейнере фиксирован.
rm -rf "$OUTPUT_DIR"/scene_bag/* "$OUTPUT_DIR"/scene_bag/.[!.]* 2>/dev/null || true
find "$OUTPUT_DIR" -maxdepth 1 -name 'scene_bag?*' -exec rm -rf {} + 2>/dev/null || true
rm -rf "$OUTPUT_DIR/.rec" "$IMG_HOST" 2>/dev/null || true
# «грязно» = в scene_bag что-то осталось, или жив сосед scene_bag_*, или scene_img
bag_dirty() {
    [ -n "$(ls -A "$OUTPUT_DIR/scene_bag" 2>/dev/null || true)" ] && return 0
    compgen -G "$OUTPUT_DIR/scene_bag?*" >/dev/null && return 0
    [ -e "$OUTPUT_DIR/.rec" ] && return 0
    [ -e "$IMG_HOST" ] && return 0
    return 1
}
if bag_dirty; then
    echo "  хост: артефакты принадлежат root — удаляю через контейнер $NAV"
    docker start "$NAV" >/dev/null 2>&1 || true
    docker exec "$NAV" bash -c \
        'rm -rf /root/sim_ws/output/scene_bag/* /root/sim_ws/output/scene_bag/.[!.]*;
         find /root/sim_ws/output -maxdepth 1 -name "scene_bag?*" -exec rm -rf {} +;
         rm -rf /root/sim_ws/output/.rec /root/sim_ws/output/scene_img' || {
        echo "ОШИБКА: не смог удалить root-артефакты (контейнер $NAV недоступен)." >&2
        echo "  вручную: docker exec $NAV rm -rf /root/sim_ws/output/scene_bag/* /root/sim_ws/output/scene_img" >&2
        exit 1
    }
fi
mkdir -p "$OUTPUT_DIR/scene_bag"   # папка живёт между прогонами (пустая — это норма)
mkdir -p "$IMG_HOST"        # каталог нужен make_video.py (пишет сюда scene.mp4)
if [ "$GDRIVE_UP" = "1" ] && [ -n "$GDRIVE_ROOT" ] && [ "$GDRIVE_ROOT" != "/" ]; then
    if rclone listremotes 2>/dev/null | grep -qx "${GDRIVE_REMOTE}:"; then
        echo "  Drive: очищаю ${GDRIVE_REMOTE}:${GDRIVE_ROOT} (только эту папку)"
        rclone purge "${GDRIVE_REMOTE}:${GDRIVE_ROOT}" 2>/dev/null || true
    else
        echo "  Drive: remote '${GDRIVE_REMOTE}:' не настроен — очистку Drive пропускаю"
    fi
fi

# ── 1. перезапуск стека ───────────────────────────────────────────────────────
if [ "$RESTART" = "1" ]; then
    # Разрешение задано → пересоздаём контейнеры (fresh-start), иначе быстрый
    # restart-all. fresh-start безопасен: критичные SITL-параметры в host-
    # смонтированном sitl-extra.parm, sim_up.sh применяет их при каждом старте.
    if [ -n "$CAMERA_W" ] || [ -n "$CAMERA_H" ]; then
        RESET_TARGET=fresh-start
        RES_NOTE=" → ${CAMERA_W}×${CAMERA_H} (fresh-start, пересоздание)"
    else
        RESET_TARGET=restart-all
        RES_NOTE=""
    fi
    log "перезапуск стека${CPU_NOTE}${RES_NOTE}"
    "${MK[@]}" "$RESET_TARGET" 2>&1 | tail -3
    "${MK[@]}" wait
else
    log "перезапуск ПРОПУЩЕН (RESTART=0) — прогон на живом стеке"
    echo "  ⚠️ без рестарта возможен рассинхрон состояния (см. дисциплину прогона в CLAUDE.md)"
    if [ -n "$CAMERA_W" ] || [ -n "$CAMERA_H" ]; then
        echo "  ⚠️ разрешение задано, но при RESTART=0 НЕ применится (нужен fresh-start)"
    fi
fi

# ── 1b. порывы ветра (WIND_GUST) — детерминированный профиль поверх WIND_SPD ──
# Публикатор src/lab/wind_gust.py живёт в контейнере simulator (там биндинги
# gz.transport13) и шлёт вектор ветра в runtime-топик плагина WindEffects.
# Расписание в АБСОЛЮТНОМ sim-времени (t=0 = старт Gazebo) → на fresh-start
# порывы двух прогонов приходят в одни и те же sim-секунды (честный A/B).
# Спека и дефолты — в шапке wind_gust.py; лог фаз — output/wind_gust.log.
# Гасится в trap cleanup на любом выходе. Оба рестарта (restart-all и
# fresh-start) перезапускают Gazebo → sim-часы с нуля, `at` честный.
# ⚠️ Только при RESTART=0 часы старые — абсолютное `at` уже уехало.
if [ -n "${WIND_GUST:-}" ]; then
    log "порывы ветра: WIND_GUST='$WIND_GUST' (лог output/wind_gust.log)"
    docker exec "$SIM" pkill -f wind_gust >/dev/null 2>&1 || true
    rm -f "$OUTPUT_DIR/wind_gust.log" 2>/dev/null || true
    python3 "$SCRIPT_DIR/wind_gust.py" "$WIND_GUST" || {
        echo "ОШИБКА: wind_gust.py не запустился (спека WIND_GUST кривая?)" >&2
        exit 3
    }
fi

# ── 2. старт записи rosbag (вокруг всей последовательности команд) ─────────────
if [ "$RECORD" = "1" ]; then
    log "старт записи rosbag $TOPIC + $POSE_TOPIC${TOPICS_EXTRA:+ + $TOPICS_EXTRA}"
    # ПИШЕМ ВО ВРЕМЕННЫЙ РОДИТЕЛЬСКИЙ КАТАЛОГ output/.rec, а не прямо в scene_bag:
    # rosbag2 (Humble) не пишет в существующий каталог («[ERROR] [ros2bag]: Output
    # folder 'scene_bag' already exists», флага overwrite нет — проверено 2026-09-07),
    # а удалять и пересоздавать живую папку нельзя: другой inode ломает всех, кто её
    # держит. Имя внутри .rec — ТО ЖЕ scene_bag, поэтому файлы получаются штатные
    # (scene_bag_0.db3 + metadata.yaml с теми же относительными путями), и шаг 4
    # просто переносит их в постоянную папку (rename на том же ФС — мгновенно).
    rm -rf "$OUTPUT_DIR/.rec" 2>/dev/null || true
    mkdir -p "$OUTPUT_DIR/.rec"
    docker exec "$NAV" bash -lc "$SRC; cd /root/sim_ws/output/.rec && exec ros2 bag record -o scene_bag $TOPIC $POSE_TOPIC $TOPICS_EXTRA" &
    sleep 3
fi

# ── 3. исполнение последовательности команд ───────────────────────────────────
log "последовательность: ${SEQ[*]}"
# Фиксированный прогрев EKF убран: команды ждут готовность ПО ФАКТУ (arm.sh —
# GUIDED-латч в бюджете sim-времени, takeoff.sh — поллинг высоты). На CPU-боксе
# (RTF≈0.07) фикс. wall-пауза = доли sim-секунды и всё равно ничего не гарантировала.
i=0
while [ "$i" -lt "${#SEQ[@]}" ]; do
    cmd="${SEQ[$i]}"
    arg=""
    case "$cmd" in
        takeoff|hover|square)
            nxt="${SEQ[$((i+1))]:-}"
            if [[ "$nxt" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then arg="$nxt"; i=$((i+1)); fi
            ;;
    esac
    echo "--- ${cmd}${arg:+ $arg} ---"
    # Проброс бюджетов ожидания (arm.sh/takeoff.sh) — таймауты, не sleep: на успехе
    # ничего не стоят, но спасают от гонки «бюджет арма vs прогрев EKF/GPS» (под
    # lockstep готовность позиции наступает чуть позже стандартных 40 sim-сек).
    # ⚠️ ПРОБРОС BS_*/ARM_* — АВТОМАТИЧЕСКИЙ, не списком. Раньше тут висел рукописный
    # белый список из ~90 `-e BS_…`, и ручка, забытая в нём, доезжала до .env и меты
    # прогона, но НЕ до ноды: прогон выглядел настроенным, а летел на дефолте. Так молча
    # пропал весь свип B3s (BS_ROLL_RATE_KI=5 → фактически 0), и так же не доезжали
    # BS_IPM_WZ_TAU и BS_FENCE — их значения просто совпадали с дефолтами конфига.
    # `docker exec -e ИМЯ` без `=` берёт значение из текущего окружения; неустановленные
    # переменные не пробрасываются вовсе, что для `[ -n "${BS_X:-}" ]` в bootstrap_arch2.sh
    # неотличимо от прежней пустой строки.
    # С 2026-09-07 ручки ноды в контейнер НЕ пробрасываются: едет только СПИСОК
    # профилей PROFILES (bootstrap_arch2.sh собирает их загрузчиком по схеме), аргументы
    # прогона BS_REPLAY_* (сценарий реплея) и ARM_* (бюджеты arm.sh/takeoff.sh — лаб-
    # скрипты, не нода). Хостовые BS_* до ноды не доезжают по построению.
    ENVS=()
    while IFS= read -r k; do ENVS+=(-e "$k"); done \
        < <(env | sed -n 's/^\(\(PROFILES\|BS_REPLAY_[A-Z]*\|ARM_[A-Z0-9_]*\)\)=.*/\1/p' | sort)
    docker exec "${ENVS[@]}" \
      "$NAV" bash /lab/"$cmd".sh $arg
    i=$((i+1))
done

# ── 4. стоп записи ────────────────────────────────────────────────────────────
if [ "$RECORD" = "1" ]; then
    log "стоп записи rosbag"
    docker exec "$NAV" pkill -INT -f "ros2 bag record" || true
    # metadata.yaml рекордер дописывает уже ПОСЛЕ SIGINT — ждём его по факту
    # (фиксированный sleep 2 на медленном боксе иногда не покрывал финализацию).
    for _ in $(seq 1 15); do
        [ -f "$OUTPUT_DIR/.rec/scene_bag/metadata.yaml" ] && break
        sleep 1
    done
    # ФАЙЛЫ — В ПОСТОЯННУЮ ПАПКУ (см. шаг 2): mv содержимого, сам каталог scene_bag
    # не трогаем, его inode тот же, что был до прогона. Изнутри контейнера (root):
    # bag пишет root, а с хоста под обычным юзером mv упирается в права.
    MOVE='set -e; shopt -s dotglob nullglob
        src=/root/sim_ws/output/.rec/scene_bag; dst=/root/sim_ws/output/scene_bag
        [ -f "$src/metadata.yaml" ] || exit 3
        mkdir -p "$dst"; mv "$src"/* "$dst"/
        rmdir "$src" /root/sim_ws/output/.rec 2>/dev/null || true'
    RC_MOVE=0
    docker exec "$NAV" bash -c "$MOVE" 2>/dev/null || RC_MOVE=$?
    if [ "$RC_MOVE" != "0" ]; then      # контейнер недоступен/права — пробуем с хоста
        ( set -e; shopt -s dotglob nullglob
          [ -f "$OUTPUT_DIR/.rec/scene_bag/metadata.yaml" ] || exit 3
          mv "$OUTPUT_DIR"/.rec/scene_bag/* "$OUTPUT_DIR"/scene_bag/
          rmdir "$OUTPUT_DIR/.rec/scene_bag" "$OUTPUT_DIR/.rec" 2>/dev/null || true
        ) 2>/dev/null && RC_MOVE=0
    fi
    case "$RC_MOVE" in
        0) echo "  bag: $BAG_HOST (папка та же, что была до прогона — inode не менялся)" ;;
        3) echo "⚠️ metadata.yaml не появился — запись не состоялась (рекордер не стартовал?)" >&2 ;;
        *) echo "⚠️ НЕ СМОГ перенести запись в $BAG_HOST (rc=$RC_MOVE) — bag лежит в" >&2
           echo "   $OUTPUT_DIR/.rec/scene_bag; забери руками, следующий прогон его сотрёт" >&2 ;;
    esac
    du -sh "$BAG_HOST" 2>/dev/null || true
else
    log "ГОТОВО (без записи: RECORD=0)"
    exit 0
fi

# ── 5. извлечение кадров → JPEG ───────────────────────────────────────────────
if [ "$SKIP_CAM" = "1" ]; then
    log "SKIP_CAM=1 — камера не писалась, извлечение кадров/mp4/заливка пропущены"
    echo "rosbag (только поза + IMU): $BAG_HOST ($(du -sh "$BAG_HOST" 2>/dev/null | cut -f1))"
    log "ГОТОВО"
    exit 0
fi
if [ "$FRAMES" = "0" ]; then
    log "FRAMES=0 — извлечение JPEG-кадров пропущено (mp4 собирается как обычно)"
else
    log "извлечение кадров по пути (шаг ${DIST_M}м, макс ${N_FRAMES})"
    docker exec \
      -e SCENE_N="$N_FRAMES" -e SCENE_DIST_M="$DIST_M" \
      -e SCENE_TOPIC="$TOPIC" -e SCENE_POSE="$POSE_TOPIC" \
      "$NAV" bash -lc "$SRC; python3 /lab/extract_frames.py" | tail -8
fi

# ── 5b. сборка mp4 из ВСЕХ кадров /image_color (полный поток камеры) ───────────
# В отличие от шага 5 (JPEG-выборка по пути), здесь кодируется весь поток камеры
# «как видела камера» за прогон. Пишется в $IMG_HOST → уедет на Drive шагом 6
# вместе с кадрами. FPS считается из sim-штампов (header.stamp) → длительность
# ролика = длительности полёта в sim-времени (не растянута низким RTF).
if [ "$MP4" = "1" ]; then
    log "сборка mp4 из всех кадров $TOPIC"
    docker exec \
      -e SCENE_TOPIC="$TOPIC" -e SCENE_MAXW="$MP4_MAXW" \
      -e SCENE_MP4="/root/sim_ws/output/scene_img/scene.mp4" \
      "$NAV" bash -lc "$SRC; python3 /lab/make_video.py" | tail -3
fi

# ── 6. заливка ТОЛЬКО видео (scene.mp4) на Google Drive ───────────────────────
# Кадры НЕ заливаем — только scene.mp4. Папка проекта на Drive уже очищена в
# начале прогона (шаг 0), так что тут только копируем видео.
log "заливка видео (scene.mp4) на Google Drive"
echo "rosbag оставлен для анализа: $BAG_HOST ($(du -sh "$BAG_HOST" 2>/dev/null | cut -f1))"

if [ "$GDRIVE_UP" = "1" ]; then
  if ! rclone listremotes 2>/dev/null | grep -qx "${GDRIVE_REMOTE}:"; then
    echo "ОШИБКА: remote '${GDRIVE_REMOTE}:' не найден в rclone.conf."
    echo "  Настрой rclone (rclone config) или положи rclone.conf в ~/.config/rclone/."
    echo "  Видео осталось локально: $IMG_HOST/scene.mp4"
    exit 1
  fi
  if [ ! -f "$IMG_HOST/scene.mp4" ]; then
    echo "⚠️ $IMG_HOST/scene.mp4 нет (MP4=$MP4?) — заливать нечего, пропускаю."
  else
    echo "Заливаю ТОЛЬКО видео $IMG_HOST/scene.mp4 → ${GDRIVE_REMOTE}:${GDRIVE_DIR}"
    rclone copy "$IMG_HOST/scene.mp4" "${GDRIVE_REMOTE}:${GDRIVE_DIR}" --progress
    echo "Готово. Ссылка на видео:"
    rclone link "${GDRIVE_REMOTE}:${GDRIVE_DIR}/scene.mp4" 2>/dev/null || true
  fi
else
  echo "GDRIVE_UP=0 — видео в $IMG_HOST/scene.mp4, заливка пропущена."
fi

log "ГОТОВО"
