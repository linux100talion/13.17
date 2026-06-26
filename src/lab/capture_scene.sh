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
TOPIC="${TOPIC:-/image_color}"  # топик камеры
POSE_TOPIC="${POSE_TOPIC:-/mavros/local_position/pose}" # поза для расчёта пути
TOPICS_EXTRA="${TOPICS_EXTRA:-}" # доп. топики в bag (через пробел), напр. диагностика IMU
SKIP_CAM="${SKIP_CAM:-0}"       # 1 = НЕ писать/не обрабатывать /image_color: лёгкий bag
                                #     (мегабайты) для анализа только по IMU/позе, напр. FFT
NAV="${NAV:-p1317_nav}"         # имя nav-контейнера
CPU="${CPU:-}"                  # CPU=1 → GPU-less режим (docker-compose.cpu.yml)
GDRIVE_UP="${GDRIVE_UP:-1}"            # 1 = заливать на Google Drive; 0 = только снять кадры
GDRIVE_REMOTE="${GDRIVE_REMOTE:-gdrive}"      # имя rclone-remote (из rclone.conf)
GDRIVE_DIR="${GDRIVE_DIR:-13.17/scene_img}"   # папка на Drive

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
        arm|land|disarm) ;;
        takeoff|hover|square)
            # опциональный числовой аргумент: если следующий токен — число, он наш
            # (takeoff=ALT, hover=SIM_SEC, square=число кругов)
            nxt="${SEQ[$((i+1))]:-}"
            [[ "$nxt" =~ ^[0-9]+(\.[0-9]+)?$ ]] && i=$((i+1))
            ;;
        *)
            echo "ОШИБКА: неизвестная команда '$cmd'." >&2
            echo "Допустимо: arm, takeoff [ALT], hover [SIM_SEC], square [LOOPS], land, disarm." >&2
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
MK=(make -C "$SIM_DIR")
[ "$CPU" = "1" ] && MK+=(CPU=1)

# окружение ROS внутри контейнера (overlay нужен для cv_bridge)
SRC='source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash'

log() { echo -e "\n=== $* ==="; }

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
    log "перезапуск стека${CPU:+ (CPU=1)}${RES_NOTE}"
    # чистим старые rosbag'ы (свежий bag этого прогона оставляем для анализа)
    [ "$RECORD" = "1" ] && { echo "  удаляю старые rosbag'ы в $OUTPUT_DIR"; rm -rf "$OUTPUT_DIR"/scene_bag*; }
    "${MK[@]}" "$RESET_TARGET" 2>&1 | tail -3
    "${MK[@]}" wait
else
    log "перезапуск ПРОПУЩЕН (RESTART=0) — прогон на живом стеке"
    echo "  ⚠️ без рестарта возможен рассинхрон состояния (см. дисциплину прогона в CLAUDE.md)"
    if [ -n "$CAMERA_W" ] || [ -n "$CAMERA_H" ]; then
        echo "  ⚠️ разрешение задано, но при RESTART=0 НЕ применится (нужен fresh-start)"
    fi
    [ "$RECORD" = "1" ] && { echo "  удаляю старые rosbag'ы в $OUTPUT_DIR"; rm -rf "$OUTPUT_DIR"/scene_bag*; }
fi

# ── 2. старт записи rosbag (вокруг всей последовательности команд) ─────────────
if [ "$RECORD" = "1" ]; then
    log "старт записи rosbag $TOPIC + $POSE_TOPIC${TOPICS_EXTRA:+ + $TOPICS_EXTRA}"
    docker exec "$NAV" bash -lc "$SRC; cd /root/sim_ws/output && exec ros2 bag record -o scene_bag $TOPIC $POSE_TOPIC $TOPICS_EXTRA" &
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
    docker exec \
      -e ARM_SIM_BUDGET="${ARM_SIM_BUDGET:-}" -e ARM_WALL_CAP="${ARM_WALL_CAP:-}" \
      "$NAV" bash /lab/"$cmd".sh $arg
    i=$((i+1))
done

# ── 4. стоп записи ────────────────────────────────────────────────────────────
if [ "$RECORD" = "1" ]; then
    log "стоп записи rosbag"
    docker exec "$NAV" pkill -INT -f "ros2 bag record" || true
    sleep 2
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
log "извлечение кадров по пути (шаг ${DIST_M}м, макс ${N_FRAMES})"
docker exec \
  -e SCENE_N="$N_FRAMES" -e SCENE_DIST_M="$DIST_M" \
  -e SCENE_TOPIC="$TOPIC" -e SCENE_POSE="$POSE_TOPIC" \
  "$NAV" bash -lc "$SRC; python3 /lab/extract_frames.py" | tail -8

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

# ── 6. заливка кадров (+ mp4) на Google Drive ─────────────────────────────────
log "заливка кадров на Google Drive"

cat > "$IMG_HOST/README.md" <<'EOF'
# scene_img — кадры камеры дрона из симуляции

Кадры с `/image_color` (camera_node, bgr8), снятые в мире `mili_fortress`.
Выборка ПО ПРОЙДЕННОМУ ПУТИ: первый кадр на старте, далее каждые ~0.5м пути
дрона (имя файла несёт дистанцию: `frame_NN_DDD.DDm.jpg`).

Назначение: диагностика инициализации VINS — посмотреть, что реально видит
камера. Снимаются скриптом `src/lab/capture_scene.sh`
(извлечение кадров — `src/lab/extract_frames.py`).
EOF

echo "rosbag оставлен для анализа: $BAG_HOST ($(du -sh "$BAG_HOST" 2>/dev/null | cut -f1))"

if [ "$GDRIVE_UP" = "1" ]; then
  if ! rclone listremotes 2>/dev/null | grep -qx "${GDRIVE_REMOTE}:"; then
    echo "ОШИБКА: remote '${GDRIVE_REMOTE}:' не найден в rclone.conf."
    echo "  Настрой rclone (rclone config) или положи rclone.conf в ~/.config/rclone/."
    echo "  Кадры остались локально: $IMG_HOST"
    exit 1
  fi
  echo "Заливаю $IMG_HOST → ${GDRIVE_REMOTE}:${GDRIVE_DIR} (кадры + scene.mp4)"
  rclone copy "$IMG_HOST" "${GDRIVE_REMOTE}:${GDRIVE_DIR}" --progress
  echo "Готово. Ссылка на папку:"
  rclone link "${GDRIVE_REMOTE}:${GDRIVE_DIR}" 2>/dev/null || true
  if [ "$MP4" = "1" ] && [ -f "$IMG_HOST/scene.mp4" ]; then
    echo "Ссылка на видео:"
    rclone link "${GDRIVE_REMOTE}:${GDRIVE_DIR}/scene.mp4" 2>/dev/null || true
  fi
else
  echo "GDRIVE_UP=0 — кадры в $IMG_HOST, заливка пропущена."
fi

log "ГОТОВО"
