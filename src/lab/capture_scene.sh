#!/usr/bin/env bash
#
# capture_scene.sh — единый АТОМАРНЫЙ прогон симуляции с хоста, разбитый на фазы.
#
# Фазы (каждую можно выключить флагом — дефолты = «полный прогон», как раньше):
#   1. перезапуск стека (RESTART)         make restart-all + make wait
#   2. арм + взлёт + ОТЧЁТ о высоте        make arm ALT=...   (печатает z vs цель)
#   3. лётная фаза + запись bag (RECORD)   облёт квадрата (FLY=1) ИЛИ висение (FLY=0)
#   4. посадка (LAND)                      make land
#   5. извлечение кадров ПО ПУТИ           extract_frames.py: кадр каждые DIST_M метров
#   6. заливка на Google Drive (GDRIVE_UP) rclone
#
# Зачем фазы: при низком RTF (≈0.05) каждая sim-секунда дорога. Чтобы дёшево
# проверить «взлетает ли дрон вообще», лётную фазу режут (FLY=0 + малый
# FLY_SECONDS), не трогая запись/заливку. Полный анализ — дефолтами.
#
# Запускать С ХОСТА из любого места:  bash src/lab/capture_scene.sh
# Примеры:
#   make capture-scene                                   # полный прогон (дефолты)
#   ALT=10 FLY=0 FLY_SECONDS=10 bash src/lab/capture_scene.sh   # взлёт 10м, висение, запись
#   GDRIVE_UP=0 bash src/lab/capture_scene.sh            # снять кадры локально, без заливки
#
set -euo pipefail

# ── параметры (можно переопределить через env) ────────────────────────────────
RESTART="${RESTART:-1}"         # 1 = перезапуск стека (restart-all+wait); 0 = на живом стеке
ALT="${ALT:-4}"                 # высота взлёта, м
FLY="${FLY:-1}"                 # 1 = облёт квадрата; 0 = висение на месте
SIZE="${SIZE:-5}"               # сторона квадрата, м (при FLY=1)
SIDE_TIME="${SIDE_TIME:-8}"     # время на сторону, с (при FLY=1)
FLY_SECONDS="${FLY_SECONDS:-55}" # длительность лётной фазы/записи bag, с (wall)
RECORD="${RECORD:-1}"           # 1 = писать rosbag /image_color
N_FRAMES="${N_FRAMES:-30}"      # макс. число кадров (0 = без лимита)
DIST_M="${DIST_M:-0.5}"         # шаг выборки кадров по пройденному пути, м
TOPIC="${TOPIC:-/image_color}"  # топик камеры
POSE_TOPIC="${POSE_TOPIC:-/mavros/local_position/pose}" # поза для расчёта пути
LAND="${LAND:-1}"               # 1 = посадка в конце
NAV="${NAV:-p1317_nav}"         # имя nav-контейнера
CPU="${CPU:-}"                  # CPU=1 → GPU-less режим (docker-compose.cpu.yml)
# Разрешение камеры (опционально). Если задано — стек ПЕРЕСОЗДАЁТСЯ (fresh-start),
# т.к. env применяется при создании контейнера, а restart-all его не перечитывает.
# Прокидываем в окружение, чтобы docker compose подставил ${CAMERA_W/H} (см. compose).
CAMERA_W="${CAMERA_W:-}"
CAMERA_H="${CAMERA_H:-}"
[ -n "$CAMERA_W" ] && export CAMERA_W
[ -n "$CAMERA_H" ] && export CAMERA_H
GDRIVE_UP="${GDRIVE_UP:-1}"            # 1 = заливать на Google Drive; 0 = только снять кадры
GDRIVE_REMOTE="${GDRIVE_REMOTE:-gdrive}"      # имя rclone-remote (из rclone.conf)
GDRIVE_DIR="${GDRIVE_DIR:-13.17/scene_img}"   # папка на Drive

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
        RES_NOTE=" → ${CAMERA_W:-?}×${CAMERA_H:-?} (fresh-start, пересоздание)"
    else
        RESET_TARGET=restart-all
        RES_NOTE=""
    fi
    log "1/6 перезапуск стека${CPU:+ (CPU=1)}${RES_NOTE}"
    # чистим старые rosbag'ы (свежий bag этого прогона оставляем для анализа)
    [ "$RECORD" = "1" ] && { echo "  удаляю старые rosbag'ы в $OUTPUT_DIR"; rm -rf "$OUTPUT_DIR"/scene_bag*; }
    "${MK[@]}" "$RESET_TARGET" 2>&1 | tail -3
    "${MK[@]}" wait
else
    log "1/6 перезапуск ПРОПУЩЕН (RESTART=0) — прогон на живом стеке"
    echo "  ⚠️ без рестарта возможен рассинхрон состояния (см. дисциплину прогона в CLAUDE.md)"
    if [ -n "$CAMERA_W" ] || [ -n "$CAMERA_H" ]; then
        echo "  ⚠️ CAMERA_W/H заданы, но при RESTART=0 НЕ применятся (нужен fresh-start)"
    fi
    [ "$RECORD" = "1" ] && { echo "  удаляю старые rosbag'ы в $OUTPUT_DIR"; rm -rf "$OUTPUT_DIR"/scene_bag*; }
fi

# ── 2. арм + взлёт + отчёт о высоте ───────────────────────────────────────────
log "2/6 арм + взлёт на ${ALT}м"
sleep 8                          # дать EKF прогреться/получить origin
"${MK[@]}" arm ALT="$ALT"

# Явный ответ на вопрос «взлетел или нет»: читаем фактическую z и сверяем с целью.
Z=$(docker exec "$NAV" bash -lc \
    "$SRC; timeout 10 ros2 topic echo --once --field pose.position.z /mavros/local_position/pose 2>/dev/null" \
    | head -1 || true)
if [ -n "$Z" ] && awk "BEGIN{exit !($Z >= $ALT * 0.9)}" 2>/dev/null; then
    echo "  ВЗЛЁТ OK: z=${Z}м (цель ${ALT}м)"
else
    echo "  ⚠️ ВЗЛЁТ НЕ ПОДТВЕРЖДЁН: z=${Z:-?}м (цель ${ALT}м) — дрон не набрал высоту"
fi

# ── 3. лётная фаза + запись rosbag ────────────────────────────────────────────
if [ "$RECORD" = "1" ]; then
    log "3/6 запись rosbag $TOPIC + $POSE_TOPIC (${FLY_SECONDS}с)"
    docker exec "$NAV" bash -lc "$SRC; cd /root/sim_ws/output && exec ros2 bag record -o scene_bag $TOPIC $POSE_TOPIC" &
    sleep 3
else
    log "3/6 лётная фаза (${FLY_SECONDS}с, без записи)"
fi

if [ "$FLY" = "1" ]; then
    echo "  облёт квадрата ${SIZE}×${SIZE}м"
    docker exec "$NAV" bash -lc "$SRC; exec python3 /lab/fly_square.py --size $SIZE --alt $ALT --side-time $SIDE_TIME" &
else
    echo "  висение на ${ALT}м (FLY=0)"
fi
sleep "$FLY_SECONDS"

# ── 4. стоп записи/полёта + посадка ───────────────────────────────────────────
log "4/6 стоп лётной фазы${LAND:+ + посадка}"
[ "$RECORD" = "1" ] && { docker exec "$NAV" pkill -INT -f "ros2 bag record" || true; sleep 2; }
[ "$FLY" = "1" ]    && { docker exec "$NAV" pkill -f "fly_square.py" || true; sleep 2; }
if [ "$LAND" = "1" ]; then
    docker exec "$NAV" bash /lab/land.sh || true
fi
[ "$RECORD" = "1" ] && du -sh "$BAG_HOST" 2>/dev/null || true

# Дальше — только если что-то записали.
if [ "$RECORD" != "1" ]; then
    log "ГОТОВО (без записи: RECORD=0)"
    exit 0
fi

# ── 5. извлечение кадров → JPEG ───────────────────────────────────────────────
log "5/6 извлечение кадров по пути (шаг ${DIST_M}м, макс ${N_FRAMES})"
docker exec \
  -e SCENE_N="$N_FRAMES" -e SCENE_DIST_M="$DIST_M" \
  -e SCENE_TOPIC="$TOPIC" -e SCENE_POSE="$POSE_TOPIC" \
  "$NAV" bash -lc "$SRC; python3 /lab/extract_frames.py" | tail -8

# ── 6. заливка кадров на Google Drive ─────────────────────────────────────────
log "6/6 заливка кадров на Google Drive"

cat > "$IMG_HOST/README.md" <<'EOF'
# scene_img — кадры камеры дрона из симуляции

Кадры с `/image_color` (camera_node, bgr8, 1280×720), снятые в мире
`mili_fortress`. Выборка ПО ПРОЙДЕННОМУ ПУТИ: первый кадр на старте, далее
каждые ~0.5м пути дрона (имя файла несёт дистанцию: `frame_NN_DDD.DDm.jpg`).

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
  echo "Заливаю $IMG_HOST → ${GDRIVE_REMOTE}:${GDRIVE_DIR}"
  rclone copy "$IMG_HOST" "${GDRIVE_REMOTE}:${GDRIVE_DIR}" --progress
  echo "Готово. Ссылка на папку:"
  rclone link "${GDRIVE_REMOTE}:${GDRIVE_DIR}" 2>/dev/null || true
else
  echo "GDRIVE_UP=0 — кадры в $IMG_HOST, заливка пропущена."
fi

log "ГОТОВО"
