#!/usr/bin/env bash
#
# capture_scene.sh — единый прогон для диагностики того, что видит камера дрона.
#
# Делает ровно то, что отлаживалось вручную:
#   1. перезапуск стека (make restart-all) + ожидание сборки (make wait)
#   2. арм + взлёт (make arm ALT=...)
#   3. запись rosbag /image_color параллельно с облётом квадрата (fly_square.py)
#   4. остановка записи + посадка
#   5. извлечение N кадров с шагом 1с → JPEG (extract_frames.py в контейнере)
#   6. копирование в src/lab/scene_img + git add/commit/push
#
# Запускать С ХОСТА из любого места:  bash src/lab/capture_scene.sh
#
set -euo pipefail

# ── параметры (можно переопределить через env) ────────────────────────────────
ALT="${ALT:-4}"                 # высота взлёта, м
SIZE="${SIZE:-5}"               # сторона квадрата, м
SIDE_TIME="${SIDE_TIME:-8}"     # время на сторону, с
FLY_SECONDS="${FLY_SECONDS:-55}" # сколько секунд летать/писать bag
N_FRAMES="${N_FRAMES:-30}"      # сколько кадров вытащить
STEP_NS="${STEP_NS:-1000000000}" # шаг между кадрами, нс (1с)
TOPIC="${TOPIC:-/image_color}"  # топик камеры
NAV="${NAV:-p1317_nav}"         # имя nav-контейнера
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

# окружение ROS внутри контейнера (overlay нужен для cv_bridge)
SRC='source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash'

log() { echo -e "\n=== $* ==="; }

# ── 0. чистка старых rosbag'ов (свежий bag этого прогона оставляем для анализа) ─
log "0/6 удаляю старые rosbag'ы в $OUTPUT_DIR"
rm -rf "$OUTPUT_DIR"/scene_bag*

# ── 1. перезапуск стека ───────────────────────────────────────────────────────
log "1/6 перезапуск стека"
make -C "$SIM_DIR" restart-all 2>&1 | tail -3
make -C "$SIM_DIR" wait

# ── 2. арм + взлёт ────────────────────────────────────────────────────────────
log "2/6 арм + взлёт на ${ALT}м"
sleep 8
make -C "$SIM_DIR" arm ALT="$ALT"

# ── 3. запись rosbag + облёт квадрата ─────────────────────────────────────────
log "3/6 запись rosbag $TOPIC + облёт квадрата (${FLY_SECONDS}с)"
docker exec "$NAV" bash -lc "$SRC; cd /root/sim_ws/output && exec ros2 bag record -o scene_bag $TOPIC" &
sleep 3
docker exec "$NAV" bash -lc "$SRC; exec python3 /lab/fly_square.py --size $SIZE --alt $ALT --side-time $SIDE_TIME" &
sleep "$FLY_SECONDS"

# ── 4. стоп записи + посадка ──────────────────────────────────────────────────
log "4/6 стоп записи + посадка"
docker exec "$NAV" pkill -INT -f "ros2 bag record" || true
sleep 2
docker exec "$NAV" pkill -f "fly_square.py" || true
sleep 2
docker exec "$NAV" bash /lab/land.sh || true
du -sh "$BAG_HOST" 2>/dev/null || true

# ── 5. извлечение кадров → JPEG ───────────────────────────────────────────────
log "5/6 извлечение $N_FRAMES кадров (шаг $((STEP_NS/1000000000))с)"
docker exec \
  -e SCENE_N="$N_FRAMES" -e SCENE_STEP_NS="$STEP_NS" -e SCENE_TOPIC="$TOPIC" \
  "$NAV" bash -lc "$SRC; python3 /lab/extract_frames.py" | tail -5

# ── 6. заливка кадров на Google Drive ─────────────────────────────────────────
log "6/6 заливка кадров на Google Drive"

cat > "$IMG_HOST/README.md" <<'EOF'
# scene_img — кадры камеры дрона из симуляции

Кадры с `/image_color` (camera_node, bgr8, 1280×720), снятые при облёте
квадрата (`fly_square.py`) в мире `mili_fortress`. Шаг ~1с между кадрами.

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
