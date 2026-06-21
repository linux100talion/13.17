#!/usr/bin/env bash
# ============================================================================
# Снимает несколько кадров из Gazebo мира и сохраняет PNG в worlds/preview/.
#
# Запуск (из корня репозитория):
#   bash docker/sim/scripts/capture_frames.sh [N_FRAMES]
#
# N_FRAMES — сколько PNG сохранить (default 5).
#
# Что делает скрипт:
#   1. Поднимает Gazebo + ros_gz_bridge внутри p1317_simulator (если ещё не запущены).
#   2. Ждёт пока /camera/image_raw начнёт публиковаться.
#   3. Записывает bag ровно столько, чтобы накопить N_FRAMES кадров.
#   4. Внутри контейнера распаковывает bag → PNG через ros2bag + python.
#   5. Копирует PNG на хост в docker/sim/worlds/preview/.
#
# Зависимости на хосте: docker (контейнер p1317_simulator должен быть запущен).
# ============================================================================
set -euo pipefail

N_FRAMES="${1:-5}"
REPO_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
PREVIEW_DIR="$REPO_ROOT/docker/sim/worlds/preview"
BAG_PATH="/root/output/capture_bag"

echo "=== capture_frames.sh: захват $N_FRAMES кадров из Gazebo ==="

# --- 1. Старт Gazebo + ros_gz_bridge (idempotent) ---
echo "[1/5] Запускаем Gazebo + ros_gz_bridge..."
docker exec -i p1317_simulator bash -s < "$REPO_ROOT/docker/sim/scripts/sim_up.sh"

# --- 2. Ждём публикацию /camera/image_raw (max 60 секунд) ---
echo "[2/5] Ожидаем /camera/image_raw..."
docker exec p1317_simulator bash -c '
    source /opt/ros/humble/setup.bash
    for i in $(seq 1 60); do
        count=$(ros2 topic info /camera/image_raw 2>/dev/null | grep "Publisher count" | grep -v "^0$" | awk "{print \$NF}")
        if [ "${count:-0}" -gt 0 ]; then
            echo "  /camera/image_raw готов (попытка $i)"
            exit 0
        fi
        echo "  ожидание... ($i/60)"
        sleep 1
    done
    echo "ОШИБКА: /camera/image_raw так и не появился за 60 сек" >&2
    exit 1
'

# --- 3+4. Прямой захват кадров через Python-подписчик (без bag) ---
echo "[3/5] Прямой захват $N_FRAMES кадров из /camera/image_raw..."
docker exec p1317_simulator bash -c "
    source /opt/ros/humble/setup.bash
    python3 - $N_FRAMES <<'PYEOF'
import sys, pathlib, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2, numpy as np
except ImportError:
    import subprocess; subprocess.check_call(['pip3', 'install', '-q', 'opencv-python-headless'])
    import cv2, numpy as np

n_target = int(sys.argv[1]) if len(sys.argv) > 1 else 5
out_dir  = pathlib.Path('/root/output/preview_frames')
out_dir.mkdir(parents=True, exist_ok=True)

class Capturer(Node):
    def __init__(self):
        super().__init__('frame_capturer')
        self.saved = 0
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        self.get_logger().info(f'Ожидаю {n_target} кадров...')

    def cb(self, msg):
        if self.saved >= n_target:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8', 'RGB8'):
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        fname = out_dir / f'frame_{self.saved:04d}.png'
        cv2.imwrite(str(fname), arr)
        print(f'  сохранён {fname}  ({msg.width}x{msg.height})', flush=True)
        self.saved += 1

rclpy.init()
node = Capturer()
deadline = time.time() + 30
while rclpy.ok() and node.saved < n_target and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)

if node.saved < n_target:
    print(f'ВНИМАНИЕ: захвачено только {node.saved}/{n_target} кадров за 30 с', flush=True)
else:
    print(f'Готово: {node.saved} кадров -> {out_dir}', flush=True)

node.destroy_node()
rclpy.shutdown()
PYEOF
"
echo "[4/5] (bag не используется — прямой захват)"

# --- 5. Копируем PNG на хост ---
echo "[5/5] Копируем PNG -> $PREVIEW_DIR"
mkdir -p "$PREVIEW_DIR"
# Очищаем старые кадры
rm -f "$PREVIEW_DIR"/frame_*.png

docker cp p1317_simulator:/root/output/preview_frames/. "$PREVIEW_DIR/"

echo ""
echo "=== Готово! Кадры сохранены в: ==="
ls -lh "$PREVIEW_DIR"/frame_*.png 2>/dev/null || echo "  (пусто)"
