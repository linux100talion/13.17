#!/usr/bin/env bash
#
# calib_run.sh — ОДИН ИМЕНОВАННЫЙ калибровочный прогон плана src/control/ToDo.md.
#
# Обёртка над capture_scene.sh, которая решает три задачи, ради которых иначе
# приходится копипастить длинную строку env:
#   1) ИМЯ. capture_scene при GDRIVE_UP=1 ЧИСТИТ всю папку 13.17 на Drive и всегда
#      льёт как scene.mp4 → каждый следующий прогон затирает предыдущий. Здесь
#      capture_scene зовётся с GDRIVE_UP=0, а заливка идёт своя: copyto с именем
#      прогона в отдельную папку (CALIB_GDIR, по умолчанию 13.17/calib). Ничего
#      не чистим — архив прогонов копится.
#   2) BAG. Свежий scene_bag переименовывается в <NAME>_bag, иначе следующий
#      прогон его сотрёт (capture_scene чистит output/scene_bag* на старте).
#   3) МЕТА. Рядом с bag'ом ложится <NAME>.env — все BS_*, с которыми он снят.
#      Без этого через неделю не восстановить, что именно измерялось.
#
# Запуск (с хоста, из любого места):
#   NAME=A1_pitch_sign BS_STAB=none BS_MISSION="climb3,mv_bkwd6,land" \
#     BS_MV_LEVEL=0.375 bash src/lab/calib_run.sh
#
#   # или имя первым позиционным аргументом:
#   BS_STAB=none BS_MISSION="climb3,hover20,land" bash src/lab/calib_run.sh A4_bias
#
#   DRY_RUN=1 …   — показать команду и env, стек не трогать (ревью перед запуском)
#
# Прогон ДОЛГИЙ (15–20 мин стенных на CPU-боксе) — запускать отвязанно, чтобы
# пережил обрыв ssh:
#   cd /root/13.17 && setsid nohup env NAME=A1_pitch_sign BS_STAB=none … \
#     bash src/lab/calib_run.sh > /root/run_A1.log 2>&1 < /dev/null & disown
#
set -euo pipefail

NAME="${1:-${NAME:-}}"
if [ -z "$NAME" ]; then
    echo "ОШИБКА: не задано имя прогона (NAME=… или 1-й аргумент)." >&2
    echo "Пример: NAME=A1_pitch_sign BS_STAB=none BS_MISSION='climb3,mv_bkwd6,land' \\" >&2
    echo "        bash src/lab/calib_run.sh" >&2
    exit 2
fi
# capture_scene чистит output/scene_bag* на старте — имя 'scene*' снесло бы само себя
case "$NAME" in
    scene*) echo "ОШИБКА: имя не может начинаться с 'scene' (конфликт с scene_bag*)." >&2; exit 2 ;;
    */*|*' '*) echo "ОШИБКА: имя без пробелов и слэшей ('$NAME')." >&2; exit 2 ;;
esac

# ── параметры (env) ───────────────────────────────────────────────────────────
CPU="${CPU:-1}"
RES="${RES:-960x540}"                   # разрешение камеры; непустое → fresh-start
FRESH="${FRESH:-1}"                     # 0 = без WxH → быстрый restart-all
CMD="${CMD:-bootstrap_arch2}"           # лётная команда capture_scene
GDRIVE_UP="${GDRIVE_UP:-1}"
GDRIVE_REMOTE="${GDRIVE_REMOTE:-gdrive}"
CALIB_GDIR="${CALIB_GDIR:-13.17/calib}" # папка на Drive; НЕ чистится (архив прогонов)
DRY_RUN="${DRY_RUN:-0}"

# Топики калибровки. Команда идёт в /flow_dbg (x=roll_off) и /flow_dbg2 (x=pitch_off) —
# оба sim-штампованы; истина по углам/скорости — /model/iris_cam/odometry; оценка
# углов у борта — /mavros/imu/data (нужна, чтобы отличить смещение объекта от смещения
# ОЦЕНКИ, см. ToDo факт 2). /flow_dbg3 ещё не публикуется (очередь п.9) — записи не мешает.
# /flow_dbg5 — УСТАВКА продольного холдера (когда стик двигает точку удержания): из
# телеметрии не восстановить, это интеграл команды по КАДРАМ. Без неё «борт не поехал»
# и «уставка не поехала» в разборе неразличимы. /flow_dbg6 — то же для КУРСА: без неё
# свип Y2 пришлось восстанавливать офлайн интегрированием /flow_dbg2, а там неизвестен
# покадровый fdt (сглаживание медианой по 5 кадрам склеивает соседние значения) —
# калибровка градусов повисла на догадке о частоте кадров.
TOPICS_EXTRA="${TOPICS_EXTRA:-/flow_dbg /flow_dbg2 /flow_dbg3 /flow_dbg4 /flow_dbg5 /flow_dbg6 /flow_dbg7 /flow_dbg8 /model/iris_cam/odometry /mavros/imu/data /mavros/global_position/rel_alt /gz_imu/data_flu /mavros/imu/data_raw}"

# Бюджеты фаз — как в эталонном прогоне bootstrap_arch2 (CPU-бокс, низкий RTF).
export BS_THROTTLE_CLIMB="${BS_THROTTLE_CLIMB:-1800}"
export BS_MODE_BUDGET="${BS_MODE_BUDGET:-80}"
export BS_ARM_BUDGET="${BS_ARM_BUDGET:-80}"
export BS_CLIMB_BUDGET="${BS_CLIMB_BUDGET:-120}"
export BS_LAND_BUDGET="${BS_LAND_BUDGET:-180}"
# Зрение поднимаем ВСЕГДА, даже когда демпфера в стеке нет: перцепт пишется в
# /flow_dbg* и меряется на том же движении, ради которого прогон и затеян.
export BS_FLOW_OBS="${BS_FLOW_OBS:-1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CAPTURE="$REPO_ROOT/src/lab/capture_scene.sh"
OUTPUT_DIR="$REPO_ROOT/docker/sim/output"
MP4_HOST="$OUTPUT_DIR/scene_img/scene.mp4"
BAG_SRC="$OUTPUT_DIR/scene_bag"
BAG_DST="$OUTPUT_DIR/${NAME}_bag"

log() { echo -e "\n############ $* ############"; }

# ── мета прогона: все BS_* + ключевые параметры обвязки ───────────────────────
META="$OUTPUT_DIR/${NAME}.env"
mkdir -p "$OUTPUT_DIR"
{
    echo "# calib_run: $NAME"
    echo "# commit: $(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo '?')"
    echo "CMD=$CMD"
    echo "RES=$RES  FRESH=$FRESH  CPU=$CPU"
    echo "TOPICS_EXTRA=$TOPICS_EXTRA"
    env | grep -E '^BS_' | sort
} > "$META"

log "КАЛИБРОВОЧНЫЙ ПРОГОН '$NAME' | $CMD | res=$RES fresh=$FRESH Drive=$GDRIVE_UP"
cat "$META"

if [ "$BAG_DST" != "$BAG_SRC" ] && [ -e "$BAG_DST" ]; then
    echo -e "\n⚠️ $BAG_DST уже есть — прогон с этим именем уже делался."
    echo "   Переименуй старый или задай другое NAME. Выхожу, чтобы не потерять данные."
    exit 3
fi

# ── 1. сам прогон (атомарный: рестарт → wait → полёт → bag → mp4) ─────────────
cs_args=()
[ "$FRESH" = "1" ] && cs_args+=("$RES")
cs_args+=("$CMD")

if [ "$DRY_RUN" = "1" ]; then
    echo -e "\n[DRY] CPU=$CPU GDRIVE_UP=0 MP4=1 N_FRAMES=1 \\"
    echo "        TOPICS_EXTRA=\"$TOPICS_EXTRA\" bash $CAPTURE ${cs_args[*]}"
    echo "[DRY] mv $BAG_SRC $BAG_DST"
    echo "[DRY] rclone copyto $MP4_HOST ${GDRIVE_REMOTE}:${CALIB_GDIR}/${NAME}.mp4"
    exit 0
fi

# GDRIVE_UP=0 инлайн: capture_scene не трогает Drive (не чистит 13.17, не льёт
# scene.mp4) — заливаем сами, ниже, под именем прогона.
CPU="$CPU" GDRIVE_UP=0 MP4=1 N_FRAMES=1 TOPICS_EXTRA="$TOPICS_EXTRA" \
    bash "$CAPTURE" "${cs_args[@]}"

# ── 2. bag под именем прогона (иначе следующий прогон его сотрёт) ─────────────
if [ -d "$BAG_SRC" ]; then
    mv "$BAG_SRC" "$BAG_DST"
    log "bag: $BAG_DST ($(du -sh "$BAG_DST" | cut -f1))"
else
    echo "⚠️ $BAG_SRC нет — запись не состоялась (RECORD=0 или прогон упал)"
fi

# ── 3. видео на Drive под именем прогона (папка НЕ чистится — архив) ──────────
if [ "$GDRIVE_UP" = "1" ]; then
    if ! rclone listremotes 2>/dev/null | grep -qx "${GDRIVE_REMOTE}:"; then
        echo "⚠️ rclone remote '${GDRIVE_REMOTE}:' не настроен — видео осталось в $MP4_HOST"
    elif [ ! -f "$MP4_HOST" ]; then
        echo "⚠️ $MP4_HOST нет (взлёта не было?) — заливать нечего"
    else
        log "заливка видео → ${GDRIVE_REMOTE}:${CALIB_GDIR}/${NAME}.mp4"
        rclone copyto "$MP4_HOST" "${GDRIVE_REMOTE}:${CALIB_GDIR}/${NAME}.mp4" --progress
        rclone link "${GDRIVE_REMOTE}:${CALIB_GDIR}/${NAME}.mp4" 2>/dev/null || true
    fi
else
    echo "GDRIVE_UP=0 — видео в $MP4_HOST, заливка пропущена"
fi

log "ГОТОВО: $NAME"
