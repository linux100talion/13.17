#!/usr/bin/env bash
#
# yaw_tune_sweep.sh — СВИП тюнинга визуального YAW-hold (фаза 2) серией ЧИСТЫХ
# атомарных прогонов. Каждый прогон = один `capture_scene.sh … liftland` с одной
# тройкой гейнов (kp, ki, smooth); после прогона считаем метрику `yaw_check.py`
# (СКО/размах/дрейф курса по ground-truth Gazebo) и заливаем видео на Google
# Drive под именем, отражающим ПОРЯДОК прогона в плане + его параметры.
#
# ── Что регулируем (см. src/lab/CLAUDE.md, alt_hold_bootstrap.py:292) ──────────
# YAW-hold = PI по yaw-СКОРОСТИ (уставка 0, «не вращаться»):
#   BS_YAWH_KP — P по yaw-скорости (демпфер yaw-rate; = D по курсу)
#   BS_YAWH_KI — I по yaw-скорости, ∫yf=курс → ДЕРЖИТ курс (= P по курсу)
#   BS_YAWH_SMOOTH — окно медианы yaw_flow (шум↓, но лаг↑; sm=1 = сглаживание ВЫКЛ)
# Знак (BS_YAWH_OSIGN=1) уже подтверждён (спина нет) — НЕ свипаем.
#
# ── Логика плана (bring-up: сначала демпфер, потом restoring) ──────────────────
#   Фаза 0 — база: воспроизвести текущую раскачку (kp=16 ki=1 sm=1) — число, что бьём.
#   Фаза 1 — демпфер: чистый демпфер yaw-rate (ki=0, sm=3), свип kp ∈ {3,6,9}.
#   Фаза 2 — сглаживание вокруг kp*: лаг vs шум, sm ∈ {1,5} (sm=3 снят в фазе 1).
#   Фаза 3 — restoring: поверх kp*+sm* добавляем ki ∈ {0.5,1,2} — убираем дрейф.
# Фазы 2–3 ЗАЯКОРЕНЫ на kp=6 / sm=3 (правь CONFIGS ниже, если победитель фазы 1
# иной, и перезапусти). Финальный «подтверждающий» прогон делаем вручную по итогам.
#
# ── Как обходим перезатирание видео ───────────────────────────────────────────
# capture_scene.sh при GDRIVE_UP=1 ЧИСТИТ всю папку 13.17 на Drive в начале КАЖДОГО
# прогона и льёт всегда как scene.mp4. Поэтому тут capture_scene зовём с GDRIVE_UP=0
# (Drive не трогается, mp4 собирается локально), а заливку с УНИКАЛЬНЫМ именем в
# отдельную папку 13.17/yaw_tune/ делаем сами (rclone copyto). Папку чистим ОДИН раз
# в начале свипа.
#
# ── Запуск (с хоста, из любого места) ─────────────────────────────────────────
#   bash src/lab/yaw_tune_sweep.sh            # весь свип
#   DRY_RUN=1 bash src/lab/yaw_tune_sweep.sh  # показать команды, не запуская (ревью)
#   GDRIVE_UP=0 bash src/lab/yaw_tune_sweep.sh # без заливки (видео/CSV только локально)
#
# Env (переопределяемые): BS_ALT(3) BS_HOLD_SEC(25) SAFE_SEC(18) CPU(1) GDRIVE_UP(1)
#   GDRIVE_REMOTE(gdrive) YAW_GDIR(13.17/yaw_tune) RES(960x540).
# ⚠️ ~9 прогонов × (рестарт+climb+hold) — на CPU-боксе (RTF низкий) это ~1.5–2 ч.
#
set -euo pipefail

# ── список прогонов: "label:KP:KI:SMOOTH" (порядок = порядок в плане) ──────────
CONFIGS=(
  # Фаза 0 — БАЗА: текущая раскачка (сырой сигнал sm=1, задран kp=16). Эталон, что бьём.
  "baseline:16:1:1"
  # Фаза 1 — ДЕМПФЕР (ki=0, sm=3): чистый демпфер yaw-rate. Ищем kp с мин. СКО без чаттера.
  "p1damp:3:0:3"
  "p1damp:6:0:3"
  "p1damp:9:0:3"
  # Фаза 2 — СГЛАЖИВАНИЕ вокруг kp=6 (ki=0): sm меньше = лаг↓/шум↑; sm=3 уже снят в фазе 1.
  "p2smooth:6:0:1"
  "p2smooth:6:0:5"
  # Фаза 3 — RESTORING (kp=6, sm=3): растим ki, пока дрейф→0, но до возврата раскачки.
  "p3rest:6:0.5:3"
  "p3rest:6:1:3"
  "p3rest:6:2:3"
)

# ── параметры (env) ───────────────────────────────────────────────────────────
CPU="${CPU:-1}"
RES="${RES:-960x540}"                 # разрешение (fresh-start только на 1-м прогоне)
BS_ALT="${BS_ALT:-3}"
BS_HOLD_SEC="${BS_HOLD_SEC:-25}"      # держим уровень, sim-сек (окно yaw_check внутри него)
SAFE_SEC="${SAFE_SEC:-18}"            # окно оценки yaw_check от взлёта, sim-сек
GDRIVE_UP="${GDRIVE_UP:-1}"
GDRIVE_REMOTE="${GDRIVE_REMOTE:-gdrive}"
YAW_GDIR="${YAW_GDIR:-13.17/yaw_tune}"
DRY_RUN="${DRY_RUN:-0}"
NAV="${NAV:-p1317_nav}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CAPTURE="$REPO_ROOT/src/lab/capture_scene.sh"
OUTPUT_DIR="$REPO_ROOT/docker/sim/output"
MP4_HOST="$OUTPUT_DIR/scene_img/scene.mp4"
CSV="$OUTPUT_DIR/yaw_tune.csv"
SRC='source /opt/ros/humble/setup.bash'

log() { echo -e "\n############ $* ############"; }

# ── фиксированная база env для КАЖДОГО capture_scene (не меняется между прогонами) ─
# TOPICS_EXTRA — ground-truth одометрия (оракул yaw_check). RECORD=1 (default) —
# нужен bag для yaw_check. Заливку из capture_scene ГЛУШИМ инлайн на его вызове
# (GDRIVE_UP=0 MP4=1), чтобы он не чистил Drive и не лил scene.mp4 — это делаем сами
# с уникальным именем. НЕ экспортируем GDRIVE_UP тут: это МОЙ флаг (purge/upload).
export_base_env() {
  export CPU BS_ALT BS_HOLD_SEC
  export BS_YAWHOLD=1 BS_GZHOLD=1 BS_YAWH_OSIGN=1
  export BS_THROTTLE_CLIMB=1800 BS_MODE_BUDGET=80 BS_ARM_BUDGET=80 \
         BS_CLIMB_BUDGET=120 BS_LAND_BUDGET=180
  export TOPICS_EXTRA="/model/iris_cam/odometry"
}

# ── очистка папки свипа на Drive (ОДИН раз) ───────────────────────────────────
purge_drive() {
  [ "$GDRIVE_UP" = "1" ] || { echo "GDRIVE_UP=0 — заливка выключена, чистку Drive пропускаю"; return; }
  if rclone listremotes 2>/dev/null | grep -qx "${GDRIVE_REMOTE}:"; then
    echo "Drive: чищу ${GDRIVE_REMOTE}:${YAW_GDIR} (только папку свипа)"
    [ "$DRY_RUN" = "1" ] || rclone purge "${GDRIVE_REMOTE}:${YAW_GDIR}" 2>/dev/null || true
  else
    echo "⚠️ rclone remote '${GDRIVE_REMOTE}:' не настроен — заливка будет пропущена"
    GDRIVE_UP=0
  fi
}

# ── заливка одного видео с уникальным именем ──────────────────────────────────
upload_video() {  # $1 = имя файла назначения
  local name="$1"
  [ "$GDRIVE_UP" = "1" ] || return 0
  if [ ! -f "$MP4_HOST" ]; then
    echo "⚠️ $MP4_HOST нет — видео не собралось (взлёта не было?), заливать нечего"
    return 0
  fi
  echo "Заливаю $MP4_HOST → ${GDRIVE_REMOTE}:${YAW_GDIR}/${name}"
  if [ "$DRY_RUN" = "1" ]; then echo "  [DRY] rclone copyto … ${name}"; return 0; fi
  rclone copyto "$MP4_HOST" "${GDRIVE_REMOTE}:${YAW_GDIR}/${name}" --progress
  rclone link "${GDRIVE_REMOTE}:${YAW_GDIR}/${name}" 2>/dev/null || true
}

# ── метрика yaw_check по свежему bag (внутри nav) ─────────────────────────────
run_yaw_check() {  # печатает "sko range drift" (или "na na na")
  if [ "$DRY_RUN" = "1" ]; then echo "na na na"; return 0; fi
  local out
  out="$(docker exec -i -e SAFE_SEC="$SAFE_SEC" "$NAV" bash -lc \
         "$SRC; python3 /lab/yaw_check.py" 2>&1 || true)"
  echo "$out" >&2   # полный вывод yaw_check — в лог
  local sko rng drift
  sko="$(echo "$out"   | grep 'СКО'    | grep -oE '[0-9]+\.[0-9]+'      | head -1)"
  rng="$(echo "$out"   | grep 'размах' | grep -oE '[0-9]+\.[0-9]+'      | head -1)"
  drift="$(echo "$out" | grep 'дрейф'  | grep -oE '[-+]?[0-9]+\.[0-9]+' | head -1)"
  echo "${sko:-na} ${rng:-na} ${drift:-na}"
}

# ── прогон ────────────────────────────────────────────────────────────────────
log "YAW-tune sweep: ${#CONFIGS[@]} прогонов | alt=${BS_ALT} hold=${BS_HOLD_SEC}s окно=${SAFE_SEC}s | Drive=${GDRIVE_UP}"
[ "$DRY_RUN" = "1" ] && echo ">>> DRY_RUN=1 — команды печатаются, стек НЕ запускается"
export_base_env
purge_drive

mkdir -p "$OUTPUT_DIR"
echo "idx,label,kp,ki,smooth,sko_deg,range_deg,drift_deg,video" > "$CSV"

idx=0
for cfg in "${CONFIGS[@]}"; do
  IFS=: read -r label kp ki sm <<< "$cfg"
  video="$(printf '%02d_%s_kp%s_ki%s_sm%s.mp4' "$idx" "$label" "$kp" "$ki" "$sm")"

  log "[$idx] $label — kp=$kp ki=$ki smooth=$sm → $video"

  # 1-й прогон задаёт разрешение → fresh-start (env применяется при создании
  # контейнера); остальные — без WxH → быстрый restart-all (960×540 уже создан).
  cs_args=()
  [ "$idx" -eq 0 ] && cs_args+=("$RES")
  cs_args+=("liftland")

  # варьируемые гейны этого прогона
  export BS_YAWH_KP="$kp" BS_YAWH_KI="$ki" BS_YAWH_SMOOTH="$sm"

  echo ">>> capture_scene: ${cs_args[*]}  (BS_YAWH_KP=$kp BS_YAWH_KI=$ki BS_YAWH_SMOOTH=$sm)"
  if [ "$DRY_RUN" = "1" ]; then
    echo "  [DRY] GDRIVE_UP=0 MP4=1 N_FRAMES=1 bash $CAPTURE ${cs_args[*]}"
  else
    # GDRIVE_UP=0 инлайн: capture_scene НЕ трогает Drive (льём/чистим сами)
    GDRIVE_UP=0 MP4=1 N_FRAMES=1 bash "$CAPTURE" "${cs_args[@]}" \
      || { echo "⚠️ прогон [$idx] $label упал — записываю na, продолжаю"; }
  fi

  # метрика по свежему bag (до следующего capture_scene, который bag перезапишет)
  read -r sko rng drift <<< "$(run_yaw_check)"
  echo "  → СКО=${sko}° размах=${rng}° дрейф=${drift}°"

  upload_video "$video"

  echo "$idx,${label},$kp,$ki,$sm,$sko,$rng,$drift,$video" >> "$CSV"
  idx=$((idx+1))
done

# ── сводка (ранжируем по СКО) ─────────────────────────────────────────────────
log "СВОДКА — CSV: $CSV"
column -t -s, "$CSV" 2>/dev/null || cat "$CSV"
echo
echo "Ранжирование по СКО (ниже = ровнее держит курс):"
tail -n +2 "$CSV" | awk -F, '$6!="na"{print $6"°  ["$1"] "$2" kp="$3" ki="$4" sm="$5}' \
  | sort -n | head -12 || true
echo
echo "Дальше: выбрать победителя из сводки → подтверждающий прогон вручную с MP4=1"
echo "  (напр.: CPU=1 BS_YAWHOLD=1 BS_GZHOLD=1 BS_YAWH_KP=<kp> BS_YAWH_KI=<ki> \\"
echo "          BS_YAWH_SMOOTH=<sm> BS_HOLD_SEC=40 TOPICS_EXTRA=/model/iris_cam/odometry \\"
echo "          GDRIVE_UP=1 MP4=1 bash src/lab/capture_scene.sh 960x540 liftland)"
