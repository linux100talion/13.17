#!/usr/bin/env bash
# ============================================================================
# freefly_lv.sh — единая обёртка над двумя «выкристаллизовавшимися» freefly-
# командами (docker/sim/Q.txt): один скрипт, флаг LV выбирает профиль.
#
#   LV=1 (default) — freefly-LV: тумблер вверх = наш стек (IPM-демпферы →
#                    VinsHold), центр CH6 = штатный LOITER-на-VINS (EK3
#                    extnav), GPS глушится в полёте по зрелости VINS.
#                    Требует VISO_TYPE=1 в eeprom — ставится автоматически.
#   LV=0           — базовый freefly: только наш стек, полётник vision не
#                    видит, GPS живой весь полёт. Требует VISO_TYPE=0 (с 1
#                    прогон без vision-фида НЕ АРМИТСЯ: «Arm: VisOdom: not
#                    healthy», ARMING_CHECK 0 чек не снимает) и возврата
#                    GPS-источников EKF после LV-полётов (POSXY/VELXY=3,
#                    SIM_GPS1_ENABLE=1) — всё ставится автоматически.
#
# Самодостаточен от ХОЛОДНОГО СТАРТА (после ребута ноута): шаг 0 поднимает
# хост и стек, если они не готовы — `make host-setup` при отсутствии
# /dev/rawbayer (v4l2loopback не персистентен, нужен sudo — спросит пароль)
# и `make up && make wait`, если контейнеры не бегут.
# Шаг 1 готовит eeprom SITL под профиль (pymavlink внутри контейнера
# simulator, scripts/sitl_lv_profile.py — SITL к этому моменту жив).
# Применяется значение на рестарте стека, который capture_scene.sh делает сам
# в начале прогона — атомарность («стек только целиком») сохранена.
# Шаг 2 — тот же атомарный прогон, что и руками: capture_scene.sh RES bootstrap_arch2.
#
# Запуск С ХОСТА из любого места:
#   bash src/lab/freefly_lv.sh              # LV=1, полный freefly-LV
#   LV=0 bash src/lab/freefly_lv.sh         # базовый freefly (GPS жив)
#   WIND_SPD=5 LV=1 bash src/lab/freefly_lv.sh
# Любой параметр (BS_*, WIND_SPD, RES, GDRIVE_UP, MP4, TOPICS_EXTRA...)
# переопределяется через env; дефолты ниже — эталонные команды из Q.txt.
# В LV=0 BS_VINS_MIN не задаётся (дефолт ноды 40, как в эталонной команде №1).
#
# После прогона — АРХИВ в docker/sim/output/joystick/<NAME>/ (шаг 4): scene.mp4,
# мета .env, bag (KEEP_BAG=0 — не забирать), joy.log, реплей-артефакты. JPEG-кадры
# не делаются (FRAMES=0; вернуть — FRAMES=1). Имя — NAME=… или автогенерат
# lv<LV>_<пилот>_<дата_время>. См. src/lab/joystick/README.md.
# ============================================================================
set -euo pipefail

LV="${LV:-1}"
RES="${RES:-960x540}"
SIM="${SIM:-p1317_simulator}"
NAV="${NAV:-p1317_nav}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIMDIR="$(cd "$SCRIPT_DIR/../../docker/sim" && pwd)"

case "$LV" in 0|1) ;; *) echo "ОШИБКА: LV=$LV (ожидаю 0 или 1)" >&2; exit 2 ;; esac
echo "=== freefly_lv: профиль LV=$LV (${RES}) ==="

# ── 0) холодный старт: хост + стек (после ребута ноута оба слетают) ──────────
# v4l2loopback не персистентен: без /dev/rawbayer контейнер nav не стартует.
if [ ! -e /dev/rawbayer ]; then
    echo "freefly_lv: нет /dev/rawbayer — make host-setup (нужен sudo)"
    make -C "$SIMDIR" host-setup
fi
# Контейнеры должны БЕЖАТЬ: SITL нужен живым для шага eeprom (сам полёт стек
# всё равно перезапустит целиком — capture_scene делает fresh-start).
RUNNING="$(docker ps --format '{{.Names}}')"
if ! grep -qx "$SIM" <<< "$RUNNING" || ! grep -qx "$NAV" <<< "$RUNNING"; then
    echo "freefly_lv: стек не поднят — make up && make wait"
    make -C "$SIMDIR" up
    make -C "$SIMDIR" wait
fi

# ── 0b) защита от наслоения прогонов ─────────────────────────────────────────
# Второй freefly_lv поверх летящего рвёт запись первого (рестарт стека посреди
# чужой лётной фазы) — 2026-08-22 три наслоённых прогона стоили bag'а полёта.
# Отдельный случай: freefly ЖДЁТ ДИЗАРМ пилота — пока его нет, нода жива и
# прогон не завершён (дизарм с пульта: газ в МИНИМУМ + yaw ВЛЕВО до упора 2–3 с).
BUSY=""
pgrep -f "capture_scene.sh" >/dev/null 2>&1 && BUSY="capture_scene на хосте"
# ps+grep вместо pgrep -f: зомби (умершая нода, которую PID1-tail не пожал —
# снесётся рестартом стека) не должны блокировать запуск; [b] — не матчить себя.
if [ -z "$BUSY" ] && docker exec "$NAV" bash -lc \
        "ps -eo stat=,cmd= | grep -v '^Z' | grep -q '[b]ootstrap_arch2'" 2>/dev/null; then
    BUSY="лётная нода bootstrap_arch2 в контейнере (freefly ждёт дизарм?)"
fi
if [ -n "$BUSY" ]; then
    echo "ОШИБКА: уже идёт прогон — $BUSY." >&2
    echo "  Заверши его (дизарм: газ min + yaw ВЛЕВО 2–3 с) или прибей:" >&2
    echo "    docker exec $NAV pkill -f bootstrap_arch2" >&2
    echo "    pkill -f capture_scene.sh" >&2
    exit 3
fi

# ── 1) eeprom SITL под профиль (VISO_TYPE и, для LV=0, возврат GPS-профиля) ──
if ! docker exec "$SIM" bash -lc \
    "PYTHONPATH=/root/ardupilot/modules/mavlink python3 /scripts/sitl_lv_profile.py $LV"; then
    echo "ОШИБКА: не удалось подготовить eeprom (SITL жив? см. make logs)." >&2
    echo "Попробуй: cd docker/sim && make restart-all && make wait — и повтори." >&2
    exit 1
fi

# ── 2) env-профиль полёта (эталон из Q.txt; всё переопределяется снаружи) ────
export WIND_SPD="${WIND_SPD:-10}"
export BS_PILOT="${BS_PILOT:-joy}"
export BS_STAB="${BS_STAB:-DpHoldM}"
export BS_MISSION="${BS_MISSION:-freefly}"
export BS_HANDOVER_VINS="${BS_HANDOVER_VINS:-1}"
export BS_GZ_CMD_GAIN="${BS_GZ_CMD_GAIN:-4}"
export BS_MODE_BUDGET="${BS_MODE_BUDGET:-80}"
export BS_IPM_DEROT="${BS_IPM_DEROT:-1.0}"
export BS_IPM_WIN="${BS_IPM_WIN:-0.5}"
export BS_IPM_WZ_TAU="${BS_IPM_WZ_TAU:-2.0}"
export BS_PITCH_OSIGN="${BS_PITCH_OSIGN:-1}"
export BS_PITCH_RATE_KI="${BS_PITCH_RATE_KI:-30}"
export BS_PITCH_RATE_KP="${BS_PITCH_RATE_KP:-30}"
export BS_PITCH_RATE_CMD_GAIN="${BS_PITCH_RATE_CMD_GAIN:-5}"
export BS_ROLL_RATE_CMD_GAIN="${BS_ROLL_RATE_CMD_GAIN:-5}"
export BS_ROLL_IMAX="${BS_ROLL_IMAX:-150}"
export BS_ROLL_OSIGN="${BS_ROLL_OSIGN:-1}"
export BS_ROLL_RATE_KI="${BS_ROLL_RATE_KI:-30}"
export BS_ROLL_RATE_KP="${BS_ROLL_RATE_KP:-30}"
export BS_SLEW="${BS_SLEW:-300}"
export BS_YAW_ARM_FRAMES="${BS_YAW_ARM_FRAMES:-5}"
export BS_YAW_KD="${BS_YAW_KD:-6}"
export BS_YAW_KI="${BS_YAW_KI:-0}"
export BS_YAW_KP="${BS_YAW_KP:-0}"
export BS_YAW_LEAK="${BS_YAW_LEAK:-8}"
export BS_YAW_MAX_RATE="${BS_YAW_MAX_RATE:-100}"
export BS_YAW_RATE_FULL="${BS_YAW_RATE_FULL:-60}"
export BS_YAW_SMOOTH="${BS_YAW_SMOOTH:-5}"
export TOPICS_EXTRA="${TOPICS_EXTRA:-/joy /odometry /model/iris_cam/odometry /flow_dbg /flow_dbg2 /flow_dbg6 /flow_dbg7 /flow_dbg8 /flow_dbg9}"
export GDRIVE_UP="${GDRIVE_UP:-0}"
export MP4="${MP4:-1}"
export FRAMES="${FRAMES:-0}"    # JPEG-кадры не нужны (просьба 2026-08-22): только mp4

if [ "$LV" = "1" ]; then                 # LV-добавки (см. Q.txt: ровно пять)
    export BS_VINS_MIN="${BS_VINS_MIN:-300}"
    export BS_FF_LOITER="${BS_FF_LOITER:-1}"
    export BS_VISION_VEL="${BS_VISION_VEL:-1}"
    export BS_VISION_POSE_SRC="${BS_VISION_POSE_SRC:-extern}"
    export BS_GPS_DISABLE="${BS_GPS_DISABLE:-1}"
fi

# ── 3) атомарный прогон (рестарт стека внутри — применяет eeprom из шага 1) ──
# Не exec: после прогона — шаг 4, архив под именем (scene.mp4/scene_bag живут
# в output/ только до следующего прогона — capture_scene чистит их на старте).
RC=0
bash "$SCRIPT_DIR/capture_scene.sh" "$RES" bootstrap_arch2 || RC=$?

# ── 4) архив прогона: docker/sim/output/joystick/<NAME>/ ─────────────────────
# Имя: NAME=… снаружи или автогенерат lv<LV>_<пилот>_<дата_время>. Внутрь едут:
# scene.mp4, мета <NAME>.env (все BS_*/WIND_ + commit; та же идея, что у
# calib_run.sh), bag (KEEP_BAG=1, default — без него разбор joystick/analyze.sh
# умрёт на следующем же прогоне), joy.log, а для BS_PILOT=replay — joy_replay.log
# и копия сценария. JPEG-кадры не делаются вовсе (FRAMES=0 в шаге 2).
# Архив копится — старые прогоны чистить руками.
NAME="${NAME:-lv${LV}_${BS_PILOT}_$(date +%Y%m%d_%H%M%S)}"
case "$NAME" in
    */*|*' '*) echo "ОШИБКА: NAME без пробелов и слэшей ('$NAME')" >&2; exit 2 ;;
esac
KEEP_BAG="${KEEP_BAG:-1}"
RUN_DIR="$SIMDIR/output/joystick/$NAME"
mkdir -p "$RUN_DIR"
{
    echo "# freefly_lv: $NAME (rc=$RC)"
    echo "# commit: $(git -C "$SCRIPT_DIR/../.." rev-parse --short HEAD 2>/dev/null || echo '?')"
    echo "LV=$LV  RES=$RES"
    env | { grep -E '^(BS_|WIND_|TOPICS_|GDRIVE_|MP4)' || true; } | sort
} > "$RUN_DIR/$NAME.env"
# Каждый артефакт — со своей громкой диагностикой: шаг 4 НЕ умирает молча и не
# молчит о пропаже (bag прогона 2026-08-22 не доехал до архива без единого слова).
if [ -f "$SIMDIR/output/scene_img/scene.mp4" ]; then
    cp "$SIMDIR/output/scene_img/scene.mp4" "$RUN_DIR/scene.mp4" \
        || echo "⚠️ scene.mp4 не скопировался в архив" >&2
else
    echo "⚠️ scene.mp4 нет (MP4=0 или прогон упал до сборки видео)" >&2
fi
if [ "$KEEP_BAG" = "1" ]; then
    if [ -d "$SIMDIR/output/scene_bag" ]; then
        if mv "$SIMDIR/output/scene_bag" "$RUN_DIR/bag"; then
            echo "    bag → joystick/$NAME/bag ($(du -sh "$RUN_DIR/bag" 2>/dev/null | cut -f1))"
        else
            echo "⚠️ bag НЕ переехал (mv не удался) — остался в output/scene_bag" >&2
        fi
    else
        echo "⚠️ output/scene_bag нет — запись не состоялась (RECORD=0 или прогон упал)" >&2
    fi
fi
if [ -f "$SIMDIR/output/joy.log" ]; then
    cp "$SIMDIR/output/joy.log" "$RUN_DIR/" || true
fi
if [ "${BS_PILOT}" = "replay" ] && [ -f "$SIMDIR/output/joy_replay.log" ]; then
    cp "$SIMDIR/output/joy_replay.log" "$RUN_DIR/" || true
fi
# копия сценария реплея (провенанс): контейнерный путь → хостовый
SCN_HOST=""
case "${BS_REPLAY_SCENARIO:-}" in
    /lab/*)                SCN_HOST="$SCRIPT_DIR/${BS_REPLAY_SCENARIO#/lab/}" ;;
    /root/sim_ws/output/*) SCN_HOST="$SIMDIR/output/${BS_REPLAY_SCENARIO#/root/sim_ws/output/}" ;;
esac
if [ -n "$SCN_HOST" ] && [ -f "$SCN_HOST" ]; then
    cp "$SCN_HOST" "$RUN_DIR/"
fi

echo "=== freefly_lv: архив прогона → docker/sim/output/joystick/$NAME/ ==="
ls -lh "$RUN_DIR" | tail -n +2 | awk '{print "    " $NF " (" $5 ")"}'
[ -d "$RUN_DIR/bag" ] && echo "    разбор: RUN=$NAME bash src/lab/joystick/analyze.sh"
exit $RC
