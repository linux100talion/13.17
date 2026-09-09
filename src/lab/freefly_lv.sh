#!/usr/bin/env bash
# ============================================================================
# freefly_lv.sh — единая обёртка над двумя «выкристаллизовавшимися» freefly-
# командами (docker/sim/doc/tmp/Q.txt): один скрипт, флаг LV выбирает профиль.
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
#   LV=2           — «GPS ОТСУТСТВУЕТ С БУТА» (модель боевого борта без
#                    приёмника): SIM_GPS1_ENABLE=0 + extnav-пара EKF ставятся
#                    в eeprom ЕЩЁ ДО старта; origin — SET_GPS_GLOBAL_ORIGIN
#                    (BS_SET_ORIGIN=1), высота миссии — сырой баро
#                    (BS_ALT_SRC=baro: global rel_alt без GPS замерзает),
#                    aiding EKF стартует НА ЗЕМЛЕ от нулевой vision_pose
#                    (мост gps_denied в ноде), с init VINS топик у ray_tracer.
#                    Центр CH6 = тот же LOITER-на-VINS, что в LV=1, но GPS
#                    не участвовал ни секунды. ⚠️ Перцепция демпфера сидит
#                    на global rel_alt (намеренно, см. bootstrap_node) — без
#                    GPS она может ослепнуть: CH6-вверх тогда ≈ чистый
#                    ALT_HOLD. Это известная цена, отдельная кампания.
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
#   bash src/lab/freefly_lv.sh              # профиль БОКСА из docker/sim/.env
#       (сеется из эталона env.default В GIT: LV=2 BS_SF_MASTER=1 — дефолт и
#       на свежем клоне); без строк в .env — LV=1
#   LV=0 bash src/lab/freefly_lv.sh         # базовый freefly (GPS жив)
#   WIND_SPD=5 LV=1 bash src/lab/freefly_lv.sh
#   BS_SF_MASTER=1 bash src/lab/freefly_lv.sh   # схема «SF-мастер»: SF (CH7) =
#       мастер сырых стиков (не-вверх = MANUAL при любом SC), SC (CH6) = потолок
#       лесенки зрелости (вверх демпфер / центр +VinsHold / вниз +LOITER).
#       ⚠️ Требует SF → CH7 в миксере EdgeTX; НЕ включать под старые реплеи
#       (их сценарии без "sf" полетят целиком на сырых стиках).
#   Кнопка SD = ВОЗВРАТ ДОМОЙ (BS_RTH_JOY='b2' = /joy buttons[2], режим —
#       BS_RTH_JOY_MODE, дефолт SMART_RTL «по следу»): фронт нажатия = тот же
#       импульс, что `make smart-rth`; повторное нажатие ОТМЕНЯЕТ возврат.
#       Карта каналов пульта целиком — docker/sim/rx.md.
#   Кнопка SA = МЯГКАЯ ПОСАДКА (BS_FF_LAND=1 — дефолт ноды): при rel_alt ≤ 5 м
#       (BS_LAND_ALT_MAX) и |v| ≤ 1 м/с (BS_LAND_V_MAX) freefly уходит в шаг
#       SoftLand: борт в LOITER → штатный LAND на EKF-от-VINS (стек пуст,
#       LAND_SPEED 15 см/с); иначе (в т.ч. ДО инициализации VINS) — снижение в
#       ALT_HOLD под нашим демпфером/VinsHold на BS_LAND_RATE 0.15 м/с, касание →
#       газ в пол → дизарм сервисом. Где кнопка в /joy — BS_LAND_JOY ('b0' =
#       buttons[0]; на пульте проекта SA измерена как b1 → docker/sim/.env
#       BS_LAND_JOY=b1; CH8 НЕ использовать — делит ось axes[6] с CH7/SF,
#       квирк HID TX12; где кнопка — src/lab/joystick/js_probe.py на хосте,
#       в ленте joy_timeline — «JOY: кнопка b<i>»). С хоста без пульта:
#       `make sa-land`. Выкл: BS_FF_LAND=0.
#   SPAWN_FROM=docker/sim/output/joystick/lv1_joy_20260824_140447 \
#       bash src/lab/freefly_lv.sh   # стартовать с места посадки того прогона
# РУЧКИ НОДЫ И ВЕТЕР (с 2026-09-07) — ТОЛЬКО профили src/control/profiles: обязательный
# env PROFILES="dphold/baseline … world/wind2_gust5" (список держит cmd/<имя>/<имя>.sh,
# запускать через него). BS_*/WIND_* из env и docker/sim/.env НЕ читаются (BS_* в .env
# игнорируются с предупреждением). Служебное (RES, GDRIVE_UP, MP4, KEEP_BAG, TOPICS_EXTRA,
# SPAWN_*, NAME, LV — профиль eeprom SITL) — env снаружи > docker/sim/.env > дефолт скрипта.
# Аргументы прогона BS_REPLAY_SCENARIO/RAW/FENCE — env (едут в контейнер как есть).
# Шпаргалки: docker/sim/env.md, src/control/profiles/README.md, cmd/README.txt.
#
# После прогона — АРХИВ в docker/sim/output/joystick/<NAME>/ (шаг 4): scene.mp4,
# мета .env, bag (KEEP_BAG=0 — не забирать), joy.log, реплей-артефакты. JPEG-кадры
# не делаются (FRAMES=0; вернуть — FRAMES=1). Имя — NAME=… или автогенерат
# lv<LV>_<пилот>_<дата_время>. См. src/lab/joystick/README.md.
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIMDIR="$(cd "$SCRIPT_DIR/../../docker/sim" && pwd)"

# ── дефолты БОКСА из docker/sim/.env (тот же файл, что читает compose) ───────
# Локальный профиль машины (в .gitignore); эталон — env.default (В GIT): при
# отсутствии .env сеем его копией эталона (то же делает make build/up/restart-
# all/fresh-start) — свежий клон летит боевым профилем сразу. Существующий
# .env не трогаем. Строки KEY=VALUE применяются ТОЛЬКО
# к незаданным переменным — env снаружи (`LV=1 bash ...`) всегда сильнее.
# Так `bash src/lab/freefly_lv.sh` без ничего летит боевым профилем бокса
# (сейчас LV=2 + BS_SF_MASTER=1), а WORLD/SPAWN_POSE, которые compose и так
# берёт из .env, заодно становятся видны скрипту (честный echo точки спавна
# и мета-архив прогона).
# ⚠️ Под BS_SF_MASTER=1 реплей СТАРОГО сценария (без "sf") летит целиком на
# сырых стиках — для таких реплеев давать BS_SF_MASTER=0 снаружи.
if [ ! -f "$SIMDIR/.env" ] && [ -f "$SIMDIR/env.default" ]; then
    cp "$SIMDIR/env.default" "$SIMDIR/.env"
    echo "freefly_lv: docker/sim/.env создан из env.default (свежий клон) —" \
         "проверь VINS_SRC/CUDA_ARCH_BIN под бокс"
fi
ENV_DEFAULTS=""
ENV_IGNORED=""
if [ -f "$SIMDIR/.env" ]; then
    while IFS= read -r line; do
        case "$line" in ''|\#*) continue ;; esac
        key="${line%%=*}"; val="${line#*=}"
        case "$key" in ''|[0-9]*|*[!A-Za-z0-9_]*) continue ;; esac
        # снять парные кавычки вокруг значения (compose их тоже снимает)
        case "$val" in
            \"*\") val="${val%\"}"; val="${val#\"}" ;;
            \'*\') val="${val%\'}"; val="${val#\'}" ;;
        esac
        case "$key" in BS_*) ENV_IGNORED="$ENV_IGNORED $key"; continue ;; esac
        if [ -z "${!key+x}" ]; then
            export "$key=$val"
            ENV_DEFAULTS="$ENV_DEFAULTS $key=$val"
        fi
    done < "$SIMDIR/.env"
fi
if [ -n "$ENV_DEFAULTS" ]; then
    echo "freefly_lv: дефолты бокса из docker/sim/.env:$ENV_DEFAULTS"
fi
if [ -n "${ENV_IGNORED:-}" ]; then
    echo "freefly_lv: ⚠️ BS_* в docker/sim/.env ИГНОРИРУЮТСЯ с 2026-09-07 (ручки ноды —" \
         "только профили):$ENV_IGNORED — убери их из .env" >&2
fi

# ── ручки ноды и ветер — ТОЛЬКО профили src/control/profiles (список PROFILES) ────
# Загрузчик собирает include + дельту строго по схеме BootstrapConfig (дубль/незнакомый/
# отсутствующий ключ = ошибка) и экспортирует BS_* + WIND_* (world/) в этот шелл: отсюда
# их читают eeprom-шаг (BS_EKF_DRAG), compose (WIND_*), имя прогона (BS_PILOT), мета.
# В контейнер едет ТОЛЬКО список (capture_scene → bootstrap_arch2.sh собирает заново) —
# хостовые BS_* до ноды не доезжают по построению. Список держит cmd/<имя>/<имя>.sh.
if [ -z "${PROFILES:-}" ]; then
    echo "ОШИБКА: PROFILES не задан — ручки ноды живут только в профилях. Запускай через" >&2
    echo "  cmd/bl/bl.sh (или cmd/<имя>/<имя>.sh); руками: PROFILES=\"dphold/baseline …\" $0" >&2
    exit 2
fi
LOADER="$SCRIPT_DIR/../control/profiles/load.py"
set -a
# shellcheck disable=SC2086
eval "$(python3 "$LOADER" $PROFILES)"
set +a
echo "freefly_lv: профили [$PROFILES] → $(env | grep -cE '^BS_') ключей BS_, ветер WIND_SPD=$WIND_SPD${WIND_GUST:+ + порывы «$WIND_GUST»}"

LV="${LV:-1}"
RES="${RES:-960x540}"
SIM="${SIM:-p1317_simulator}"
NAV="${NAV:-p1317_nav}"

case "$LV" in 0|1|2) ;; *) echo "ОШИБКА: LV=$LV (ожидаю 0, 1 или 2)" >&2; exit 2 ;; esac
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

# ── 1) eeprom SITL под профиль (VISO_TYPE; LV=0 — возврат GPS-профиля;
# LV=2 — глушение GPS + extnav-пара EKF ещё до бута) ─────────────────────────
# SITL поднимается десятки секунд ПОСЛЕ «nav: готово» (make wait ждёт только
# nav_up) — ждём порт 5762 сами, иначе eeprom-шаг стучится рано и сдаётся
# (два ложных «SITL мёртв» 2026-08-22; ретраев самого sitl_lv_profile мало).
wait_sitl() {
    for _ in $(seq 1 45); do
        docker exec "$SIM" python3 -c \
            "import socket; s=socket.socket(); s.settimeout(1); s.connect(('127.0.0.1',5762))" \
            2>/dev/null && return 0
        sleep 2
    done
    return 1
}
wait_sitl || echo "freefly_lv: 5762 так и не открылся — пробую eeprom-шаг как есть"
# BS_EKF_DRAG (drag-фьюжн ветра EKF3, стрелка ветра HUD) — в eeprom-шаг LV2.
# BS_FCU_PARAMS («NAME=VALUE …», loiter/) — параметры самого полётника в eeprom: там,
# где командует FCU, а не нода (возврат домой RTL, cmd/rth). Стоковые значения стоят в
# loiter/baseline.txt явно — иначе кандидат остался бы в eeprom у соседних прогонов.
EEPROM_CMD="PYTHONPATH=/root/ardupilot/modules/mavlink BS_EKF_DRAG='$BS_EKF_DRAG' BS_FCU_PARAMS='$BS_FCU_PARAMS' python3 /scripts/sitl_lv_profile.py $LV"
if ! docker exec "$SIM" bash -lc "$EEPROM_CMD"; then
    # SITL часто мёртв после аварийно размотанного прогона (краш физики/зависший
    # арм) при живых контейнерах — лечится полным рестартом стека, делаем сами.
    echo "freefly_lv: SITL недоступен — make restart-all && make wait и повтор"
    make -C "$SIMDIR" restart-all 2>&1 | tail -2
    make -C "$SIMDIR" wait
    if ! docker exec "$SIM" bash -lc "$EEPROM_CMD"; then
        echo "ОШИБКА: eeprom не подготовлен и после рестарта (см. make logs)." >&2
        exit 1
    fi
fi

# ── 2) ручки ноды и ветер — В ПРОФИЛЯХ (PROFILES, см. выше). Здесь только служебное. ──
# 2026-09-07: 48 export BS_* и LV-блоки (BS_GPS_DENIED/SET_ORIGIN/ALT_SRC/…) удалены —
# значения живут в src/control/profiles/{mission,vins,loiter,dphold,…}, ветер — world/,
# LV=2 (GPS отсутствует с бута) запечён в loiter/baseline.txt; LV здесь выбирает только
# eeprom-профиль SITL (шаг 1). История чисел — шапки профилей и git log этого файла.

# /mavros/state (1 Гц) — для разбора joystick-серии: латчи режимов (LOITER!) и
# арм/дизарм видны в bag (двойной щелчок CH6 в полёте 182409 без него не объяснить).
# /mission/status — лесенка SF-мастера + гейт LOITER-на-VINS от лётной ноды
# (debug-HUD): joy_timeline показывает переходы «HUD: LOITER READY» / «HUD: ярус …».
# /feature — счётчик фич трекера для строки FEAT пост-рендера HUD (hud_video.py);
# PointCloud 10 sim-Гц — копейки против /image_color.
export TOPICS_EXTRA="${TOPICS_EXTRA:-/joy /mavros/state /mission/status /feature /odometry /model/iris_cam/odometry /flow_dbg /flow_dbg2 /flow_dbg6 /flow_dbg7 /flow_dbg8 /flow_dbg9 /flow_dbg10 /mavros/imu/data /nn1/bridge /vins/sane}"
export GDRIVE_UP="${GDRIVE_UP:-0}"
export MP4="${MP4:-1}"
export FRAMES="${FRAMES:-0}"    # JPEG-кадры не нужны (просьба 2026-08-22): только mp4

# ── 2b) ТОЧКА СПАВНА: где сел — там и стартуем ──────────────────────────────
# SPAWN_FROM=<каталог прогона|bag|.db3> — взять МЕСТО ПОСАДКИ того прогона
# (истинная поза Gazebo из bag'а, src/lab/spawn_pose.py) и спавнить борт там же
# с тем же курсом. SPAWN_POSE="x y z r p y" — та же поза руками (оси мира
# Gazebo: x-восток, y-север, yaw 0 = нос на восток).
#   SPAWN_FROM=docker/sim/output/joystick/lv1_joy_20260824_140447 bash src/lab/freefly_lv.sh
# ЧАСТО ПРОЩЕ: сохранить точку под именем один раз и звать по имени —
#   python3 src/lab/spawn_save.py <прогон> among_trees   (прогон дальше не нужен)
#   SPAWN_POSE=among_trees bash src/lab/freefly_lv.sh
# Пусто — штатный спавн в центре площадки. Постоянный дефолт для всех прогонов
# кладётся строкой SPAWN_POSE=... в docker/sim/.env (её читает compose).
# Применяет позу scripts/sim_up.sh (патчит КОПИЮ мира в /tmp); env доезжает до
# контейнера только при ПЕРЕСОЗДАНИИ — capture_scene делает fresh-start, т.к.
# RES задан всегда.
# ⚠️ Требует ВЕТРА (WIND_SPD ≠ 0, здесь дефолт 5): в безветренном прогоне борт на
# земле ничем не удерживается и уезжает — трения о землю в этой связке нет,
# демпфирует только плагин ветра. Подробности и замеры — в sim_up.sh.
if [ -n "${SPAWN_FROM:-}" ]; then
    if [ -n "${SPAWN_POSE:-}" ]; then
        echo "freefly_lv: заданы и SPAWN_POSE, и SPAWN_FROM — беру SPAWN_FROM" >&2
    fi
    SPAWN_LINE="$(python3 "$SCRIPT_DIR/spawn_pose.py" "$SPAWN_FROM")" || {
        echo "ОШИБКА: место посадки из '$SPAWN_FROM' не достаётся (см. выше)" >&2
        exit 4; }
    eval "$SPAWN_LINE"
fi
export SPAWN_POSE="${SPAWN_POSE:-}"
if [ -n "$SPAWN_POSE" ]; then
    # тут может быть и ИМЯ ПРЕСЕТА (SPAWN_POSE=among_trees) — его разрешает уже
    # sim_up.sh по файлу docker/sim/output/spawn/<имя> (пишет spawn_save.py)
    echo "freefly_lv: точка старта — $SPAWN_POSE"
else
    echo "freefly_lv: спавн штатный (центр площадки)"
fi

# ── 3) атомарный прогон (рестарт стека внутри — применяет eeprom из шага 1) ──
# Не exec: после прогона — шаг 4, архив под именем (scene.mp4/scene_bag живут
# в output/ только до следующего прогона — capture_scene чистит их на старте).
RC=0
bash "$SCRIPT_DIR/capture_scene.sh" "$RES" bootstrap_arch2 || RC=$?

# ── 3.5) пост-рендер debug-HUD на видео из bag (HUD_MP4=0 — выключить) ──────
# HUD живёт только в FPV-потоке :5600 (в bag не пишется), а scene.mp4 —
# чистая камера. scene_hud.mp4 = тот же полёт глазами пилота OpenHD:
# hud_video.py восстанавливает оверлей из топиков bag ТЕМ ЖЕ кодом
# (nav_pkg/hud_renderer.py), что рисует живой поток.
if [ "${HUD_MP4:-1}" = "1" ] && [ -f "$SIMDIR/output/scene_bag/metadata.yaml" ]; then
    echo "=== пост-рендер debug-HUD (scene_hud.mp4) ==="
    docker exec "$NAV" bash -lc 'source /opt/ros/humble/setup.bash;
        source /opt/overlay/install/setup.bash;
        source /root/sim_ws/install/setup.bash;
        python3 /lab/hud_video.py' \
        || echo "⚠️ hud_video.py упал — scene_hud.mp4 не будет (прогон цел)" >&2
fi

# ── 3.6) пост-рендер канала вида сверху (scene_ipm.mp4; IPM_MP4=0 — выключить) ─
# Варп в bag не пишется — ipm_video.py пересчитывает его БОЕВЫМ FlowEstimator и
# рисует общей с офлайн-стендом рисовалкой (ipm_panel.py): слева кадр с полосой
# земли, справа выпрямленный варп + лётные значения из /flow_dbg8|9 рядом с
# истиной Gazebo. Конфиг канала — из ЭТОГО окружения (те же BS_*, что летели),
# поэтому проброс BS_* тем же автосписком, что в capture_scene.sh (рукописный
# белый список уже терял ручки молча).
if [ "${IPM_MP4:-1}" = "1" ] && [ -f "$SIMDIR/output/scene_bag/metadata.yaml" ]; then
    echo "=== пост-рендер канала вида сверху (scene_ipm.mp4) ==="
    # лётный конфиг канала — из тех же профилей (BootstrapConfig.from_run → PROFILES)
    docker exec -e PROFILES "$NAV" bash -lc 'source /opt/ros/humble/setup.bash;
        source /opt/overlay/install/setup.bash;
        source /root/sim_ws/install/setup.bash;
        python3 /lab/ipm_video.py' \
        || echo "⚠️ ipm_video.py упал — scene_ipm.mp4 не будет (прогон цел)" >&2
fi

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
    echo "PROFILES=$PROFILES"
    # полный снимок ручек ноды и ветра — ровно то, что собрал загрузчик (все поля схемы)
    python3 "$LOADER" --format plain $PROFILES
    env | { grep -E '^(BS_REPLAY_|SPAWN_|TOPICS_|GDRIVE_|MP4|HUD_MP4|IPM_MP4)' || true; } | sort
} > "$RUN_DIR/$NAME.env"
# Каждый артефакт — со своей громкой диагностикой: шаг 4 НЕ умирает молча и не
# молчит о пропаже (bag прогона 2026-08-22 не доехал до архива без единого слова).
if [ -f "$SIMDIR/output/scene_img/scene.mp4" ]; then
    cp "$SIMDIR/output/scene_img/scene.mp4" "$RUN_DIR/scene.mp4" \
        || echo "⚠️ scene.mp4 не скопировался в архив" >&2
else
    echo "⚠️ scene.mp4 нет (MP4=0 или прогон упал до сборки видео)" >&2
fi
if [ -f "$SIMDIR/output/scene_img/scene_hud.mp4" ]; then
    cp "$SIMDIR/output/scene_img/scene_hud.mp4" "$RUN_DIR/scene_hud.mp4" \
        || echo "⚠️ scene_hud.mp4 не скопировался в архив" >&2
elif [ "${HUD_MP4:-1}" = "1" ]; then
    echo "⚠️ scene_hud.mp4 нет (hud_video.py упал или bag не писался)" >&2
fi
if [ -f "$SIMDIR/output/scene_img/scene_ipm.mp4" ]; then
    cp "$SIMDIR/output/scene_img/scene_ipm.mp4" "$RUN_DIR/scene_ipm.mp4" \
        || echo "⚠️ scene_ipm.mp4 не скопировался в архив" >&2
elif [ "${IPM_MP4:-1}" = "1" ]; then
    echo "⚠️ scene_ipm.mp4 нет (ipm_video.py упал или bag не писался)" >&2
fi
if [ "$KEEP_BAG" = "1" ]; then
    if [ -f "$SIMDIR/output/scene_bag/metadata.yaml" ]; then
        # КОПИЯ, А НЕ ПЕРЕЕЗД (2026-09-07): в архив едет копия, а НАСТОЯЩИЙ bag
        # остаётся в output/scene_bag до следующего прогона. Раньше был mv, и
        # привычный путь исчезал ровно в тот момент, когда он нужен больше всего —
        # сразу после записи: `ros2 bag play output/scene_bag`, analyze.sh без RUN и
        # всё, что знает SCENE_BAG по умолчанию (drift_check, attitude, hud_video,
        # ipm_video, extract_frames, analyze_*.py в output/), приходилось переучивать
        # на новый путь. Вариант «оставить симлинк на архив» отвергнут (решение
        # 2026-09-07): играть и разбирать — из настоящего каталога, не через ссылку.
        # Цена — двойной объём (bag 2+ ГБ) до старта следующего
        # прогона, который снесёт output/scene_bag* (capture_scene, шаг 0); архив цел.
        # Копируем СНАЧАЛА С ХОСТА (после capture_scene файлы уже наши: chown_output
        # в его trap) — так копия сразу host-owned. Если прав нет (chown не прошёл,
        # bag остался root'овым) — тем же копированием изнутри контейнера, там мы root,
        # и возвращаем владельца хосту. Между попытками чистим цель: cp -r в
        # СУЩЕСТВУЮЩИЙ каталог кладёт вложенный scene_bag/, а не заменяет.
        BAG_OK=0
        rm -rf "$RUN_DIR/bag" 2>/dev/null || true
        if cp -r "$SIMDIR/output/scene_bag" "$RUN_DIR/bag" 2>/dev/null; then
            BAG_OK=1
        else
            rm -rf "$RUN_DIR/bag" 2>/dev/null || true
            # именно `if`, а не `cmd && BAG_OK=1`: под `set -e` упавший последний
            # AND-список в теле else уронил бы весь шаг 4 (архив остался бы без
            # joy.log/сценария), а мы хотим предупредить и доделать остальное
            if docker exec "$NAV" bash -lc "rm -rf '/root/sim_ws/output/joystick/$NAME/bag';
                    cp -r /root/sim_ws/output/scene_bag '/root/sim_ws/output/joystick/$NAME/bag' &&
                    chown -R $(id -u):$(id -g) '/root/sim_ws/output/joystick/$NAME/bag'" 2>/dev/null; then
                BAG_OK=1
            fi
        fi
        if [ "$BAG_OK" = "1" ]; then
            echo "    bag → joystick/$NAME/bag ($(du -sh "$RUN_DIR/bag" 2>/dev/null | cut -f1)),"
            echo "         оригинал остался в output/scene_bag — играть можно сразу:"
            echo "         ros2 bag play docker/sim/output/scene_bag  (живёт до следующего прогона)"
        else
            echo "⚠️ bag НЕ скопировался в архив — есть только output/scene_bag," >&2
            echo "   и его сотрёт следующий прогон: забери руками (cp -r) прямо сейчас" >&2
        fi
    else
        # каталог output/scene_bag теперь живёт между прогонами (пустой — норма),
        # поэтому судим по metadata.yaml: нет его — записи не было
        echo "⚠️ в output/scene_bag нет metadata.yaml — запись не состоялась " \
             "(RECORD=0 или прогон упал)" >&2
    fi
fi
if [ -f "$SIMDIR/output/joy.log" ]; then
    cp "$SIMDIR/output/joy.log" "$RUN_DIR/" || true
fi
# лог порывов ветра (sim_t фаз каждого порыва — джойнится с bag'ом по sim-времени)
if [ -n "${WIND_GUST:-}" ]; then
    if [ -f "$SIMDIR/output/wind_gust.log" ]; then
        cp "$SIMDIR/output/wind_gust.log" "$RUN_DIR/" || true
    else
        echo "⚠️ WIND_GUST задан, а output/wind_gust.log нет — публикатор не жил?" >&2
    fi
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
