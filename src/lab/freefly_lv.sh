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
#   SPAWN_FROM=docker/sim/output/joystick/lv1_joy_20260824_140447 \
#       bash src/lab/freefly_lv.sh   # стартовать с места посадки того прогона
# Любой параметр (BS_*, WIND_SPD, RES, GDRIVE_UP, MP4, TOPICS_EXTRA...)
# переопределяется через env; дефолты ниже — эталонные команды из Q.txt.
# Приоритет: env снаружи > docker/sim/.env (локальный профиль бокса, gitignore —
# тот же файл, что читает compose: WORLD/SPAWN_POSE едут туда же) > дефолт скрипта.
# Шпаргалка по .env (ключи, правила разбора, грабли) — docker/sim/env.md.
# В LV=0 BS_VINS_MIN не задаётся (дефолт ноды 40, как в эталонной команде №1).
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
        if [ -z "${!key+x}" ]; then
            export "$key=$val"
            ENV_DEFAULTS="$ENV_DEFAULTS $key=$val"
        fi
    done < "$SIMDIR/.env"
fi
if [ -n "$ENV_DEFAULTS" ]; then
    echo "freefly_lv: дефолты бокса из docker/sim/.env:$ENV_DEFAULTS"
fi

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
EEPROM_CMD="PYTHONPATH=/root/ardupilot/modules/mavlink python3 /scripts/sitl_lv_profile.py $LV"
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

# ── 2) env-профиль полёта (эталон из Q.txt; всё переопределяется снаружи) ────
# WIND_SPD: дефолт снижен 10 → 5 (2026-08-23, просьба Андрея) — 10 был из
# LV-серии (стресс ветром), для регулярных прогонов серии joystick хватает 5.
export WIND_SPD="${WIND_SPD:-5}"
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
# ── ГЕЙНЫ ДЕМПФЕРА: kp 30 → 90 (шаг 1 ЗАШЁЛ), ki 30 (шаг 1b откачен) ────────
# ШАГ 1 ЗАШЁЛ (прогон 061854 против 034514, окно взлёта): пик боковой 1.07 →
# 0.78 м/с и приходит раньше (2.4 → 1.8 с), снос за 5 с 3.8 → 2.8 м, откат после
# торможения −0.64 → −0.22 м/с — звон практически ушёл. Осталась ровно ПЕРВАЯ
# ПЯТЁРКА СЕКУНД, и у неё своё имя: ДОЛГ ИНТЕГРАТОРА. Ветер в этом мире стоит
# 43-44 PWM (замер: γ=0.65 м/с² ÷ α=0.0147 — совпал в обоих прогонах), держать
# их может ТОЛЬКО интегратор: чтобы столько выдал P, нужна постоянная ошибка
# 44/90 = 0.5 м/с, то есть борт обязан ехать. А интегратор набирает их за
# I/ki = 1.45 м ИЗМЕРЕННОГО пути ≈ 2.6 м истинного (канал на разгоне видит 0.55
# истины) — ровно наблюдаемые 2.8 м. От kp это не зависит вовсе.
# ШАГ 1b (ki 30 → 60) ОТКАЧЕН В ДЕФОЛТЕ — прогон 064122 показал ОБЕ стороны:
#   подлёт стал лучше ровно как предсказано (пик 0.78 → 0.68 м/с, снос за 5 с
#   2.83 → 1.60 м, долг 1.44 → 0.78 м измеренного пути),
#   НО затухание село в ноль: ζ 0.11 → 0.02, период 10.2 → 7.8 с. Борт перестал
#   УСПОКАИВАТЬСЯ: после любого толчка качается ±0.2-0.5 м/с и не гаснет — это и
#   ощущается как «стало хуже», хотя первые 5 с объективно лучше.
# ⚠️ Формула ζ ≈ (kp/2)·√(α/ki) ЗАВЫШАЕТ: по трём прогонам измеренное ζ = 0.14 /
#   0.11 / 0.02 против «идеальных» 0.33 / 0.98 / 0.63, зато ω_n = √(α·ki) сходится
#   (0.59 / 0.62 / 0.80 против 0.65 / 0.65 / 0.84). Значит kp затухания почти НЕ
#   даёт — его съедает запаздывание канала (фаза ∝ ω·τ_s), а ki поднимает ω и тем
#   ухудшает ζ. То есть ki торгует подлёт против успокоения НАПРЯМУЮ, и узкое
#   место теперь одно — τ_s (0.60-1.05 с).
# Дефолт вернулся на ki=30 (последняя конфигурация, которая УСПОКАИВАЕТСЯ).
# A/B руками (гейны едут в мету прогона, NAME — чтобы архив сам себя называл):
#   A  BS_ROLL_RATE_KP=90 BS_ROLL_RATE_KI=30 BS_PITCH_RATE_KP=90 \
#      BS_PITCH_RATE_KI=30 NAME=ab_kp90_ki30 bash src/lab/freefly_lv.sh
#   B  BS_ROLL_RATE_KP=90 BS_ROLL_RATE_KI=60 BS_PITCH_RATE_KP=90 \
#      BS_PITCH_RATE_KI=60 NAME=ab_kp90_ki60 bash src/lab/freefly_lv.sh
# Сравнивать честно можно только ОДИНАКОВЫЕ полёты: отрыв → руки прочь от крена
# → висеть ~30 с на одной высоте → посадка. Прогоны 061854/064122 этим и
# отличались (161 и 99 с с манёврами до 4.5 и 1.35 м) — участок удержания в них
# несравним, сравнимо только окно взлёта.
# Следующая ступень по существу — резать τ_s: BS_IPM_WIN 0.5 → 0.3 при ki=60.
# ⚠️ ki НЕ ОБНУЛЯТЬ, и это не небрежность. ki по ошибке СКОРОСТИ — это СКРЫТЫЙ
# ПОЗИЦИОННЫЙ контур (интеграл скорости = смещение), то есть контур уже второго
# порядка, а единственное демпфирование в нём — kp. Разбор прогона 034514:
# пики боковой +1.07 → −0.64 → +0.49 → −0.26, период 10.7 с, ζ ≈ 0.14 (комфорт
# 0.7), команда отстаёт от скорости на 2.2 с = четверть периода (подпись чистого
# интегратора). Поднять ОБА гейна = поднять частоту звона; лечит только kp.
# Идентификация контура по тому же бэгу: v̇ = −0.0141·PWM + 0.023·v + 0.675;
# аэродемпфера нет (β>0, см. память «Земля в симе без трения»), постоянные
# 0.675 м/с² — ветер, и держит их именно интегратор (0.675/0.0141 = 48 PWM,
# ровно наблюдаемое установившееся смещение). Отсюда ki=0 НЕЛЬЗЯ: контрфактный
# свип по этой модели даёт 23 м сноса за 45 с; ki=15 тоже хуже 30 (медленнее
# набирает противоветер). kp=90 при ki=30: пик подлёта 0.77 → 0.41 модельных.
# ⚠️ Старый свип крена в config.py (120 хуже 60 хуже 30) НЕ противоречит: он
# мерился при ki=0 и на канале ДО латча высоты и ipm_acc_tau, то есть на сигнале
# со смещением. Поэтому идём лесенкой по прогону на ступень и смотрим ПЕРИОД:
# упал с 10.7 с до ~5 с и PWM залупил — упёрлись в запаздывание канала
# (τ_s ≈ 0.55-1.05 с), ступень назад. Дальше по остатку: если долг интегратора
# всё ещё виден — не ki=90 (звенит), а СТАНЦИЯ-КИПИНГ BS_ROLL_POS_KP=0.3-0.5
# поверх ki (явный позиционный контур с ограничением скорости возврата), либо
# резать сам τ_s: BS_IPM_WIN 0.5 → 0.3.
export BS_PITCH_RATE_KI="${BS_PITCH_RATE_KI:-30}"
export BS_PITCH_RATE_KP="${BS_PITCH_RATE_KP:-90}"
export BS_PITCH_RATE_CMD_GAIN="${BS_PITCH_RATE_CMD_GAIN:-5}"
export BS_ROLL_RATE_CMD_GAIN="${BS_ROLL_RATE_CMD_GAIN:-5}"
export BS_ROLL_IMAX="${BS_ROLL_IMAX:-150}"
export BS_ROLL_OSIGN="${BS_ROLL_OSIGN:-1}"
export BS_ROLL_RATE_KI="${BS_ROLL_RATE_KI:-30}"
export BS_ROLL_RATE_KP="${BS_ROLL_RATE_KP:-90}"
export BS_SLEW="${BS_SLEW:-300}"
export BS_YAW_ARM_FRAMES="${BS_YAW_ARM_FRAMES:-5}"
export BS_YAW_KD="${BS_YAW_KD:-6}"
export BS_YAW_KI="${BS_YAW_KI:-0}"
export BS_YAW_KP="${BS_YAW_KP:-0}"
export BS_YAW_LEAK="${BS_YAW_LEAK:-8}"
export BS_YAW_MAX_RATE="${BS_YAW_MAX_RATE:-100}"
export BS_YAW_RATE_FULL="${BS_YAW_RATE_FULL:-60}"
export BS_YAW_SMOOTH="${BS_YAW_SMOOTH:-5}"
# /mavros/state (1 Гц) — для разбора joystick-серии: латчи режимов (LOITER!) и
# арм/дизарм видны в bag (двойной щелчок CH6 в полёте 182409 без него не объяснить).
# /mission/status — гейт LOITER-на-VINS от лётной ноды (debug-HUD): joy_timeline
# показывает переходы «VINS READY t=…» в ленте событий.
# /feature — счётчик фич трекера для строки FEAT пост-рендера HUD (hud_video.py);
# PointCloud 10 sim-Гц — копейки против /image_color.
export TOPICS_EXTRA="${TOPICS_EXTRA:-/joy /mavros/state /mission/status /feature /odometry /model/iris_cam/odometry /flow_dbg /flow_dbg2 /flow_dbg6 /flow_dbg7 /flow_dbg8 /flow_dbg9}"
export GDRIVE_UP="${GDRIVE_UP:-0}"
export MP4="${MP4:-1}"
export FRAMES="${FRAMES:-0}"    # JPEG-кадры не нужны (просьба 2026-08-22): только mp4

if [ "$LV" = "1" ] || [ "$LV" = "2" ]; then   # общие vision-добавки (Q.txt)
    export BS_VINS_MIN="${BS_VINS_MIN:-300}"
    export BS_FF_LOITER="${BS_FF_LOITER:-1}"
    export BS_VISION_VEL="${BS_VISION_VEL:-1}"
    export BS_VISION_POSE_SRC="${BS_VISION_POSE_SRC:-extern}"
fi
if [ "$LV" = "1" ]; then                 # GPS есть на буте, глушится В ПОЛЁТЕ
    export BS_GPS_DISABLE="${BS_GPS_DISABLE:-1}"
fi
if [ "$LV" = "2" ]; then                 # GPS ОТСУТСТВУЕТ С БУТА (см. шапку)
    export BS_GPS_DENIED="${BS_GPS_DENIED:-1}"
    export BS_GPS_DISABLE="${BS_GPS_DISABLE:-0}"   # глушить нечего
    export BS_SET_ORIGIN="${BS_SET_ORIGIN:-1}"     # origin руками (GPS не поставит)
    export BS_ALT_SRC="${BS_ALT_SRC:-baro}"        # миссия: баро (независим от EKF)
    # перцепция: EKF local z — гладко И без лага (замер 2026-08-24); global в
    # LV=2 тоже жив (EKF с origin), но local не требует global-канала вовсе
    export BS_PERC_ALT_SRC="${BS_PERC_ALT_SRC:-local}"
    # ноль высоты перцепции по арму: EKF local z смещён вниз на 0.2-0.3 м,
    # и на полёте ниже полуметра гейт земли IPM не открывался вовсе
    # (прогоны 183305/185921 — демпфер 0 PWM); пишем явно, чтобы мета
    # прогона отвечала «латч был или нет» без раскопок в дефолтах
    export BS_PERC_ALT_ZERO="${BS_PERC_ALT_ZERO:-1}"
fi

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
if [ "${HUD_MP4:-1}" = "1" ] && [ -d "$SIMDIR/output/scene_bag" ]; then
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
if [ "${IPM_MP4:-1}" = "1" ] && [ -d "$SIMDIR/output/scene_bag" ]; then
    echo "=== пост-рендер канала вида сверху (scene_ipm.mp4) ==="
    IPM_ENVS=()
    while IFS= read -r k; do IPM_ENVS+=(-e "$k"); done \
        < <(env | sed -n 's/^\(BS_[A-Z0-9_]*\)=.*/\1/p' | sort)
    docker exec "${IPM_ENVS[@]}" "$NAV" bash -lc 'source /opt/ros/humble/setup.bash;
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
    env | { grep -E '^(BS_|WIND_|SPAWN_|TOPICS_|GDRIVE_|MP4|HUD_MP4|IPM_MP4)' || true; } | sort
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
    if [ -d "$SIMDIR/output/scene_bag" ]; then
        # mv — ИЗНУТРИ контейнера (root): bag создан root'ом, а перенос каталога
        # в другой родитель требует записи на сам каталог (обновляется его "..") —
        # с хоста (andriy) это Permission denied. Так пропал bag прогона 182409.
        if docker exec "$NAV" mv /root/sim_ws/output/scene_bag \
                "/root/sim_ws/output/joystick/$NAME/bag" 2>/dev/null \
           || mv "$SIMDIR/output/scene_bag" "$RUN_DIR/bag" 2>/dev/null; then
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
