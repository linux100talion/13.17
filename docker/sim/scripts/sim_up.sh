#!/usr/bin/env bash
# ============================================================================
# Запуск симуляции ВНУТРИ контейнера simulator: Gazebo + ArduPilot SITL + мост.
# Исполняется так:  docker exec -i p1317_simulator bash -s < scripts/sim_up.sh
#                   (или make sim)
#
# Все процессы — в фоне (nohup), логи в /root/output (== docker/sim/output).
# bash -s = неинтерактивный шелл: ENV из Dockerfile есть (PATH к SITL),
# но .bashrc НЕ читается — ROS подключаем явно.
# ============================================================================
set -eo pipefail
source /opt/ros/humble/setup.bash

LOG=/root/output; mkdir -p "$LOG"
WORLD="${WORLD:-/root/worlds/mili_fortress.sdf}"

# Разрешение камеры: env CAMERA_W/CAMERA_H (default 1280×720). В GPU-less прогоне
# (llvmpipe) CPU-оверрайд compose ставит 320×180 — в ~16 раз меньше пикселей под
# софтрендер. SDF статичен (gz не подставляет env), поэтому при не-дефолтном
# разрешении кладём ПАТЧЕНУЮ копию модели iris_cam в /tmp и выводим её первой в
# GZ_SIM_RESOURCE_PATH — репозиторную модель не трогаем (git чист).
PATCH=/tmp/sim_models
CAM_W="${CAMERA_W:-1280}"; CAM_H="${CAMERA_H:-720}"
if [ "$CAM_W" != "1280" ] || [ "$CAM_H" != "720" ]; then
    rm -rf "$PATCH"; mkdir -p "$PATCH"
    cp -a /root/worlds/iris_cam "$PATCH/iris_cam"
    sed -i "s|<width>1280</width>|<width>${CAM_W}</width>|; \
            s|<height>720</height>|<height>${CAM_H}</height>|" \
        "$PATCH/iris_cam/model.sdf"
    export GZ_SIM_RESOURCE_PATH="$PATCH:${GZ_SIM_RESOURCE_PATH}"
    echo "  камера: SDF пропатчен до ${CAM_W}x${CAM_H} (модель из $PATCH)"
fi

# ── ВЕТЕР (Gazebo WindEffects) ────────────────────────────────────────────────
# WIND_SPD=0 → выкл (мир и модель не трогаем вовсе). Иначе тем же приёмом, что и
# разрешение: патченые копии в /tmp, репозиторные файлы чистые.
#
# Три части, без любой из них ветра НЕ будет:
#   1) <wind><linear_velocity> в мире — сам вектор ветра (МИРОВЫЕ оси);
#   2) плагин gz-sim-wind-effects-system — он и прикладывает силу;
#   3) <enable_wind>true</enable_wind> в звене — плагин действует ТОЛЬКО на такие.
#      base_link живёт в iris_with_standoffs (модель из образа ardupilot_gazebo),
#      поэтому её копию тоже патчим.
#
# WIND_DIR_DEG — куда ДУЕТ ветер, в мировых осях (0 = вдоль +X, 90 = вдоль +Y).
# Дефолт 98° = «боковой к носу»: борт на висении держит курс +8°, плюс 90°.
# ⚠️ Ветер задаётся в МИРОВЫХ осях, а курс за 40 с гуляет 2..15°, поэтому боковым
# он остаётся лишь приблизительно.
#
# WIND_FACTOR — force_approximation_scaling_factor, ОТКАЛИБРОВАН ЗАМЕРОМ, не формулой.
# Ходовое «сила = mass·factor·(v_ветра − v_звена)» НЕВЕРНО: отклик КВАДРАТИЧЕН по factor.
# Замер тремя прогонами с оракулом (BS_STAB=GzPosHold, ветер 3 м/с; сила = средний крен
# удержания × g × масса 1.75 кг):
#   factor 0.15 → крен 0.43° → 0.13 Н      (формула обещала 0.68)
#   factor 0.30 → крен 1.40° → 0.42 Н
#   factor 0.80 → крен 9.41° → 2.85 Н
# Отношение F/factor² = 5.8 / 4.7 / 4.5 — то есть F ≈ 4.6·factor² Н при ветре 3 м/с.
# Значит factor масштабирует не силу, а СКОРОСТЬ, к которой уже применяется квадратичное
# лобовое. Наивная линейная прикидка ошибается впятеро в одну сторону при малых factor и
# недооценивает при больших.
# Цель — реалистичный бриз: ½ρv²·Cd·A = 0.5·1.225·9·1.0·0.1 ≈ 0.55-0.8 Н на 3 м/с, то
# есть 0.31-0.45 м/с² и крен удержания 1.8-2.6° (настоящий борт в 3 м/с держит 3-6°).
# Отсюда 0.4 (расчётно 0.74 Н, крен ≈2.4°). При 0.8 выходило 9.4° — это уже ~7 м/с.
# ⚠️ Закон квадратичный, поэтому factor нельзя пересчитывать пропорцией: меняешь —
# перемеряй тем же прогоном с оракулом.
WIND_SPD="${WIND_SPD:-0}"
if [ "$WIND_SPD" != "0" ]; then
    WIND_DIR_DEG="${WIND_DIR_DEG:-98}"; WIND_FACTOR="${WIND_FACTOR:-0.4}"
    mkdir -p "$PATCH"
    read -r WX WY <<<"$(awk -v s="$WIND_SPD" -v d="$WIND_DIR_DEG" \
        'BEGIN{r=d*3.14159265358979/180; printf "%.4f %.4f", s*cos(r), s*sin(r)}')"
    # 1+2: мир — вектор ветра и плагин, сразу после открывающего <world>
    cp "$WORLD" "$PATCH/world_wind.sdf"
    python3 - "$PATCH/world_wind.sdf" "$WX" "$WY" "$WIND_FACTOR" <<'PYEOF'
import re, sys
path, wx, wy, factor = sys.argv[1:5]
s = open(path).read()
block = (f'\n    <wind><linear_velocity>{wx} {wy} 0</linear_velocity></wind>\n'
         '    <plugin filename="gz-sim-wind-effects-system"\n'
         '            name="gz::sim::systems::WindEffects">\n'
         f'      <force_approximation_scaling_factor>{factor}'
         '</force_approximation_scaling_factor>\n'
         '    </plugin>\n')
s = re.sub(r'(<world[^>]*>)', lambda m: m.group(1) + block, s, count=1)
open(path, 'w').write(s)
PYEOF
    # 3: корпус — разрешить ветру действовать на base_link
    rm -rf "$PATCH/iris_with_standoffs"
    cp -a /root/ardupilot_gazebo/models/iris_with_standoffs "$PATCH/iris_with_standoffs"
    sed -i "s|<link name='base_link'>|<link name='base_link'><enable_wind>true</enable_wind>|" \
        "$PATCH/iris_with_standoffs/model.sdf"
    export GZ_SIM_RESOURCE_PATH="$PATCH:${GZ_SIM_RESOURCE_PATH}"
    WORLD="$PATCH/world_wind.sdf"
    echo "  ВЕТЕР: ${WIND_SPD} м/с, куда дует ${WIND_DIR_DEG}° (мир: ${WX} ${WY} 0), factor ${WIND_FACTOR}"
else
    echo "  ветер: выкл (WIND_SPD=0)"
fi

# ── ТОЧКА СПАВНА: «где сел — там и стартуем» ─────────────────────────────────
# SPAWN_POSE="x y z roll pitch yaw" — где поставить борт, в осях МИРА Gazebo
# (x-восток, y-север, z-вверх; yaw 0 = нос на восток, радианы). Пусто = штатный
# спавн из SDF (0 0 0.245 0 0 0 — центр площадки).
#
# Позу места посадки достаёт из bag'а прошлого прогона src/lab/spawn_pose.py,
# freefly_lv.sh умеет одним env: SPAWN_FROM=<каталог прогона>.
#
# Вместо чисел можно дать ИМЯ ПРЕСЕТА: SPAWN_POSE=among_trees → берётся
# docker/sim/output/spawn/among_trees (пресеты пишет src/lab/spawn_save.py —
# он вынимает из прогона только позу, дальше прогон можно удалять).
#
# Патчим тем же приёмом, что ветер и разрешение: КОПИЯ мира в /tmp, репозиторный
# SDF чист. Проверено 2026-08-24: борт встаёт ровно в заданную точку с заданным
# курсом и стоит неподвижно (bit-in-bit 48 с).
#
# ⚠️ ТОЛЬКО С ВЕТРОМ (WIND_SPD ≠ 0, дефолт freefly — 5). В БЕЗВЕТРЕННОМ прогоне
#    борт на земле не удерживает ничто: трения о землю в этой связке (dartsim +
#    <surface> из SDF) фактически нет, сопротивления воздуха у модели тоже нет —
#    единственное демпфирование даёт плагин WindEffects (сила ∝ разности скоростей
#    ветра и звена). Замер 2026-08-24, WIND_SPD=0: борт получает при подключении
#    SITL толчок ~0.06 м/с и едет ВЕЧНО (скорость не падает за 45 с, 13 м за
#    4 мин) — и это НЕ связано со спавном: при WIND_SPD=0 так же уезжает штатный
#    спавн в начале координат. С WIND_SPD=5 обе конфигурации стоят намертво.
# ⚠️ z — точка ПОКОЯ (верх земли + клиренс ног 0.195). На ровной травяной
#    площадке это 0.245; spawn_pose.py берёт фактическое z посадки.
# ⚠️ env применяется при СОЗДАНИИ контейнера → менять только через fresh-start
#    (capture_scene.sh делает его сам, когда задано разрешение — freefly_lv
#    задаёт всегда).
# ⚠️ Истинная поза Gazebo теперь начинается НЕ с нуля (скрипты разбора, если они
#    считают старт началом координат, надо кормить той же SPAWN_POSE). Полётнику
#    сдвиг безразличен: home/origin EKF он ставит на буте от своей точки (LV=1 —
#    по SIM-GPS, LV=2 — SET_GPS_GLOBAL_ORIGIN от ноды), локальные координаты
#    снова начинаются с нуля в точке спавна.
SPAWN_POSE="${SPAWN_POSE:-}"
# Вместо шести чисел можно дать ИМЯ пресета — файла в output/spawn/ (это
# /root/output/spawn в контейнере, каталог смонтирован с хоста). Пресеты пишет
# src/lab/spawn_save.py: «взять из прогона только позу и сохранить под именем»,
# после чего сам прогон можно удалять. Формат файла: комментарии «#» + одна
# строка «x y z r p y».
case "$SPAWN_POSE" in
    ''|*[0-9]*[!0-9.eE+\ -]*|*[!0-9.eE+\ -]*)
        if [ -n "$SPAWN_POSE" ]; then
            SPAWN_NAME="$SPAWN_POSE"
            PRESET="/root/output/spawn/$SPAWN_NAME"
            if [ ! -f "$PRESET" ]; then
                echo "  ОШИБКА: нет пресета спавна '$SPAWN_NAME' ($PRESET)" >&2
                echo "  есть: $(ls /root/output/spawn 2>/dev/null | tr '\n' ' ')" >&2
                echo "  сделать: python3 src/lab/spawn_save.py <прогон> $SPAWN_NAME" >&2
                exit 2
            fi
            SPAWN_POSE="$(grep -vE '^[[:space:]]*(#|$)' "$PRESET" | head -1 \
                          | tr -s ' ' | sed 's/^ *//; s/ *$//')"
            echo "  спавн из пресета '$SPAWN_NAME' ($PRESET)"
        fi
        ;;
esac
if [ -n "$SPAWN_POSE" ]; then
    if [ "$(wc -w <<< "$SPAWN_POSE")" != "6" ]; then
        echo "  ОШИБКА: SPAWN_POSE='$SPAWN_POSE' — жду 6 чисел «x y z r p y»" >&2
        exit 2
    fi
    [ "${WIND_SPD:-0}" = "0" ] && echo "  ⚠️ СПАВН при WIND_SPD=0: борт поедет (см. комментарий в sim_up.sh)" >&2
    mkdir -p "$PATCH"
    cp "$WORLD" "$PATCH/world_spawn.sdf"
    python3 - "$PATCH/world_spawn.sdf" "$SPAWN_POSE" <<'PYEOF'
import re, sys
path, pose = sys.argv[1], sys.argv[2]
s = open(path).read()
# <include> именно с <name>iris_cam</name> — прочие модели мира не трогаем
pat = re.compile(r'(<include>(?:(?!</include>).)*?<name>iris_cam</name>'
                 r'(?:(?!</include>).)*?<pose>)([^<]*)(</pose>)', re.S)
s, n = pat.subn(lambda m: m.group(1) + pose + m.group(3), s, count=1)
if n != 1:
    sys.exit("ОШИБКА: не нашёл <pose> модели iris_cam в мире")
open(path, 'w').write(s)
PYEOF
    WORLD="$PATCH/world_spawn.sdf"
    echo "  СПАВН: борт в ($SPAWN_POSE) вместо штатного 0 0 0.245 0 0 0"
else
    echo "  спавн: штатный (центр площадки, SPAWN_POSE не задан)"
fi

# 1. Gazebo Harmonic — мир + дрон с камерой (ArduPilotPlugin слушает 9002).
if ! pgrep -f "gz sim" >/dev/null; then
    nohup gz sim -s --headless-rendering -v4 -r "$WORLD" >"$LOG/gz_sim.log" 2>&1 &
    echo "  gz sim   -> $LOG/gz_sim.log"
else
    echo "  gz sim   уже запущен"
fi

sleep 5  # дать Gazebo поднять физику/плагины до подключения SITL

# 2. ArduPilot SITL — подключается к ArduPilotPlugin@9002, MAVLink на tcp:5760.
#    --no-mavproxy: телеметрию разводит mavlink_router, консоль MAVProxy не нужна.
if ! pgrep -f "sim_vehicle" >/dev/null; then
    # SITL пишет eeprom.bin (accel-калибровка + параметры) в cwd. Запускаем из
    # /root/sitl_state — это named volume sitl_eeprom, поэтому eeprom переживает
    # fresh-start (одноразовая калибровка не повторяется на каждом старте).
    # --defaults (--add-param-file) применяется на каждом boot поверх eeprom, так
    # что правки sitl-extra.parm по-прежнему подхватываются, а калибровка (её нет
    # в .parm) сохраняется в eeprom. См. docker/sim/todo.txt.
    mkdir -p /root/sitl_state
    # SIM_HOME — ДОМ SITL «lat,lon,alt,heading». Держать согласованным с
    # <spherical_coordinates> мира и origin_lat/lon/alt ноды: SITL рисует от дома
    # магнитное поле, EK3 строит WMM от origin — разъедутся, и арма нет
    # («PreArm: Check mag field», урок lv2_replay_20260824_034433). Дефолт — Киев,
    # та же точка, что начало координат Gazebo (см. worlds/mili_fortress.sdf).
    # Без ключа SITL брал CMAC (Канберра) из Tools/autotest/locations.txt.
    SIM_HOME="${SIM_HOME:-50.450100,30.523400,180,0}"
    echo "  дом SITL: $SIM_HOME (= начало координат мира)"
    ( cd /root/sitl_state && nohup sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
        --custom-location="$SIM_HOME" \
        --add-param-file=/root/ardupilot/Tools/autotest/default_params/gazebo-iris.parm \
        --add-param-file=/root/sitl-extra.parm \
        --no-rebuild --no-mavproxy >"$LOG/sitl.log" 2>&1 & )
    echo "  SITL     -> $LOG/sitl.log (cwd=/root/sitl_state, eeprom персистентный)"
else
    echo "  SITL     уже запущен"
fi

# 3. Мост Gazebo -> ROS2: камера + /clock (источник sim-времени) + ground-truth
#    одометрия дрона (gz-position-hold) + IMU дрона @250Гц (для VINS — обход
#    MAVLink-телеметрии, которая капается до ~21-50 Гц; реальный борт берёт IMU с
#    FCU на ~200Гц, в sim телеметрийный тракт столько не может → кормим gz-IMU).
#    gz IMU-сенсор уже есть (его юзает ArduPilotPlugin), публикует 250Гц sim;
#    мостим его длинный gz-топик в чистый ROS /gz_imu/data (remap).
GZ_IMU="/world/default/model/iris_cam/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu"
if ! pgrep -f "ros_gz_bridge" >/dev/null; then
    nohup ros2 run ros_gz_bridge parameter_bridge \
        "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image" \
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
        "/model/iris_cam/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
        "${GZ_IMU}@sensor_msgs/msg/Imu[gz.msgs.IMU" \
        --ros-args -r "${GZ_IMU}:=/gz_imu/data" \
        >"$LOG/ros_gz_bridge.log" 2>&1 &
    echo "  ros_gz_bridge -> $LOG/ros_gz_bridge.log"
else
    echo "  ros_gz_bridge уже запущен"
fi

echo "simulator: готово. Логи: docker/sim/output/"
