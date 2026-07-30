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
# WIND_FACTOR — force_approximation_scaling_factor. Плагин считает силу как
# mass·factor·(v_ветра − v_звена), то есть factor это обратное время нарастания, а
# дефолтная единица дала бы 3 м/с² при ветре 3 м/с — вдвое больше веса рамы в
# горизонте. Физика: лобовое сопротивление квадрокоптера на 3 м/с ≈ ½ρv²·Cd·A =
# 0.5·1.225·9·1.0·0.1 ≈ 0.55 Н, при массе 1.75 кг это 0.31 м/с² → factor ≈ 0.1.
# Берём 0.15 (0.79 Н, 0.45 м/с², удержание требует крена ≈2.6°) — заметный, но
# реалистичный бриз. Проверяется замером: сколько градусов держит оракул.
WIND_SPD="${WIND_SPD:-0}"
if [ "$WIND_SPD" != "0" ]; then
    WIND_DIR_DEG="${WIND_DIR_DEG:-98}"; WIND_FACTOR="${WIND_FACTOR:-0.15}"
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
    ( cd /root/sitl_state && nohup sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
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
