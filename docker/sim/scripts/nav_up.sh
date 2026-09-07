#!/usr/bin/env bash
# ============================================================================
# Запуск nav-стороны ВНУТРИ контейнера nav: colcon build + ноды + MAVROS.
# Исполняется так:  docker exec -i p1317_nav bash -s < scripts/nav_up.sh
#                   (или make nav)
#
# Порядок source важен: ROS -> overlay (cv_bridge против CUDA-OpenCV) -> ws.
# ============================================================================
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/overlay/install/setup.bash

LOG=/root/sim_ws/output; mkdir -p "$LOG"
cd /root/sim_ws

# 1a. vins_oss — монтируется из хоста через bind mount (../../src/vins_oss).
#     При fresh-start bind mount переподключается автоматически — клонировать не нужно.
#     Все патчи (Humble, QoS, IMU skip, debug) живут в ветке 1317_debug форка:
#     https://github.com/linux100talion/VINS-MONO-ROS2/tree/1317_debug
if [ ! -d src/vins_oss/vins_estimator ]; then
    echo "  ОШИБКА: src/vins_oss не смонтирован (проверь bind mount в docker-compose.yml)"
    exit 1
fi
echo "  vins_oss: $(git -C src/vins_oss log --oneline -1 2>/dev/null || echo 'не git-репо')"

# 1b. image_transport — нужен feature_tracker/pose_graph из форка linux100talion.
#     Должен быть в Dockerfile; fallback на apt если образ старый (до rebuild).
if ! ros2 pkg list 2>/dev/null | grep -q "^image_transport$"; then
    echo "  image_transport не найден, ставим через apt..."
    apt-get update -q && apt-get install -y -q ros-humble-image-transport
fi

# 1d. numpy<2 — cv_bridge в overlay собран против NumPy 1.x; NumPy 2.x → ABI-краш.
#     Проверяем один раз: если уже <2 — ничего не делаем.
if python3 -c "import numpy; exit(0 if tuple(int(x) for x in numpy.__version__.split('.')[:2]) < (2,0) else 1)" 2>/dev/null; then
    echo "  numpy: $(python3 -c 'import numpy; print(numpy.__version__)') — OK"
else
    echo "  numpy>=2 обнаружен, даунгрейд до <2..."
    pip3 install 'numpy<2' -q
fi

# 1e. Сборка workspace — только если ещё не собран.
#     Для пересборки: make nav-rebuild (или rm -rf install внутри).
if [ ! -f install/setup.bash ]; then
    echo "  colcon build ..."
    colcon build --packages-ignore ar_demo
fi
source install/setup.bash

# 1f. control_pkg + mission_pkg + nav_pkg (правятся на хосте через bind mount).
#     ИНКРЕМЕНТАЛЬНО и ВСЕГДА: полная сборка выше идёт только при пустом install/,
#     но install/ живёт в персистентном volume (выживает fresh-start) → новые
#     пакеты иначе не подхватятся. colcon пропускает неизменённое (быстро),
#     собирает новое/правленое. Домен чистый (без rclpy) — build лишь ставит модули.
if [ -d src/control ] || [ -d src/mission ]; then
    echo "  colcon build (control_pkg, mission_pkg, nav_pkg) ..."
    if colcon build --packages-select control_pkg mission_pkg nav_pkg 2>&1 | tail -3; then
        source install/setup.bash
    else
        echo "  ⚠️ сборка control_pkg/mission_pkg не удалась (см. выше) — ARCH2 недоступен"
    fi
fi

# 1g. Форк VINS (vins_oss, C++) — ТОЖЕ инкрементально и всегда, по той же
#     причине: исходники правятся на хосте (bind mount), а install/ живёт в
#     volume — фикс в форке иначе не попадает в бинарь до ручного nav-rebuild.
#     make инкрементален: без изменений это <1 c, с правкой — десятки секунд.
if [ -d src/vins_oss/feature_tracker ]; then
    echo "  colcon build (feature_tracker, vins_estimator) ..."
    if colcon build --packages-select feature_tracker vins_estimator 2>&1 | tail -3; then
        source install/setup.bash
    else
        echo "  ⚠️ сборка форка VINS не удалась (см. выше) — летим на прежнем бинаре"
    fi
fi

# 2. Байеризатор: Gazebo RGB → /dev/rawbayer (v4l2loopback).
#    Запускается ВНЕ sim_nav.launch.py: если запустить внутри launch, его крах
#    убивает весь launch (camera_node + VINS). Здесь он изолирован.
#    Eager-init в __init__: открывает /dev/rawbayer, пишет нулевой кадр —
#    только после этого v4l2loopback разрешает G_FMT на стороне capture.
if ! pgrep -f "bayerizer.py" >/dev/null; then
    # Разрешение — из env CAMERA_W/CAMERA_H (default 1280×720). Должно совпадать
    # с SDF-камерой Gazebo и camera_node (CPU-оверрайд compose ставит 320×180).
    nohup python3 /root/sim_ws/src/sim/bayerizer.py \
        --ros-args \
        -p input_topic:=/camera/image_raw \
        -p device:=/dev/rawbayer \
        -p pattern:=GRBG \
        -p width:=${CAMERA_W:-1280} \
        -p height:=${CAMERA_H:-720} \
        -p use_sim_time:=true \
        >"$LOG/bayerizer.log" 2>&1 &
    echo "  bayerizer -> $LOG/bayerizer.log"
    # Ждём пока байеризатор активирует capture-сторону v4l2loopback (eager-init).
    echo -n "  ожидаем /dev/rawbayer..."
    for i in $(seq 1 60); do
        if v4l2-ctl -d /dev/rawbayer --get-fmt-video >/dev/null 2>&1; then
            echo " готово (${i}с)"
            break
        fi
        sleep 1
        echo -n "."
    done
else
    echo "  bayerizer уже запущен"
fi

# 2b. Конвертер gz-IMU FRD→FLU: /gz_imu/data (250Гц, мост) → /gz_imu/data_flu для
#     VINS (drop-in замена /mavros/imu/data_raw в том же фрейме, но 250Гц вместо ~21).
#     См. src/sim/imu_frd_to_flu.py и todo3.
if ! pgrep -f "imu_frd_to_flu.py" >/dev/null; then
    nohup python3 /root/sim_ws/src/sim/imu_frd_to_flu.py \
        >"$LOG/imu_frd_to_flu.log" 2>&1 &
    echo "  imu_frd_to_flu -> $LOG/imu_frd_to_flu.log"
else
    echo "  imu_frd_to_flu уже запущен"
fi

# 3. Все sim-ноды (camera_node + feature_tracker + vins_estimator),
#    уже с use_sim_time:=true.
if ! pgrep -f "sim_nav.launch" >/dev/null; then
    nohup ros2 launch /root/sim_ws/src/sim/sim_nav.launch.py \
        >"$LOG/sim_nav.log" 2>&1 &
    echo "  sim_nav.launch -> $LOG/sim_nav.log"
else
    echo "  sim_nav.launch уже запущен"
fi

# 4. MAVROS — отдельно, тоже с sim-временем (вход от mavlink_router udp:14540).
#    conn/timesync_mode:=NONE — MAVROS не синхронизирует часы с FCU, а ставит
#    ros_now() (Gazebo sim-время) на каждый пакет. Нужно потому что SITL с
#    JSON-протоколом возвращает no_time_sync и использует wall-time FCU-часы,
#    отличные от Gazebo sim-времени. При дефолтном MAVLINK-режиме offset дрейфует
#    → IMU timestamps уходят назад → VINS получает "imu message in disorder".
if ! pgrep -f "mavros_node" >/dev/null; then
    nohup ros2 run mavros mavros_node --ros-args \
        -p use_sim_time:=true \
        -p fcu_url:="udp://:14540@127.0.0.1" \
        -p conn/timesync_mode:=NONE \
        >"$LOG/mavros.log" 2>&1 &
    echo "  MAVROS   -> $LOG/mavros.log"
    # ── ПОТОКИ ТЕЛЕМЕТРИИ FCU — В ФОРГРАУНДЕ, «nav: готово» ТОЛЬКО ПОСЛЕ НИХ ──
    # ArduPilot шлёт RAW_IMU/ATTITUDE/LOCAL_POSITION_NED только по запросу (SR0_*
    # в eeprom нули, пока sitl_lv_profile их не записал). Раньше этот цикл жил в
    # фоне `( … ) &`, а «nav: готово» печаталось сразу: прогон 122716 (2026-09-07)
    # — «готово» за 7 с ДО бута FCU, узел стартовал до heartbeat, цикл дал потоки
    # через 3.5 мин, и узел 200 с «ждал EKF» при живом мосте позы. Теперь: готово
    # = потоки идут; не пошли — «nav: ОШИБКА» (make wait падает, секвенсор не летит).
    # Лог цикла — свой файл: mavros_node держит mavros.log открытым через `>` и
    # затирал строки, дописанные сюда через `>>` (потому в 122716 следов и не было).
    SLOG="$LOG/stream_rate.log"; : > "$SLOG"
    echo "  потоки FCU: жду MAVROS connected (до 180 с)..."
    ok=0
    for _ in $(seq 1 60); do
        [ "$(ros2 topic echo --once --field connected /mavros/state 2>/dev/null | head -1)" = "True" ] && { ok=1; break; }
        sleep 3
    done
    if [ "$ok" != 1 ]; then
        echo "nav: ОШИБКА: MAVROS не подключился к FCU за 180 с — см. $LOG/mavros.log, $LOG/sitl.log"
        exit 1
    fi
    ok=0
    for i in $(seq 1 8); do
        echo "--- попытка $i $(date +%T)" >> "$SLOG"
        while read -r sid rate; do
            ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
                "{stream_id: $sid, message_rate: $rate, on_off: true}" >> "$SLOG" 2>&1 || true
        done <<'STREAMS'
1 200
6 25
10 50
2 2
11 5
STREAMS
        ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
            '{message_id: 168, message_rate: 5.0}' >> "$SLOG" 2>&1 || true
        hz=$(python3 /scripts/imu_rate.py 40 15 2>>"$SLOG" | tail -1)
        echo "  IMU ≈ ${hz:-?} sim-Гц" >> "$SLOG"
        if [ -n "$hz" ] && awk "BEGIN{exit !(${hz:-0}>=15)}"; then
            ok=1
            echo "  stream_rate: IMU идёт ${hz} sim-Гц (попытка $i) — RAW_SENS/POSITION/EXTRA1/EXT_STAT/EXTRA2 + WIND запрошены"
            break
        fi
        sleep 3
    done
    if [ "$ok" != 1 ]; then
        echo "nav: ОШИБКА: потоки FCU не пошли за 8 попыток (IMU молчит) — см. $SLOG и $LOG/mavros.log"
        exit 1
    fi
else
    echo "  MAVROS   уже запущен"
fi
echo "nav: готово. Логи: docker/sim/output/"
