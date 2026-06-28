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
    # Поднять частоты потоков MAVLink. КРИТИЧНО ждать коннект MAVROS<->FCU ПО ФАКТУ,
    # а не фиксированным sleep: раньше был `sleep 20` — на медленном старте / низком
    # RTF MAVROS к 20-й секунде ещё НЕ подключён к FCU, REQUEST_DATA_STREAM уходит в
    # никуда, и /mavros/imu/data_raw остаётся ПУСТЫМ весь прогон (источник флаки
    # "в части прогонов IMU не пишется"). Ниже: ждём connected=True, затем ставим
    # потоки с РЕТРАЯМИ, пока IMU реально не пойдёт (запрос может потеряться на UDP).
    # (accel-калибровка делается ОДИН раз через make sitl-cal и живёт в
    #  персистентном eeprom — на каждом старте не повторяется, FCU не ребутит.)
    (
        for _ in $(seq 1 60); do
            [ "$(ros2 topic echo --once --field connected /mavros/state 2>/dev/null | head -1)" = "True" ] && break
            sleep 3
        done
        # Частоты потоков ставим per-message через SET_MESSAGE_INTERVAL (MAV_CMD 511,
        # /mavros/cmd/command) — НАДЁЖНЕЕ устаревшего REQUEST_DATA_STREAM
        # (set_stream_rate): в ArduPilot 4.8 он часто не отрабатывает, из-за чего
        # /mavros/imu/data_raw оставался ~33 Гц (мало для VINS и для FFT гиро).
        # interval_us=5000 = запрос 200 Гц; FCU отдаёт НЕ выше SCHED_LOOP_RATE (тут
        # 100) → реальный потолок IMU ~100 sim-Гц. msgid: 27=RAW_IMU
        # (→ /mavros/imu/data_raw, вход VINS), 30=ATTITUDE (→ /mavros/imu/data),
        # 32=LOCAL_POSITION_NED, 33=GLOBAL_POSITION_INT (→ pose + rel_alt).
        set_interval() {  # $1=msgid  $2=interval_us
            ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
                "{command: 511, param1: $1.0, param2: $2.0}" >> "$LOG/mavros.log" 2>&1
        }
        for _ in $(seq 1 20); do
            # IMU: интервал на ВСЕ источники — mavros строит /imu/data_raw из того,
            # что реально шлёт FCU (26=SCALED_IMU, 27=RAW_IMU, 105=HIGHRES_IMU,
            # 116/129=SCALED_IMU2/3). 5000us = запрос 200 Гц (FCU cap ~SCHED_LOOP=100).
            for mid in 26 27 105 116 129; do set_interval "$mid" 5000; done
            set_interval 30  5000     # ATTITUDE → /mavros/imu/data (ориентация)
            set_interval 32 40000     # LOCAL_POSITION_NED  → 25 Гц
            set_interval 33 40000     # GLOBAL_POSITION_INT → 25 Гц
            # подстраховка устаревшим REQUEST_DATA_STREAM: stream 1 = RAW_SENSORS (IMU)
            ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
                '{stream_id: 1, message_rate: 200, on_off: true}' >> "$LOG/mavros.log" 2>&1
            # Подтверждаем ПО SIM-ЧАСТОТЕ (не по факту наличия): на низком RTF wall-rate
            # мизер, поэтому imu_rate.py считает Гц из header.stamp. Цель >= 80 sim-Гц.
            hz=$(python3 /scripts/imu_rate.py 40 15 2>/dev/null | tail -1)
            echo "  stream_rate: /mavros/imu/data_raw ≈ ${hz:-?} sim-Гц" >> "$LOG/mavros.log"
            if [ -n "$hz" ] && awk "BEGIN{exit !(${hz:-0}>=80)}"; then
                echo "  stream_rate: IMU подтверждён ${hz} sim-Гц"; break
            fi
            sleep 3
        done
    ) &
else
    echo "  MAVROS   уже запущен"
fi

echo "nav: готово. Логи: docker/sim/output/"
