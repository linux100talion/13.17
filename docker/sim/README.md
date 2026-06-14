# Симуляционный стек (SITL + Gazebo Harmonic)

Запуск всей навигационной системы на ноутбуке **без реального железа**:
ArduPilot SITL эмулирует полётный контроллер, Gazebo Harmonic рендерит мир и
виртуальную камеру, VINS-Mono и нейросети работают как на дроне.

Параллелен боевому стеку `docker/orin/` (Jetson Orin + реальная камера/полётник).

## Архитектура

```
┌─────────────────────────┐     MAVLink tcp:5760      ┌──────────────────┐
│  simulator              │ ────────────────────────► │ mavlink_router   │
│  ┌───────────────────┐  │                           │  ├─ udp:14550 QGC│
│  │ Gazebo Harmonic   │  │                           │  ├─ udp:14540 ───┼──┐
│  │  + камера (SDF)   │  │                           │  └─ udp:14541 cpp│  │
│  │ ArduPilot SITL    │  │                           └──────────────────┘  │
│  └───────────────────┘  │                                                 │
│   GPU (рендер) + X11     │   /camera/image_raw (ros_gz_bridge)            │
└───────────┬─────────────┘ ──────────────────────────┐                    │
            │                                          ▼                    ▼
            │                              ┌────────────────────────────────────┐
            │                              │  nav (CUDA)                         │
            │                              │   feature_tracker → vins_estimator  │
            │                              │   нейросети (YOLO / DINOv2 / FAISS) │
            │                              │   MAVROS ◄── udp:14540              │
            └──────────────────────────────  GPU (тензоры, без графики)         │
                                           └────────────────────────────────────┘
```

## Контейнеры

| Сервис           | База                          | GPU            | Роль |
|------------------|-------------------------------|----------------|------|
| `simulator`      | osrf/ros:humble-desktop       | graphics+compute | Gazebo Harmonic + ArduPilot SITL + ros_gz_bridge |
| `mavlink_router` | radarku/mavlink-router        | —              | Раздача MAVLink по UDP |
| `nav`            | nvidia/cuda 12.2 + ros-base   | compute        | VINS-Mono + нейросети + MAVROS |

## Запуск

```bash
# 1. Разрешить контейнеру доступ к дисплею (для GUI Gazebo)
xhost +local:root
export DISPLAY=:0

# 2. Сборка (долго — SITL и Gazebo собираются из исходников)
cd docker/sim
docker compose build

# 3. Поднять стек
docker compose up -d

# 4. Запустить Gazebo + SITL внутри контейнера simulator
docker exec -it p1317_simulator bash
#   (внутри) gz sim -r <world>.sdf &
#   (внутри) sim_vehicle.py -v ArduCopter -f gazebo-iris --console

# 5. Собрать и запустить ноды в nav
docker exec -it p1317_nav bash
#   (внутри) cd /root/sim_ws && colcon build && source install/setup.bash
#   (внутри) ros2 launch mavros apm.launch fcu_url:=udp://:14540@
```

## Виртуальная камера (camera_node работает "как есть")

В симуляции камера-нода (`camera_pkg`) НЕ переписывается. Вместо реального
ArduCam она читает кадры из виртуального V4L2-устройства `/dev/rawbayer`,
которое создаёт модуль ядра **v4l2loopback**. Байеризатор берёт RGB из Gazebo,
"портит" его до сырого Bayer (как реальный сенсор) и пишет в это устройство.

```
Gazebo /camera/image_raw (RGB)
  → bayerizer.py        RGB→Bayer16
  → write() → /dev/rawbayer (v4l2loopback)
  → camera_node  (v4l2-ctl читает BA10 → CUDA debayer → /image_mono + OpenHD)
```

Нода меняется только параметром: `device:=/dev/rawbayer` (на железе — `/dev/video0`).

### Настройка на ХОСТЕ (v4l2loopback — модуль ядра, не ставится в docker)

```bash
# 1. Установить и загрузить модуль (один раз)
sudo apt install v4l2loopback-dkms v4l2loopback-utils
# Создаём устройство с фиксированным именем /dev/rawbayer.
# Подбери номер устройства так, чтобы не конфликтовал с вебкамерой.
sudo modprobe v4l2loopback devices=1 video_nr=9 card_label="rawbayer" exclusive_caps=0

# 2. Симлинк на фиксированное имя (docker пробрасывает именно /dev/rawbayer)
sudo ln -sf /dev/video9 /dev/rawbayer
```

> ⚠️ Формат/размер кадра: camera_node берёт `Sizeimage` из `v4l2-ctl
> --get-fmt-video`, поэтому ему важно, чтобы устройство отдавало
> `width*height*2` байт (16-бит на пиксель). Если negotiation формата
> BA10 на loopback не проходит — проверь это первым делом.

### Запуск в контейнере nav

```bash
docker exec -it p1317_nav bash
#   (внутри) cd /root/sim_ws && colcon build && source install/setup.bash
#   (внутри) терминал 1 — байеризатор:
python3 src/nav/bayerizer.py --ros-args \
    -p input_topic:=/camera/image_raw -p device:=/dev/rawbayer -p pattern:=GRBG
#   (внутри) терминал 2 — камера-нода как на железе, но с другим device:
ros2 run camera_pkg camera_node --ros-args -p device:=/dev/rawbayer
```

> Если цвета перепутаны — поменяй `pattern` (GRBG/RGGB/BGGR/GBRG).

### Запуск VINS в контейнере nav

Конфиг симуляции — `sim.yaml` (нулевая дисторсия, интринсики Gazebo-камеры,
пути `/root/sim_ws/`). Лежит рядом с боевым:
`src/vins/VINS-MONO-ROS2/config_pkg/config/sim.yaml`.

```bash
CFG=/root/sim_ws/src/vins/VINS-MONO-ROS2/config_pkg/config/sim.yaml
ros2 run feature_tracker feature_tracker --ros-args -p config_file:=$CFG
ros2 run vins_estimator vins_estimator --ros-args -p config_file:=$CFG \
    --remap /feature_tracker/feature:=/feature \
    --remap /feature_tracker/restart:=/restart
```

## Открытые задачи (грабли симуляции)

1. **Разрешение камеры.** `camera_node` по умолчанию ждёт 1280×720 (захардкожено
   в конструкторе) — под это посчитан `sim.yaml`. Камера Gazebo в `model.sdf` —
   1920×1200. Согласовать: проще выставить Gazebo-камеру в 1280×720. Если оставить
   1920×1200 — поправить `width_`/`height_` в ноде И интринсики в `sim.yaml`
   (fx=fy=960, cx=960, cy=600).
2. **`use_sim_time:=true` всем нодам.** Gazebo публикует `/clock`. Без этого
   таймстампы кадров и IMU разойдутся с wall-clock → VINS диверджит молча.
3. **IMU-источник.** VINS ждёт IMU на `/mavros/imu/data_raw` из SITL.
   Проверить частоту и шум — `sim.yaml` задаёт заниженный шум (нет вибраций рамы),
   при расхождении подстроить под модель IMU из ardupilot_gazebo.
4. **Мир и модель дрона.** Положить SDF-мир (лесополосы) и модель iris с
   камерой из `concept.txt` в `worlds/`.

## Порты MAVLink

| Порт        | Назначение |
|-------------|------------|
| tcp 5760    | ArduPilot SITL (сервер) ← mavlink_router подключается клиентом |
| udp 14550   | QGroundControl |
| udp 14540   | MAVROS (контейнер nav) |
| udp 14541   | Кастомные узлы / прямые MAVLink-команды |
