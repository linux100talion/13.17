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

## Открытые задачи (грабли симуляции — НЕ закрыты этим стеком)

Инфраструктура контейнеров готова, но узлы под симуляцию ещё надо адаптировать:

1. **Камера: V4L2 → Gazebo-топик.** `camera_pkg` (CUDA+V4L2) в симуляции не
   используется. Нужен `ros_gz_bridge` с `/camera/image_raw` (Gazebo) на
   `/image_mono` (вход VINS). Класть в `src/nav` или отдельный launch.
2. **`use_sim_time:=true` всем нодам.** Gazebo публикует `/clock`. Без этого
   таймстампы кадров и IMU разойдутся с wall-clock → VINS диверджит молча.
3. **Интринсики камеры.** `dummy_13_7.yaml` — калибровка реального ArduCam.
   Нужен `config/sim.yaml` с параметрами Gazebo-камеры (1920x1200, fov 1.5708
   из `model.sdf`).
4. **IMU-источник.** VINS ждёт IMU. В симе — `/mavros/imu/data` из SITL.
   Проверить частоту и ремап под `feature_tracker`/`vins_estimator`.
5. **Мир и модель дрона.** Положить SDF-мир (лесополосы) и модель iris с
   камерой из `concept.txt` в `worlds/`.

## Порты MAVLink

| Порт        | Назначение |
|-------------|------------|
| tcp 5760    | ArduPilot SITL (сервер) ← mavlink_router подключается клиентом |
| udp 14550   | QGroundControl |
| udp 14540   | MAVROS (контейнер nav) |
| udp 14541   | Кастомные узлы / прямые MAVLink-команды |
