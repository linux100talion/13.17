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

## Быстрый старт (Makefile)

Обёртка `docker/sim/Makefile` + `scripts/` автоматизируют всю
последовательность ниже. Скрипты исполняются ВНУТРИ контейнеров через
`docker exec -i ... bash -s` (по stdin), ноды уходят в фон, логи — в `output/`.

```bash
cd docker/sim
make host-setup     # хост: xhost + v4l2loopback (/dev/rawbayer), нужен sudo
make build          # собрать образы (долго: SITL+Gazebo+OpenCV из исходников)
make run            # up + sim (Gazebo+SITL+мост) + nav (colcon+ноды+MAVROS)
make logs           # хвост логов всех нод (output/*.log)
make stop           # погасить ноды (контейнеры живут); make restart — перезапуск
make down           # удалить контейнеры
make help           # все цели
```

> ⚠️ Не прогонялось без Gazebo. Спорные места в обёртке: `sim_vehicle.py
> --no-mavproxy` (телеметрию ждём через mavlink_router), пауза 5 c перед SITL,
> фон через `nohup`. Если что-то не поднимается — смотреть `output/*.log`
> и чек-листы ниже.

## Запуск (вручную, по шагам)

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
python3 src/sim/bayerizer.py --ros-args \
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

## Мир: Military fortress

Мир `worlds/mili_fortress.sdf` — портированная под Harmonic карта из
[engcang/gazebo_maps](https://github.com/engcang/gazebo_maps) (`mili_tech`),
та самая, на которой в демо-видео работает VINS-Fusion + YOLO. Ассеты
вендорены в `worlds/mili_tech/` (~27 МБ). Атрибуция — `worlds/mili_tech/ATTRIBUTION.md`.

Что было портировано из оригинального Gazebo Classic `mili.world`:
- добавлены обязательные system-плагины gz-sim (physics/scene/sensors/imu/...)
- `<population>` деревьев/огня заменены явной расстановкой (gz-sim не
  поддерживает `<population>`)
- `<road>` удалён, физика `ode` → дефолт DART
- материалы `grass_plane`/`digital_wall` → PBR
- убрана битая ссылка `model://home`

### Запуск мира (контейнер simulator)

```bash
docker exec -it p1317_simulator bash
#   (внутри) gz sim -v4 -r /root/worlds/mili_fortress.sdf
```

### ⚠️ Чек-лист проверки на ноуте (порт не тестировался без Gazebo)

1. **Мир грузится без ошибок** — смотреть лог `gz sim -v4`. Частые места:
   - `model://...` не резолвится → проверь `GZ_SIM_RESOURCE_PATH` содержит
     `/root/worlds` и `/root/worlds/mili_tech`.
   - **Вложенные include в `mili_map`** не подхватились → если карта (стены,
     танки, дома) не появилась, расплющить `mili_map/model.sdf` в world-level
     include внутри `mili_fortress.sdf` (позы сложить со смещением `-35 -5 0`).
   - **`mt_background` (heightmap)** ломает загрузку → закомментировать его
     include в `mili_fortress.sdf`.
2. **Текстуры мешей** (танки/дома/деревья) видны — если меш серый, текстура
   из `.dae` не нашлась (проверь пути в Collada/наличие jpg/png рядом).
3. **Земля/стены** с PBR-текстурой (не однотонные). Земля может быть размытой —
   gz-sim не тайлит текстуру по грани бокса (для VINS вниз — слабо, но камера
   смотрит вперёд на меши).

## Дрон: iris + камера

Модель `worlds/iris_cam/` — это `iris_with_ardupilot` из ardupilot_gazebo
(проверенная под SITL: моторы, IMU, плагин `ArduPilotPlugin` на 127.0.0.1:9002)
**плюс камера пилота**. Камера — параметры из `concept.txt` (поза `0.15 0 0.05`,
наклон 0.26 рад, fov 1.5708, clip 0.1–10000), но разрешение **1280×720** под
`camera_node`/`sim.yaml`. Жёстко прикреплена к `iris_with_standoffs::base_link`.
Меши корпуса/винтов берутся из `model://iris_with_standoffs` (в образе ardupilot_gazebo).

Спавнится в `mili_fortress.sdf` (`<include> model://iris_cam` на `0 0 0.2`).
Камера публикует gz-топик `camera/image_raw`.

### Мост Gazebo → ROS (контейнер simulator)

```bash
# Камера + часы симуляции в ROS2
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

## Полный пайплайн запуска

```
gz sim (mili_fortress.sdf, дрон с камерой)
  → SITL (sim_vehicle.py) ←→ ArduPilotPlugin (управление + IMU)
  → ros_gz_bridge: camera/image_raw + /clock → ROS2
  → [nav] bayerizer: RGB → /dev/rawbayer
  → [nav] camera_node: /dev/rawbayer → /image_mono (+ OpenHD)
  → [nav] feature_tracker + vins_estimator (sim.yaml, use_sim_time:=true)
  → MAVROS ← mavlink_router ← SITL
```

1. **simulator**: `gz sim -v4 -r /root/worlds/mili_fortress.sdf`
2. **simulator**: `sim_vehicle.py -v ArduCopter -f gazebo-iris --console`
3. **simulator**: `ros_gz_bridge` (команда выше) — публикует `/clock`. **Этой
   ноде `use_sim_time` НЕ ставится — она источник часов.**
4. **nav**: один launch на все ноды (bayerizer + camera_node + feature_tracker +
   vins_estimator), уже с `use_sim_time:=true`:
   ```bash
   ros2 launch /root/sim_ws/src/sim/sim_nav.launch.py
   ```
5. **nav**: MAVROS — отдельно, тоже с sim-временем:
   ```bash
   ros2 run mavros mavros_node --ros-args \
     -p use_sim_time:=true -p fcu_url:="udp://:14540@127.0.0.1:14555"
   ```

> **use_sim_time** прописан всем нодам через `src/sim/sim_nav.launch.py`
> (camera_node штампует кадр через `get_clock()->now()` → уважает sim-время).
> Единственное исключение — `ros_gz_bridge` (источник `/clock`).

### ⚠️ Проверить на ноуте

- **`/clock` идёт**: `ros2 topic hz /clock` (после моста). Ноды должны
  показывать sim-время: `ros2 param get /vins_estimator use_sim_time` → true.
- **Имя модели/линка**: камера цепляется к `iris_with_standoffs::base_link`.
  Если ardupilot_gazebo обновил структуру — свериться (`gz sim`-лог о joint).
- **SITL соединилась**: в логе `gz sim` — "ArduPilot... connected", в `sim_vehicle`
  телеметрия идёт. Порт 9002.
- **Камера-топик**: `ros2 topic hz /camera/image_raw` (~30 Гц) после моста.

## Открытые задачи

1. **IMU-источник.** VINS ждёт IMU на `/mavros/imu/data_raw` из SITL.
   Проверить частоту и шум — `sim.yaml` задаёт заниженный шум, при расхождении
   подстроить под модель IMU из ardupilot_gazebo. ⚠️ MAVROS под `use_sim_time`
   — момент тонкий (часть таймстампов приходит от FCU); если IMU/камера не
   синхронизируются, проверить штампы `/mavros/imu/data_raw` первым делом.
2. **Тюнинг полёта/сцены** после первого успешного прогона VINS.

## Порты MAVLink

| Порт        | Назначение |
|-------------|------------|
| tcp 5760    | ArduPilot SITL (сервер) ← mavlink_router подключается клиентом |
| udp 14550   | QGroundControl |
| udp 14540   | MAVROS (контейнер nav) |
| udp 14541   | Кастомные узлы / прямые MAVLink-команды |

## nav_pkg (NN1/NN2) и работа без GPU

> Раскладка и роли нод подробно — в корневом `CLAUDE.md` (раздел
> «OpenHD-оверлей и nav_pkg»). Кратко:
> - `nav_pkg/nn1/` — NN1, якорная локализация: `nn1_anchor` (SuperPoint+LightGlue)
>   + `ray_tracer` (засечка по ориентиру → поправка VINS).
> - `nav_pkg/nn2/` — NN2, топокарта: `nn2_scene` (DINOv2 + FAISS) + `relocalizer`.
> - `nav_pkg/openhd_streamer.py` — даунлинк в OpenHD с оверлеем детекций.
> - `tools/nn1`, `tools/nn2_scene`, `tools/nn2_route` — офлайн-скрипты (сборка
>   баз/карт, обучение, оценка).
>
> Запуск: `ros2 launch nav_pkg nav.launch.py use_sim_time:=true` (камеру/VINS не
> поднимает; в симуляции включается из `src/sim/sim_nav.launch.py`).

### GPU vs CPU — где NN1/NN2 упираются в отсутствие GPU

Контекст: ветка `nn2_c3_cpu` — GPU-less прогон на боксе без NVIDIA, пока T4 в
дефиците. Ядро `gazebo→SITL→VINS` от GPU отвязано; ниже — про nn-сторону.

**Ядро `gazebo→SITL→VINS` в GPU-стену НЕ упирается.** `nav.launch` оно не
поднимает. VINS — Ceres+LK (CPU), камера имеет CPU drop-in (`camera_node_cpu`),
Gazebo — софтовый llvmpipe. Барьер тут — fps софтрендера (perf-гейт), а не
отсутствие GPU.

**Стена №1 — тривиальная: РЕШЕНА авто-фолбэком cuda→cpu.** Дефолты нод
по-прежнему `"cuda"` (боевой Orin / GPU-sim не трогаем):
- `nav_pkg/nn1/nn1_anchor.py` — `declare_parameter("device", "cuda")` →
  `anchor_matcher: LightGlue(...).to(device)`
- `nav_pkg/nn2/nn2_scene.py` — `declare_parameter("device", "cuda")` →
  `SceneEncoder: DINOv2 .to(device)`

Раньше на машине без GPU это валило ноду на загрузке модели (`torch: no CUDA
device`). Теперь `device` проходит через `nav_pkg/device_util.py::resolve_device()`:
если просили cuda, но `torch.cuda` недоступна — молча уходим на cpu (с warn в
лог), как в `tools/nn2_route/train_route_coords.py:120`. Так что на CPU-боксе
ноды поднимаются БЕЗ ручного `device:=cpu`; на GPU поведение не меняется.
Офлайн-тулзы (`build_scene_map`, `eval_isometry`, `visualize_fiber`) фолбэка
пока НЕ имеют — argparse `--device default="cuda"`, передавать `--device cpu`
вручную.

**Стена №2 — настоящая: пропускная способность, а не «не запустится».** Torch и
FAISS на CPU РАБОТАЮТ, вопрос в скорости (×10–50):
- **nn2 / DINOv2 ViT-S/14**: ~0.2–1 с/кадр на CPU. Живой инференс @3 с —
  терпимо. Сборка карты из bag (`build_scene_map` по сотням-тысячам кадров) и
  обучение топографа на реальных bag'ах — минуты→десятки минут.
- **nn1 / SuperPoint+LightGlue**: ~0.5–2 с/матч на CPU, цель ~1 Гц → на грани;
  против многокадровой reference-базы — мимо каденса.
- **FAISS** — уже faiss-cpu, поиск по карте микросекунды: стены нет в принципе.
- **Геометрия** (`nn1/geo.py`, `ray_tracer`, `nn2/metric_decode` — чистый numpy)
  — CPU-native, стены нет.

**Вывод.** Жёсткой «не поедет» стены у NN1/NN2 нет — у всего есть CPU-путь.
- Мгновенная мелочь: дефолт `device="cuda"` → краш (решено фолбэком).
- Мягкая throughput-стена: для СМОУК-ТЕСТА (нода поднялась, топики идут, FAISS
  отвечает, матчинг/трейсинг считают верно) CPU-бокса хватает; для ОБУЧЕНИЯ
  топографа на реальных bag'ах и REAL-TIME релокализации — нет, вот за этим
  возвращаемся на T4.
