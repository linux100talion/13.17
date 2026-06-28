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
make up             # создать контейнеры; sim_up.sh + nav_up.sh стартуют сами
make wait           # ждать «nav: готово» (до 5 мин)
make logs           # хвост логов всех нод (output/*.log)
make restart-all    # быстрый перезапуск (stop→start, ephemeral state жив)
make fresh-start    # полный сброс (down→up, ephemeral state теряется)
make down           # удалить контейнеры
make help           # все цели
```

> CPU-бокс без NVIDIA (ветка `nn2_c3_cpu`): добавляй `CPU=1` к любой цели —
> детали в [`CLAUDE.md`](CLAUDE.md) (раздел «CPU-режим»).

> 💬 Частые вопросы по обёртке Makefile (`restart-all` vs `fresh-start`, что
> нужно сделать до первого `restart-all`) — см. [`FAQ.md`](FAQ.md).

> ⚠️ Не прогонялось без Gazebo. Спорные места в обёртке: `sim_vehicle.py
> --no-mavproxy` (телеметрию ждём через mavlink_router), пауза 5 c перед SITL,
> фон через `nohup`. Если что-то не поднимается — смотреть `output/*.log`
> и чек-листы ниже.

## Запуск (вручную, по шагам)

> ⚠️ **Справочно — что делает Makefile под капотом.** Штатно стек поднимается
> ТОЛЬКО целиком через `make` (см. «⚠️ Дисциплина прогона» в корневом
> `CLAUDE.md`): ручной перезапуск отдельной ноды внутри контейнера
> рассинхронизирует стек и даёт ложные диагнозы. Шаги ниже — для понимания
> устройства, не для «прогона по кусочкам».

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

В симуляции камера-нода (`camera_pkg`) НЕ переписывается: вместо ArduCam читает
кадры из виртуального V4L2-устройства `/dev/rawbayer` (модуль ядра
**v4l2loopback** на хосте). Байеризатор берёт RGB из Gazebo, «портит» до сырого
Bayer и пишет туда. Нода меняется только параметром `device:=/dev/rawbayer`.

```
Gazebo /camera/image_raw (RGB) → bayerizer.py → /dev/rawbayer (v4l2loopback)
  → camera_node → /image_mono (VINS) + /image_color (nav)
```

> Подробности механики и грабли v4l2loopback (формат BGR4, padding 208 байт,
> `ready_for_capture` после первого `write()`, задержка camera_node 4 c) — в
> корневом `CLAUDE.md`, решение №4. Хост-настройка автоматизирована в
> `make host-setup` (`scripts/host_setup.sh`: `modprobe v4l2loopback` + симлинк
> `/dev/rawbayer`). Конфиг VINS под Gazebo — `sim.yaml` (нулевая дисторсия,
> интринсики камеры), см. там же раздел «sim.yaml».

## Мир: Military fortress

Мир `worlds/mili_fortress.sdf` — портированная под Harmonic карта `mili_tech` из
[engcang/gazebo_maps](https://github.com/engcang/gazebo_maps), на которой в
демо-видео работает VINS-Fusion + YOLO (текстуры уже годятся для VINS). Ассеты
вендорены в `worlds/mili_tech/` (~27 МБ, атрибуция — `ATTRIBUTION.md`).

> Детали портирования Classic→Harmonic (system-плагины, `<population>`→явная
> расстановка, `ode`→DART, Ogre→PBR) — в корневом `CLAUDE.md`, раздел «Мир —
> Military fortress».

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

Модель `worlds/iris_cam/` — `iris_with_ardupilot` из ardupilot_gazebo
(проверена под SITL: моторы, IMU, `ArduPilotPlugin`@9002) **плюс камера пилота**
(параметры из `concept.txt`, разрешение 1280×720 под `camera_node`/`sim.yaml`,
крепление к `iris_with_standoffs::base_link`). Спавнится в `mili_fortress.sdf`,
публикует gz-топик `camera/image_raw`.

> Детали (поза/наклон/fov камеры, источник мешей) — в корневом `CLAUDE.md`,
> раздел «Дрон — worlds/iris_cam/».

## Полный пайплайн запуска

```
gz sim (mili_fortress.sdf, дрон с камерой)
  → SITL ←→ ArduPilotPlugin@9002 (управление + IMU) → mavlink_router → MAVROS
  → ros_gz_bridge: camera/image_raw + /clock → ROS2
  → [nav] bayerizer: RGB → /dev/rawbayer
  → [nav] camera_node: /dev/rawbayer → /image_mono (VINS) + /image_color (nav)
  → [nav] feature_tracker + vins_estimator (sim.yaml, use_sim_time:=true)
```

Под капотом это поднимают `scripts/sim_up.sh` (Gazebo + SITL + `ros_gz_bridge`)
и `scripts/nav_up.sh` (bayerizer + `sim_nav.launch.py` + MAVROS) — автоматически
при `make up`/`restart-all`. Полная схема пайплайна и обоснования — в корневом
`CLAUDE.md` (раздел «Полный пайплайн»).

> **use_sim_time** прописан всем нодам через `src/sim/sim_nav.launch.py`.
> Единственное исключение — `ros_gz_bridge` (источник `/clock`, ему sim-время НЕ
> ставится). Тонкость IMU-таймсинка под SITL — решение №11 в корневом `CLAUDE.md`.

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
