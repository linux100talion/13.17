# src/lab — управляющие скрипты для симуляции

Скрипты для армирования, полёта и мониторинга VINS внутри симуляционного стека.
Монтируются в nav-контейнер как `/lab:ro` (bind mount, read-only).

## Быстрый старт

```bash
cd docker/sim
make restart-all && make wait   # поднять стек, дождаться сборки
make arm                        # GUIDED + арм + взлёт 3м
make fly                        # квадрат 5×5м (держит VINS на треке)
make vins-watch                 # смотреть инициализацию в реальном времени
make land                       # посадка
```

## Скрипты

### `arm_takeoff.sh`
Переводит дрон в режим GUIDED, армирует, взлетает на заданную высоту.

> ⏱ Ждёт по ФАКТУ (поллинг `mode`/`armed`/высоты `z` из `/mavros/...`), а не
> фиксированными `sleep` — RTF-независимо. При низком RTF фикс. `sleep` означал
> бы доли sim-секунды, и «взлёт» завершался бы у земли. Поллинг высоты заодно
> гарантирует, что EKF получил origin (см. ограничение ниже — теперь снято).

```bash
make arm            # взлёт на 3м (default)
make arm ALT=5      # взлёт на 5м
# или напрямую:
docker exec p1317_nav bash /lab/arm_takeoff.sh 5
```

### `capture_scene.sh` (+ `extract_frames.py`)
Единый АТОМАРНЫЙ прогон диагностики камеры «от рестарта до заливки на Google
Drive», разбитый на 6 ФАЗ (каждую можно выключить флагом, дефолты = полный прогон):

```
1. рестарт стека      (RESTART)   make restart-all + make wait
2. арм + взлёт        + ОТЧЁТ      make arm ALT=… → печатает факт. z vs цель
3. лётная фаза + bag  (RECORD)    облёт квадрата (FLY=1) ИЛИ висение (FLY=0)
4. посадка            (LAND)      make land
5. извлечение кадров  ПО ПУТИ     extract_frames.py: кадр каждые DIST_M метров
6. заливка на Drive   (GDRIVE_UP) rclone
```

Зачем фазы: при низком RTF (≈0.05) каждая sim-секунда дорога. Чтобы дёшево
проверить «взлетает ли дрон вообще», лётную фазу режут (`FLY=0` + малый
`FLY_SECONDS`), не трогая запись/заливку. После арма скрипт печатает фактическую
`z` против цели — явный ответ «взлетел/нет».

#### Запуск

```bash
cd docker/sim && make capture-scene        # полный прогон с дефолтами
# или напрямую с хоста (параметры через env):
ALT=10 FLY=0 FLY_SECONDS=10 bash src/lab/capture_scene.sh  # взлёт 10м, висение, запись+заливка
DIST_M=1.0 bash src/lab/capture_scene.sh   # выборка кадров реже — каждый метр пути
GDRIVE_UP=0 bash src/lab/capture_scene.sh  # снять кадры локально, без заливки
RECORD=0 FLY=0 FLY_SECONDS=5 bash src/lab/capture_scene.sh # дешёвая проверка взлёта (без bag)
```

#### Параметры (env)

| Env | Default | Что |
|---|---|---|
| `RESTART` | 1 | 1 = перезапуск стека (restart-all+wait); 0 = на живом стеке (⚠️ рассинхрон) |
| `ALT` | 4 | высота взлёта, м |
| `FLY` | 1 | 1 = облёт квадрата; 0 = висение на месте |
| `SIZE` / `SIDE_TIME` | 5 / 8 | геометрия квадрата (при `FLY=1`): сторона, м / время на сторону, с |
| `FLY_SECONDS` | 55 | длительность лётной фазы и записи bag, с (wall) |
| `RECORD` | 1 | 1 = писать rosbag (`/image_color` + поза) |
| `DIST_M` | 0.5 | **шаг выборки кадров по пройденному пути, м** |
| `N_FRAMES` | 30 | макс. число кадров (0 = без лимита) |
| `LAND` | 1 | 1 = посадка в конце |
| `TOPIC` | `/image_color` | топик камеры |
| `POSE_TOPIC` | `/mavros/local_position/pose` | поза для расчёта пути |
| `CPU` | — | `CPU=1` → GPU-less режим (накладывает `docker-compose.cpu.yml`) |
| `CAMERA_W` / `CAMERA_H` | — (1280×720 / CPU 320×180) | разрешение камеры; если задано → фаза 1 делает `fresh-start` (см. ниже) |
| `GDRIVE_UP` | 1 | 1 = заливать на Drive; 0 = только снять кадры |
| `GDRIVE_REMOTE` / `GDRIVE_DIR` | `gdrive` / `13.17/scene_img` | rclone-remote и папка на Drive |

Дефолты совпадают с прежним поведением — `make capture-scene` не изменился (кроме
выборки кадров: теперь по пути, см. ниже). При `RECORD=1` старые rosbag'ы
(`output/scene_bag*`) удаляются в начале; свежий bag этого прогона (2+ ГБ)
**остаётся** в `docker/sim/output/scene_bag` для анализа.

#### Разрешение камеры (`CAMERA_W` / `CAMERA_H`)
Разрешение задаётся ОДНОЙ парой env, которая растекается по всем 5 точкам (SDF-
камера Gazebo, `bayerizer`, `camera_node`, интринсики VINS, `CameraInfo`).
`docker-compose.yml` интерполирует их из env хоста (`${CAMERA_W:-1280}` / `:-720`;
CPU-оверрайд → `:-320` / `:-180`). **Подвох:** env применяется при СОЗДАНИИ
контейнера — `restart-all` (stop/start) его не перечитывает. Поэтому при заданном
`CAMERA_W`/`CAMERA_H` `capture_scene.sh` в фазе 1 делает `fresh-start`
(пересоздание), а не `restart-all`. Это безопасно: критичные SITL-параметры лежат
в host-смонтированном `config/sitl-extra.parm` и применяются при каждом старте.

```bash
CAMERA_W=640 CAMERA_H=360 bash src/lab/capture_scene.sh        # 640×360 на GPU
CPU=1 CAMERA_W=320 CAMERA_H=180 bash src/lab/capture_scene.sh  # CPU-бокс
```
⚠️ При `RESTART=0` разрешение не применится (нет пересоздания) — скрипт предупредит.

#### Выборка кадров — ПО ПУТИ, а не по времени (`extract_frames.py`)
Кадры для заливки выбираются по **пройденному пути дрона**, не по таймеру:
- фаза 3 пишет в bag два топика — `/image_color` И `/mavros/local_position/pose`;
- `extract_frames.py` (внутри nav, `rosbag2_py` + `cv_bridge`) копит 3D-длину пути
  между позами и сохраняет: первый кадр (старт) + каждый раз, как с прошлого
  сохранения набежало ≥ `DIST_M` метров;
- имя файла несёт пройденный путь: `frame_03_001.50m.jpg`;
- **дрон не двигался** (не взлетел) или позы в bag нет → останется ТОЛЬКО первый
  кадр + предупреждение. На время НЕ откатываемся.

Это удобнее для анализа сцены: кадры равномерны в пространстве (а не во времени),
число кадров ∝ длине маршрута. Извлечение можно перезапустить отдельно по уже
снятому bag (с другим шагом), не делая новый прогон — внутри nav-контейнера:

```bash
docker exec -e SCENE_DIST_M=0.5 -e SCENE_N=0 p1317_nav bash -lc \
  'source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash; \
   source /root/sim_ws/install/setup.bash; python3 /lab/extract_frames.py'
```

Env `extract_frames.py`: `SCENE_BAG` (default `…/output/scene_bag`), `SCENE_OUT`
(`…/output/scene_img`), `SCENE_TOPIC` (`/image_color`), `SCENE_POSE`
(`/mavros/local_position/pose`), `SCENE_DIST_M` (0.5), `SCENE_N` (30; 0 = без
лимита). Требует overlay `/opt/overlay` (cv_bridge против CUDA-OpenCV).

#### Бюджет времени прогона (~4–7 мин)
Складывается из стадий скрипта + физического прогрева FCU (значения — из скрипта
и таймстампов `mavros.log`):

| Стадия | Откуда | Время |
|---|---|---|
| `restart-all` (stop→start контейнеров) | docker compose | ~10–20с |
| `make wait` → «nav: готово» (старт MAVROS/VINS/камеры, colcon инкрементально) | до старта нод | ~60–120с |
| прогрев EKF: origin set ~+35с, «is using GPS» ~+85с после старта MAVROS | таймстампы FCU | ~85с |
| `arm_takeoff.sh`: GUIDED→arm→takeoff (циклы с ретраями до `TIMEOUT=180`) | мои циклы | 10–40с при успехе; до 180с при отказе |
| облёт + запись bag | `FLY_SECONDS=55` | 55с |
| извлечение кадров по пути из bag (`extract_frames.py`) | внутри nav | ~15с |
| заливка на Drive (rclone) | из лога | ~18с |

Разброс даёт `arm_takeoff.sh`: при успешном взлёте весь прогон ≈ **4–5 мин**;
при зависшем `takeoff` (ретрай до таймаута 180с) → **+3 мин** → ~6–7 мин.
Когда взлёт стабилен, время режется: убрать лишний `sleep 8` в `capture_scene.sh`
(арминг сам ждёт готовность), уменьшить `TIMEOUT`, `FLY_SECONDS` для диагностики
хватит 30с.

#### Настройка Google Drive (rclone, разово)
Заливка идёт через `rclone` (remote по умолчанию `gdrive:`). Бокс headless,
поэтому OAuth проходим в Google Cloud Shell и копируем готовый `rclone.conf`:

```bash
# В Google Cloud Shell:
curl https://rclone.org/install.sh | sudo bash
rclone config            # n → имя gdrive → drive → scope drive → auto config Yes
# Скопировать конфиг на бокс (попадёт в домашку SSH-юзера):
gcloud compute scp ~/.config/rclone/rclone.conf \
    dev-workspace-1317:~/rclone.conf \
    --zone europe-west4-a --project drone-13-17-workspace-2026
# На боксе (под root) положить в дефолтный путь:
mkdir -p /root/.config/rclone && mv /home/*/rclone.conf /root/.config/rclone/
```

Проверка: `rclone listremotes` должен показать `gdrive:`.

### `fly_square.py`
Непрерывный облёт квадрата через `setpoint_position/local`.
Нужен для инициализации VINS: создаёт параллакс и IMU excitation.
Работает пока не прервать Ctrl+C.

> ⏱ Работает на **sim-времени** (`use_sim_time` ставится в ноде): таймер, отсчёт
> сторон и штамп setpoint — в sim-часах. Поэтому `--side-time` — это секунды
> SIM-времени; при низком RTF (GPU-less, ветка `nn2_c3_cpu`) квадрат всё равно
> проходится корректно в sim-пространстве, просто дольше по реальным часам.
> Паблиш 10 sim-Гц (для GUIDED-таргета достаточно). См. `src/sim/CLAUDE.md`.

```bash
make fly                                    # квадрат 5×5м, высота 3м, 8с на сторону
make fly FLYARGS="--size 8 --alt 4"        # квадрат 8×8м, высота 4м
make fly FLYARGS="--size 5 --side-time 5"  # быстрее — 5с на сторону
# или напрямую:
docker exec -it p1317_nav python3 /lab/fly_square.py --size 5 --alt 3 --side-time 8
```

Параметры:
| Параметр | Default | Описание |
|---|---|---|
| `--size` | 5.0 | сторона квадрата, м |
| `--alt` | 3.0 | высота полёта, м |
| `--side-time` | 8.0 | время на каждую сторону, с |

### `land.sh`
Переводит в режим LAND. Дрон садится на месте.

```bash
make land
# или напрямую:
docker exec p1317_nav bash /lab/land.sh
```

### `vins_watch.sh`
Мониторинг VINS в реальном времени:
- фильтрует `sim_nav.log` по ключевым событиям
  (`Initialization`, `NON_LINEAR`, `disorder`, `unstable`, `reboot`, `IMU excitation`)
- параллельно показывает частоту `/vins_estimator/odometry`

```bash
make vins-watch
# или напрямую:
docker exec -it p1317_nav bash /lab/vins_watch.sh
```

## Сценарии

### Инициализация VINS после взлёта

VINS требует для инициализации движение с параллаксом и IMU excitation.
Дрон стоя не инициализируется. Типичный сценарий:

```
[terminal 1]  make vins-watch        # смотреть лог
[terminal 2]  make arm && make fly   # взлететь и начать квадрат
```

Ждать строку `Initialization finish!` → потом `solver_flag: NON_LINEAR`.
После инициализации `/vins_estimator/odometry` начнёт публиковаться.

### Проверка после пересборки VINS

После `make nav-rebuild` или `make fresh-start` нужно убедиться что патчи
(IMU skip, шум sim.yaml) работают:

```bash
make arm && make fly &
make vins-watch
# ожидаем: нет "numerical unstable", нет "imu message in disorder"
# ожидаем: "Initialization finish!" → "NON_LINEAR" через ~30с движения
```

## Зависимости

Скрипты запускаются внутри `p1317_nav` контейнера.
Требуют:
- `ros-humble-mavros` + `mavros_msgs` (есть в образе)
- `rclpy` (есть в образе)
- MAVROS подключён к FCU (`make status` показывает `FCU: ArduCopter`)

## Известные ограничения

- `fly_square.py` использует локальные координаты (`map` frame) — нужен EKF
  origin. Раньше требовалось вручную ждать 3–5 с после `make arm`; теперь
  `arm_takeoff.sh` поллит высоту до набора, а к этому моменту origin уже есть,
  так что ручная пауза не нужна.
- `arm_takeoff.sh` требует `ARMING_CHECK 0` в SITL (задано в `sitl-extra.parm`).
- При потере VINS трекинга (`system reboot!`) — остановить `fly`, сделать
  `make land`, потом `make arm && make fly` заново.
