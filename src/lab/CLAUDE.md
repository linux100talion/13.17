# src/lab — управляющие скрипты для симуляции

Скрипты для армирования, полёта и мониторинга VINS внутри симуляционного стека.
Монтируются в nav-контейнер как `/lab:ro` (bind mount, read-only).

## Быстрый старт

```bash
cd docker/sim
make restart-all && make wait   # поднять стек, дождаться сборки
make arm                        # GUIDED + арм (без взлёта)
make takeoff ALT=3              # взлёт на 3м
make fly                        # квадрат 5×5м (держит VINS на треке)
make vins-watch                 # смотреть инициализацию в реальном времени
make land                       # посадка
make disarm                     # дизарм
```

## Скрипты

### Лётные команды: `arm` / `takeoff` / `hover` / `land` / `disarm`
Пять чистых атомарных команд, каждая = свой скрипт в `/lab/`. Используются
самостоятельно (`make`/`docker exec`) или как звенья секвенсора `capture_scene.sh`.

> ⏱ Все команды ждут по ФАКТУ (поллинг `mode`/`armed`/`z` из `/mavros/...`;
> `hover` — по `/clock`), а не фиксированными `sleep` — RTF-независимо. При низком
> RTF фикс. `sleep` означал бы доли sim-секунды, и «взлёт» завершался бы у земли.

| Команда | Скрипт | Что делает |
|---|---|---|
| `arm` | `arm.sh` | GUIDED + арм (БЕЗ взлёта) |
| `takeoff [ALT]` | `takeoff.sh` | взлёт на `ALT` м (default 3); нужен предварительный `arm` |
| `hover [SIM_SEC]` | `hover.sh` | висение `SIM_SEC` секунд **sim-времени** (default 10) |
| `square [LOOPS]` | `square.sh` | облёт квадрата `SQ_SIZE`×`SQ_SIZE` м @ `SQ_ALT` м, `LOOPS` кругов (default 1); нужен предварительный `takeoff` |
| `land` | `land.sh` | посадка (режим LAND) |
| `disarm` | `disarm.sh` | дизарм (`cmd/arming false`) |

`hover` = sim-секунды (не wall): в GUIDED коптер сам удерживает точку после
takeoff, поэтому `hover.sh` просто ждёт прироста sim-времени по `/clock`. При
низком RTF фикс. wall-секунды были бы мизером sim-времени.

```bash
make arm                              # GUIDED + арм
make takeoff ALT=5                    # взлёт на 5м
make hover SEC=10                     # висеть 10 sim-секунд
make land && make disarm              # посадка + дизарм
# или напрямую:
docker exec p1317_nav bash /lab/takeoff.sh 5
docker exec p1317_nav bash /lab/hover.sh 10
```

### `capture_scene.sh` (+ `extract_frames.py`) — СЕКВЕНСОР команд
Единый АТОМАРНЫЙ прогон диагностики камеры «от рестарта до заливки на Google
Drive». Проигрывает заданную ПОСЛЕДОВАТЕЛЬНОСТЬ лётных команд, а запись
rosbag + извлечение кадров по пути + заливка идут автоматически вокруг неё.

```
capture_scene.sh [WxH] <команда> [арг] <команда> [арг] ...
```

- `WxH` — ОПЦ. 1-й позиц. аргумент: разрешение камеры (напр. `640x480`). Если
  задано → стек ПЕРЕСОЗДАЁТСЯ (`fresh-start`), т.к. env применяется при создании
  контейнера; иначе быстрый `restart-all`.
- команды — `arm`, `takeoff [ALT]`, `hover [SIM_SEC]`, `land`, `disarm`
  (см. таблицу выше). `takeoff`/`hover` съедают следующий числовой токен;
  неизвестная команда → ошибка ещё ДО рестарта стека (стек впустую не поднимаем).

Поток: рестарт → старт записи bag (`RECORD=1`) → исполнение последовательности
команд по порядку → стоп записи → извлечение кадров по пути → сборка mp4 из всего
потока камеры (`MP4=1`) → заливка кадров + `scene.mp4` на Drive.

#### Запуск

```bash
cd docker/sim && make capture-scene                          # дефолтная последовательность (CSARGS)
make capture-scene CSARGS="640x480 arm takeoff 5 hover 2 land"
# или напрямую с хоста:
bash src/lab/capture_scene.sh 640x480 arm takeoff 5 hover 2 land   # 640×480 (fresh-start)
bash src/lab/capture_scene.sh arm takeoff 3 hover 20 land disarm   # без смены разрешения (restart-all)
DIST_M=1.0 bash src/lab/capture_scene.sh arm takeoff 4 hover 20 land  # выборка кадров реже
GDRIVE_UP=0 bash src/lab/capture_scene.sh arm takeoff 4 hover 20 land # снять кадры локально, без заливки
RECORD=0 bash src/lab/capture_scene.sh arm takeoff 3 land             # дешёвая проверка взлёта (без bag)
```

#### Параметры (env)

Полётные параметры теперь ПОЗИЦИОННЫЕ (команды + `WxH`); env управляет только
рестартом / записью / извлечением / заливкой:

| Env | Default | Что |
|---|---|---|
| `RESTART` | 1 | 1 = перезапуск стека (restart-all/fresh-start + wait); 0 = на живом стеке (⚠️ рассинхрон) |
| `RECORD` | 1 | 1 = писать rosbag (`/image_color` + поза) вокруг всей последовательности |
| `MP4` | 1 | 1 = собрать `scene.mp4` из ВСЕХ кадров `/image_color` и залить с кадрами; 0 = выкл |
| `MP4_MAXW` | 1280 | макс. ширина кадра в mp4, px (0 = не масштабировать) |
| `DIST_M` | 0.5 | **шаг выборки кадров по пройденному пути, м** |
| `N_FRAMES` | 30 | макс. число кадров (0 = без лимита) |
| `TOPIC` | `/image_color` | топик камеры |
| `POSE_TOPIC` | `/mavros/local_position/pose` | поза для расчёта пути |
| `TOPICS_EXTRA` | — | доп. топики в bag через пробел (напр. `"/mavros/imu/data /mavros/imu/data_raw"` для диагностики IMU) |
| `SKIP_CAM` | 0 | `1` = не писать/не обрабатывать `/image_color`: лёгкий bag (мегабайты) для анализа только по IMU/позе (напр. FFT гироскопа). Гасит запись камеры, mp4, извлечение кадров и заливку |
| `CPU` | — | `CPU=1` → GPU-less режим (накладывает `docker-compose.cpu.yml`) |
| `GDRIVE_UP` | 1 | 1 = заливать на Drive; 0 = только снять кадры |
| `GDRIVE_REMOTE` / `GDRIVE_DIR` | `gdrive` / `13.17/scene_img` | rclone-remote и папка на Drive |

Разрешение задаётся ПОЗИЦИОННО (`WxH`), не через env (см. ниже). При `RECORD=1`
старые rosbag'ы (`output/scene_bag*`) удаляются в начале; свежий bag этого
прогона (2+ ГБ) **остаётся** в `docker/sim/output/scene_bag` для анализа.

#### Разрешение камеры (`WxH`, 1-й позиционный аргумент)
Разрешение задаётся первым позиционным аргументом `WxH`; `capture_scene.sh`
парсит его в `CAMERA_W`/`CAMERA_H`, которые растекаются по всем 5 точкам (SDF-
камера Gazebo, `bayerizer`, `camera_node`, интринсики VINS, `CameraInfo`).
`docker-compose.yml` интерполирует их из env хоста (`${CAMERA_W:-1280}` / `:-720`;
CPU-оверрайд → `:-320` / `:-180`). **Подвох:** env применяется при СОЗДАНИИ
контейнера — `restart-all` (stop/start) его не перечитывает. Поэтому при заданном
`WxH` `capture_scene.sh` в фазе рестарта делает `fresh-start` (пересоздание), а не
`restart-all`. Это безопасно: критичные SITL-параметры лежат в host-смонтированном
`config/sitl-extra.parm` и применяются при каждом старте.

```bash
bash src/lab/capture_scene.sh 640x360 arm takeoff 4 hover 20 land        # 640×360 на GPU
CPU=1 bash src/lab/capture_scene.sh 320x180 arm takeoff 4 hover 20 land  # CPU-бокс
```
⚠️ При `RESTART=0` разрешение не применится (нет пересоздания) — скрипт предупредит.

#### Выборка кадров — ПО ПУТИ, а не по времени (`extract_frames.py`)
Кадры для заливки выбираются по **пройденному пути дрона**, не по таймеру:
- запись пишет в bag два топика — `/image_color` И `/mavros/local_position/pose`;
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

#### Видео из всего потока камеры (`make_video.py`)
Параллельно JPEG-выборке `capture_scene` собирает **`scene.mp4`** — ВЕСЬ поток
`/image_color` за прогон («как видела камера»), не выборку по пути. Пишется в
`…/output/scene_img/scene.mp4`, заливается на Drive вместе с кадрами (`MP4=1` по
умолчанию; `MP4=0` — выключить, `MP4_MAXW` — даунскейл по ширине).

**FPS считается из `header.stamp` (sim-время камеры), а НЕ из времени записи bag.**
Тонкость: 3-й элемент `read_next()` (bag-receive-время) — это **wall**-время; на
низком RTF (CPU-бокс, RTF≈0.07) оно растянуто в ~14× → если по нему считать fps,
выйдет «слайдшоу» ~2 fps вместо реальных ~30 sim-Гц. По `header.stamp` длительность
ролика = длительности полёта в sim-времени. Заодно это диагностика для VINS: видно
реальный sim-Гц камеры (на 960×540 ~30 Гц, втрое выше нужных VINS 10 Гц).

Можно пересобрать отдельно по уже снятому bag (внутри nav):
```bash
docker exec -e SCENE_MAXW=1280 p1317_nav bash -lc \
  'source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash; \
   source /root/sim_ws/install/setup.bash; python3 /lab/make_video.py'
```
Env `make_video.py`: `SCENE_BAG`, `SCENE_MP4` (`…/scene_img/scene.mp4`),
`SCENE_TOPIC` (`/image_color`), `SCENE_FPS` (0 = авто из sim-штампов),
`SCENE_MAXW` (1280; 0 = без масштабирования). Кодек mp4v, требует overlay.

#### Бюджет времени прогона (~4–7 мин)
Складывается из стадий скрипта + физического прогрева FCU (значения — из скрипта
и таймстампов `mavros.log`):

| Стадия | Откуда | Время |
|---|---|---|
| `restart-all` (stop→start контейнеров; `fresh-start` при смене разрешения чуть дольше) | docker compose | ~10–20с |
| `make wait` → «nav: готово» (старт MAVROS/VINS/камеры, colcon инкрементально) | до старта нод | ~60–120с |
| прогрев EKF: origin set ~+35с, «is using GPS» ~+85с после старта MAVROS | таймстампы FCU | ~85с |
| `arm.sh`+`takeoff.sh`: GUIDED→arm→takeoff (циклы с ретраями до 180 итераций) | поллинг | 10–40с при успехе; до 180с при отказе |
| `hover.sh` (sim-секунды) + запись bag | по `SIM_SEC` и RTF | зависит |
| извлечение кадров по пути из bag (`extract_frames.py`) | внутри nav | ~15с |
| сборка `scene.mp4` из всего потока (`make_video.py`, `MP4=1`) | внутри nav | ~10–20с |
| заливка на Drive (rclone) | из лога | ~18с |

Разброс даёт взлёт (`arm.sh`+`takeoff.sh`): при успешном весь прогон ≈ **4–5 мин**;
при зависшем `takeoff` (ретрай до таймаута) → **+3 мин** → ~6–7 мин. Когда взлёт
стабилен, время режется: убрать лишний `sleep 8` в `capture_scene.sh` (команды
сами ждут готовность), уменьшить `SIM_SEC` у `hover`.

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
По умолчанию работает пока не прервать Ctrl+C; **`--loops N`** — выйти после N
полных кругов (круг считается по возврату к `(0,0)` → дрон финиширует у старта).

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
| `--loops` | 0 | число полных кругов; 0 = бесконечно (до Ctrl+C) |

### `square.sh`
ОГРАНИЧЕННАЯ обёртка над `fly_square.py` для секвенсора `capture_scene` (в
отличие от `make fly`, который крутит бесконечно). Команда `square [LOOPS]` летит
квадрат `LOOPS` кругов и выходит сама → встраивается в атомарный прогон между
`takeoff` и `land`. Размер/высота/скорость — через env `SQ_SIZE` (2), `SQ_ALT`
(5), `SQ_SIDE` (6 с/сторона). Нужен предварительный `takeoff` (EKF origin для
local-координат `map`).

```bash
# в секвенсоре capture_scene (запись bag + кадры по пути вдоль квадрата + Drive):
CPU=1 bash src/lab/capture_scene.sh 640x480 arm takeoff 5 square 1 land
# отдельно (1 круг 2×2 @ 5м):
docker exec p1317_nav bash /lab/square.sh 1
SQ_SIZE=4 SQ_ALT=6 docker exec p1317_nav bash /lab/square.sh 2   # 4×4 @ 6м, 2 круга
```

### `land.sh`
Переводит в режим LAND. Дрон садится на месте.

```bash
make land
# или напрямую:
docker exec p1317_nav bash /lab/land.sh
```

### `bootstrap.sh` / `alt_hold_bootstrap.py` — взлёт без GPS, init VINS в полёте
Взлёт в **ALT_HOLD** и инициализация VINS в полёте (без GPS), под боевую
GPS-denied-архитектуру. Обоснование и теория — `src/nav/FAQ_gps.md`, план —
`src/nav/todo.txt`. Ветка `nn2_c3_vins_althold`.

**Почему отдельно от `arm`/`takeoff`:** те работают в GUIDED (позиционный режим —
без GPS/сошедшегося VINS не латчится). ALT_HOLD держит высоту по баро и НЕ требует
горизонтальной позиции → можно оторваться и СОЗДАТЬ движение, нужное монокуляру
для init. Но в ALT_HOLD нет авто-взлёта: высота — throttle-стиком (пружинный,
центр=hold) → нужен **непрерывный RC override** (`/mavros/rc/override`, 20 Гц).
Поэтому это нода (а не bash): она держит override весь полёт, иначе FCU по таймауту
вернётся к своему RC и дрон просядет. `bootstrap` **сам владеет всей лётной фазой**
(arm→climb→раскачка→ждёт VINS) → перед ним НЕ нужны `arm`/`takeoff`.

Автомат: `PREARM(ALT_HOLD,газ=min) → ARM → CLIMB(газ>центр до alt) →
EXCITE(газ=центр + импульсы roll/pitch) — ждём сходимости VINS →` далее по флагу:
- **без handover (default):** `OBSERVE` (держит высоту `BS_OBSERVE` sim-сек) `→ LAND`
  — самодостаточно, дрон садится сам; в секвенсоре после `bootstrap` НЕ добавлять `land`;
- **`BS_HANDOVER=1`:** после init → `GUIDED` (самоудержание), дрон остаётся в воздухе
  — тут проявляется **рывок** (кадр VINS не выровнен к NED, yaw-коррекция в ray_tracer
  ещё не реализована); дальше можно `square`/`hover`/`land`.

Высота меряется по `/mavros/global_position/rel_alt` (баро, доступна БЕЗ origin/GPS).
Сходимость VINS — по устойчивому потоку `/vins_estimator/odometry`. Бюджеты — в
sim-времени (`/clock`), RTF-независимо (как `arm.sh`).

```bash
make bootstrap                          # climb→init→observe→land (без рывка), alt=3
make bootstrap BS_ALT=4 BS_HANDOVER=1   # после init → GUIDED (наблюдать рывок)
# в секвенсоре (запись bag + кадры + Drive вокруг всего bootstrap):
bash src/lab/capture_scene.sh bootstrap                 # без handover (сам садится)
BS_HANDOVER=1 bash src/lab/capture_scene.sh bootstrap square 1 land   # handover → квадрат
# напрямую:
docker exec p1317_nav bash /lab/bootstrap.sh
docker exec p1317_nav python3 /lab/alt_hold_bootstrap.py --alt 3 --handover
```

| Env (`bootstrap.sh`) | Default | Что |
|---|---|---|
| `BS_ALT` | 3 | целевая высота climb, м |
| `BS_HANDOVER` | 0 | 1 = после init перейти в GUIDED (иначе OBSERVE→LAND) |
| `BS_EXCITE` | 80 | амплитуда импульсов roll/pitch, PWM от центра (1500) |
| `BS_OBSERVE` | 15 | держать высоту после init перед посадкой, sim-сек (без handover) |
| `BS_VINS_TO` | 90 | таймаут ожидания сходимости VINS, sim-сек (по нему → LAND) |

> ⚠️ **Проверить на первом прогоне:** принимает ли этот SITL RC override (нода логит
> `rc/in throttle=…` в CLIMB; если высота не растёт — override не проходит, возможно
> нужен `SYSID_MYGCS` или `RC_NOCHANGE=65535` в `alt_hold_bootstrap.py`). И хватает
> ли climb+раскачки для init монокуляра (иначе поднять `BS_EXCITE` / `--excite-period`).

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
[terminal 1]  make vins-watch                       # смотреть лог
[terminal 2]  make arm && make takeoff && make fly  # арм, взлёт, квадрат
```

Ждать строку `Initialization finish!` → потом `solver_flag: NON_LINEAR`.
После инициализации `/vins_estimator/odometry` начнёт публиковаться.

### Проверка после пересборки VINS

После `make nav-rebuild` или `make fresh-start` нужно убедиться что патчи
(IMU skip, шум sim.yaml) работают:

```bash
make arm && make takeoff && make fly &
make vins-watch
# ожидаем: нет "numerical unstable", нет "imu message in disorder"
# ожидаем: "Initialization finish!" → "NON_LINEAR" через ~30с движения
```

## Диагностические инструменты (по bag / live)

Питон-утилиты в `src/lab/` (примонтированы как `/lab`), запускать ВНУТРИ
`p1317_nav` с overlay (для `cv_bridge`):

```bash
SRC='source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash'
docker exec p1317_nav bash -lc "$SRC; python3 /lab/<tool> ..."
```

| Инструмент | Что | Пример |
|---|---|---|
| `two_clocks.py [topic]` | wall-fps vs sim-Гц топика (разрыv RTF) | `/lab/two_clocks.py /mavros/imu/data_raw` |
| `grab_live.py [out.png]` | снять 1 живой кадр `/image_color` + метрики (ORB/резкость/цвет → детект «оранж-фриза» рендера) | `/lab/grab_live.py` |
| `gyro_fft.py [bag] [imu]` | FFT гироскопа из bag по окнам ground/air/late (осцилляции rate-loop, см. `docker/sim/FAQ_rate_loop.md`) | `/lab/gyro_fft.py` |
| `bag_frames.py "n:wall,…"\|N` | кадры `/image_color` из bag по wall-моментам (= эпоха логов VINS) + монтаж + метрики | `/lab/bag_frames.py "init:1782653941,reboot:1782654163"` |

Нужен IMU в bag для FFT — писать с `TOPICS_EXTRA="/mavros/imu/data /mavros/imu/data_raw"`.
IMU sim-частоту в рантайме подтверждает `docker/sim/scripts/imu_rate.py` (его зовёт
`nav_up.sh`: цель ≥80 sim-Гц через `MAV_CMD_SET_MESSAGE_INTERVAL`).

## Зависимости

Скрипты запускаются внутри `p1317_nav` контейнера.
Требуют:
- `ros-humble-mavros` + `mavros_msgs` (есть в образе)
- `rclpy` (есть в образе)
- MAVROS подключён к FCU (`make status` показывает `FCU: ArduCopter`)

## Известные ограничения

- `fly_square.py` использует локальные координаты (`map` frame) — нужен EKF
  origin. Раньше требовалось вручную ждать 3–5 с после `make arm`; теперь
  `takeoff.sh` поллит высоту до набора, а к этому моменту origin уже есть,
  так что ручная пауза не нужна.
- `arm.sh` требует `ARMING_CHECK 0` в SITL (задано в `sitl-extra.parm`).
- При потере VINS трекинга (`system reboot!`) — остановить `fly`, сделать
  `make land`, потом `make arm && make takeoff && make fly` заново.
