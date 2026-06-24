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
Единый прогон диагностики камеры «от рестарта до заливки на Google Drive»:
рестарт стека → арм/взлёт → запись rosbag `/image_color` параллельно с облётом
квадрата → стоп/посадка → извлечение 30 кадров с шагом 1с в JPEG → заливка папки
на Google Drive через `rclone`. `extract_frames.py` запускается внутри
nav-контейнера (читает bag через `rosbag2_py`, декодит `cv_bridge`).

```bash
cd docker/sim && make capture-scene        # полный прогон с дефолтами
# или напрямую с хоста (параметры через env):
ALT=4 FLY_SECONDS=55 N_FRAMES=30 bash src/lab/capture_scene.sh
GDRIVE_UP=0 bash src/lab/capture_scene.sh  # только снять кадры, без заливки
```

Параметры (env): `ALT`, `SIZE`, `SIDE_TIME`, `FLY_SECONDS`, `N_FRAMES`,
`STEP_NS`, `TOPIC`, `NAV`, `GDRIVE_UP` (1/0), `GDRIVE_REMOTE` (default `gdrive`),
`GDRIVE_DIR` (default `13.17/scene_img`).
В начале прогона старые rosbag'ы (`output/scene_bag*`) удаляются; свежий bag
этого прогона (2+ ГБ) **остаётся** в `docker/sim/output/scene_bag` для анализа.

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
| извлечение 30 кадров из bag (`extract_frames.py`) | внутри nav | ~15с |
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
