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
| `FRAMES` | 1 | `0` = совсем НЕ извлекать JPEG-кадры (mp4 не затронут; так летает joystick-серия) |
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

### `yaw_tune_sweep.sh` — СВИП тюнинга визуального YAW-hold (фаза 2)
Серия ЧИСТЫХ атомарных прогонов `capture_scene … liftland`, по одному на тройку
гейнов `(BS_YAWH_KP, BS_YAWH_KI, BS_YAWH_SMOOTH)`. После каждого — метрика
`yaw_check.py` (СКО/размах/дрейф курса по ground-truth `/model/iris_cam/odometry`)
и заливка видео на Drive под именем с ПОРЯДКОВЫМ номером прогона + параметрами
(`00_baseline_kp16_ki1_sm1.mp4`, …) в папку `13.17/yaw_tune/`. Результаты копятся
в `output/yaw_tune.csv`, в конце — ранжирование по СКО.

Логика (bring-up: сначала демпфер, потом restoring; знак `osign` подтверждён,
не свипаем):
- **Фаза 0** — база: воспроизвести текущую раскачку (`kp=16 ki=1 sm=1`);
- **Фаза 1** — демпфер (`ki=0 sm=3`): свип `kp∈{3,6,9}`;
- **Фаза 2** — сглаживание вокруг `kp=6`: `sm∈{1,5}`;
- **Фаза 3** — restoring (`kp=6 sm=3`): `ki∈{0.5,1,2}`.

Фазы 2–3 заякорены на `kp=6/sm=3` — если победитель фазы 1 иной, правь массив
`CONFIGS` в шапке скрипта и перезапусти. Финальный подтверждающий прогон — вручную.

> ⚠️ Обходит перезатирание видео: `capture_scene` при `GDRIVE_UP=1` чистит всю
> папку `13.17` на Drive и льёт как `scene.mp4`. Поэтому свип зовёт его с
> `GDRIVE_UP=0 MP4=1` (Drive не трогается, mp4 локально), а заливку с уникальным
> именем делает сам; папку `13.17/yaw_tune` чистит ОДИН раз в начале.

```bash
DRY_RUN=1 bash src/lab/yaw_tune_sweep.sh   # показать команды, не запуская (ревью)
bash src/lab/yaw_tune_sweep.sh             # весь свип (~9 прогонов; на CPU-боксе ~1.5–2 ч)
GDRIVE_UP=0 bash src/lab/yaw_tune_sweep.sh  # без заливки (видео/CSV только локально)
```
Env: `BS_ALT(3)`, `BS_HOLD_SEC(10)`, `SAFE_SEC(8)`, `CPU(1)`, `GDRIVE_UP(1)`,
`GDRIVE_REMOTE(gdrive)`, `YAW_GDIR(13.17/yaw_tune)`, `RES(960x540)`.

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
EXCITE(газ=центр + station-keeping forward/back + медленный yaw) — ждём сходимости
VINS →` далее по флагу:
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

**Station-keeping в EXCITE:** дрон не «жмёт стик вперёд» (улетел бы за край сцены
в жёлтый экран), а раскачивается **forward/back с возвратом**. В ALT_HOLD стик
pitch = угол наклона = ускорение (двойной интегратор), поэтому симметричный
«вперёд τ / назад τ» НЕ возвращает позицию (уносит ~v·τ за цикл). Используется
профиль ускорения **+τ / −2τ / +τ** (translate, длительность 4τ): скорость
0→+→−→0 и позиция возвращается в исходную к концу цикла → дрон держится в круге
~R(peak ≈ a·τ²) около старта. `BS_EXCITE` масштабирует радиус.

⚠️ Компенсация импульсов работает ТОЛЬКО при **постоянном курсе** (вперёд/назад
гасятся, лишь когда смотрят в одну сторону в мировой системе). Поэтому yaw подаётся
НЕ непрерывно во время translate (это проворачивало бы ось внутри цикла → снос →
дрон уезжает), а **отдельным импульсом в точке возврата между translate-циклами**:
курс меняется ступенькой, сам translate идёт при фиксированном курсе. `BS_YAW` —
амплитуда импульса, `BS_YAW_DUR` (sim-сек) — его длительность; знак чередуется
каждый цикл → «подметаем» сцену ± (translate-ось разворачивается → параллакс на
2D-диске). `BS_YAW=0` → чистый translate без поворотов.

| Env (`bootstrap.sh`) | Default | Что |
|---|---|---|
| `BS_ALT` | 3 | целевая высота climb, м |
| `BS_HANDOVER` | 0 | 1 = после init перейти в GUIDED (иначе OBSERVE→LAND) |
| `BS_EXCITE` | 80 | амплитуда forward/back раскачки, PWM от центра (1500) — масштаб радиуса |
| `BS_YAW` | 30 | амплитуда yaw-импульса в EXCITE, PWM от центра (0 = без yaw) |
| `BS_YAW_DUR` | 1.5 | длительность yaw-импульса МЕЖДУ translate-циклами, sim-сек |
| `BS_EXCITE_PERIOD` | 3 | базовая τ профиля +τ/−2τ/+τ, sim-сек (translate = 4τ) |
| `BS_OBSERVE` | 15 | держать высоту после init перед посадкой, sim-сек (без handover) |
| `BS_VINS_TO` | 60 | таймаут ожидания сходимости VINS, sim-сек (по нему → LAND) |
| `BS_THROTTLE_CLIMB` | 1650 | PWM газа на подъёме (climb) |
| `BS_MODE_BUDGET` | 40 | бюджет латча режима, sim-сек |
| `BS_ARM_BUDGET` | 40 | бюджет арминга, sim-сек |
| `BS_CLIMB_BUDGET` | 60 | бюджет набора высоты, sim-сек |
| `BS_LAND_BUDGET` | 120 | бюджет посадки, sim-сек |

> ⚠️ **Проверить на первом прогоне:** принимает ли этот SITL RC override (нода логит
> `rc/in throttle=…` в CLIMB; если высота не растёт — override не проходит, возможно
> нужен `SYSID_MYGCS` или `RC_NOCHANGE=65535` в `alt_hold_bootstrap.py`). И хватает
> ли climb+раскачки для init монокуляра (иначе поднять `BS_EXCITE` / `--excite-period`).

#### Полный прогон bootstrap с записью (атомарный, через capture_scene)
Эталонная команда (CPU-бокс, 960×540, station-keeping + диагностика IMU в bag):
```bash
CPU=1 BS_EXCITE=40 BS_YAW=30 BS_EXCITE_PERIOD=3 \
  BS_THROTTLE_CLIMB=1800 BS_MODE_BUDGET=80 BS_ARM_BUDGET=80 \
  BS_CLIMB_BUDGET=120 BS_VINS_TO=150 BS_LAND_BUDGET=180 \
  TOPICS_EXTRA="/mavros/imu/data /mavros/imu/data_raw" \
  GDRIVE_UP=1 MP4=1 \
  bash src/lab/capture_scene.sh 960x540 bootstrap
```
Это **атомарный прогон** (рестарт стека → `wait` → bootstrap → bag → кадры → mp4 →
Drive), как требует дисциплина прогона — пошагово вручную не запускать.

**Подкрутка по `scene.mp4`** (радиус/раскачка station-keeping):
- уезжает за край сцены → уменьшить `BS_EXCITE` (25–30) или `BS_EXCITE_PERIOD` (2);
- мало параллакса / VINS не сходится → поднять `BS_EXCITE` или `BS_EXCITE_PERIOD`;
- скорость разворота головы → `BS_YAW` (0 = совсем без yaw).

### `freefly_lv.sh` — единая обёртка пилотных freefly-прогонов (флаг `LV=0/1`)

Один скрипт вместо двух эталонных env-блоков (см. `docker/sim/doc/tmp/Q.txt`).
Дефолты берёт лесенкой: **env снаружи > `docker/sim/.env` (локальный профиль
бокса, gitignore — тот же файл, что читает compose) > дефолт скрипта (LV=1)** —
шпаргалка по `.env` (ключи, правила разбора, грабли): `docker/sim/env.md`.
Эталон `.env` — **`docker/sim/env.default` (в git)**: при отсутствии `.env`
сеется его копией автоматически (make/сам скрипт), так что `LV=2` и
`BS_SF_MASTER=1` — дефолт и на свежем клоне: голый `bash src/lab/freefly_lv.sh`
летит боевым профилем (⚠️ реплей СТАРЫХ сценариев без "sf" запускать с
`BS_SF_MASTER=0`):

```bash
bash src/lab/freefly_lv.sh          # профиль бокса из docker/sim/.env
                                    #   (без строк в .env — LV=1: freefly-LV,
                                    #   центр CH6 = LOITER-на-VINS, GPS глушится
                                    #   в полёте)
LV=0 bash src/lab/freefly_lv.sh     # базовый freefly: только наш стек, GPS жив
LV=2 bash src/lab/freefly_lv.sh     # GPS ОТСУТСТВУЕТ С БУТА: eeprom глушит приёмник
                                    #   до старта, origin руками (BS_SET_ORIGIN=1),
                                    #   высота — сырой баро (BS_ALT_SRC=baro), aiding
                                    #   EKF с земли от нулевой vision_pose (мост
                                    #   gps_denied), с init VINS топик у ray_tracer;
                                    #   LOITER-на-VINS без секунды GPS
WIND_SPD=5 bash src/lab/freefly_lv.sh   # любой env поверх дефолтов
BS_LAND_JOY=b1 bash src/lab/freefly_lv.sh   # кнопка SA (мягкая посадка) — на buttons[1]
                                    #   (так измерено на пульте проекта 2026-08-30 →
                                    #   уже в docker/sim/.env; дефолт ноды b0; CH8
                                    #   делит ось с SF — не годится; где кнопка —
                                    #   src/lab/joystick/js_probe.py на хосте);
                                    #   BS_FF_LAND=0 — выкл (шапка скрипта)
BS_SF_MASTER=1 bash src/lab/freefly_lv.sh   # схема «SF-мастер»: SF (CH7)
                                    #   не-вверх = СЫРЫЕ СТИКИ при любом SC;
                                    #   SF вверх → SC (CH6) = потолок лесенки
                                    #   зрелости (вверх = демпфер, центр =
                                    #   +VinsHold, вниз = +LOITER; борт на
                                    #   лучшей ДОСТУПНОЙ ступени). Нужен микс
                                    #   SF→CH7 в EdgeTX; НЕ под старые реплеи
                                    #   (сценарии без "sf" = всё сырые стики)
```

Перед атомарным прогоном (`capture_scene.sh 960x540 bootstrap_arch2`) сама
готовит **eeprom SITL** под профиль (`docker/sim/scripts/sitl_lv_profile.py`,
pymavlink в контейнере simulator, tcp:5762):
- `LV=1` → `VISO_TYPE=1` (без него «Loiter failed: requires position»;
  остальное самовосстанавливает очередь bootstrap_node до арма);
- `LV=0` → `VISO_TYPE=0` (**с 1 без vision-фида не армится**: «Arm: VisOdom:
  not healthy», `ARMING_CHECK 0` чек не снимает) + возврат GPS-профиля EKF
  (`EK3_SRC1_POSXY/VELXY=3`, `SIM_GPS1_ENABLE=1`) — в LV=0 очередь ноды не
  работает и сама его не вернёт;
- `LV=2` → `VISO_TYPE=1` + `SIM_GPS1_ENABLE=0` + extnav-пара
  (`EK3_SRC1_POSXY=6`, `VELXY=0`) **ещё до бута**: борт стартует безжпсным,
  EKF живёт на vision с земли. Очередь ноды (режим `gps_denied`) эти же
  значения лишь самовосстанавливает и датирует `extnav_ready` зрелостью
  VINS (те же ~600 odom, что в LV=1 — гейт LOITER/HUD READY не упрощён).
  ⚠️ Перцепция демпфера сидит на global rel_alt (намеренно) — без GPS
  может ослепнуть: CH6-вверх тогда ≈ чистый ALT_HOLD (отдельная кампания
  «перцепция на баро»).

Записанные значения применяются на рестарте стека в начале прогона — отдельный
ребут не нужен, дисциплина «стек только целиком» сохранена.

Самодостаточен от **холодного старта** (после ребута ноута): сам делает
`make host-setup`, если нет `/dev/rawbayer` (v4l2loopback не персистентен;
нужен sudo — спросит пароль), и `make up && make wait`, если контейнеры не
бегут. Убирает и ручной pymavlink-шаг, который LV-серия делала между
профилями трижды, и ручную возню с подъёмом стека.

### `spawn_save.py` / `spawn_pose.py` — стартовать С МЕСТА ПОСАДКИ прошлого прогона

Серия «полёт за полётом»: борт появляется там и с тем курсом, где его посадили
в предыдущем прогоне, а не в центре площадки.

**Рабочий способ — сохранить точку под именем** (`spawn_save.py`): из прогона
берётся ТОЛЬКО поза, кладётся в `docker/sim/output/spawn/<имя>`, дальше сам
прогон (видео, десятки ГБ bag'а) можно удалять.

```bash
python3 src/lab/spawn_save.py docker/sim/output/joystick/lv1_joy_20260824_140447
#   под каким именем сохранить точку старта? among_trees
#   сохранено: docker/sim/output/spawn/among_trees
#     место:  x=-44.907 y=-31.536 z=0.245 м, курс -15.14°
python3 src/lab/spawn_save.py <прогон> among_trees   # имя сразу, без вопроса
python3 src/lab/spawn_save.py --list                 # какие точки есть
python3 src/lab/spawn_save.py --show among_trees     # что внутри

# старт оттуда — по имени
SPAWN_POSE=among_trees bash src/lab/freefly_lv.sh
SPAWN_POSE=among_trees make -C docker/sim fresh-start
```

Пресет — текстовый файл: комментарии (`из прогона`, дата, место словами) плюс
одна строка `x y z roll pitch yaw`. Имя разрешает `scripts/sim_up.sh`: не шесть
чисел → ищет `/root/output/spawn/<имя>` (тот же каталог, смонтирован в
контейнер). `docker/sim/output/` в `.gitignore`, поэтому пресеты локальные —
на другой бокс копировать каталог руками. Шпаргалка — `docker/sim/spawn.md`.

**Разовый вариант, без имени:**

```bash
# что за поза (печатает готовую строку env; ROS не нужен, читает sqlite bag'а)
python3 src/lab/spawn_pose.py docker/sim/output/joystick/lv1_joy_20260824_140447
#   # xyz:   (-44.907, -31.536, 0.245) м, курс -15.14° (ENU: 0 = нос на восток)
#   # покой: сдвиг за хвост 0.000 м, курс ±0.00°
#   SPAWN_POSE="-44.9074 -31.5355 0.2450 0.0000 0.0000 -0.26422"

# или сразу в полёт от места посадки конкретного прогона
SPAWN_FROM=docker/sim/output/joystick/lv1_joy_20260824_140447 bash src/lab/freefly_lv.sh
```

Из каталога прогона берётся РОВНО ОДНО: последнее сообщение
`/model/iris_cam/odometry` в `bag/*.db3` — семь чисел (xyz + кватернион).
Ни видео, ни `.env`, ни остальные топики не нужны.

Как устроено: `spawn_pose.py` берёт из bag'а последнюю **истинную позу Gazebo**
(`/model/iris_cam/odometry`, мировые оси ENU; fallback —
`/mavros/local_position/pose` + `--origin`), зануляет roll/pitch (`--keep-rp` —
оставить) и печатает `SPAWN_POSE`. `scripts/sim_up.sh` подставляет её в `<pose>`
модели `iris_cam` в КОПИИ мира в `/tmp` (репозиторный SDF чист). Постоянный
дефолт для всех прогонов — строкой `SPAWN_POSE=...` в `docker/sim/.env`.
Проверено 2026-08-24: борт встаёт ровно в точку с её курсом и стоит неподвижно.

> ⚠️ **Только с ветром** (`WIND_SPD ≠ 0`, дефолт freefly — 5). В безветренном
> прогоне борт на земле не удерживает ничто: трения о землю в связке
> Gazebo+dartsim фактически нет (`<surface><friction>` не помогает), своего
> сопротивления воздуха у модели нет — демпфирует только плагин `WindEffects`.
> Замер: `WIND_SPD=0` → борт получает при подключении SITL толчок 0.06 м/с и
> едет ВЕЧНО (13 м за 4 мин), причём и при ШТАТНОМ спавне в начале координат.
> С `WIND_SPD=5` обе конфигурации стоят намертво (bit-in-bit 48 с).

> ⚠️ Истинная поза Gazebo в таком прогоне начинается НЕ с нуля — скрипты
> разбора, считающие старт началом координат, надо кормить той же `SPAWN_POSE`.
> Полётнику сдвиг безразличен: home/origin EKF ставится на буте от своей точки,
> локальные координаты снова начинаются с нуля в точке спавна.

### `joystick/` — запись и реплей пульта (freefly без человека)
Пилот летает freefly руками (TX12 → `/joy`) → `joystick/analyze.sh` разбирает
bag (лента событий: CH6/жесты/высоты + черновик сценария + сырой таймлайн) →
сценарий правится и кладётся в `joystick/scenarios/` → `BS_PILOT=replay`
проигрывает его виртуальным пилотом: `joy_replay.py` публикует `/joy` вместо
`joy_linux_node`, для JoyPilot неотличимо от живого пульта. Якоря замкнуты
(arm/disarm по `/mavros/state`, wait_alt по gt, wait_mode по режиму FCU) —
тайминги RTF-независимы, таймаут якоря → аварийная посадка. Формат сценария и
детали: `src/lab/joystick/README.md`.

Каждый прогон `freefly_lv.sh` архивируется в `output/joystick/<NAME>/`
(scene.mp4 + **scene_hud.mp4** — пост-рендер debug-HUD из bag «глазами пилота
OpenHD», `hud_video.py` тем же кодом `nav_pkg/hud_renderer.py`, что живой FPV;
`HUD_MP4=0` — выключить; + **scene_ipm.mp4** — то же для КАНАЛА ВИДА СВЕРХУ
(`ipm_video.py`: полоса земли на кадре + выпрямленный варп + лётные значения
`/flow_dbg8|9` рядом с истиной; `IPM_MP4=0` — выключить); + мета `.env` + bag + joy.log; `NAME=…` или автогенерат
`lv<LV>_<пилот>_<дата_время>`; `KEEP_BAG=0` — не забирать bag) — иначе
видео/bag живут только до следующего прогона (capture_scene чистит на старте).
JPEG-кадры в этой серии не делаются (`FRAMES=0`). Запуск поверх летящего
прогона блокируется; freefly завершается ДИЗАРМОМ пилота (газ min + yaw ВЛЕВО
2–3 с — тем же руддером, что арм, только в другую сторону).

```bash
bash src/lab/freefly_lv.sh                 # 1) ручной полёт → архив прогона
RUN=<NAME> bash src/lab/joystick/analyze.sh  # 2) разбор bag из архива
BS_PILOT=replay BS_REPLAY_SCENARIO=/lab/joystick/scenarios/x.json \
  bash src/lab/freefly_lv.sh               # 3) реплей — тот же атомарный прогон
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
| `gyro_fft.py [bag] [imu]` | FFT гироскопа из bag по окнам ground/air/late (осцилляции rate-loop, см. `docker/sim/doc/tmp/FAQ_rate_loop.md`) | `/lab/gyro_fft.py` |
| `bag_frames.py "n:wall,…"\|N` | кадры `/image_color` из bag по wall-моментам (= эпоха логов VINS) + монтаж + метрики | `/lab/bag_frames.py "init:1782653941,reboot:1782654163"` |
| `phase_stats.py <bag>…` | тайминги фаз freefly-прогона по bag'ам (prearm → ALT_HOLD → ARMED → отрыв → init VINS → st=READY → CH6-центр → LOITER) + сводка mean/min/max; старые bag без `/mission/status` — частично | `python3 /lab/phase_stats.py /root/sim_ws/output/joystick/*/bag` |
| `hud_video.py` | пост-рендер debug-HUD на видео из bag → `scene_hud.mp4` (HUD живёт только в FPV :5600, в bag его нет; рисует тем же `nav_pkg/hud_renderer.py`; зелёные точки фич трекера — из каналов `/feature`). Env: `SCENE_BAG`, `SCENE_HUD_MP4`, `SCENE_MAXW`, `SCENE_FPS`, `SCENE_FEAT_DOTS` | `SCENE_BAG=…/joystick/<RUN>/bag python3 /lab/hud_video.py` |
| `ipm_video.py` | пост-рендер КАНАЛА ВИДА СВЕРХУ из bag → `scene_ipm.mp4`: слева кадр с нарисованной полосой земли (боевой `_ipm_px`), справа выпрямленный варп + `ipm` (реплей) / `rec` (`/flow_dbg8\|9`, что канал выдал в полёте) / `true` (истина Gazebo). Варп в bag не пишется — считает настоящий `FlowEstimator._ipm_update` лётным конфигом (`BS_*` окружения, для архива — из его `<NAME>.env`; сегодняшний дефолт помечается `*`). Углы/ω — истина Gazebo (в freefly-бэгах нет `/mavros/imu/data`). Env: `SCENE_BAG`, `SCENE_IPM_MP4`, `IPM_ZOOM`, `IPM_PAD`, `IPM_ALL`, `IPM_ALT_SRC` | `SCENE_BAG=…/joystick/<RUN>/bag python3 /lab/ipm_video.py` |
| `gain_sim.py <bag>` | **гейны демпфера**: идентификация контура по бэгу (`v̇ = α·PWM + β·v + γ`, запаздывание канала τ_s) + КОНТРФАКТНЫЙ свип kp/ki/pos_kp по тому же порыву — «что было бы», не тратя прогон. Печатает ζ/период/снос по записи, **долг интегратора** (ветер = γ/α PWM набирается за (γ/α)/ki метров пути — цена первых секунд после отрыва) и свёрку модели с записью. Лётные гейны берёт из меты прогона. Env: `GS_WIN` (окно с отрыва, 12 с), `GS_AXIS` (roll\|pitch), `GS_SWEEP`, `GS_KP`/`GS_KI` (если меты нет). ⚠️ числа — отношения строк, не абсолют (контур FCU по углу не смоделирован) | `python3 /lab/gain_sim.py …/joystick/<RUN>/bag` |
| `ipm_alt_replay.py` | стенд A/B/C «что было бы при другой высоте перцепции» (EKF / истина / латч по арму) на ОДНОМ потоке кадров; `IA_WARP_MP4` — то же видео канала. Рисовалка общая с `ipm_video.py` — `ipm_panel.py` | `IA_BAG=…/bag PYTHONPATH=/root/sim_ws/src/control:$PYTHONPATH python3 /lab/ipm_alt_replay.py` |
| `vins_sane_replay.py <bag>` | **ГЕЙТ ЗДОРОВЬЯ VINS** по bag: настоящий `Handover.vins_sane` (потолок / физика висения / занижение против IPM) на сетке 0.05 с — скорость VINS по штампам (`VinsTrack`), IPM из `/flow_dbg8\|9`, высота перцепции и стики из `/mission/status`, рядом истина Gazebo. Печатает фронты sane→insane с причиной и срабатывания чека занижения; ручки гейта — аргументами. Так валидирован чек занижения (114248 — срабатывание на 106 с; контрольные и серии 09-03/04 — 0) | `python3 /lab/vins_sane_replay.py /root/sim_ws/output/joystick/<RUN>/bag [--scale-ratio 0.5 …]` |
| `gust_hold_compare.py <RUN>…` | **КАК ДЕРЖАТ ЯРУСЫ ПОД ПОРЫВАМИ** (DpHold / DpVins / LOITER) по bag'ам freefly: окно = непрерывный сегмент яруса (`tier=`) между концом набора и посадкой; внутри — каждый цикл порыва (`WIND_GUST` из `.env` прогона) отдельно: пик смещения от точки на фронте (жёсткость) и остаток в конце цикла (якорь), плюс exc/final/vmax окна и здоровье VINS (flips/reb/scl/масштаб). Серия dphold_vs_dpvins 2026-09-05: DpHold пик 2.4–2.8 м vs DpVins 6.3–9.5, но DpHold без рамы копит 0.8–1.5 м/цикл по ветру, DpVins возвращается (0.1–0.4). `--csv DIR` — окна/циклы построчно С 2026-09-06 — секунды фазы BRAKE из поля `brk=` статуса: за цикл порыва (`b`) и за окно (`brk_s`; `nan` — поля в bag нет) | `python3 /lab/gust_hold_compare.py /root/sim_ws/output/joystick/dphold_vs_dpvins/wind_left/{1,5,10,20} …` |
| `dpvins_gust_stand.py` | **СТЕНД DpVins под порывами** без полётов и ROS: настоящий `DpVins` на планте проекта (100 PWM = 1 м/с², лаг наклона 0.26 с, лаг измерения 0.35 с — ИЗМЕРЕН по bag cmd/1, прежние 0.15 льстили), порывы по огибающей `wind_gust.py`, свип ki/kp/brake/brake_t; калибровка — ki 6 даёт пик 9.6 м как в полётах, ki 30 раскачка как cmd/1. Итоги в докстринге: ki 15 (cmd/2), BRAKE 5 кап 2 (cmd/3), запирание BRAKE и brake_t, хвост брейка как у демпфера (`--brake-t -1`, ki 8 → 2.4 м = уровень демпфера; cmd/4) | `python3 src/lab/dpvins_gust_stand.py [--gust 75] [--ki 8,15] [--kp 40,32] [--brake 5] [--brake-vmax 2] [--brake-t -1] [--trim0 -56]` |
| `stick_lateral.py` | **БОКОВОЙ СНОС НА СТИКЕ** — держит ли стаб линию, пока пилот рулит другой осью: сегменты стика (|rcp|/|rcr| > 20) ≥ N с, на каждом ярус, скорость вперёд, боковая скорость тела (right+, истина Gazebo в осях курса), RMS, снос ∫v_lat, трим стрелки ветра начало→конец и его модуль (стоит / вращается с курсом / учится). Повод — cmd/4 в движении (113224): DpVins на стике морозил трим по обеим осям, снос 0.56–0.76 м/с против 0.16 у DpHold → кандидат latch_axis (cmd/5) | `docker exec p1317_nav bash -lc "source …; python3 /lab/stick_lateral.py /root/sim_ws/output/joystick/<RUN> [--min-seg 3]"` |
| `brake_phase.py` | **РАЗБОР ПОРЫВОВ ПО ФАЗЕ СТАНЦИИ** (поля `brk=`/`ifz=` статуса, с 2026-09-06): лента переходов фаз, по циклам порыва — через сколько после фронта вошёл BRAKE, секунды BRAKE по осям, заморозка трима в BRAKE, и положение ОТНОСИТЕЛЬНО ГВОЗДЯ (DpHold — рама `sx/sy/spx/spy`; DpVins — гвоздь = VINS-поза первого `hold`) проекцией на ось ветра: e(фронт), e при входе в BRAKE, e(max), перелёт возврата. Первый прогон 130326: DpVins на фронте стоит 0.8 м против ветра (остаток перелёта), порыв гонит К гвоздю без тормоза, BRAKE лишь за гвоздём на 1.2 м/с; вход в ярус без гвоздя под порыв = 8 м | `docker exec p1317_nav bash -lc "source …; python3 /lab/brake_phase.py /root/sim_ws/output/joystick/<RUN> [--wind-deg 98] [--no-trans]"` |
| `vins_twist_check.py` | **TWIST ОДОМЕТРИИ VINS против разности позы и истины** — годится ли twist источником скорости (`BS_VINS_VEL_SRC=twist`, cmd/6): МНК-поворот рамы к истине для twist и для разности (совпали → одна рама), лаг и масштаб кросс-корреляцией |v| по штампам, шум на висении. Bag 130326: поворот +3.2/+2.7°, лаг 0.00/0.14 с, масштаб 1.008, шум 0.008±0.005 против 0.025±0.049 м/с — twist в 10 раз тише и без лага | `docker exec p1317_nav bash -lc "source …; python3 /lab/vins_twist_check.py /root/sim_ws/output/joystick/<RUN>"` |
| `ipm_band_ab.py` | стенд **ГЕОМЕТРИИ ПОЛОСЫ IPM** — A/B ширины (`yhalf`), длины/дальности (`x0..x1`) и лимита фич на ОДНИХ кадрах bag, боевым `FlowEstimator` в лётном конфиге (`BS_*` + мета `.env` прогона): коды брака, углы полосы за кадром (чёрные клинья), скорости против истины Gazebo и СЫРОЙ сигнал (приращение пути за кадр − истина·dt) с разложением на Δpitch/Δroll/ω_z/масштаб + попарная разница с базой (= пиксельная часть). Ответ 2026-08-28 (yaw/swing/1): ширина — НЕ рычаг (разница 0.2–0.6 мм при ошибке 7 мм/кадр), дальше/длиннее — хуже ∝1/sin²α; шум = тайминг углов. Читает bag из sqlite без `rosbag2_py`/`mavros_msgs` → бегает и на ХОСТЕ (venv с cv2 + `source /opt/ros/jazzy/setup.bash`) при погашенном nav. Env: `IB_BAG`, `IB_VARIANTS` («yhalf:feats:x0:x1,…»), `IB_ALT_SRC` (latch\|true), `IB_DETAIL`, `IB_CSV` | `IB_BAG=docker/sim/output/joystick/<RUN>/bag python3 src/lab/ipm_band_ab.py` |
| `yaw_spring_check.py <root>` | **ПРУЖИНА курса** по joystick-серии `spring/{long,short}/{left,right}/N`: на каждое нажатие yaw-стика — истинный Δpress/Δback (Gazebo), return%, намотка ошибки `err@rel` и знак PWM после отпускания (`/flow_dbg6`) + конфиденс (исключить гейтинг). Доказал пружину 2026-08-27 (возврат 92–96%, err до 430°) и фикс 2026-08-28 (`joystick/yaw/*`: возврат 4%, err@rel=0) — см. control.md. Env: `BAG` — один bag вместо раскладки | `docker run --rm -v …/spring:/spring:ro -v …/src/lab:/lab:ro sim-nav:latest bash -lc 'source /opt/ros/humble/setup.bash; python3 /lab/yaw_spring_check.py /spring'` |

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
- `arm.sh` требует отключённых pre-arm чеков в SITL: в ArduPilot 4.8 это
  `ARMING_SKIPCHK -1` (бывш. `ARMING_CHECK 0` — переименован и молча
  игнорировался; задано в `sitl-extra.parm` + продублировано в eeprom).
- При потере VINS трекинга (`system reboot!`) — остановить `fly`, сделать
  `make land`, потом `make arm && make takeoff && make fly` заново.
