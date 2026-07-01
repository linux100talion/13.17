# docker/sim/ — симуляционный стек (SITL + Gazebo + VINS)

Локальный контекст каталога. Архитектура, решения и пайплайн целиком — в
корневом `CLAUDE.md` (раздел «Симуляция — `docker/sim/`»). Здесь — раскладка,
рабочий цикл и грабли при правке файлов *в этом каталоге*.

## Что где лежит

```
docker-compose.yml          — базовый стек (3 сервиса: simulator, mavlink_router, nav)
docker-compose.cpu.yml      — CPU-оверрайд (накладывается ПОВЕРХ базового, см. ниже)
Makefile                    — единая точка входа (все прогоны — через make)
config/
  sitl-extra.parm           — параметры SITL (переживают fresh-start)
  mavlink-router.conf       — раздача MAVLink по UDP
scripts/                    — entrypoint'ы (монтируются как /scripts:ro, без пересборки)
  sim_up.sh                 — simulator: Gazebo + SITL + ros_gz_bridge
  nav_up.sh                 — nav: VINS + камера + MAVROS + bayerizer (nohup)
  host_setup.sh             — хост: xhost + v4l2loopback (/dev/rawbayer)
  capture_frames.sh, sitl_accel_cal.py
simulator/  nav/  mavlink_router/  — Dockerfile'ы образов
worlds/                     — SDF-миры и модель дрона (iris_cam)
output/                     — логи нод (make logs читает отсюда)
FAQ*.txt, README.md, todo.txt — заметки по отладке
```

## Рабочий цикл (всё через Makefile)

```bash
make host-setup     # один раз: v4l2loopback + xhost (нужен sudo)
make build          # собрать образы (долго: SITL+Gazebo+OpenCV из исходников)
make up             # поднять контейнеры; sim_up.sh + nav_up.sh стартуют сами
make wait           # ждать «nav: готово» (до 5 мин)
make logs           # хвост output/*.log
make restart-all    # быстрый перезапуск (stop→start, ephemeral state жив)
make fresh-start    # полный сброс (down→up, ephemeral state теряется)
```

Лётные команды (после `wait`): `make arm takeoff hover land disarm`,
`make fly` (облёт квадрата для инициализации VINS), `make vins-watch`.

## CPU-режим (ветка nn2_c3_cpu)

GPU-less прогон на машине без NVIDIA-драйвера: добавляй **`CPU=1` к ЛЮБОМУ
таргету** — Makefile подключит `docker-compose.cpu.yml` (`DC := docker compose
-f docker-compose.yml -f docker-compose.cpu.yml`). Держи флаг во ВСЕХ командах
сессии, иначе compose адресует другой набор файлов и контейнеров:

```bash
make CPU=1 build && make CPU=1 up && make CPU=1 wait && make CPU=1 logs
```

Что меняет оверрайд: `runtime: nvidia → runc`, софтовый GL (llvmpipe) для
Gazebo, drop-in `CAMERA_NODE=camera_node_cpu` (cv::* вместо cv::cuda::*).
Разрешение камеры `CAMERA_W/H` занижено (320×180) — llvmpipe не тянет 1280×720
на fps; `sim_up.sh` патчит SDF. Боевой Orin и GPU-sim остаются на базовом compose.

## capture_scene.sh — атомарный прогон-секвенсор

`src/lab/capture_scene.sh` — единый АТОМАРНЫЙ прогон с хоста: перезапуск стека →
последовательность лётных команд → запись rosbag → извлечение кадров → mp4 →
(опц.) Google Drive. Запускать С ХОСТА из любого места. Соблюдает «дисциплину
прогона»: не дёргать ноды по кускам, всё одной командой.

```
bash src/lab/capture_scene.sh [WxH] <команда> [арг] <команда> [арг] ...
```

- **`WxH`** (опц., 1-й позиц.) — разрешение камеры, напр. `640x480`. Задано →
  стек ПЕРЕСОЗДАЁТСЯ (`fresh-start`, env применяется при создании контейнера);
  не задано → `restart-all`.
- **команды** (каждая = скрипт в `src/lab/`): `arm` · `takeoff [ALT=3]` ·
  `hover [SIM_SEC=10]` · `square [LOOPS]` · `land` · `disarm` · `bootstrap` (взлёт
  в ALT_HOLD + раскачка для init VINS без GPS) · `liftland` (см. ниже). Числовой
  аргумент привязывается к предыдущей команде.

### `liftland` — взлёт → держать уровень → посадка (БЕЗ раскачки)
Диагностический сценарий ALT_HOLD: «просто взлетаем и садимся, никуда не летим».
Изолирует ДРЕЙФ ALT_HOLD (остаточная скорость / наклон AHRS) от excite-раскачки
`bootstrap`: если дрон при нулевом excite всё равно уезжает за край сцены — причина
в AHRS-уровне/скорости, а не в управлении. Реюзает машинерию `alt_hold_bootstrap.py`
(`src/lab/liftland.sh` → `--hold-only`): arm → climb до `BS_ALT` → держать центр
стиков `BS_HOLD_SEC` sim-сек → land. Никаких движений по roll/pitch/yaw.

Эталонная команда (CPU-бокс, 960×540, с диагностикой IMU в bag):
```bash
CPU=1 BS_HOLD_SEC=30 \
  BS_THROTTLE_CLIMB=1800 BS_MODE_BUDGET=80 BS_ARM_BUDGET=80 \
  BS_CLIMB_BUDGET=120 BS_LAND_BUDGET=180 \
  TOPICS_EXTRA="/mavros/imu/data /mavros/imu/data_raw" \
  GDRIVE_UP=1 MP4=1 \
  bash src/lab/capture_scene.sh 960x540 liftland
```
Env: `BS_ALT` (3) — высота; `BS_HOLD_SEC` (30) — сколько висеть; бюджеты/газ — как у
`bootstrap`. `960x540` → fresh-start → применяется `RC1_DZ=RC2_DZ=RC4_DZ=0` из
`config/sitl-extra.parm` (мёртвая зона RC обнулена, иначе мелкий override-наклон
её не пробивает). Детали раскачки/дрейфа — `src/lab/CLAUDE.md` (раздел `bootstrap`).

### gz-position-hold — стабилизатор «стоять на месте» (СИМ-костыль)
**Зачем.** В GPS-denied EKF не держит горизонт: не отличает наклон от
горизонт-ускорения (tilt/accel ambiguity, нет опорной скорости) → мелкий bias
интегрируется в растущий наклон → дрон сам себя разгоняет и **уезжает за край
сцены** (карта `grass_plane` 150×150 м; подтверждено ИСТИННОЙ позой Gazebo —
SITL-`SIMSTATE` по позиции врёт при внешней физике). `AHRS_TRIM=0`/`SIM_WIND=0`
ни при чём — лечится только референсом скорости/позиции (на боевом борту — VINS).

**Костыль (ТОЛЬКО sim).** На время бутстрапа подменяем «идеальным VINS» = истинной
позой Gazebo. `liftland`+`BS_GZHOLD=1` включает PID position-hold по истинной позе:
- `worlds/iris_cam/model.sdf`: `gz-sim-odometry-publisher` → `/model/iris_cam/odometry`
  (поза+скорость тела в world @50Гц), мост в `sim_up.sh` → `nav_msgs/Odometry`;
- `alt_hold_bootstrap.py` (`--gz-hold`): ошибка+скорость world → тело (по yaw) →
  PWM-смещения roll/pitch. Гейны `BS_GZ_KP`(40)/`KD`(120)/`KI`(8)/`IMAX`(100);
  знаки `BS_GZ_PSIGN`/`RSIGN` (оба **+1**, выверены). I-член убирает статич. ошибку.

Результат (истина Gazebo): дрон держится в **~0.2 м** (без костыля — runaway 50м+).
На боевом Orin костыля НЕТ — там референс даёт реальный VINS.

**Эталонная команда (gz-hold, 960×540, с заливкой видео на Google Drive):**
```bash
CPU=1 BS_GZHOLD=1 BS_HOLD_SEC=40 \
  BS_THROTTLE_CLIMB=1800 BS_MODE_BUDGET=80 BS_ARM_BUDGET=80 \
  BS_CLIMB_BUDGET=120 BS_LAND_BUDGET=180 \
  TOPICS_EXTRA="/mavros/imu/data /mavros/imu/data_raw" \
  GDRIVE_UP=1 MP4=1 \
  bash src/lab/capture_scene.sh 960x540 liftland
```
`GDRIVE_UP=1 MP4=1` → собирает `scene.mp4` (весь поток камеры) и заливает на Drive
(remote `gdrive:`, см. настройку rclone в `src/lab/CLAUDE.md`); в конце лога — ссылка.
Для лёгкого прогона только по телеметрии: `SKIP_CAM=1 GDRIVE_UP=0 MP4=0`.

Запись/кадры/mp4/заливка идут автоматически вокруг всей последовательности,
управляются env:

| env | дефолт | что делает |
|---|---|---|
| `CPU` | — | `CPU=1` → GPU-less режим (как `make CPU=1`); держать во всех вызовах |
| `GDRIVE_UP` | 1 | заливать кадры на Google Drive; `0` — только локально |
| `MP4` | 1 | собирать mp4 из всех кадров `/image_color`; `0` — выключить |
| `N_FRAMES` | 30 | макс. число кадров (`0` = без лимита) |
| `TOPICS_EXTRA` | — | доп. топики в bag через пробел (диагностика IMU и т.п.) |
| `SKIP_CAM` | 0 | `1` → лёгкий bag без `/image_color` (анализ по IMU/позе, FFT) |
| `ARM_SIM_BUDGET` | 40 | sim-секунд на готовность EKF + GUIDED/arm (см. ниже) |
| `ARM_WALL_CAP` | 1200 | абсолютный потолок ожидания арма в wall-секундах |

**`ARM_SIM_BUDGET` / `ARM_WALL_CAP`** (проброс в `arm.sh`): бюджет арминга
считается в **sim-секундах** (`SIM_BUDGET`) — он переносится между RTF без
правок; на низком RTF (llvmpipe/lockstep) фиксированный wall-таймаут давал
~2 sim-сек, EKF не успевал сойтись и арм отваливался `armed=False`. `WALL_CAP` —
страховка от зависания, если `/clock` не читается. На медленном CPU/lockstep
поднимай оба.

Примеры:

```bash
# базовый прогон с пересозданием под 640x480, без заливки
GDRIVE_UP=0 bash src/lab/capture_scene.sh 640x480 arm takeoff 5 hover 2 land

# CPU-режим, штатное низкое разрешение
CPU=1 bash src/lab/capture_scene.sh 320x180 arm takeoff 3 hover 5 land

# lockstep + длинный бюджет арма + диагностика IMU в bag, без видео/заливки/кадров
ARM_SIM_BUDGET=100 ARM_WALL_CAP=2400 \
  TOPICS_EXTRA="/mavros/imu/data /mavros/imu/data_raw" \
  GDRIVE_UP=0 MP4=0 N_FRAMES=0 \
  bash src/lab/capture_scene.sh arm takeoff 3 square 1 land
```

> Lockstep-контекст: под него конфиг уже выставлен — `lock_step=1` +
> `no_time_sync=0` (`worlds/iris_cam/model.sdf`), `SCHED_LOOP_RATE 100`, и в
> `capture_scene.sh` проброшены `ARM_SIM_BUDGET`/`ARM_WALL_CAP` + `TOPICS_EXTRA`.
> Подтверждено: дрон армится (`armed=True`) и идёт на взлёт — lockstep арм не
> ломает.

> ⚠️ **Полный прогон на CPU работает ТОЛЬКО с поднятым бюджетом времени.**
> Ключевые тут именно ПЕРВЫЕ два параметра — `ARM_SIM_BUDGET=100`
> `ARM_WALL_CAP=2400`. На CPU (llvmpipe + lockstep) RTF низкий, поэтому EKF
> сходится и арм/взлёт укладываются в окно ТОЛЬКО при большом бюджете wall-
> времени; с дефолтами (`ARM_SIM_BUDGET=40`/`ARM_WALL_CAP=1200`) прогон не
> доходит до конца. Остальные env в примере (`TOPICS_EXTRA`/`GDRIVE_UP=0`/
> `MP4=0`/`N_FRAMES=0`) — для диагностики/облегчения bag, на сам факт прохождения
> прогона не влияют. Рабочая форма полного CPU-прогона:
>
> ```bash
> ARM_SIM_BUDGET=100 ARM_WALL_CAP=2400 \
>   TOPICS_EXTRA="/mavros/imu/data /mavros/imu/data_raw" \
>   GDRIVE_UP=0 MP4=0 N_FRAMES=0 \
>   bash src/lab/capture_scene.sh arm takeoff 3 square 1 land
> ```

### roll-excite — калибровочное возбуждение потока (system-ID)
Открытый контур для калибровки планта флоу-демпфера (`flow_calib.py`, слой C):
под gz-hold-pitch на roll подаётся ЗАДАННЫЙ `roll_off` (демпфер ВЫКЛ) → `roll_off`
экзогенный → чистая идентификация `k/s/τ/d`. Полный контекст и три бэга кампании —
`docker/sim/HowToFlow_PID_synth.md`. `/flow_dbg` в bag обязателен (фактический
`roll_off`).

**Эталонная команда (balanced — позиция ВОЗВРАЩАЕТСЯ, дрон на сцене):**
```bash
CPU=1 BS_GZHOLD=1 BS_ROLL_EXCITE=1 BS_RE_MODE=balanced BS_HOLD_SEC=40 \
  BS_RE_AMP=50 BS_RE_TAU=2 BS_RE_NREP=3 \
  TOPIC="/image_color" \
  TOPICS_EXTRA="/gz_imu/data_flu /model/iris_cam/odometry /flow_dbg" \
  BS_THROTTLE_CLIMB=1800 BS_MODE_BUDGET=80 BS_ARM_BUDGET=80 \
  BS_CLIMB_BUDGET=120 BS_LAND_BUDGET=180 GDRIVE_UP=1 MP4=1 N_FRAMES=0 \
  bash src/lab/capture_scene.sh 960x540 liftland
# калибровка (после сохранения bag): docker exec -i -e SAFE_SEC=35 -e R_SAFE=40 \
#   p1317_nav bash -lc 'source /opt/ros/humble/setup.bash; python3 /lab/flow_calib.py'
```
`balanced` = профиль `+τ/−2τ/+τ` (как bootstrap EXCITE): скорость 0→+→−→0, позиция
возвращается каждый цикл (4τ), сторона чередуется каждые `BS_RE_NREP` циклов.

**Вариант chirp (СНОСИТ — не для калибровки):** `BS_RE_MODE=chirp BS_RE_AMP=50
BS_RE_F0=0.15 BS_RE_F1=1.5 BS_RE_CHIRP=25 BS_RE_STEP=3`. Линейный чирп f0→f1 богат
спектром, НО уносит позицию ∝ `amp/f₀²` (ускорение — двойной интегратор) → дрон за
сцену за ~12с. Оставлен для полноты; предпочитать `balanced`.

Env (`BS_RE_*` → `--roll-excite-*`): `MODE` (balanced/chirp), `AMP` (PWM),
`TAU` (сек, цикл=4τ), `NREP` (циклов на сторону), `F0/F1/CHIRP/STEP` (для chirp).
Реализация — `alt_hold_bootstrap.py:_roll_excite_cmd`.

**Персист bag** (иначе следующий `capture_scene` сотрёт `output/scene_bag*`):
```bash
cp -r docker/sim/output/scene_bag docker/sim/bags/roll_excite_$(date +%Y%m%d)
```
`docker/sim/bags/` — в `.gitignore` (бэги крупные, вне git).

## EEPROM SITL (персистентная accel-калибровка)

**Зачем.** На свежем SITL ArduCopter режет арм обязательной проверкой
`"Arm: 3D Accel calibration needed"` — её НЕ снимает ни `ARMING_CHECK 0`, ни
параметры (`INS_ACCOFFS/ACCSCAL`): «калибровка выполнена» — это внутреннее
состояние eeprom, а не параметр. Снимается только level-cal
(`PREFLIGHT_CALIBRATION param5=4`), но та принимается лишь когда EKF сошёлся,
ребутит FCU и ненадёжна через MAVROS на низком RTF. Поэтому делаем её ОДИН раз
надёжно (pymavlink прямо к SITL `tcp:5762`) и ПЕРСИСТИМ результат.

**Как устроено.** SITL пишет `eeprom.bin` (калибровка + параметры) в cwd;
`sim_up.sh` запускает SITL из `/root/sitl_state` — это named volume
`sitl_eeprom`, поэтому eeprom переживает `fresh-start`. `--defaults`
(`config/sitl-extra.parm`) применяется поверх eeprom на каждом boot, так что
правки `.parm` продолжают работать. Per-boot MAVROS-калибровка в `nav_up.sh`
УБРАНА (с персистом не нужна, ребутила FCU).

**Как пересобрать** (`scripts/sitl_accel_cal.py` ← `make sitl-cal`): ждёт латч
GUIDED (готовность FCU), шлёт level-cal до `ACK result=0`, проверяет что
`"3D Accel cal needed"` ушёл, пишет в volume.

```bash
make CPU=1 fresh-start && make CPU=1 wait   # создаёт volume + свежий eeprom
make CPU=1 sitl-cal                         # ОДИН раз: accel-cal → eeprom
# дальше любой fresh-start стартует с откалиброванным eeprom
```

**Когда пересобирать.** Только когда volume `sitl_eeprom` пропал или сброшен:
после `make clean` (`down --volumes`), на новом боксе при первой настройке, или
если арм снова падает на «3D Accel calibration needed». Обычный `fresh-start`
(`down/up` без `-v`) volume СОХРАНЯЕТ — повторять `sitl-cal` НЕ нужно.

## Грабли (быстрое напоминание, детали — в корневом CLAUDE.md)

Короткий локальный чек-лист; полное обоснование каждого пункта — в корневом
`CLAUDE.md` (раздел «Симуляция», нумерованные решения и «⚠️ Дисциплина прогона»).

- **Прогон атомарен** — стек только целиком через make, ручной перезапуск
  отдельной ноды ЗАПРЕЩЁН. Без рестарта — только смотреть логи. → «Дисциплина прогона».
- **scripts/ правятся на хосте**, применяются на `restart-all` без пересборки
  образа (bind-mount `/scripts:ro`). → решение №10.
- **Критичные `apt`/`pip`-пакеты — в Dockerfile**: рантайм-установки теряются на
  `fresh-start`. → раздел volumes.
- **SITL `.parm` частично ephemeral**: `default_params/gazebo-iris.parm` теряется
  на `fresh-start`; `config/sitl-extra.parm` переживает. → решение №12.
- **Принудительный colcon** — `make nav-rebuild`; полный снос томов — `make clean`.
  → раздел volumes.
