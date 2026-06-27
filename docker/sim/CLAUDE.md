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

## Грабли (не наступать)

- **Прогон атомарен.** Поднимать стек ТОЛЬКО целиком через make (`restart-all`/
  `fresh-start` → `wait` → лётные команды → bag). Ручной перезапуск отдельного
  процесса/ноды внутри контейнера ЗАПРЕЩЁН — стек рассинхронизируется (см.
  «Дисциплина прогона» в корневом CLAUDE.md). Без перезапуска можно только
  СМОТРЕТЬ логи (`make logs/status`, `tail output/*.log`).
- **scripts/ редактируются на хосте** и применяются на следующем `restart-all`
  без пересборки образа (bind-mount `/scripts:ro`). Пересобирать образ ради
  правки скрипта не нужно.
- **Критичные пакеты — в Dockerfile, не в рантайме.** Всё, что ставится
  `apt`/`pip` внутри живого контейнера, теряется на `fresh-start`.
- **SITL .parm частично ephemeral** — `default_params/gazebo-iris.parm`
  применяется в контейнере при первом запуске и теряется на `fresh-start`
  (TODO: зафиксировать в образе). `config/sitl-extra.parm` — переживает.
- **named volumes** `nav_colcon_build`/`nav_colcon_install` НЕ удаляются на
  `fresh-start`; снести всё (вкл. тома) — `make clean`. Принудительный colcon —
  `make nav-rebuild`.
