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

```bash
make arm            # взлёт на 3м (default)
make arm ALT=5      # взлёт на 5м
# или напрямую:
docker exec p1317_nav bash /lab/arm_takeoff.sh 5
```

### `fly_square.py`
Непрерывный облёт квадрата через `setpoint_position/local` на 10 Гц.
Нужен для инициализации VINS: создаёт параллакс и IMU excitation.
Работает пока не прервать Ctrl+C.

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

- `fly_square.py` использует локальные координаты (`map` frame) — сразу после
  взлёта EKF может не иметь origin, первые секунды дрон может не реагировать.
  Решение: после `make arm` подождать 3–5 с перед `make fly`.
- `arm_takeoff.sh` требует `ARMING_CHECK 0` в SITL (задано в `sitl-extra.parm`).
- При потере VINS трекинга (`system reboot!`) — остановить `fly`, сделать
  `make land`, потом `make arm && make fly` заново.
