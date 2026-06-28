# FAQ — `bootstrap` (взлёт без GPS + инициализация VINS в полёте)

Сценарий `bootstrap` поднимает дрон **без GPS** и инициализирует VINS прямо в
полёте — под боевую GPS-denied архитектуру. Реализация: нода
`src/lab/alt_hold_bootstrap.py` (обёртка `src/lab/bootstrap.sh`). Теория и
обоснование — `FAQ_gps.txt`, `src/nav/FAQ_gps.md`. Ветка `nn2_c3_vins_althold`.

## Что это за сценарий? Арминг, взлёт, посадка?

Да — арминг + взлёт + (раскачка для VINS) + посадка, но **всё одним автоматом**,
а не отдельными командами `arm`/`takeoff`/`land`. Нода `alt_hold_bootstrap`
**сама владеет всей лётной фазой** от арма до посадки. Поэтому в секвенсоре
`capture_scene.sh` команда `bootstrap` идёт одна — перед ней НЕ нужны
`arm`/`takeoff`, после неё (без handover) НЕ нужен `land`.

### Машина состояний

```
PREARM  → газ в минимум, латч режима ALT_HOLD
  ↓
ARM     → арминг моторов (в ALT_HOLD)
  ↓
CLIMB   → throttle > центра (default 1650, --throttle-climb) → набор до BS_ALT
  ↓
EXCITE  → throttle = центр (hold) + импульсы roll/pitch (±BS_EXCITE PWM)
          → параллакс + IMU-excitation, ЖДЁМ сходимости VINS (бюджет BS_VINS_TO)
  ↓
далее по флагу BS_HANDOVER:
  • 0 (default): OBSERVE (держит высоту BS_OBSERVE sim-сек) → LAND → DONE
                 (самодостаточно: дрон садится сам)
  • 1:           HANDOVER → GUIDED (остаётся в воздухе; тут проявляется «рывок» —
                 кадр VINS не выровнен к NED, yaw-коррекция в ray_tracer не сделана)
  ↓
DONE    → печатает ИТОГ: VINS_OK / VINS_TIMEOUT / CLIMB_FAIL
```

### Почему ALT_HOLD, а не GUIDED (как `arm`/`takeoff`)

`arm`/`takeoff` работают в **GUIDED** — позиционном режиме: без GPS или
сошедшегося VINS он не латчится. **ALT_HOLD** держит высоту по барометру и НЕ
требует горизонтальной позиции → можно оторваться от земли и СОЗДАТЬ движение,
нужное монокуляру для инициализации. В ALT_HOLD нет авто-взлёта (высота — это
throttle-стик, центр = hold), поэтому нода непрерывно шлёт **RC override 20 Гц**
(`/mavros/rc/override`) всю лётную фазу — иначе FCU по таймауту вернётся к своему
RC и дрон просядет. Высоту меряем по `/mavros/global_position/rel_alt` (баро,
доступна БЕЗ origin/GPS); сходимость VINS — по устойчивому `/vins_estimator/odometry`.
Все бюджеты — в **sim-времени** (`/clock`), RTF-независимо.

## Запуск

```bash
make bootstrap                          # climb→init→observe→land (без рывка), alt=3
make bootstrap BS_ALT=4 BS_HANDOVER=1   # после init → GUIDED (наблюдать рывок)

# в секвенсоре capture_scene (rosbag + кадры вокруг всего bootstrap):
bash src/lab/capture_scene.sh bootstrap                # сам садится
BS_HANDOVER=1 bash src/lab/capture_scene.sh bootstrap square 1 land

# напрямую:
docker exec p1317_nav bash /lab/bootstrap.sh
docker exec p1317_nav python3 /lab/alt_hold_bootstrap.py --alt 3 --handover
```

### Параметры (env `bootstrap.sh` → `--флаг` ноды)

| Env | Default | Что |
|---|---|---|
| `BS_ALT` | 3 | целевая высота climb, м |
| `BS_HANDOVER` | 0 | 1 = после init перейти в GUIDED (иначе OBSERVE→LAND) |
| `BS_EXCITE` | 80 | амплитуда импульсов roll/pitch, PWM от центра (1500) |
| `BS_OBSERVE` | 15 | держать высоту после init перед посадкой, sim-сек (без handover) |
| `BS_VINS_TO` | 90 | таймаут ожидания сходимости VINS, sim-сек (по нему → LAND) |
| `BS_THROTTLE_CLIMB` | 1650 | газ в CLIMB (PWM); >центра+deadzone, иначе не растёт высота |
| `BS_MODE_BUDGET` / `BS_ARM_BUDGET` / `BS_CLIMB_BUDGET` / `BS_LAND_BUDGET` | 40/40/60/120 | бюджеты фаз, sim-сек |

CPU-прогон: добавлять `CPU=1` ко всем вызовам (RTF≈0.07 → бюджеты в sim-сек дают
десятки минут wall; это норма).

## Грабли (подтверждено прогонами)

### ⚠️ RC override молча отбрасывается → дрон не взлетает (`MAV_GCS_SYSID`)

**Симптом:** автомат армится и уходит в CLIMB, нода публикует `/mavros/rc/override`
с throttle (напр. 1800), но `rel_alt ≈ 0` — дрон не отрывается. На исходе
climb-бюджета → `⚠️ не взлетели (rel_alt≈0) — RC override не принят? аборт→LAND`,
ИТОГ `CLIMB_FAIL`, `odom=0`. `/mavros/rc/in` при этом практически пуст.

**Причина:** ArduCopter принимает `RC_CHANNELS_OVERRIDE` только если sysid
отправителя == `MAV_GCS_SYSID` (дефолт **255**). MAVROS шлёт как `system_id=1` →
1≠255 → КАЖДЫЙ override дропается → throttle остаётся на собственном RC FCU
(min) → моторы у земли. **В ArduPilot 4.8 параметр переименован
`SYSID_MYGCS → MAV_GCS_SYSID`** — старое имя прошивкой игнорируется (несуществующий
параметр отвергается при загрузке), поэтому строка `SYSID_MYGCS 1` НЕ действовала.

**Диагностика (надёжно, минуя зависающий на низком RTF mavros param-pull):**
pymavlink напрямую к SITL `tcp:127.0.0.1:5762` (5760 занят mavlink_router'ом),
изнутри контейнера `simulator` с `PYTHONPATH=/root/ardupilot/modules/mavlink`.
Шлём `RC_CHANNELS_OVERRIDE` и смотрим отражение в `RC_CHANNELS.chan3_raw`:

```
A/B-тест (подтверждено):
  override от source_system=1   (как mavros)  → chan3 = 1000  ОТБРОШЕН ❌
  override от source_system=255 (= MAV_GCS_SYSID) → chan3 = 1800  ПРИНЯТ ✅
  MAV_GCS_SYSID читался как 255; SYSID_MYGCS — не существует.
```

> Почему mavros param-сервис «висит» на низком RTF: `/mavros/param/get` отдаёт из
> кэша, который наполняется только после полной выкачки param-таблицы FCU. На
> lockstep+RTF≈0.07 MAVLink тактируется sim-временем, и pull ~1400 параметров не
> укладывается в wall-ретраи mavros (`request param #0 timeout … 1396 params still
> missing`). Поэтому FCU-параметры читаем напрямую через SITL TCP, а не через mavros.

**Фикс:** в `config/sitl-extra.parm` использовать АКТУАЛЬНОЕ имя:
```
MAV_GCS_SYSID 1
```
Применяется `sim_up.sh` поверх eeprom при каждом старте SITL (переживает
`fresh-start`). Арм/setpoint НЕ гейтятся sysid (GPS-полёт работал при sysid 1) —
ломался ТОЛЬКО override. После фикса дрон набирает высоту, проходит CLIMB→EXCITE,
VINS доходит до `Initialization finish!`.

### throttle в CLIMB и throttle-deadzone

`--throttle-climb` должен быть выше throttle-deadzone ALT_HOLD (~центр ±`THR_DZ`;
при `THR_DZ=100` это 1400–1600). Дефолт 1650 — на границе; 1800 — заведомо выше,
быстрее набор. НО: если высота не растёт даже на 1800 — дело НЕ в throttle, а в
приёме override (см. `MAV_GCS_SYSID` выше).

### Режим слетает в STABILIZE при арминге

В фазе ARM FCU кратко показывает дефолтный `STABILIZE`, пока латч `ALT_HOLD` не
«прилип» — нода ловит это и переставляет режим (`режим=STABILIZE ≠ ALT_HOLD —
ре-ассерт`). Это транзиент, не застревание: переход `ARM → CLIMB` нода делает
только убедившись, что режим=ALT_HOLD и armed. `FLTMODE_CH 0` в `sitl-extra.parm`
убирает симулированный RC-переключатель режима, который иначе каждый тик перебивал
set_mode.

### После init — `unstable features tracking`

VINS может выдать `Initialization finish!`, а затем `unstable features tracking,
please slowly move your device!` — раскачка EXCITE слишком резкая для монокуляра.
Лечится тюнингом EXCITE: снизить `BS_EXCITE`, увеличить `--excite-period`. Это
отдельный от override вопрос (всплывает уже ПОСЛЕ того, как дрон реально летает).

## Где смотреть

- лог автомата (фазы `>>> X → Y`, ИТОГ) — stdout прогона / `make logs`;
- VINS init-маркеры (`Initialization finish!`, `NON_LINEAR`, `unstable`,
  `excitation`) — `output/sim_nav.log` (`make vins-watch`);
- высота — `/mavros/global_position/rel_alt` (QoS `sensor_data` / BEST_EFFORT);
- override — `/mavros/rc/override`; приём на FCU — `/mavros/rc/in` (на низком RTF
  редкий) или напрямую через SITL `tcp:5762`.
