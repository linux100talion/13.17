# src/control — архитектура управления (hexagonal + DDD)

Статус: **проектный документ**, код ещё не написан. Ветка `nn2_c3_control`.
Источник: рефакторинг монолита `src/lab/alt_hold_bootstrap.py` (~865 строк) в
переиспользуемое боевое ядро управления.

## Зачем

`alt_hold_bootstrap.py` был сделан на скорую руку и сплавил в одном классе-ноде
четыре разные вещи:

1. **Конечный автомат миссии** — `PREARM→ARM→CLIMB→EXCITE→HANDOVER/OBSERVE→LAND→DONE`.
2. **Законы управления** — gz-position-hold (PID world→body), flow-damper, yaw-hold,
   генераторы возбуждения (pulse/chirp, translate +τ/−2τ/+τ, shuttle, круговая траектория).
3. **ROS2-инфраструктуру** — подписки с QoS, sim-clock, сервис-клиенты, `OverrideRCIn`.
4. **Конфигурацию** — ~60 argparse-флагов.

Цель — разнести это по hexagonal-слоям так, чтобы:
- **законы управления тестировались на числах** без Gazebo/ROS;
- **одно ядро работало и в симе, и на боевом Orin** (там нет ground-truth Gazebo —
  источник позы VINS): меняется только адаптер, домен идентичен;
- **режимы стабилизации/движения свободно комбинировались** в рантайме без лестниц
  `if gz_hold / elif flow_hold / elif ...`;
- **реальный пульт подключался как стратегия** — управление можно переключить на
  живого пилота, стабилизация при этом остаётся доступной (assisted-режим).

Прецедент в репо уже есть: `src/lab/flow_estimator.py` — чистый перцепт-сервис
(только numpy+cv2, ноль ROS), шарится между 5+ инструментами. Мы распространяем
эту культуру на весь слой управления.

## Ключевая модель: три роли, а не две

Наивная декомпозиция «движение + стабилизация», где итоговая команда =
`rc_motion + rc_stab − центр`, **физически неверна**: в исходном коде движение и
стабилизация комбинируются ТРЕМЯ разными способами:

1. **Впрыск сетпойнта** (shuttle, circle): траектория не добавляется к RC — она
   *двигает уставку* position-hold PID, а PID её отслеживает (`sp += offset`, затем
   `e = pose − sp`). Складывать выходы = двойной счёт.
2. **Аддитивно** (pitch-excite): зонд кладётся ПОВЕРХ выхода стабилизатора
   (`ADDITIVE, НЕ open-loop, иначе runaway`).
3. **Замена оси** (roll-excite, ALT_HOLD translate): зонд ВЫТЕСНЯЕТ стабилизатор на
   оси. Для system-ID это принципиально — контроллер не должен гасить экзогенный сигнал.

Недостающее измерение — **политика композиции по оси**. Отсюда три роли:

- **Trajectory (Motion)** → выдаёт *намерение*: смещение уставки / скорость. Знает
  геометрию манёвра, не знает про PWM. Сюда садится и **пульт** (источник намерения).
- **Stabilization** → берёт намерение + обратную связь → RC по регулируемым осям.
  gz-hold, flow-damper, yaw-hold, будущий vins-hold, pilot-passthrough.
- **Excitation** → экзогенный зонд для system-ID (pulse/chirp) с явной политикой на
  ось (`ADDITIVE` / `REPLACE`). Это НЕ движение — другая эпистемическая задача
  (возбудить, а не долететь).

### Проверка на всех режимах монолита

| Режим `alt_hold_bootstrap` | Trajectory | Stabilization | Excitation |
|---|---|---|---|
| ALT_HOLD translate (+τ/−2τ/+τ) | — (баро) | — | Translate (replace pitch/yaw) |
| gz-hold (чистый) | Static | GzPositionHold | — |
| gz-hold + shuttle / circle | Shuttle / Circle | GzPositionHold | — |
| roll-excite | Static | Gz (только pitch) | Pulse (**replace** roll) |
| pitch-excite | Static | Gz (обе оси) | Pulse (**additive** pitch) |
| flow-hold | — | FlowDamper (roll) | — |
| yaw-hold | … | + YawHold (yaw) | … |
| **assisted (пульт)** | **RcTransmitter** | VinsHold / FlowDamper | — |
| **ручной (пульт)** | RcTransmitter | **PilotPassthrough** | — |
| **автономный боевой** | Shuttle / Circle / waypoint | VinsHold | — |

Всё встаёт без единого `if`. Комбинаторика при этом **разрежена** (flow/gz
взаимоисключающи, roll-excite требует gz и т.д.) — валидные тройки собирает
`recipes.py`, а не «любая клетка легальна».

## Пульт как стратегия (центральная цель)

Реальный пульт — не «режим сбоку», а **источник намерения** на месте Shuttle/Circle.
Два уровня авторитета выражаются штатно, из тех же кирпичей:

- **Assisted**: `Trajectory = RcTransmitter` (стики → `MotionIntent`) +
  `Stabilization = VinsHold/FlowDamper` (борт исполняет чисто).
- **Ручной**: `Stabilization = PilotPassthrough` (сырые стики → RC, обратной связи нет).

Адаптер пилота (`/mavros/rc/in` → `RcCommand`) работает **идентично** в симе
(SITL-стики) и на борту (радио) — разница только в том, что на борту стики двигает
человек. Это и есть «переключить управление на реальный пульт» без изменения ядра.

## Слои и порты

```
        driving (входящий)                         driven (исходящие)
   timer 20Гц → MissionRunner.tick        RcOutput · FlightMode · Clock · Logger · DebugSink
        │                                          ▲
        ▼                                          │
   ┌─────────────── application ───────────────────┴───┐
   │  MissionRunner (FSM, в mission_pkg) · ControlStack · Arbiter │
   └───────────────────────┬───────────────────────────┘
                           │ (только доменные типы)
   ┌───────────────────── domain ──────────────────────┐
   │  RcCommand · DroneState · Setpoint/MotionIntent    │
   │  Stabilization/Trajectory/Excitation (ABC + impl)  │
   │  ports.py (Protocol-контракты)                     │
   └────────────────────────────────────────────────────┘
                           ▲ реализуют
   ┌───────────────── infrastructure ──────────────────┐
   │  ros_node · mavros_actuator · ros_telemetry        │
   │  ros_perception (FlowEstimator) · ros_pilot · clock │
   └────────────────────────────────────────────────────┘
```

Правило: `domain/` не импортит ни `rclpy`, ни `mavros_msgs`, ни `cv2`. Проверяется
дисциплиной + простым import-тестом.

### Value objects (`domain/`)

```python
# rc.py
RC_CENTER, RC_MIN_THR, RC_NOCHANGE = 1500, 1000, 65535

@dataclass
class RcCommand:
    roll: int = RC_CENTER
    pitch: int = RC_CENTER
    throttle: int = RC_CENTER
    yaw: int = RC_CENTER

# state.py — снапшот телеметрии, домен читает каждый тик (адаптер наполняет)
@dataclass
class DroneState:
    mode: str | None = None
    armed: bool = False
    rel_alt: float | None = None
    rcin_throttle: int | None = None
    # VINS
    vins_odom_count: int = 0
    vins_last_sim: float = -1e9
    # Ground-truth (СИМ; на Orin пусто → источник позы = VINS-адаптер, домен не меняется)
    gt_valid: bool = False
    gt_x: float = 0.0; gt_y: float = 0.0; gt_yaw: float = 0.0
    gt_vx: float = 0.0; gt_vy: float = 0.0
    # Поток: СЫРЫЕ агрегаты от FlowEstimator (PID теперь в домене, не в колбэке)
    flow_lateral: float = 0.0; flow_yaw: float = 0.0; flow_conf: float = 0.0
    flow_seq: int = 0            # счётчик кадров (см. «тонкость 1»)
    flow_dt: float = 0.0         # интервал последнего кадра
    now_sim: float = 0.0         # проставляет адаптер из Clock

# setpoint.py
class AxisPolicy(Enum):
    REGULATE = auto()   # стабилизатор ведёт ось к уставке
    ADDITIVE = auto()   # зонд ПОВЕРХ выхода стабилизатора (pitch-excite)
    REPLACE  = auto()   # зонд ВЫТЕСНЯЕТ стабилизатор на оси (roll-excite, translate)

@dataclass
class MotionIntent:      # чего хочет Trajectory — смещение уставки от точки входа, в ТЕЛЕ
    d_fwd: float = 0.0
    d_right: float = 0.0

@dataclass
class Setpoint:          # абсолютная цель в МИРЕ (ControlStack = origin + intent)
    x: float = 0.0
    y: float = 0.0
```

### Порты (`domain/ports.py`, Protocol)

```python
class Clock(Protocol):
    def now_sim(self) -> float: ...

class Telemetry(Protocol):
    def snapshot(self) -> DroneState: ...

class RcOutput(Protocol):
    def publish(self, cmd: RcCommand) -> None: ...

class FlightMode(Protocol):                # КОМАНДЫ (не RC) — отдельный порт
    def set_mode(self, mode: str) -> None: ...
    def arm(self) -> None: ...
    def ready(self) -> bool: ...

class PilotInput(Protocol):                # пульт: сим (SITL) и боевой (радио) — одинаково
    def sticks(self) -> RcCommand: ...     # сырой PWM с /mavros/rc/in
    def mode_switch(self) -> int: ...      # тумблер авто/ручной — для арбитража

class Logger(Protocol):
    def info(self, m: str) -> None: ...
    def warn(self, m: str) -> None: ...
    def error(self, m: str) -> None: ...

class DebugSink(Protocol):
    def publish(self, roll_off: float, flow_off: float, conf: float, stamp: float) -> None: ...
```

### Контракты трёх ролей (`domain/control/base.py`)

```python
class TrajectoryStrategy(ABC):
    @abstractmethod
    def intent(self, s: DroneState, t: float) -> MotionIntent: ...
    def done(self, t: float) -> bool: return False        # челнок сам говорит «отлетал»

class StabilizationStrategy(ABC):
    axes: frozenset[str]                                  # {"roll","pitch","yaw"}
    def enter(self, s: DroneState) -> None: ...           # сброс интеграторов при switch
    @abstractmethod
    def update(self, s: DroneState, sp: Setpoint, dt: float) -> RcCommand: ...

class ExcitationStrategy(ABC):
    @abstractmethod
    def offset(self, s: DroneState, t: float) -> dict[str, tuple[int, AxisPolicy]]: ...
    def done(self, t: float) -> bool: return False        # excite_total → триггер land
```

## Приложение (`application/`)

```python
# control_stack.py — сведённые StabilizationManager + MotionManager + Excitation-слот.
# Композиция трёх ролей в строгом порядке с политикой осей. Юзабелен и БЕЗ миссии.
class ControlStack:
    def switch_stabilization(self, s): ...    # горячая замена в рантайме
    def switch_trajectory(self, t): ...
    def switch_excitation(self, e): ...
    def enter(self, s: DroneState): ...        # захват origin/yaw0/t0, stab.enter()
    def update(self, s: DroneState) -> RcCommand:
        intent = self.traj.intent(s, t)                     # намерение (тело)
        sp     = self._origin_plus(intent, s)               # → world Setpoint
        rc     = self.stab.update(s, sp, dt)                # регулируемые оси
        for axis, (off, pol) in self.excite.offset(s, t).items():
            rc = _compose(rc, axis, off, pol)               # REGULATE/ADDITIVE/REPLACE
        return rc
    def excite_done(self) -> bool: ...

# arbiter.py — SAFETY: авто ↔ ручной по mode_switch пилота.
# На БОЕВОМ дроне пилот выхватывает управление безусловно; FLTMODE_CH остаётся
# включённым (в отличие от сима, где мы его обнулили). Заложить с самого начала.
class Arbiter:
    def resolve(self, s: DroneState, autonomous: RcCommand) -> RcCommand: ...
```

```python
# mission_pkg: mission_runner.py — FSM фаз. Потребитель ControlStack. Control о
# миссии НЕ знает (зависимость строго mission → control).
class MissionRunner:
    def __init__(self, cfg, clock: Clock, mode: FlightMode, stack: ControlStack, log: Logger): ...
    def tick(self, s: DroneState) -> RcCommand: ...   # автомат фаз; в EXCITE → stack.update
    @property
    def finished(self) -> bool: ...
    @property
    def result(self) -> str: ...
```

## Поток данных (один тик)

```
timer 20Гц (ros_node) → telemetry.snapshot() + perception.merge → DroneState
  → MissionRunner.tick(state):
        фазы-команды (PREARM/ARM/CLIMB/LAND) → FlightMode.set_mode/arm + простой RcCommand
        фаза EXCITE                          → ControlStack.update → RcCommand
  → Arbiter.resolve(state, rc)   # уступить пилоту, если тумблер в «ручной»
  → RcOutput.publish(rc)  +  DebugSink.publish(...)
wall-цикл в main → RcOutput.publish(тот же rc)   # свежесть override на FCU (как сейчас)
```

Детерминизм override сохраняется: точки СМЕНЫ значения задаёт sim-таймер (tick),
wall-цикл лишь ре-публикует неизменное между тиками значение для свежести на FCU.

## Две тонкости (иначе сломается)

1. **Flow-PID интегрирует ПО КАДРАМ, а не по тикам.** Сейчас PID в `_on_flow_image`
   с `dt=res['dt']`, чтобы не интегрировать одну ошибку 20 раз/с на стоячем сигнале.
   Переносим PID в домен (`FlowDamper`), но адаптер кладёт `flow_seq`+`flow_dt`, а
   `FlowDamper.update` продвигает интегратор ТОЛЬКО при смене `flow_seq`. Стратегия
   остаётся чистой (вызов из тика), физика per-frame сохранена.

2. **Точку входа (origin/yaw0/t0) владеет `ControlStack.enter()`, не стратегии.**
   Trajectory отдаёт смещение относительно входа (тело), стабилизатор — абсолютную
   world-уставку. Это текущая логика `hold_sp`/`hold_yaw0`, но в одном месте.

## Пакеты и упаковка

```
src/control/                     # ament_python пакет control_pkg
  package.xml setup.py setup.cfg resource/control_pkg
  architecture.md                # этот файл
  control_pkg/                   # модуль-дир = имя пакета (конвенция репо, ср. nav_pkg)
    domain/                      # ✅ срез 1: чистый, 0 импортов rclpy/cv (проверено grep)
      rc.py  state.py  setpoint.py  ports.py
      control/  base.py  stabilization.py  trajectory.py  excitation.py
    application/
      control_stack.py           # ✅ срез 1;  arbiter.py — срез 2 (пилот)
    infrastructure/              # ✅ срез 1: ros_clock/ros_telemetry/mavros_actuator/ros_io
      ros_perception.py ros_pilot.py   # — срез 2/3
    nodes/
      control_node.py            # — срез 2: bare пилот+стабилизация, БЕЗ FSM
  test/
    test_gz_shuttle_equiv.py     # ✅ числовая эквивалентность закона с монолитом (Δ=0)

src/mission/                     # ament_python пакет mission_pkg (рядом, потребитель control)
  package.xml setup.py setup.cfg resource/mission_pkg
  mission_pkg/
    config.py                    # BootstrapConfig (срез 1)
    application/  mission_runner.py     # ✅ FSM фаз bootstrap
    nodes/        bootstrap_node.py      # ✅ точка входа (console_script bootstrap_arch2)
  test/
    test_bootstrap_fsm.py        # ✅ оффлайн smoke автомата PREARM→DONE на фейках
```

### Статус реализации

- **Срез 1 (gz-hold + shuttle): КОД ГОТОВ, оффлайн-гейты зелёные.** Домен+приложение
  побитово воспроизводят закон монолита (5 кейсов, Δroll/pitch=0); автомат фаз
  проходит PREARM→…→DONE на фейках-портах. Осталось: colcon-интеграция (mounts в
  `docker/sim/docker-compose.yml` + сборка `control_pkg`/`mission_pkg`) и АТОМАРНЫЙ
  прогон в симе (`ros2 run mission_pkg bootstrap_arch2`) со сверкой по метрике.
- **Срез 2 (пульт-как-стратегия): КОД ГОТОВ, оффлайн-гейты зелёные.** Добавлены:
  `RcTransmitter` (trajectory: стик=скорость уставки, интеграл → assisted-режим),
  `PilotPassthrough` (stabilization: сырые стики → RC, manual), `Arbiter`
  (application: safety-seize по тумблеру), порт-адаптеры `RosPilot`
  (/mavros/rc/in — борт) и `ScriptedPilot` (профиль стиков — sim без живого пульта),
  pilot-поля в `DroneState`, `recipes.build_control_stack` (shuttle|assisted|manual).
  Нода: `--control-mode`, `--pilot`, Arbiter в контуре тика. Тесты: pilot-стратегии
  (RcTransmitter/PilotPassthrough/Arbiter) + FSM-smoke assisted/manual/seize. Осталось:
  атомарный прогон в симе (`--control-mode assisted`). Срез 3 (excitation) — далее.

- **Гранулярность:** один модуль на роль (все стратегии стабилизации в одном файле),
  а не файл-на-стратегию — это код, который мутирует; дробить в 15 крошек вредно.
  Взорвём позже, если роль разрастётся.
- **colcon:** `control_pkg` и `mission_pkg` монтируются и собираются в ОБОИХ
  контейнерах (`nav` в симе, `vins_project_13_7` на Orin) — как сейчас `src/nav`.
  Интеграция по существующему шаблону (Dockerfile/compose mounts +
  `colcon build --packages-select control_pkg mission_pkg`).
- `flow_estimator.py` переиспользуется как есть (перцепт-сервис), не переписывается.

## Совместимость и план миграции

- **CLI не ломаем.** За ~60 argparse-флагов держатся `bootstrap.sh`, `liftland.sh`,
  `yaw_tune_sweep.sh`. Точка входа сохраняет флаги 1:1. На первом этапе флаг
  `--arch2` включает новый путь, без флага — старый монолит.
- **Инкрементально, вертикальными срезами.** Каждый срез доводится до рабочего в
  симе и сверяется с текущим поведением метрикой (`yaw_check.py`, `drift_check.py`,
  `scene.mp4` из `capture_scene`), а не на глаз.
- **Срез 1 (сейчас): `gz-hold + shuttle`** — трогает Stabilization + Trajectory
  (впрыск сетпойнта) + порты RcOutput/FlightMode/Clock/Telemetry. Валидирует ядро
  композиции до переписывания всех 865 строк.
- **Срез 2: пульт-как-стратегия** (`RcTransmitter` / `PilotPassthrough`) — доказывает
  главную цель, без ground-truth.
- Когда срезы зелёные — переключаем дефолт на `--arch2` и выпиливаем монолит.

## Инварианты (не нарушать при реализации)

1. `domain/` без `rclpy`/`mavros_msgs`/`cv2`.
2. Зависимость строго `mission_pkg → control_pkg`, никогда обратно.
3. `FlightMode` (команды set_mode/arm) — отдельный порт от `RcOutput` (RC).
4. Все бюджеты/таймеры — в sim-времени по `/clock` (RTF-независимо).
5. На боевом борту пилот выхватывает управление безусловно (Arbiter + FLTMODE_CH вкл).
6. `ControlStack` самодостаточен — работает и без `MissionRunner`.
