# src/control — архитектура управления (hexagonal + DDD)

**Living design-of-record.** Тело описывает АКТУАЛЬНОЕ состояние кода; история — в
разделе «Статус» внизу. Ветка `nn2_c3_control`. Источник: рефакторинг монолита
`src/lab/alt_hold_bootstrap.py` (~865 строк) в переиспользуемое боевое ядро управления.

## Зачем и северная звезда

`alt_hold_bootstrap.py` сплавил в одном классе-ноде четыре разные вещи: конечный
автомат миссии, законы управления, ROS2-инфраструктуру и ~60 argparse-флагов. Цель
рефактора — разнести это по hexagonal-слоям так, чтобы законы тестировались на числах
без Gazebo, одно ядро работало в симе и на борту, режимы свободно комбинировались без
лестниц `if gz/elif flow/…`, и — главное — **реальный пульт подключался как стратегия**.

**Северная звезда (боевое назначение):** до инициализации VINS дать пилоту НАШ простой
стабилизатор для стиков. На борту до VINS нет опоры позиции (ни GPS, ни VINS, ни gt) →
«наш простой» = **демпфер сноса по оптическому потоку** (даёт скорость/курс без GPS).
Пилот рулит, флоу убирает дрейф ALT_HOLD. Отсюда лестница опор — **один пилот-слой,
сменный стабилизатор**:

| Стабилизатор | Опора | Когда |
|---|---|---|
| `GzPositionHold` | ground-truth Gazebo (СИМ) | **оракул** для тюнинга законов, sim-only |
| `FlowDamper` + `YawHold` | камера (борт) | **ДО init VINS — наш пре-VINS демпфер (ГЛАВНОЕ)** |
| `VinsHold` | VINS | ПОСЛЕ init: точный hold (рантайм switch — `VinsHandover`) |

```
взлёт ─► [FlowDamper+YawHold, пилот рулит] ──VINS сошёлся──► switch_stabilization(VinsHold) ─► [точный hold / авто]
              наш пре-VINS демпфер            (рантайм hot-swap)      пилот RcTransmitter НЕ меняется
```

Прецедент чистого перцепт-сервиса уже был: `flow_estimator.py` (numpy+cv2, ноль ROS).
Он и стал каноничным в `control_pkg/perception/` (см. «Пакеты»).

## Ключевая модель: три роли, а не две

Наивная декомпозиция «движение + стабилизация» с суммой `rc_motion + rc_stab − центр`
**физически неверна**: движение и стабилизация комбинируются ТРЕМЯ способами — впрыск
сетпойнта (shuttle двигает уставку PID), аддитивно (pitch-excite поверх выхода), замена
оси (roll-excite вытесняет). Недостающее измерение — **политика композиции по оси**.
Отсюда три роли:

- **Trajectory** → *намерение*: смещение уставки (`d_*`, для position-hold) и/или
  нормированная скорость-команда (`c_*`, для velocity-damp). Не знает про PWM. Сюда
  садится и **пульт** (`RcTransmitter` — источник намерения).
- **Stabilization** → намерение + обратная связь → RC **по своим осям** (`axes`).
  `GzPositionHold`, `FlowDamper`, `YawHold`, будущий `VinsHold`.
- **Excitation** → экзогенный зонд для system-ID (pulse/chirp) с политикой на ось
  (`ADDITIVE`/`REPLACE`). Не движение — другая задача (возбудить, не долететь).

### Режимы (recipes) → тройки стратегий

| Режим (`recipes.build_control_stack`) | Trajectory | Stabilization (список) | Excitation |
|---|---|---|---|
| `shuttle` (sim system-ID) | `Shuttle` | `[GzPositionHold]` (roll+pitch) | — |
| `assisted` (пульт+gt) | `RcTransmitter` | `[GzPositionHold]` | — |
| `manual` | `StaticSetpoint` | `[]` — всё пилоту | — |
| **`flow_assist`** (пре-VINS, борт) | `RcTransmitter` | `[FlowDamper(roll), YawHold(yaw)]` | — |
| (будущий) авто-манёвр | `Shuttle`/`Circle`/waypoint | `[VinsHold]` | — |
| (system-ID оси) | `Static` | `[GzPositionHold]` | `Pulse`/`Chirp` |

Комбинаторика разрежена (flow/gz взаимоисключающи и т.п.) — валидные тройки собирает
`recipes.py`, а не «любая клетка легальна».

## Пульт и per-axis композиция (центральная цель)

Пульт — не «режим сбоку», а **источник намерения** + **подложка стека**. `ControlStack`
композит по осям: **база = сырые стики пилота**, каждый стабилизатор перезаписывает
СВОИ оси, excitation — сверху. Отсюда без единого `if`:

- **незанятая ось → сырой стик пилота** (ручной наклон);
- **flow-ось → velocity-assist**: стик задаёт цель (`c_*`·gain), флоу гасит к ней; стик
  в центре → демпф сноса к нулю;
- **`manual` = пустой список** стабилизаторов (пилот владеет всем; `PilotPassthrough`
  стал избыточен, оставлен для совместимости);
- **микс**: «пульт + только yaw» = `[YawHold]`; «пульт + flow(roll)» = `[FlowDamper]`.

Плюс `Arbiter` (safety-supervisor поверх миссии): тумблер MANUAL → сырые стики (incl
throttle) безусловно. Адаптер пилота (`/mavros/rc/in` → `RcCommand`) идентичен в симе
(SITL/скрипт) и на борту (радио) — «переключить на реальный пульт» без изменения ядра.

## Слои и порты

```
        driving (входящий)                         driven (исходящие)
   timer 20Гц → node._tick                 RcOutput · FlightMode · Clock · Logger · DebugSink
        │                                          ▲
        ▼                                          │
   ┌─────────────── application ───────────────────┴───┐
   │  MissionRunner (FSM, mission_pkg) · ControlStack · Arbiter │
   └───────────────────────┬───────────────────────────┘
                           │ (только доменные типы)
   ┌───────────────────── domain ──────────────────────┐
   │  RcCommand · DroneState · Setpoint/MotionIntent    │
   │  Stabilization/Trajectory/Excitation (ABC + impl)  │
   │  ports.py (Protocol) · perception/flow_estimator   │
   └────────────────────────────────────────────────────┘
                           ▲ реализуют
   ┌───────────────── infrastructure ──────────────────┐
   │  ros_clock · ros_telemetry · mavros_actuator · ros_io │
   │  ros_perception (FlowEstimator) · ros_pilot           │
   └────────────────────────────────────────────────────┘
```

Инвариант: `domain/` не импортит `rclpy`/`mavros_msgs` (perception — только numpy/cv2).
Проверяется grep'ом. Точка входа/проводка (composition root) — `mission_pkg/nodes/bootstrap_node.py`.

### Value objects (`domain/`)

```python
# rc.py
RC_CENTER, RC_MIN_THR, RC_NOCHANGE = 1500, 1000, 65535
@dataclass
class RcCommand:
    roll: int = RC_CENTER; pitch: int = RC_CENTER
    throttle: int = RC_CENTER; yaw: int = RC_CENTER

# state.py — снапшот, домен читает каждый тик (адаптеры наполняют)
@dataclass
class DroneState:
    mode: str|None = None; armed: bool = False; rel_alt: float|None = None; rcin_throttle: int|None = None
    vins_odom_count: int = 0; vins_last_sim: float = -1e9
    gt_valid: bool = False; gt_x=gt_y=gt_yaw=0.0; gt_vx=gt_vy=0.0   # СИМ; на Orin gt_valid=False
    flow_lateral=flow_yaw=flow_conf=0.0                            # СЫРЫЕ агрегаты (PID в домене)
    flow_seq: int = 0; flow_dt: float = 0.0                        # покадровая интеграция (тонкость 1)
    pilot_roll=pilot_pitch=pilot_throttle=pilot_yaw = RC_CENTER    # стики (ScriptedPilot/RosPilot)
    pilot_switch: int = 0                                          # тумблер авто/ручной (Arbiter)
    now_sim: float = 0.0

# setpoint.py
class AxisPolicy(Enum): REGULATE=auto(); ADDITIVE=auto(); REPLACE=auto()
@dataclass
class MotionIntent:
    d_fwd=d_right = 0.0          # ПОЗИЦИЯ уставки от входа, тело (position-hold Gz/Vins)
    c_fwd=c_right=c_yaw = 0.0    # СКОРОСТЬ-команда, нормир.[-1..1] (velocity-damp Flow/Yaw)
@dataclass
class Setpoint:
    x=y = 0.0                    # абсолютная цель в МИРЕ (ControlStack = origin + d_*)
    c_fwd=c_right=c_yaw = 0.0    # скорость-команда, прокинутая из intent
```

### Порты (`domain/ports.py`, Protocol)

`Clock.now_sim`, `Telemetry.snapshot`, `RcOutput.publish`, `FlightMode.set_mode/arm/ready`
(КОМАНДЫ, отдельно от RC), `PilotInput.sticks/mode_switch` (сим и борт — одинаково),
`Logger.info/warn/error`, `DebugSink.publish`.

### Контракты трёх ролей (`domain/control/base.py`)

```python
class TrajectoryStrategy(ABC):
    def intent(self, s, t) -> MotionIntent: ...
    def done(self, t) -> bool: return False               # челнок сам говорит «отлетал»

class StabilizationStrategy(ABC):
    axes: frozenset                                       # какие оси регулирует
    def enter(self, s) -> None: ...                       # сброс интеграторов при switch/входе
    def update(self, s, sp, dt) -> RcCommand: ...         # значимы только поля из axes

class ExcitationStrategy(ABC):
    def offset(self, s, t) -> dict: ...                   # ось → (pwm, AxisPolicy)
    def done(self, t) -> bool: return False
```

Стабилизаторы и их оси: `GzPositionHold`={roll,pitch}, `FlowDamper`={roll},
`YawHold`={yaw}, `PilotPassthrough`={roll,pitch,yaw} (легаси).

## Приложение (`application/`)

```python
# control_stack.py — per-axis композиция. Стабилизаторов СПИСОК. Юзабелен и без миссии.
class ControlStack:
    def __init__(self, stabilization, trajectory, excitation):
        self.stabs = _as_list(stabilization)              # один | список | []
    def switch_stabilization(self, s): self.stabs = _as_list(s)   # ГОРЯЧАЯ замена (пока не вызывается)
    def switch_trajectory(self, t): ...
    def switch_excitation(self, e): ...
    def enter(self, s):                                   # захват origin/yaw0/t0 + stab.enter() каждому
        ...
    def update(self, s) -> RcCommand:
        intent = self.traj.intent(s, t)                   # позиция d_* + скорость c_*
        sp = self._origin_plus(intent)                    # world-уставка + c_* passthrough
        rc = RcCommand(roll=s.pilot_roll, pitch=s.pilot_pitch,   # БАЗА = стики пилота
                       throttle=RC_CENTER, yaw=s.pilot_yaw)
        for st in self.stabs:                             # каждый перезаписывает СВОИ оси
            out = st.update(s, sp, dt)
            for ax in st.axes: setattr(rc, ax, getattr(out, ax))
        for axis,(off,pol) in self.excite.offset(s,t).items():
            rc = _compose(rc, axis, off, pol)             # ADDITIVE/REPLACE
        return rc

# arbiter.py — SAFETY: тумблер MANUAL → сырые стики (incl throttle) безусловно.
# На борту FLTMODE_CH остаётся ВКЛ (в симе обнулён) — второй барьер на уровне FCU.
class Arbiter:
    def resolve(self, s, autonomous) -> RcCommand: ...

# mission_pkg/mission_runner.py — FSM фаз, потребитель ControlStack. Control о миссии НЕ знает.
class MissionRunner:                                      # PREARM→ARM→CLIMB→EXCITE→LAND→DONE
    def tick(self, s) -> RcCommand: ...                   # команды-фазы → FlightMode; EXCITE → stack.update
```

## Поток данных (один тик)

```
timer 20Гц → telemetry.snapshot() → DroneState
  → perception.merge(s)   # только flow_assist: камера → flow_* (+ flow_seq++)
  → s.pilot_* ← pilot.sticks()/mode_switch()             # стики в снапшот (как телеметрия)
  → MissionRunner.tick(s):  PREARM/ARM/CLIMB/LAND → FlightMode + простой RcCommand
                            EXCITE                 → ControlStack.update → RcCommand
  → Arbiter.resolve(s, rc)                                # уступить пилоту, если MANUAL
  → RcOutput.publish(rc) + DebugSink.publish(...)
wall-цикл в main → RcOutput.publish(тот же rc)            # свежесть override на FCU
```

Детерминизм override: точки СМЕНЫ значения задаёт sim-таймер (tick), wall-цикл лишь
ре-публикует неизменное между тиками значение.

## Две тонкости (иначе сломается)

1. **Flow-PID интегрирует ПО КАДРАМ, не по тикам.** Адаптер кладёт `flow_seq`+`flow_dt`,
   а `FlowDamper/YawHold` продвигают интегратор ТОЛЬКО при смене `flow_seq` (иначе 20-Гц
   тик накрутил бы одну ошибку на стоячем сигнале). Стратегия чистая, физика per-frame.
2. **Точку входа (origin/yaw0/t0) владеет `ControlStack.enter()`, не стратегии.**
   Trajectory отдаёт смещение относительно входа (тело), стек — абсолютную world-уставку.

## Пакеты и упаковка

```
src/control/                       # ament_python пакет control_pkg
  package.xml setup.py setup.cfg resource/control_pkg  architecture.md
  control_pkg/
    domain/          rc.py state.py setpoint.py ports.py
      control/       base.py stabilization.py trajectory.py excitation.py
    application/     control_stack.py arbiter.py
    infrastructure/  ros_clock.py ros_telemetry.py mavros_actuator.py ros_io.py
                     ros_pilot.py ros_perception.py
    perception/      flow_estimator.py           # КАНОНИЧНАЯ копия (борт self-contained; вариант A)
  test/              test_gz_shuttle_equiv.py test_pilot_strategies.py
                     test_flow_strategies.py test_multiaxis_stack.py

src/mission/                       # ament_python пакет mission_pkg (потребитель control)
  package.xml setup.py setup.cfg resource/mission_pkg
  mission_pkg/
    config.py  recipes.py
    application/  mission_runner.py
    nodes/        bootstrap_node.py               # console_script bootstrap_arch2
  test/  test_bootstrap_fsm.py test_pilot_fsm.py
```

- **Гранулярность:** модуль на роль (все стабилизаторы в одном файле) — код мутирует,
  дробить в крошки вредно.
- **colcon:** оба пакета bind-mount в `nav` (сим) и на Orin, инкрементальная сборка в
  `nav_up.sh` (`--packages-select control_pkg mission_pkg`).
- **Зависимость строго `mission_pkg → control_pkg`**, никогда обратно.
- `flow_estimator.py` — вариант A: канонично в `control_pkg/perception/`; легаси-копия
  `src/lab/flow_estimator.py` живёт для монолит-инструментов до их вывода.

## Совместимость и запуск

- **Монолит `alt_hold_bootstrap.py` жив параллельно** (`bootstrap`/`liftland` его гоняют).
  Новое ядро — отдельная нода: `ros2 run mission_pkg bootstrap_arch2` (обёртка
  `src/lab/bootstrap_arch2.sh`, команда `bootstrap_arch2` в `capture_scene.sh`).
- CLI-флаги новой ноды совместимы с монолитом (подмножество) + `--control-mode
  {shuttle|assisted|manual|flow_assist}`, `--pilot {scripted|ros}`.
- Сверка поведения — метрикой (`drift_check.py`/`yaw_check.py`/`scene.mp4`), не на глаз.

## Статус

**Все три среза закодированы, покрыты 6 оффлайн-гейтами и прогнаны в симе.** Домен
чист (grep), зависимость `mission→control` соблюдена.

| Срез | Что | Оффлайн | Прогон в симе |
|---|---|---|---|
| 1 · `shuttle` | gz-hold + челнок; ядро композиции | эквивалентность Δ=0 vs монолит (5 кейсов) + FSM | ✅ `HOLD_DONE` |
| 2 · `assisted`/`manual` | пульт-как-стратегия + Arbiter | pilot-стратегии + FSM assisted/manual/seize | ✅ `HOLD_DONE` |
| 3 · `flow_assist` | пре-VINS флоу-демпфер + per-axis стек | флоу-стратегии + per-axis композиция | ✅ (см. ниже) |

**Оффлайн-гейты (чистый python, без ROS):** `test_gz_shuttle_equiv` (закон побитово =
монолит), `test_bootstrap_fsm`, `test_pilot_strategies`, `test_pilot_fsm`,
`test_flow_strategies`, `test_multiaxis_stack`, `test_handover` (switch Flow→Vins).

**Рантайм switch `Flow→Vins` — РЕАЛИЗОВАН (оффлайн-проверен).** `VinsHold` (position-hold
по VINS, СВОЯ опора в vins-фрейме, захват в момент switch) + `VinsHandover` (application:
детектор «VINS ready» = N odom + свежесть → ОДНОКРАТНО `stack.switch_stabilization`).
`MissionRunner` зовёт `handover.maybe_switch` в EXCITE; пилот (`RcTransmitter`) при switch
не меняется. Флаг `--handover-vins` (flow_assist). Это первый реальный вызов `switch_*`.
⚠️ Sim-демо самого switch требует сценария, где VINS СХОДИТСЯ (нужно движение/параллакс;
нейтральный flow_assist его не даёт) — механизм готов, демо-прогон отдельно.

**Подтверждённые факты (drift_check в симе):**
- `flow_osign = −1` — знак торможения флоу-демпфера. `+1` РАЗГОНЯЛ снос (метрика
  боковая/продольная RMS_v = 3.47, дрон улетел на 25м), `−1` ГАСИТ (0.49 < 1.0). Монолит
  помечал «TODO tune» — теперь подтверждён, дефолт `−1`.
- `yaw ki = 0` — победитель свипа [[yaw-hold-tuning]] (интеграл вреден, bias yaw_flow).

**Известные лимиты v0:**
- Продольная ось (pitch) НЕ демпфируется — looming не портирован → дрон уходит по pitch
  (drift_check это учитывает: pitch = встроенный baseline).
- Флоу-метрика 0.49 vs монолит-цель 0.21 — зазор от run-to-run variance / интринсик
  (монолит fx=640 на 960-кадрах, у нас честный fx=W/2) / недобора гейнов, НЕ архитектуры.

**Дальше (не начато):**
1. Добор флоу до ~0.21 (свип gains / прогон на 1280) + порт продольного looming (pitch).
2. Sim-демо switch `Flow→Vins` в сценарии со сходящимся VINS (движущийся пилот → параллакс).
3. Excitation (`Pulse`/`Chirp`/`Translate`) — контракт `offset()` готов, реализаций нет.
4. `control_node.py` (bare пилот+стабилизация без FSM) — не написан.
5. Порт на боевой Orin (colcon в orin-контейнере; VinsHold уже на VINS-позе, не gt).

## Инварианты (не нарушать)

1. `domain/` без `rclpy`/`mavros_msgs`; perception — только numpy/cv2.
2. Зависимость строго `mission_pkg → control_pkg`.
3. `FlightMode` (команды) — отдельный порт от `RcOutput` (RC).
4. Все бюджеты/таймеры — в sim-времени по `/clock` (RTF-независимо).
5. На борту пилот выхватывает управление безусловно (Arbiter + FLTMODE_CH вкл).
6. `ControlStack` самодостаточен — работает и без `MissionRunner`.
7. Пилот — БАЗА стека; незанятая ось = сырой стик, flow-ось = velocity-assist.
