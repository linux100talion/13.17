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
| `Gz*` (позиция) | ground-truth Gazebo (СИМ) | **оракул** для тюнинга законов, sim-only |
| `Dp*` (демпфер) | оптический поток (борт) | **ДО init VINS — наш пре-VINS демпфер (ГЛАВНОЕ)** |
| `VinsHold` | VINS | ПОСЛЕ init: точный hold (рантайм switch — `VinsHandover`) |

Два семейства стабилизации именуются `<Источник><Ось>Hold`: **`Gz*`** держит ПОЗИЦИЮ по
gazebo (`GzPosHold` все оси / `GzRollHold` / `GzPitchHold` / `GzYawHold`); **`Dp*`** —
ДЕМПФЕР скорости к нулю по ОПТИЧЕСКОМУ ПОТОКУ (`DpHold` все / `DpRollHold`=flow_lateral /
`DpPitchHold`=looming / `DpYawHold`=flow_yaw). Gz = позиция/gt (sim), Dp = скорость/поток (борт).

```
взлёт ─► [Dp*-демпфер, пилот рулит] ──VINS сошёлся──► switch_stabilization(VinsHold) ─► [точный hold / авто]
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

- **Trajectory** → *намерение* как **стик-профиль** (`c_*` [-1..1], PROFILE-ONLY; метрик нет).
  Позиц-холдер интегрирует его в уставку, демпфер берёт целью. Источник взаимозаменяем:
  `ProfileTrajectory` (скрипт) / `Shuttle` / `RcTransmitter` (пилот) / (будущее) NN2.
- **Stabilization** → намерение + обратная связь → RC **по своим осям** (`axes`).
  `Gz*` (позиция/gt), `Dp*` (демпфер/поток), `VinsHold` (после init).
- **Excitation** → экзогенный зонд для system-ID (pulse/chirp) с политикой на ось
  (`ADDITIVE`/`REPLACE`). Не движение — другая задача (возбудить, не долететь).

### Режимы (recipes) → тройки стратегий

| Режим (`recipes.build_control_stack`) | Trajectory | Stabilization (список) | Excitation |
|---|---|---|---|
| `shuttle` (sim system-ID) | `Shuttle` | `[GzHold(roll+pitch)]` | — |
| `assisted` (пульт+gt) | `RcTransmitter` | `[GzHold(roll+pitch)]` | — |
| `manual` | `StaticSetpoint` | `[]` — всё пилоту | — |
| **`flow_assist`** (пре-VINS, борт) | `RcTransmitter` | `[DpRollHold, DpYawHold]` | — |
| (будущий) авто-манёвр | `Shuttle`/`Circle`/waypoint | `[VinsHold]` | — |
| (system-ID оси) | `Static` | `[GzHold]` | `Pulse`/`Chirp` |

Комбинаторика разрежена (flow/gz взаимоисключающи и т.п.) — валидные тройки собирает
`recipes.py`, а не «любая клетка легальна».

### Ортогональный путь: стабилизатор × миссия (BS_STAB / BS_MISSION)

`control_mode` — ЛЕГАСИ-ярлык: он слепляет в одно слово ДВЕ роли (стабилизатор +
траекторию). Рекомендуемый путь разносит их на две независимые ручки:

- **`BS_STAB`** → `recipes.build_stabilizers(cfg, spec)` — стабилизатор ПО ИМЕНИ, с
  `+`-склейкой: `GzPosHold`, `GzRollHold`, `DpHold`, `DpRollHold+DpYawHold`, `VinsHold`,
  `manual` (`[]`). Так «пульт + только yaw» = `DpYawHold`, «пульт + flow(roll)» =
  `DpRollHold`, боевой пре-VINS = `DpRollHold+DpYawHold`.
- **`BS_MISSION`** → `plan/mission_plan.compile_mission` — плейлист **профиль-токенов**
  как ДАННЫЕ: `Mission1` (реестр `MISSIONS`) или инлайн `climb3,mv_fwd2,mv_bkwd4,landing3`.
  Компилится в шаги PlanRunner `prearm→arm→<токены>→land`; каждый `mv_*`/`hover` = свой
  `Control`-сегмент со СВЕЖИМ `ControlStack(build_stabilizers(spec), ConstProfile)`.

Грамматика токена = глагол+число (`climb`=метры; `mv_*`/`hover`=секунды; уровень стика —
глобальный `cfg.mv_level`): `climb<h>`, `mv_fwd/bkwd/left/right/cw/ccw<t>`, `hover<t>`,
`land|landing<x>`. Один стабилизатор-spec на всё задание, любая траектория-профиль на
любой стек — то самое «везде только профили, любой профиль → любой стек».

Так `bootstrap` — просто одна из `MISSIONS` (`climb→hover→land`), а `Mission1` из диалога
= взлёт 3м → вперёд 2с → назад 4с → посадка. Легаси-`build_control_stack` оставлен для
валидированных прогонов (shuttle/assisted/flow_assist); профиль-миссии его не трогают.

## Пульт и per-axis композиция (центральная цель)

Пульт — не «режим сбоку», а **источник намерения = Trajectory**. `ControlStack`
композит по осям: **база = намерение траектории (оператор), `c_*`→PWM**, каждый
стабилизатор перезаписывает СВОИ оси, excitation — сверху. Отсюда без единого `if`:

- **незанятая ось → намерение траектории открытым контуром** (наклон оператора). Оператор
  взаимозаменяем: скрипт (`ConstProfile`/`Shuttle`) / живой пульт (`RcTransmitter` читает
  `s.pilot_*`) / (будущее) NN2 — profile-only до конца, единый «язык» `c_*`;
- **Dp-ось → velocity-assist**: стик = цель скорости, флоу гасит к ней; центр → демпф к нулю;
- **Gz/Vins-ось → интеграл**: стик = скорость уставки, холдер интегрирует → держит точку (Loiter);
- **`manual` = пустой список** стабилизаторов + `RcTransmitter` (оператор владеет всем);
- **микс**: «оператор + только yaw» = `[DpYawHold]`; боевой пре-VINS = `[DpRollHold,DpYawHold]`
  (pitch = оператор открытым контуром).

Живой пилот входит ТОЛЬКО как `Trajectory` (`RcTransmitter`), не как отдельная база стека —
поэтому «профиль» и «оператор» суть одно. `s.pilot_*` в снапшоте читает лишь `RcTransmitter`
(и `Arbiter` для safety-seize).

Плюс `Arbiter` (safety-supervisor поверх миссии): тумблер MANUAL → сырые стики (incl
throttle) безусловно. Адаптеры пилота взаимозаменяемы за портом `PilotInput`
(«переключить на реальный пульт» без изменения ядра): `ScriptedPilot` (headless-сим) /
`JoyPilot` (живой пульт: TX USB-джойстиком → `/joy`, МИМО FCU). ⚠️ `RosPilot`
(`/mavros/rc/in`) — ЛЕГАСИ: под активным override ArduPilot отдаёт в `RC_CHANNELS`
подменённые значения → нода читала бы СВОЮ команду как стик (петля; в MANUAL —
защёлка). Пред-override RC в MAVLink-телеметрии не существует, поэтому живые стики
входят только мимо FCU; нода — единственный писатель `/mavros/rc/override`.
Детали и наземный чек-лист — `docker/sim/laptop_move.md` §3.

## Слои и порты

```
        driving (входящий)                         driven (исходящие)
   timer 20Гц → node._tick                 RcOutput · FlightMode · Clock · Logger · DebugSink
        │                                          ▲
        ▼                                          │
   ┌─────────────── application ───────────────────┴───┐
   │  PlanRunner+Step (план фаз, mission_pkg) · ControlStack · Arbiter │
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
    flow_lateral=flow_longitudinal=flow_yaw=flow_conf=0.0          # СЫРЫЕ агрегаты (PID в домене)
    flow_seq: int = 0; flow_dt: float = 0.0                        # покадровая интеграция (тонкость 1)
    pilot_roll=pilot_pitch=pilot_throttle=pilot_yaw = RC_CENTER    # стики (ScriptedPilot/RosPilot)
    pilot_switch: int = 0                                          # тумблер авто/ручной (Arbiter)
    now_sim: float = 0.0

# setpoint.py
class AxisPolicy(Enum): REGULATE=auto(); ADDITIVE=auto(); REPLACE=auto()
@dataclass
class MotionIntent:
    c_fwd=c_right=c_yaw = 0.0    # PROFILE-ONLY: нормир. стик-уровень [-1..1], тело (единый «язык»)
@dataclass
class Setpoint:
    c_fwd=c_right=c_yaw = 0.0    # стик-команда, прокинутая стабилизатору (Dp=цель / Gz=интеграл)
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

Стабилизаторы: семейство `Gz*` (позиция/gt) — `GzHold(axes)` + алиасы GzPos/Roll/Pitch/Yaw;
семейство `Dp*` (демпфер/поток) — `DpRollHold`{roll}/`DpPitchHold`{pitch}/`DpYawHold`{yaw}/`DpHold`{все};
`VinsHold`{roll,pitch}; `PilotPassthrough` (легаси).

## Приложение (`application/`)

```python
# control_stack.py — per-axis композиция. Стабилизаторов СПИСОК. Юзабелен и без миссии.
class ControlStack:
    def __init__(self, stabilization, trajectory, excitation):
        self.stabs = _as_list(stabilization)              # один | список | []
    def switch_stabilization(self, s): self.stabs = _as_list(s)   # ГОРЯЧАЯ замена (пока не вызывается)
    def switch_trajectory(self, t): ...
    def switch_excitation(self, e): ...
    def enter(self, s):                                   # t0 (время траектории) + stab.enter() каждому
        ...
    def update(self, s) -> RcCommand:
        intent = self.traj.intent(s, t)                   # стик-профиль c_*
        sp = self._origin_plus(intent)                    # world-уставка + c_* passthrough
        rc = RcCommand(roll=_cmd_to_pwm(intent.c_right),         # БАЗА = намерение траектории
                       pitch=_cmd_to_pwm(intent.c_fwd),          # (оператор), c_*→PWM
                       throttle=RC_CENTER, yaw=_cmd_to_pwm(intent.c_yaw))
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

# mission_pkg/plan/ — полётное задание = СПИСОК Step'ов; PlanRunner их гоняет (не FSM-класс!).
# Step: AwaitMode/Arm/Climb/Control/Land/Hover — примитив фазы (tick → rc + NEXT/GOTO/FINISH).
# bootstrap_plan.build_bootstrap_plan(cfg, stack, handover) = [prearm,arm,climb,control,land].
# mission_plan.compile_mission(cfg, tokens, stab_spec, handover) — плейлист профиль-токенов
#   (BS_MISSION) → шаги; стабилизатор ортогонален (BS_STAB). MISSIONS — реестр заданий.
# Новое задание = ДАННЫЕ (список шагов/токенов), а не новый класс. Control-шаг питает ControlStack.
class PlanRunner:
    def tick(self, s) -> RcCommand: ...                   # текущий шаг → rc; переход по статусу шага
```

## Поток данных (один тик)

```
timer 20Гц → telemetry.snapshot() → DroneState
  → perception.merge(s)   # только flow_assist: камера → flow_* (+ flow_seq++)
  → s.pilot_* ← pilot.sticks()/mode_switch()             # стики в снапшот (как телеметрия)
  → PlanRunner.tick(s):  шаг prearm/arm/climb/land → FlightMode + простой RcCommand
                            EXCITE                 → ControlStack.update → RcCommand
  → Arbiter.resolve(s, rc)                                # уступить пилоту, если MANUAL
  → RcOutput.publish(rc) + DebugSink.publish(...)
wall-цикл в main → RcOutput.publish(тот же rc)            # свежесть override на FCU
```

Детерминизм override: точки СМЕНЫ значения задаёт sim-таймер (tick), wall-цикл лишь
ре-публикует неизменное между тиками значение.

## Две тонкости (иначе сломается)

1. **Flow-PID интегрирует ПО КАДРАМ, не по тикам.** Адаптер кладёт `flow_seq`+`flow_dt`,
   а `Dp*`-демпферы продвигают интегратор ТОЛЬКО при смене `flow_seq` (иначе 20-Гц
   тик накрутил бы одну ошибку на стоячем сигнале). Стратегия чистая, физика per-frame.
2. **Опору владеет КАЖДЫЙ позиц-холдер сам** — интегрирует стик-профиль `c_*` от своей
   опоры в своём фрейме (Gz в gt, Vins в vins). ControlStack лишь тактует время. Это
   разъединяет фреймы Gz↔Vins (раньше общий origin в стеке коряво увязывал их).

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
  test/              test_profile_motion.py test_pilot_strategies.py test_families.py
                     test_flow_strategies.py test_multiaxis_stack.py test_handover.py

src/mission/                       # ament_python пакет mission_pkg (потребитель control)
  package.xml setup.py setup.cfg resource/mission_pkg
  mission_pkg/
    config.py  recipes.py
    plan/         step.py runner.py bootstrap_plan.py   # план фаз (bootstrap = один план)
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
  {shuttle|assisted|manual|flow_assist}`, `--pilot {scripted|joy|ros}` (joy = живой
  пульт через `/joy`; ros — легаси, петля под override).
- Сверка поведения — метрикой (`drift_check.py`/`yaw_check.py`/`scene.mp4`), не на глаз.

## Статус

**Все три среза закодированы, покрыты 6 оффлайн-гейтами и прогнаны в симе.** Домен
чист (grep), зависимость `mission→control` соблюдена.

| Срез | Что | Оффлайн | Прогон в симе |
|---|---|---|---|
| 1 · `shuttle` | gz-hold + челнок; ядро композиции | profile-motion + FSM (эквивалентность Δ=0 сняла profile-рефакторинг) | ✅ `HOLD_DONE` |
| 2 · `assisted`/`manual` | пульт-как-стратегия + Arbiter | pilot-стратегии + FSM assisted/manual/seize | ✅ `HOLD_DONE` |
| 3 · `flow_assist` | пре-VINS флоу-демпфер + per-axis стек | флоу-стратегии + per-axis композиция | ✅ (см. ниже) |

**Оффлайн-гейты (10, чистый python, без ROS):** `test_profile_motion` (интеграл стик-профиля
+ холд + симм. челнок), `test_bootstrap_fsm`, `test_pilot_strategies`, `test_pilot_fsm`,
`test_flow_strategies`, `test_multiaxis_stack`, `test_handover` (switch Flow→Vins),
`test_families` (Gz*/Dp* per-axis + DpPitchHold/DpHold), `test_plan` (PlanRunner NEXT/GOTO/FINISH),
`test_mission_plan` (реестр `build_stabilizers` + '+'-склейка + токен-грамматика + прогон Mission1).

**ПЛАН-СЛОЙ (переход):** захардкоженный `MissionRunner` FSM заменён на `PlanRunner` +
`Step`-примитивы (`AwaitMode`/`Arm`/`Climb`/`Control`/`Land`/`Hover`). Полётное задание =
**список шагов** (`bootstrap_plan.build_bootstrap_plan`), а не класс; новое задание = данные.
Переходы NEXT/GOTO(по имени)/FINISH. bootstrap = один план. `test_plan` + переписанные
`test_bootstrap_fsm`/`test_pilot_fsm` на `PlanRunner`.

**Ортогональный stab×mission — РЕАЛИЗОВАН (оффлайн-проверен).** Полётное задание как
ДАННЫЕ: `BS_MISSION` (плейлист профиль-токенов `climb3,mv_fwd2,mv_bkwd4,landing3` или имя
из `MISSIONS`) × `BS_STAB` (стабилизатор по имени, `+`-склейка). `compile_mission` строит
шаги PlanRunner, каждый `mv_*`/`hover` — `Control`-сегмент со свежим стеком из
`build_stabilizers(spec)` + `ConstProfile`. Разнял две роли, слитые в `control_mode` (тот
оставлен как легаси-ярлык). `test_mission_plan`. **Подтверждён в симе** (`GzPosHold` ×
`climb3,mv_fwd2,mv_bkwd4,landing3`): план отсеквенировался → `MISSION_DONE`, каждый
`mv_*` сам завершился по длительности токена, `GzPosHold` проинтегрировал профиль
(смещ за окно ≈0.1–0.2м — ушёл вперёд и вернулся). Осталось: библиотека готовых заданий
(waypoint/return-home) + Circle/GoTo-траектории (относительные, до NN1).

**PROFILE-ONLY движение (переход):** метрический канал `d_*` убран — движение везде это
**стик-профиль** `c_*` (`ProfileTrajectory`/`Shuttle`/`RcTransmitter`). Позиц-холдеры
(`Gz*`/`Vins`) интегрируют профиль в уставку от своей опоры (разъединяет фреймы Gz↔Vins);
демпферы (`Dp*`) берут его целью скорости. Устаревший `test_gz_shuttle_equiv` (проверял
d_*-модель монолита) снят → `test_profile_motion`. Любой профиль → любой стек.

**Рантайм switch `Flow→Vins` — РЕАЛИЗОВАН (оффлайн-проверен).** `VinsHold` (position-hold
по VINS, СВОЯ опора в vins-фрейме, захват в момент switch) + `VinsHandover` (application:
детектор «VINS ready» = N odom + свежесть → ОДНОКРАТНО `stack.switch_stabilization`).
`Control`-шаг зовёт `handover.maybe_switch`; пилот (`RcTransmitter`) при switch
не меняется. Флаг `--handover-vins` (flow_assist). Это первый реальный вызов `switch_*`.
⚠️ Sim-демо самого switch требует сценария, где VINS СХОДИТСЯ (нужно движение/параллакс;
нейтральный flow_assist его не даёт) — механизм готов, демо-прогон отдельно.

**Живой пульт = `JoyPilot` — ЛЕТАЕТ (TX12, два живых полёта в симе 2026-08-16).**
Петля `rc/override → rc/in` подтверждена как устройство ArduPilot (override замещает
`radio_in`, `RC_CHANNELS` отдаёт его же) → `RosPilot` понижен до легаси, живые стики
идут `/joy` → `joy_sticks()` (чистое ядро, покрыто `test_pilot_strategies`) →
`JoyPilot`. Обвязка: `joy_linux_node` из `bootstrap_arch2.sh` (`BS_PILOT=joy`),
`ros-humble-joy-linux` в образе `nav`, `/dev/input` hotplug-каталогом в compose.
Знаки осей выверены полётами → `JOY_SIGNS_DEFAULT` (roll/yaw зеркальны).
Осталось: газ пилота в assisted (сейчас — только MANUAL, без защёлки центра),
MANUAL-seize в воздухе не испытан, доставка намерения на боевой Orin мимо FCU
(открытая ветка — laptop_move.md §3).

**Подтверждённые факты (drift_check в симе):**
- `roll_osign = −1` — знак торможения флоу-демпфера. `+1` РАЗГОНЯЛ снос (метрика
  боковая/продольная RMS_v = 3.47, дрон улетел на 25м), `−1` ГАСИТ (0.49 < 1.0). Монолит
  помечал «TODO tune» — теперь подтверждён, дефолт `−1`.
- `yaw_ki = 0` — победитель свипа [[yaw-hold-tuning]] (интеграл вреден, bias yaw_flow).

**Параметры демпфера — ТРИ НЕЗАВИСИМЫЕ ОСИ.** В конфиге нет общего «flow_*»: есть
`roll_*`, `pitch_*`, `yaw_*`, каждый со своим полным набором (kp/ki/kd/imax/max/
conf_min/conf_full/osign/cmd_gain/smooth; у yaw нет kd — `DpYawHold` это PI).
Дублирование намеренное: тюнинг у осей разный, а общий префикс это маскировал
(`flow_kp` правил разом roll и pitch, при этом yaw брал пороги `flow_conf_*`).
Env-ручки прогона — `BS_ROLL_*` / `BS_PITCH_*` / `BS_YAW_*` (`bootstrap_arch2.sh`).
`flow_lateral/flow_longitudinal/flow_yaw/flow_conf` в `DroneState` — это СИГНАЛЫ
перцепта, имена не менялись.

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
6. `ControlStack` самодостаточен — работает и без `PlanRunner`.
7. Пилот — БАЗА стека; незанятая ось = сырой стик, flow-ось = velocity-assist.
