---
name: control-refactor-arch
description: "рефакторинг alt_hold_bootstrap.py → src/control (hexagonal/DDD), ветка nn2_c3_control, пульт-как-стратегия"
metadata: 
  node_type: memory
  type: project
  originSessionId: 3e4cb44f-d2bd-4acf-889e-3a7656f65ac8
  modified: 2026-07-27T04:15:48.322Z
---

Рефакторим монолит `src/lab/alt_hold_bootstrap.py` (~865 строк) в production-пакет
`src/control` (control_pkg) — hexagonal + DDD. Ветка **`nn2_c3_control`** (от
nn2_c3_vins_althold_5). Проектный документ: **`src/control/architecture.md`** (читать первым).

**Почему:** ядро управления должно тестироваться на числах без Gazebo, работать
одинаково в симе и на боевом Orin (источник позы: gt в симе / VINS на борту меняется
адаптером), режимы свободно комбинироваться без if-лестниц, и — главная цель —
**реальный пульт подключаться как стратегия** (assisted: стики→MotionIntent+стабилизация;
ручной: PilotPassthrough). Адаптер /mavros/rc/in идентичен в симе и на борту.

**Ключевое решение — ТРИ роли, не две:** Trajectory (намерение) / Stabilization
(намерение+ОС→RC) / Excitation (экзогенный зонд с политикой осей ADDITIVE/REPLACE).
Плоская сумма motion+stab−центр физически неверна (shuttle/circle впрыскивают
СЕТПОЙНТ в PID, pitch-excite аддитивен, roll-excite вытесняет ось).

**Пакеты:** control_pkg (ядро) + mission_pkg (FSM bootstrap — потребитель), зависимость
строго mission→control. Оба ament_python, colcon в обоих контейнерах (nav сим + orin).

**План:** миграция вертикальными срезами под флаг `--arch2`, CLI 1:1 (не ломать
bootstrap.sh/liftland.sh/yaw_tune_sweep.sh). Сверка метрикой: yaw_check.py/drift_check.py/scene.mp4.

**СРЕЗ 1 (gz-hold + shuttle): КОД ГОТОВ (commit 46b3abe), оффлайн-гейты зелёные.**
Пакеты `src/control` (control_pkg) + `src/mission` (mission_pkg), оба ament_python.
Ноды-точка входа: `ros2 run mission_pkg bootstrap_arch2` (console_script). Тесты (чистый
python, без ROS): `src/control/test/test_gz_shuttle_equiv.py` — закон побитово=монолит
(Δ=0 на 5 кейсах); `src/mission/test/test_bootstrap_fsm.py` — автомат PREARM→DONE на фейках.
**СРЕЗ 1 ПОДТВЕРЖДЁН В СИМЕ (2026-07-26).** colcon-интеграция сделана (commit 365c7c6):
mounts src/control+src/mission в docker/sim/docker-compose.yml, инкрементальная сборка в
nav_up.sh, команда `bootstrap_arch2` в capture_scene.sh + обёртка src/lab/bootstrap_arch2.sh.
Атомарный прогон (CPU-бокс, 960x540, fresh-start) прошёл всю миссию PREARM→ARM→CLIMB(3.2м)→
EXCITE(gz-hold+shuttle)→LAND→DONE, ИТОГ HOLD_DONE, видео залито на Drive. odom=0 — норма
(gz-hold по gt Gazebo, не VINS). Запуск: `ros2 run mission_pkg bootstrap_arch2` в p1317_nav
(via /lab/bootstrap_arch2.sh). Монолит alt_hold_bootstrap.py остаётся параллельно.
**СРЕЗ 2 (пульт-как-стратегия): КОД ГОТОВ (commit 2f81241), оффлайн-гейты зелёные.**
RcTransmitter (trajectory: стик=скорость уставки, интеграл → assisted), PilotPassthrough
(stabilization: сырые стики → RC, manual), Arbiter (application: safety-seize по тумблеру
MANUAL → сырые стики incl throttle), адаптеры RosPilot(/mavros/rc/in — борт) + ScriptedPilot
(профиль стиков — sim без пульта), pilot-поля в DroneState, recipes.build_control_stack
(shuttle|assisted|manual). Запуск: `ros2 run mission_pkg bootstrap_arch2 --control-mode assisted`
(или BS_CONTROL_MODE=assisted через capture_scene). Тесты: src/control/test/test_pilot_strategies.py,
src/mission/test/test_pilot_fsm.py. Срез 1 без регресса (mode=shuttle дефолт).
ОСТАЛОСЬ: атомарный прогон --control-mode assisted в симе.

**СЕВЕРНАЯ ЗВЕЗДА (уточнено 2026-07-26):** боевое назначение всего этого — **до init
VINS дать пилоту НАШ простой стабилизатор для стиков**. На борту до VINS нет опоры позиции
(ни GPS/VINS/gt) → «наш простой» = ДЕМПФЕР СНОСА ПО ОПТИЧЕСКОМУ ПОТОКУ: FlowDamper(боковой)+
YawHold(курс), даёт скорость/курс без GPS. Это уже наполовину в монолите (flow_estimator.py/
flow_hold/yaw_hold [[yaw-hold-tuning]]) — надо ПОРТИРОВАТЬ в control_pkg как StabilizationStrategy.
Расклад стабилизаторов: GzPositionHold=sim-оракул(gt) для тюнинга; FlowDamper+YawHold=пре-VINS
борт (ГЛАВНОЕ); VinsHold=после init. RcTransmitter(пилот) ОДИН для всех — меняется только
стабилизатор. Таймлайн: взлёт→[Flow+пилот]→VINS ready→switch_stabilization(VinsHold)→[точный
hold/авто-манёвры]. Тут hot-swap ControlStack.switch_* (п.2, пока НЕ вызывается) обретает смысл.
shuttle/excitation/circle = для sim-калибровки + будущих авто-манёвров.

ТОНКОСТЬ перед портом FlowDamper: RcTransmitter сейчас интегрирует стик в СМЕЩЕНИЕ ПОЗИЦИИ
(под position-hold Gz/Vins). FlowDamper даёт только СКОРОСТЬ → намерение должно стать
СКОРОСТЬЮ (MotionIntent → vx/vy/yaw_rate, как MotionCommand у ChatGPT); position-hold её
интегрируют/берут feedforward. Развилка в контракте MotionIntent — принять ДО порта FlowDamper.

**СРЕЗ 3 (пре-VINS флоу + per-axis): ДОМЕН ГОТОВ (commit 2c3c417), 6 оффлайн-гейтов зелёные.**
ControlStack теперь PER-AXIS: стабилизаторов СПИСОК, каждый владеет осями (axes); база стека
= сырые стики пилота (незанятая ось → ручной наклон); стабилизатор перезаписывает свои оси.
Решения юзера: незанятая ось = сырой стик; flow-ось = velocity-assist (стик=цель, флоу гасит к
ней). manual = [] (PilotPassthrough избыточен), flow_assist = [FlowDamper(roll),YawHold(yaw)]+пилот.
FlowDamper/YawHold — порт flow_hold/yaw_hold монолита (yaw ki=0 [[yaw-hold-tuning]]), ПОКАДРОВАЯ
интеграция (flow_seq), conf/stale-fade. MotionIntent/Setpoint: +c_fwd/c_right/c_yaw (нормир.
скорость-команда); d_* остаётся для Gz/Vins → срез1 Δ=0 сохранён после мульти-стек рефактора.
Тесты: test_flow_strategies.py, test_multiaxis_stack.py.
RosPerception СДЕЛАН (commit db2bb67, вариант A: flow_estimator.py скопирован в
control_pkg/perception/ — канонично, self-contained для борта; src/lab-копия = легаси).
Камера(mono8)+гиро(FLU)→FlowEstimator→flow_* в DroneState через merge(), flow_seq++ на кадр.
Интринсики из разрешения (fx=W/2), R_cam_imu+rotsign из монолита. Нода: RosPerception только
для flow_assist, нейтральный пилот (флоу гасит снос сам).

**СРЕЗ 3 ПОДТВЕРЖДЁН В СИМЕ (2026-07-26, commit 9a1d077).** Атомарный flow_assist-прогон +
drift_check: **flow_osign=+1 РАЗГОНЯЛ снос (метрика 3.47), −1 ГАСИТ (0.49<1.0)** → знак был
единственной проблемой (монолит помечал «TODO tune»). Дефолт flow_osign → −1 (config+node).
Флоу-демпфер (пре-VINS, per-axis с пилотом) валидирован end-to-end. ⚠️ Продольная ось (pitch)
НЕ демпфируется (v0, не портирована looming) → дрон уходит по pitch; drift_check это учитывает
(pitch=baseline). Метрика 0.49 vs монолит-цель 0.21 — зазор от run-to-run variance/интринсик
(монолит юзал fx=640 на 960-кадрах, у нас честный fx=480)/тюнинга, НЕ архитектуры.

**РАНТАЙМ SWITCH Flow→Vins РЕАЛИЗОВАН (commit 6154ecf, 8 оффлайн-гейтов зелёные).** VinsHold
(position-hold по VINS, зеркало Gz, но СВОЯ опора в vins-фрейме — захват в enter() на момент
switch; ControlStack-origin в gt не годится, на борту gt=0) + VinsHandover (application:
детектор «VINS ready»=N odom+свежесть → ОДНОКРАТНО stack.switch_stabilization). Первый реальный
вызов switch_*. MissionRunner зовёт в EXCITE; пилот RcTransmitter при switch не меняется. Флаг
--handover-vins (flow_assist). DroneState +vins_x/y/yaw/vx/vy; RosTelemetry._on_odom заполняет.
⚠️ Sim-демо switch требует сценария со СХОДЯЩИМСЯ VINS (движение/параллакс; нейтральный flow_assist
не даёт) — механизм готов, демо отдельно.

**СЕМЕЙСТВА Gz*/Dp* (commit 7aa72b3):** демпфер гасит скорость по ОПТИЧЕСКОМУ ПОТОКУ
(подтверждено src/lab/flow_damp_node: res['lateral'], scale-free) → FlowDamper переименован
в DpRollHold, YawHold в DpYawHold. Систематика <Источник><Ось>Hold. Gz* = позиция по gt
(GzHold(axes) база + GzPosHold/GzRollHold/GzPitchHold/GzYawHold; GzPosHold добавляет курс-холд
по yaw — новый P-закон, знак не выверен). Dp* = демпфер по потоку (DpRollHold=flow_lateral,
DpPitchHold=looming/flow_longitudinal НОВЫЙ не летан, DpYawHold=flow_yaw, DpHold=композит).
+flow_longitudinal в DroneState/RosPerception. test_families.py. 8/8 гейтов, Δ=0 держится.
Полный список стратегий — в architecture.md «Статус».

**PROFILE-ONLY ДВИЖЕНИЕ (commit 74c752e):** движение везде = нормир. стик-профиль c_* [-1..1],
метрики (d_*) убраны (до NN1 расстояния не опираемы). MotionIntent/Setpoint = c_* only.
Позиц-холдеры Gz*/Vins ИНТЕГРИРУЮТ c_* в свою уставку от СВОЕЙ опоры в своём фрейме (cmd_gain=0.8)
→ побочно разъединяет фреймы Gz↔Vins (общий origin в ControlStack убран). Dp* берут c_* целью.
trajectory: ProfileTrajectory (скрипт-профиль), Shuttle (level/leg/pause как стик-профиль),
RcTransmitter без интеграла. Любой профиль → любой стек. test_gz_shuttle_equiv снят (проверял
d_*-модель монолита) → test_profile_motion. 8/8 гейтов. Полётные задания = профили (см. диалог:
Gz мог бы в метрах, но решили нет). Слой планов (Step/PlanRunner) — ещё НЕ начат.

**ПЛАН-СЛОЙ (commit 13d63d4):** MissionRunner FSM УДАЛЁН, заменён на PlanRunner + Step-примитивы
(mission_pkg/plan/: step.py AwaitMode/Arm/Climb/Control/Land/Hover, runner.py, bootstrap_plan.py).
Полётное задание = СПИСОК Step'ов (данные, не класс). Переходы NEXT/GOTO(по имени)/FINISH.
bootstrap = один план. Control-шаг питает ControlStack (+VinsHandover), wait_gt только для gz.
Нода на PlanRunner. Тесты: test_plan + переписанные test_bootstrap_fsm/test_pilot_fsm. 9/9 гейтов.
ДАЛЬШЕ по планам: библиотека готовых планов (waypoint/return-home) + Circle/GoTo-траектории
(относительные, до NN1).

**ВЕСЬ РЕФАКТОР (profile-only + plan) ПОДТВЕРЖДЁН В СИМЕ (shuttle-прогон, HOLD_DONE):** план
prearm→arm→climb→control→land отсеквенировался, Shuttle-профиль отработал полный цикл (motion_done),
GzHold-интегратор держал позицию в пределах 1.2м (окно control/10с; 25с-окно даёт 5.7м — это
дрейф фазы land, не регресс). drift_check метрика ≈1 бессмысленна для shuttle (это флоу-диагностика,
обе оси gz-held). Регресса нет.

**ОРТОГОНАЛЬНЫЙ путь stab×mission РЕАЛИЗОВАН (оффлайн, 10/10 гейтов).** По просьбе пользователя
разнёс две роли, слитые в BS_CONTROL_MODE, на две независимые ручки:
- BS_STAB → recipes.build_stabilizers(cfg, spec): стабилизатор по ИМЕНИ с '+'-склейкой
  (GzPosHold|GzRollHold|DpHold|DpRollHold+DpYawHold|VinsHold|manual). Реестр _STAB.
- BS_MISSION → plan/mission_plan.compile_mission: плейлист профиль-токенов как ДАННЫЕ
  (имя из MISSIONS: Mission1/square/bootstrap, ИЛИ инлайн 'climb3,mv_fwd2,mv_bkwd4,landing3').
  Грамматика глагол+число: climb=МЕТРЫ, mv_*/hover=СЕКУНДЫ, уровень стика глобальный cfg.mv_level
  (default 0.3). Направления mv_fwd/bkwd/left/right/cw/ccw. → шаги prearm→arm→<токены>→land,
  каждый mv_*/hover = Control-сегмент со СВЕЖИМ ControlStack(build_stabilizers(spec)+ConstProfile).
Новый примитив траектории ConstProfile (постоянная c_* dur сек, done). control_mode оставлен
легаси-ярлыком (валидированные shuttle/assisted/flow_assist не трогаю). Обёртки: BS_STAB/
BS_MISSION/BS_MV_LEVEL в bootstrap_arch2.sh + capture_scene.sh. wait_gt=('Gz' in spec).
ПОДТВЕРЖДЁН В СИМЕ (BS_STAB=GzPosHold BS_MISSION=climb3,mv_fwd2,mv_bkwd4,landing3, 960×540 CPU):
план prearm→arm→climb0(3.2м)→mv_fwd_1→mv_bkwd_2→land отсеквенировался, MISSION_DONE. Каждый
mv-сегмент сам завершился по длительности токена (2с/4с sim). drift_check: смещ за окно ≈0.1–0.2м
(ушёл вперёд и вернулся — GzPosHold проинтегрировал профиль), макс 0.8м = намеренная экскурсия.
По дороге фикс бага двойного резолва (list in MISSIONS unhashable → resolve_mission идемпотентна, 9069caf).

БОЕВОЙ НАБОР Dp — прогоны в симе (960×540 CPU, drift_check SAFE_SEC=8, gt odom):
- DpRollHold+DpYawHold: MISSION_DONE. Боковая (roll ДЕМПФЕР) 0.3м / RMS_v 0.14 ✅ гасит снос по потоку.
  Продольная (pitch) убежала 8.4м — осью НИКТО не владеет (mv_fwd/bkwd командуют pitch, но в наборе
  roll+yaw её не держит ни стабилизатор, ни пилот; профиль≠пилот, живого пульта в симе нет; ALT_HOLD
  без position-hold → импульс интегрируется). Это боевая правда: в пре-VINS pitch даёт ОПЕРАТОР.
  Видео 1_DMrxN931-Gl1ZRv3tpjTivbqNbPZhpW.
- DpHold (roll+pitch+yaw): РЕГРЕСС. DpPitchHold НЕ гасит 8.4м, а РАЗГОНЯЕТ до 14.5м (унаследовал
  flow_osign=-1 от roll — для looming знак НЕВЕРЕН, положит. ОС). Разгон по pitch сломал и roll
  (0.3м→14.2м): дрон ушёл с текстуры, поток выродился. Норм. метрика 1.17 (≈1.0=нет демпфа).
  ВЫВОД: DpPitchHold нельзя тюнить через общие flow_* (флип flow_osign перевернул бы и roll).
  Нужен СПЛИТ параметров (pitch_osign/kp/ki) + вероятно другой закон (looming/дивергенция ≠
  поперечная трансляция). Пока боевой = DpRollHold+DpYawHold, pitch на операторе. DpHold в бой НЕ годится.

СТАБИЛИЗАЦИЯ НАБОРА/СБРОСА (f7c2c4a): Climb/Land раньше слали центр-стики (горизонт держал
только Control) → дрейф копился на наборе. Теперь Climb/Land проводят hold-стек (StaticSetpoint +
те же стабилизаторы) через _overlay_stack — горизонт с отрыва до касания. ПОДТВЕРЖДЕНО в симе
(GzPosHold, та же миссия, 1280×720): продольное конечное +5.85м→+1.38м, макс гориз. 5.97м→1.45м.
Остаток 1.4м = амплитуда самой mv_fwd-команды (0.6·0.8·3с=1.44м), не снос. Юзер прав был.

ПРОФИЛЬ = ОПЕРАТОР (2c7912f): база ControlStack теперь = намерение траектории (c_*→PWM),
незанятая ось = наклон оператора открытым контуром (было: сырой пилот, профиль на незанятой
оси отбрасывался). Живой пилот входит только через RcTransmitter; manual→RcTransmitter.
LAND-ФИКС (7a4fbf3): Land ловит касание по gt_z (истинная высота), не только по баро rel_alt
(врало/лагало → дрон за краем сцены зависал на 180-сек бюджете). Бюджет 120→45 (бэкстоп).
ПОДТВЕРЖДЕНО в симе: land закрылся по касанию при rel_alt=0.937/gt_z=0.262, прогон
самозавершился (раньше висел ~час). STATION-KEEP (47dfe0c) — примитив StationKeep (+τ/−2τ/+τ) + токены sk_fwd/bkwd/right/left.
В ИДЕАЛЕ (двойной интегратор) возвращает v,x→0 (юнит ok). НО СИМ ОПРОВЕРГ автономное
удержание: DpRoll+DpYaw + sk_fwd level0.3 → дрон РАЗОГНАЛСЯ монотонно до 577м (хуже 82м от
постоянного mv). Причина: идеальная отмена держится на «стик=мгновенное симм. ускорение при
постоянном курсе»; реально — лаг наклона + как только тронулся и ушёл со сцены, флоу roll/yaw
теряют поток → курс плывёт → ускорение в вертящемся фрейме не отменяется → скорость копится,
докатился на LAND. ЖЕЛЕЗНЫЙ ВЫВОД: открытый контур на pitch НЕ держит станцию (ни постоянный
наклон, ни station-keeping) — продольная требует ЗАМКНУТОГО контура: живой оператор (боевой
пре-VINS, закрывает глазами) или позиц-опора VINS/gz. Это и есть северная звезда: до VINS
ведёт оператор, флоу лишь демпфирует roll/yaw снос. StationKeep оставлен как короткий
excite-импульс (не для удержания). land-фикс (gt_z) работал во всех прогонах — не висли.

ДАЛЬШЕ (не начато): (1) добор флоу до ~0.21 + looming(pitch); (2) sim-демо switch с движущимся
пилотом; (3) excitation Pulse/Chirp/Translate (контракт offset() готов); (4) control_node.py bare;
(5) порт на боевой Orin (colcon в orin-контейнере).

Инварианты: domain/ без rclpy/cv2; FlightMode(команды) отдельно от RcOutput(RC); бюджеты
в sim-времени; на борту пилот выхватывает управление безусловно (Arbiter+FLTMODE_CH вкл).
Прецедент чистого домена в репо — [[imu-sim-freq-sim]] контекст: `flow_estimator.py`
переиспользуется как перцепт-сервис.
