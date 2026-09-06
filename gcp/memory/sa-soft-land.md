---
name: sa-soft-land
description: "Мягкая посадка по кнопке SA (SoftLand) — две ветки по семантике стиков в LAND ArduCopter; hover throttle учится сам (MOT_HOVER_LEARN=2, eeprom в volume)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 21c1a194-d0f7-45b5-b758-827f7ea9a513
  modified: 2026-09-01T15:41:33.824Z
---

Реализовано 2026-08-30 (ветка nn2_c3_laptop_yaw, НЕ доказано полётом — только офлайн-тест
`src/mission/test/test_freefly_land.py`): кнопка SA (TX12) в freefly → при rel_alt ≤ 5 м
и |v| ≤ 1 м/с (`BS_LAND_ALT_MAX`/`BS_LAND_V_MAX`; с 2026-08-30 по просьбе — было 1 м/0.3; бюджет
шага = max(land_budget, 2·alt_max/rate) ≈ 67 с) шаг `SoftLand` (`plan/step.py`).

**Why:** «плавно = 70–80 % газа ховера» физически не работает: 80 % ховера с 1 м в
разомкнутом контуре = 0.2 g → 2 м/с об землю; мягко (0.3–0.5 м/с) — только замкнутым
контуром vz, который у FCU уже есть (газ = MOT_THST_HOVER + PID vz). Ловушка №2: у LAND
ArduCopter две ветки по `position_ok()` на входе — position-LAND трактует roll/pitch как
УСТАВКУ СКОРОСТИ (LAND_REPOSITION=1; трим демпфера 44 PWM ≈ 0.14 м/с постоянного хода),
nogps-LAND — как наклон; ветку FCU наружу не отдаёт, а в LV=2 берёт position-ветку даже
на нулевой позе моста до VINS (ekf=1 весь полёт в bag lv2_joy_20260830_101919).

**How to apply:** ветка `pos` только когда FCU уже в LOITER (латч доказал позицию):
set_mode LAND, стек пуст, LAND_SPEED 15 (sitl-extra.parm; диапазон 30-200 — лишь метаданные,
прошивка пола не ставит). ⚠️ 2026-09-01: `LAND_SPEED` в этой прошивке ПЕРЕИМЕНОВАН в
`LAND_SPD_MS` (СИ) — строка игнорируется, фактический спуск LAND 0.5 м/с (дефолт),
втрое быстрее задуманного ([[ardupilot-48-param-renames]]). Иначе — в т.ч. «сесть сразу до VINS» — ветка `alt`: ALT_HOLD,
газ 1381 PWM (land_rate 0.15 м/с: центр − dz − 0.15/3.16·span, формула
AltHold), стек демпфер/VinsHold по готовности, касание (баро | gt | `/mavros/extended_state`
ON_GROUND — стрим 2 запрошен в nav_up.sh) → газ в пол → arm(False) через 1 с, force через
5 с. Кнопка: `BS_LAND_JOY` (дефолт `b0`). КВИРК TX12 (HID-дескриптор снят 2026-08-30): осей в
дескрипторе 8, но CH7 и CH8 — обе `Slider` → Linux кладёт их на один ABS_THROTTLE → joydev
видит 7 осей, CH8 ДЕРЁТСЯ с CH7 (SF-мастер) за axes[6] — CH8 не использовать; кнопки
b0..b23 = каналы после осей (по коду EdgeTX CH9..CH32, нажата при канале > 0) — но индекс
только по пробнику `src/lab/joystick/js_probe.py` (хост, /dev/input/js0): ИЗМЕРЕНО
2026-08-30 — SA приходит как buttons[1] (7 чистых нажатий, без дрожи осей) → в
docker/sim/.env бокса BS_LAND_JOY=b1 (дефолт ноды b0; при переносе SA — перемерить).
В ленте joy_timeline — «JOY: кнопка b1». С хоста без пульта `make sa-land`. HUD: баннер `LANDING …`, `land=`/`sa=` в статусе. MAVROS не декодирует
EKF_STATUS_REPORT (только ESTIMATOR_STATUS PX4) — флагов EKF ArduPilot в ROS нет.
Hover throttle: MOT_HOVER_LEARN=2 (дефолт), учится в ALT_HOLD/LOITER (наш демпфер тоже),
сохраняется на дизарме в eeprom (volume sim_sitl_eeprom, живёт через fresh-start);
прочитано 0.3378 (30.08). Связано: [[sf-master-ladder]], [[openhd-debug-hud]].

2026-09-06 (просьба пилота): (1) ОТМЕНА повторным нажатием SA до касания — в любой ветке и ярусе:
SoftLand ловит второй фронт (зажатая на входе кнопка — не фронт), в ветке pos шлёт keep (ALT_HOLD)
сразу (Freefly «уважает» LAND и сам из него не выводит), goto freefly с LAND_CANCEL; Freefly.enter в
воздухе: стек+опора от текущей точки, «возврат в свободный полёт», _stack_applied — первый
_ladder_apply перепринимает стек (после ветки pos он пуст), LOITER вернётся по своему гейту; после
касания отмены нет (газ в полу). Ручка BS_FF_LAND_CANCEL (дефолт 1). Тест test_freefly_land §10–12.
(2) LAND_SPD_MS: пробовали 0.30, пилот вернул 0.15 = land_rate ветки alt (одинаково на всех ярусах).
ПОЛЁТ 180821: отмена на ярусах 0/1 работает (damper 26 с, vinshold 56 с → freefly); в LOITER НЕ
сработала: SoftLand слал один async set_mode(ALT_HOLD) из LAND, а Freefly._mode_target LAND «уважает»
(EKF-failsafe) — борт продолжал садиться, стек в position-LAND командовал бы скорость. ФИКС: Freefly.enter
в LAND → окно LAND_EXIT_SEC 5 с: _mode_target возвращает keep (ре-ассерт каждые 2 с) + set_mode сразу,
стики в центре пока mode==LAND; не вышел за 5 с → error, дальше LAND уважаем. Тест §13. Полётом не
проверено (bag 181111 на момент разбора ещё писался).
ПОЛЁТ 182028 с фиксом (ре-ассерт keep 5 с): «так не работает» — отмена из LAND в LOITER всё равно не
срабатывает; bag 181111/182028 обрываются на входе в LOITER (запись не дописана после kill стека /
ещё идёт) — что ответил FCU, не видно. По просьбе пилота кнопка SA на ярусе LOITER ОТКЛЮЧЕНА
(BS_LAND_IN_LOITER, дефолт 0: отказ с подсказкой «сажай руками или CH6 вниз → ярус 0/1 → SA»),
тест §14; ветка pos SoftLand и код отмены из LAND остаются под ручкой land_in_loiter=1 до разбора
поведения FCU (гипотезы: ArduCopter не принимает смену режима из LAND по нашему SetMode — проверять
по mavros.log «Mode change failed»/COMMAND_ACK; или наш set_mode шлётся до того, как FCU залатчил
LAND, и потом LAND ре-ассертится SoftLand'ом… нет — SoftLand уже завершён).