cmd/att_out — A/B выхода домена: override (PWM) против углов (SET_ATTITUDE_TARGET), 2026-09-30
============================================================================================

ЗАЧЕМ. Переезд на углы (docker/sim/laptop_move.md §5.7), схема по решению пилота:
  * ALT_HOLD — ТОЛЬКО ПИЛОТ: стики с RC-входа полётника (в симе — мост /joy → RC SITL,
    src/sim/joy_rc_bridge.py; на борту — приёмник), нода вне цепочки, override НЕТ вообще;
  * нода управляет только в GUIDED_NOGPS углами (порт AttitudeOutput, пересчёт PWM → углы
    как в ALT_HOLD — control_pkg/infrastructure/att_convert.py) — и на земле: арм НОДОЙ по
    жесту пилота (газ в пол + курс вправо 2 с) и её готовности (телеметрия, EKF, параметры
    углов, кадры камеры — control_pkg/application/node_arm.py), взлёт газом пилота, посадка,
    дизарм нодой по жесту; полётнику арм со стиков запрещён (ARMING_RUDDER 0 — в симе ставит
    sitl_lv_profile.py по BS_ATT_OUT);
  * LOITER/LAND/RTL/SMART_RTL/возврат GUIDED — штатные режимы полётника, нода в них молчит.
Маршрутизация режима — control_pkg/application/att_mode.py: план заявляет «держать ALT_HOLD»,
а это GUIDED_NOGPS при SF вверх (всегда, и на земле) и ALT_HOLD «только пилот» при SF не вверх.

ЧТО МЕНЯЕТ. Стек = cmd/bl бит в бит; между сторонами один ключ BS_ATT_OUT
(mission/replay → mission/att_out_replay). Маршрут — реплей, по умолчанию vins_init.json.

КАК ЛЕТАТЬ.
  bash cmd/att_out/att_out.sh rc     # override, как всегда
  bash cmd/att_out/att_out.sh att    # углы
  BS_REPLAY_SCENARIO=/lab/joystick/scenarios/vinshold_ring.json bash cmd/att_out/att_out.sh att

ЧЕМ СУДИТЬ. В логе ноды «ВЫХОД В УГЛАХ» и «углы: параметры FCU прочитаны»; в mavros.log НЕТ
«ignore_thrust» (без thrust_scaling MAVROS отбрасывает все углы — см. результат); переход
ALT_HOLD → GUIDED_NOGPS после взлёта, обратно после дизарма; путь плеча, наклон, разгон и
остановка против стороны rc (истина Gazebo); в dataflash ATT.DesRoll/DesPitch не нулевые.

РЕЗУЛЬТАТ (2026-09-30).
1. attout_att_20260930_153618 — ПРОВАЛ канала: MAVROS отбрасывал каждый SET_ATTITUDE_TARGET
   («Recieved thrust, but ignore_thrust is true»): mavros_node шёл без apm_config.yaml,
   setpoint_raw.thrust_scaling = nan. Полётник в GUIDED_NOGPS держал DesRoll 0 (как при обрыве
   потока), демпфер упирался в +150 PWM впустую, ветер унёс борт за геозабор 40 м. Исправлено:
   docker/sim/scripts/mavros_params.yaml (thrust_scaling 1.0), nav_up.sh --params-file.
2. attout_att_20260930_154029 против vins_init_rcb1_20260930_151317 (override, с мостом):
   плечо 15 с вперёд 16.5 / 15.9 м, инерция 1.4 / 1.4 м, скорость 1.30 / 1.28 м/с, наклон
   4.2 / 4.3°, разгон 2.6 / 2.3 с, остановка 2.2 / 2.1 с, высота 3.84–4.09 / 3.99–4.12 м;
   висение 0.62 / 0.37 м/с — в пределах разброса дня (0.37–0.85). Знаки верные; руддер-арм и
   дизарм по радио, посадка в GUIDED_NOGPS, override ни разу. Канал углов = override.
3. attout_att_20260930_155629 — АРМ НОДОЙ: весь полёт в GUIDED_NOGPS (ALT_HOLD ни разу): арм
   2.8 с «арм нодой по жесту пилота (готовность ОК)» → «FCU: Arming motors» через 0.02 с, взлёт
   газом пилота, плечо 17.2 м, посадка, «дизарм нодой» → «FCU: Disarming motors» через 0.02 с;
   ARMING_RUDDER 2 → 0 в eeprom.
Открыто: живой отказ арма неготовой нодой (проверен офлайн test_node_arm.py); аварийная
остановка моторов мимо ноды (RCx_OPTION 31) — канал решить с картой пульта борта; сторона rc
отдельной парой в одной серии.
