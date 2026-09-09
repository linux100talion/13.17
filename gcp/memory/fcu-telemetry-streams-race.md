---
name: fcu-telemetry-streams-race
description: "«EKF не захватил позицию за 120 с» в LV=2 (прогоны 121511/122716, 2026-09-07) — не EKF, а MAVROS 200 с не получал потоков FCU (IMU/ATTITUDE/LOCAL_POSITION) при живом мосте позы; «nav: готово» печаталось до бута FCU, MAV1_* в eeprom стояли, первопричина на стороне FCU не установлена; три страховки (MAV1_* в eeprom, готово после потоков, сторож в ноде, tel= в HUD)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 90283c05-f1a8-4568-8d80-50e31402d9bc
  modified: 2026-09-07T10:13:32.845Z
---

Прогоны `lv2_joy_20260907_121511` и `_122716` (первые после рефакторинга профилей,
коммит 1fc5734): `ekf_warmup: EKF не захватил позицию за 120 с`. Разбор 2026-09-07:
профили ни при чём (мета — только добавленные ключи, все = дефолты ноды; код между
прогонами не менялся). Мост нулевой позы (`_pose_bridge`) отработал: FCU-тексты в
mavros.log на 5-й с после бута — «EKF3 IMU0 origin set», «is using external nav
data», «initial pos NED 0,0,0». Но MAVROS 200 с не получал НИ ОДНОГО потока (в bag
первый `/mavros/imu/data` на t=200.7 с, `local_position` тоже; вчера 231055 — IMU с
10 с, `ekf=1` на 14-й). ArduPilot шлёт RAW_IMU/ATTITUDE/LOCAL_POSITION_NED только по
`REQUEST_DATA_STREAM`/`SET_MESSAGE_INTERVAL` (SR0_* в eeprom нули); запрашивал фоновый
цикл `( … ) &` в `nav_up.sh`, а «nav: готово» печаталось сразу: 12:18:53 готово,
12:18:57 старт узла, 12:19:00 первый heartbeat FCU (SITL блокируется на «Waiting for
connection» до TCP роутера), 12:22:13 все потоки разом, 12:22:19 цикл отчитался
«IMU идёт». Почему первые ~5 итераций не дали потоков — не восстановить: цикл писал
в mavros.log через `>>`, а mavros_node держит файл через `>` и затирал строки.

**Попутно:** SITL умер в 12:43:01 уже после полёта — «Floating point exception -
aborting» (роутер: Connection closed → refused; Gazebo: «ArduPilot controller has
reset»); dumpstack не собрался, причина не разобрана. 23 строки «Loaded defaults from»
в консоли SITL — штатно (reload_defaults_file при регистрации динамических поддеревьев
параметров на буте), не рестарты.

**Why:** `local_position` — поток телеметрии, а не факт позиции: его нет и без
позиции EKF, и когда MAVROS вообще молчит. Одно и то же «ekf=0» маскировало разные
причины, а порядок старта (готово → узел → бут FCU → запрос потоков) был гонкой.

**How to apply:** починено 2026-09-07 тремя страховками (проверить первым же
полётом): (1) `sitl_lv_profile.py` пишет `MAV1_RAW_SENS 200 / POSITION 25 / EXTRA1 50 /
EXT_STAT 2 / EXTRA2 5` в eeprom (4.8: SRn_→MAVn_, с единицы; `SR0_*` прошивка не знает) —
они УЖЕ стояли (persist_streamrates сохраняет REQUEST_DATA_STREAM), т.е. eeprom не был
причиной 122716 — первопричина на стороне FCU (потоки голодали при живых heartbeat/params?)
не установлена; (2)
`nav_up.sh` ждёт потоки В ФОРГРАУНДЕ (лог `output/stream_rate.log`), «nav: готово»
только после них, иначе «nav: ОШИБКА» и `make wait` падает (секвенсор не летит);
(3) узел — сторож `_telemetry_watch` (`TEL_STREAMS` в bootstrap_node): пока молчит
`/mavros/imu/data` > 2 с — раз в 3 с `SET_MESSAGE_INTERVAL` на всё, что читает
нода; `WaitEkfPos` пишет «телеметрия FCU молчит», а не «EKF не захватил»; в статусе
`tel=`, в HUD красный `FCU TELEMETRY SILENT`. Диагноз «EKF не захватил» теперь
верить только при `tel=1`. Связано: [[lv2-gps-denied]], [[openhd-debug-hud]],
[[freefly-phase-stats]].
