---
name: ardupilot-48-param-renames
description: "ArduPilot 4.8 переименовал параметры в SI-имена (LOIT_SPEED→LOIT_SPEED_MS, LAND_SPEED→LAND_SPD_MS, ARMING_CHECK→ARMING_SKIPCHK) — старые строки в .parm МОЛЧА игнорируются; как проверять"
metadata: 
  node_type: memory
  type: project
  originSessionId: 21c1a194-d0f7-45b5-b758-827f7ea9a513
  modified: 2026-09-01T15:41:14.179Z
---

Прошивка SITL в симе (сборка ~4.8-dev) переименовала часть параметров в
SI-единицы, старые имена в `--defaults`/parm-файлах **молча игнорируются**
(ошибки нет нигде). Пойманные случаи:

- `ARMING_CHECK 0` → `ARMING_SKIPCHK -1` (найдено раньше, [[lv2-gps-denied]]);
- `LOIT_SPEED 500` → `LOIT_SPEED_MS` (м/с!) — фактически летали на дефолте
  12.5 м/с, пилот в LOITER разгонялся до 8.8 м/с (разбор eagle/4 2026-09-01,
  [[loiter-yaw-dive]]); соседи тоже SI: `LOIT_ACC_MAX_M`, `LOIT_BRK_ACC_M`;
- `LAND_SPEED 15` → `LAND_SPD_MS` (м/с) — фактический спуск LAND 0.5 м/с
  (дефолт), не 0.15, ломает расчёт мягкости [[sa-soft-land]].

`PSC_ANGLE_MAX`, `PILOT_Y_RATE`, `SCHED_LOOP_RATE` и прочие из
sitl-extra.parm не переименованы — применяются.

**Why:** каждая такая строка — «фикс», который в полёте не работает, и это
невидимо до замера (мы дважды строили выводы на неприменённом параметре).

**How to apply:** после правки sitl-extra.parm сверять ИМЕНА против живой
прошивки: `ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull
'{force_pull: true}'`, затем `ros2 param list /mavros/param | grep <имя>` и
`ros2 param get /mavros/param <имя>` (в p1317_nav; ParamGet-сервиса нет —
параметры FCU видны как ROS-параметры узла /mavros/param). Новые значения в
единицах СИ (м/с, м), не см/с.
