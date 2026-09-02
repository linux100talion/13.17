---
name: joystick-base-dataset
description: Базовый датасет joystick-прогонов docker/sim/output/joystick/base/ — семантика подкаталогов 0/1/2/3
metadata: 
  node_type: memory
  type: project
  originSessionId: 47154f72-2494-4536-9b19-2c3ac4908ee1
  modified: 2026-08-25T10:17:55.324Z
---

`docker/sim/output/joystick/base/` — базовый датасет joystick-прогонов freefly
(записан 2026-08-25, 24 прогона, ~217 ГБ). Семантика подкаталогов:

- `0` — без стабилизации (сырые стики), 5 прогонов
- `1` — наш демпфер, 5 прогонов
- `2` — VinsHold, 6 прогонов
- `3` — LOITER, 8 прогонов

Каждый прогон `lv1_joy_*/`: `bag/`, `joy.log`, `cmd_line_log.txt`, `scene.mp4`,
`scene_hud.mp4`, снимок `.env`.

**Важно:** `.env` у всех четырёх категорий идентичны (кроме комментария с именем
прогона) — категории различаются действиями пилота в полёте (позиция лесенки
SC/CH6 при SF-мастере, см. [[sf-master-ladder]]), а не конфигурацией. Определять
категорию прогона по `.env` нельзя — только по каталогу или CH6 в bag/joy.log.

Связано: [[joystick-replay-series]].
