---
name: sf-master-ladder
description: "Схема «SF-мастер» селектора пульта — SF (CH7) мастер сырых стиков, SC (CH6) потолок лесенки демпфер/VinsHold/LOITER; BS_SF_MASTER=1 дефолт бокса через docker/sim/.env"
metadata: 
  node_type: memory
  type: project
  originSessionId: 11cb5749-21ae-4d24-aff8-4fdb04aa27c6
  modified: 2026-08-26T14:20:01.242Z
---

Схема «SF-мастер» (2026-08-24, ветка nn2_c3_laptop_vins) — ✅ ДОКАЗАНА полётами:
lv1_joy_20260824_224118/224925 (BS_SF_MASTER=1) + прогон со спавном на курсе
−169° после yaw-якоря ([[vins-frame-yaw-anchor]]) — LOITER пришёл автоматически
по лесенке и держал.

- SF (CH7, axes[6]) центр/вниз/оси нет = СЫРЫЕ СТИКИ (MANUAL-seize) при любом SC; SF вверх = стабилизация, SC (CH6) задаёт ПОТОЛОК лесенки: вверх=демпфер, центр=+VinsHold по готовности, вниз=+штатный LOITER по зрелости. Борт всегда на лучшей ДОСТУПНОЙ ступени (позиция loiter до зрелости ≠ голый ALT_HOLD — лечит дрейф прогона 2026-08-20).
- Ярус LOITER пустит стек только ПОСЛЕ фактического латча режима FCU (урок LoiterHold); вниз с VinsHold — гистерезис 3×fresh; выход из MANUAL — пересев опор от текущей точки.
- Ключевые места: `ros_pilot.joy_master` (чистое ядро), `Freefly._ladder_*` (step.py), `VinsHandover.vins_stabs`, поле `DroneState.pilot_level`, флаг `config.sf_master` (BS_SF_MASTER / --sf-master).
- Тесты: `src/mission/test/test_freefly_ladder.py` + секция joy_master в `test_pilot_strategies.py`.

**Why:** сырые стики как крайнее положение SC заставляли проскакивать через центр (loiter); выделенный мастер = перехват одним щелчком + освободил позицию под «только демпфер» (недоступный раньше A/B-режим).

**How to apply:** с 2026-08-26 `BS_SF_MASTER=1` (вместе с `LV=2`) — дефолт РЕПО:
эталон `docker/sim/env.default` (в git) сеется в gitignored `.env` автоматически
(make ensure-env / freefly_lv), freefly_lv.sh читает .env как слой дефолтов (env
снаружи сильнее), голый `bash src/lab/freefly_lv.sh` = LV=2+SF-мастер даже на
свежем клоне. Существующий .env не перетирается (правки дефолта переносить руками). ⚠️ Реплей СТАРЫХ
сценариев требует явного `BS_SF_MASTER=0` ([[joystick-replay-series]]): старые
записи/сценарии несут 6 осей и их CH6=+1 значил MANUAL — под схемой они летят
целиком на сырых стиках. Новые записи несут 7 осей (joy_timeline пишет axes[:7]), сценарии — ключ "sf" (0/1). На пульте нужен микс SF→CH7 в EdgeTX; наземная сверка — joy_check.py (столбец SF:). Ярусу LOITER нужен BS_FF_LOITER=1 (профили LV=1/2 ставят сами). Control-шаги (не-freefly) лесенку не ведут: SF вверх = наш стек+handover.
