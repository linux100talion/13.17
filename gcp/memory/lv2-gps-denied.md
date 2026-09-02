---
name: lv2-gps-denied
description: "Профиль LV=2 «GPS отсутствует с бута» ДОКАЗАН (2026-08-24): LOITER-на-VINS без секунды GPS, старт→LOITER 72 с; уроки origin/WMM и ARMING_SKIPCHK"
metadata: 
  node_type: memory
  type: project
  originSessionId: 94d73fde-a611-4729-9add-e48af5a39a84
  modified: 2026-08-24T03:36:56.325Z
---

Профиль LV=2 в `freefly_lv.sh` — модель боевого борта БЕЗ приёмника GPS —
**доказан двумя полётами 2026-08-24** (реплей lv_flight2): арм за ~8 с от
старта ноды, init VINS на отрыве (0.2–1.2 с), READY на 600 odom, штатный
LOITER залатчился, чистый прогон `lv2_replay_20260824_041803` — FREEFLY_DONE
exit 0, bag 5.2G. **Старт→LOITER 72 с против 96–116 у LV=1** — вся экономия
из исчезнувшего GPS-прогрева EKF (8 с вместо 43). В полёте ни одного
EKF-failsafe: незрелую фазу VINS пережило якорение кадра в ray_tracer.

Механика: eeprom (`sitl_lv_profile.py 2`) глушит `SIM_GPS1_ENABLE=0` и ставит
extnav-пару (POSXY=6/VELXY=0) ДО бута; нода в режиме `gps_denied`
(BS_GPS_DENIED) шлёт **мост нулевой позы** (0,0,баро) в vision_pose с земли до
первой одометрии VINS (EK3 стартует aiding только на земле — LV4), потом топик
навсегда у ray_tracer; POSXY=6 переписывается по зрелости только чтобы
extnav_ready/HUD READY открылись на тех же 600 odom. Плюс BS_SET_ORIGIN=1 и
BS_ALT_SRC=baro (миссия) + BS_PERC_ALT_SRC=local (перцепция).
**Кампания «перцепция на баро» ЗАКРЫТА (2026-08-24):** перцепция в LV=2 на
global НЕ слепла (EKF с origin публикует global без GPS — conf~1.0, демпфер
жив, bag 053321), но зависимость от EKF-канала сняли: замер 4 каналов бок о
бок (bag 062957) доказал механизм улётов 2026-08-19 — сырой баро ломает IPM
межкадровой производной (25.6 см p95 = фантомные скорости), EMA чинит
производную, но лаг 0.35 с рушит масштаб/полосу на наборе; EKF local z —
гладко (7.6 см) И без лага (0.03 с), вертикаль переживает смерть VINS.
Новая ручка perc_alt_src (global|local|baro, отдельно от alt_src миссии),
LV=2 → local; A/B-прогон 063604 — полёт неотличим от global-бейзлайна.
Таблица замера — комментарий в RosPerception.

**Урок 1 — origin НЕ условный** (прогон 034433, 59 ГБ земли, удалён): EKF
строит из origin модель магнитного поля WMM и сверяет с магнитометром;
захардкоженный «Киев» при доме SITL CMAC дал «PreArm: Check mag field
(z diff:999>200)» — арм невозможен. Origin обязан быть примерной реальной
точкой старта (BS_ORIGIN_LAT/LON/ALT, дефолт = CMAC). На боевом борте — то же.

**Урок 2 — ARMING_CHECK не работал НИКОГДА**: в ArduPilot 4.8 параметр
переименован в `ARMING_SKIPCHK` с инверсией (маска «что пропускать», −1 = все
немандаторные); строка `ARMING_CHECK 0` в sitl-extra.parm молча игнорировалась,
все PreArm-чеки были включены — LV-прогоны армились лишь потому, что с GPS
чеки честно проходили. Починено: `.parm` + продублировано в eeprom (eeprom
сильнее --defaults). Та же ловушка, что SYSID_MYGCS→MAV_GCS_SYSID.

Не забыть: рендер видео из bag прерванного прогона — только ПОСЛЕ SIGINT
рекордеру (kill ноды его не останавливает — писал в перенесённый каталог ещё
15 мин, «database is locked»). Связано: [[lv-loiter-series]],
[[joystick-replay-series]], [[freefly-phase-stats]], [[run-video-discipline]].
