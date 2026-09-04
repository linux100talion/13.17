---
name: ekf3-drag-wind
description: "Ветер EKF3 drag-фьюжн НАБЛЮДАЕТСЯ на VINS-external-nav (GPS off); блокером был стрим-рейт WIND msg 168, не физика. BS_EKF_DRAG→BCOEF, /mavros/wind_estimation. Источник стрелки ветра HUD в LOITER"
metadata: 
  node_type: memory
  type: project
  originSessionId: 790e40a9-7511-4c26-b8ed-594eb205c6fa
  modified: 2026-09-04T18:38:40.545Z
---

Стрелка ветра HUD (2026-09-04, ветка nn2_c3_laptop_wind, коммит e45b5e2) —
источник ПО ЯРУСУ: ярусы 0/1 наш ТРИМ стабилизатора (контур держит вживую),
ярус 2 LOITER — ветер **EKF3 drag-фьюжна** (стек пуст, трим DpVins замерзает и
порыв не ловит — прогон 191327: застыл 2.7 м/с сквозь порывы 11, борт кренился
9.5°). См. [[dpvins-wind-trim-learn]], [[sim-wind-gusts]], src/control/windspeed.md.

**Проба Ф0 — ГЛАВНАЯ находка:** EKF3 drag-фьюжн ОЦЕНИВАЕТ ветер на чистом
VINS-external-nav (SIM_GPS1_ENABLE=0, EK3_SRC1_POSXY=6, VELXY=0) — сошёлся к
ИСТИНЕ 10.0 м/с @98° за ~30 с в висении. GPS для drag-ветра НЕ нужен.

**Why (грабли):** блокером была НЕ наблюдаемость, а СТРИМ-РЕЙТ — WIND (msg 168)
не в стриме роутер-канала SITL (ровно как ATTITUDE/IMU, урок sitl-extra.parm
«Стрим IMU»). Топик /mavros/wind_estimation молчал, пока не запросили msg 168.
WIND_COV (231) ArduPilot НЕ шлёт вовсе — только WIND (168), без ковариации
(mavros ставит −1).

**How to apply:**
- `BS_EKF_DRAG` (дефолт 32 кг/м², 0=выкл) → EK3_DRAG_BCOEF_X/Y в eeprom LV2
  (sitl_lv_profile.py). MCOEF=0. ⚠️ ВКЛЮЧАЕТ drag-фьюжн EKF3 ПО УМОЛЧАНИЮ для
  LV2 — меняет EKF (добавляет drag-измерение); flight-failsafe в валидации не
  было, но для УПРАВЛЕНИЯ доверять после A/B по удержанию (пока только HUD).
- WIND(168) в стрим — nav_up.sh (EXTRA2/id11 + set_message_interval 168).
- /mavros/wind_estimation = TwistWithCovarianceStamped (не TwistStamped!),
  BEST_EFFORT QoS; скорость воздушной массы в мире ENU (куда дует).

**ДОКАЗАНО ПОЛЁТОМ пилота (2026-09-04):** ветер показывает во ВСЕХ режимах;
наш ТРИМ на DpHold (ярус 0) — «точнее всего и быстрее всего» (подтверждает
выбор оставить трим на 0/1, а EKF лишь в LOITER). EKF-ветер точен как readout
ВИСЕННОГО ветра (медленный — главный юзкейс). На резком движении/разносе VINS
дёргается (в валидации завышал до 16.5 при разносе VINS — EKF-скорость мусор →
ветер мусор; drag-ветер ровно настолько хорош, насколько EKF-скорость).
Сим-драг сублинеен по v (speed.md) → вне точки калибровки поплывёт; боевой
борт ближе к v² (там BCOEF/MCOEF — лётная калибровка).

**Открыто (Ф1, если понадобится точность в движении):** проба VELXY=6
(vision-скорость даёт EKF честную скорость на манёврах) — но память
предупреждает про фантомы vision_vel → EKF failsafe, осторожная проба.
