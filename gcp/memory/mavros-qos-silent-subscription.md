---
name: mavros-qos-silent-subscription
description: Топики MAVROS публикуются BEST_EFFORT — подписка с дефолтным RELIABLE не получает НИЧЕГО и молча
metadata:
  type: project
---

Все сенсорные топики MAVROS (`/mavros/imu/data`, `/mavros/global_position/rel_alt`,
`/mavros/local_position/pose`, …) публикуются **BEST_EFFORT**. `create_subscription(..., 10)`
даёт RELIABLE — пара **не согласуется**, и подписка не получает ни одного сообщения.
Отказ ПОЛНОСТЬЮ МОЛЧАЛИВЫЙ: ни ошибки, ни варна, топик в `ros2 topic list` есть,
`ros2 topic echo` (сам best-effort) работает. Правильно — `qos_profile_sensor_data`.

**Why:** так `ray_tracer` жил С САМОГО НАЧАЛА ветки возврата домой: `att_yaw` был `None`
всегда → седьмое поле `/nn1/bridge` уходило как `-` (`brdn=--`, критерий зрелости по Δyaw
слеп), `latch_yaw()` на первом открытии моста не вызывался НИКОГДА (ветка «курса AHRS нет»
→ `anchor.reset()` → 2 с сырого VINS в EKF — фикс 5c1d562 деградировал обратно), засечка
NN1 выходила по `R_enu_body is None`. На спавне «на восток» Δyaw≈0 и всё выглядело
исправным; вскрыл прогон `SPAWN_POSE=diagonal` 164742 (дрейф 7.63 м против 0.87 м,
ошибка курса 8.73° против 0.82°). Класс ошибки — «фикс есть, но его ветка недостижима».

**How to apply:** любую подписку на MAVROS писать с `qos_profile_sensor_data`. Проверка
живьём — `ros2 topic info -v <топик>`: сравнить `Reliability` у PUBLISHER и SUBSCRIPTION
(скрипт-обход всех `/mavros/*` есть в разборе `cmd/yaw_vins/README.txt`). Новый вход,
который может молчать, снабжать СТОРОЖЕМ (в `ray_tracer` — таймер 30 с, ошибка в лог):
молчаливый отказ обязан становиться громким. Связано: [[fcu-telemetry-streams-race]],
[[ekf-frame-adopt-or-reset]], [[vins-frame-yaw-anchor]].
