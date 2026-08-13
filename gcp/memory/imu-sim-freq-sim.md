---
name: imu-sim-freq-sim
description: "IMU sim-частота в SITL-симуляции ≈50Гц raw / 30Гц filtered, потолок телеметрии; доки про ≥80 устарели"
metadata: 
  node_type: memory
  type: project
  originSessionId: 23847c91-aec3-4a50-9af7-539d49f00c31
---

Ветка `nn2_c3_vins_althold_2`, sim-стек `docker/sim/`. Реально достигнутая
IMU sim-частота (из bag, по header.stamp):
- `/mavros/imu/data_raw` (вход VINS) ≈ **50 sim-Гц**
- `/mavros/imu/data` (ATTITUDE/EXTRA1) ≈ **30 sim-Гц**

Запрос в `nav_up.sh` через `set_stream_rate`: RAW_SENSORS@200 → реально ~50
(SITL режет; потолок телеметрии, физпотолок `SCHED_LOOP_RATE=100`), EXTRA1@50 → ~30.

**Доки/код устарели — ПОПРАВИТЬ:** корневой `CLAUDE.md` пишет «цель ≥80 sim-Гц»,
а реальный гейт приёмки в `nav_up.sh` — всего `≥15`. Инлайн-коммент «~24–34 Гц»
тоже неточен (сейчас 50).

**Риск для VINS:** камера ≈30 sim-Гц, IMU 50 → ratio ≈1.6× (на боевом борту IMU
250 Гц). <2 IMU-сэмпла на интервал между кадрами → грубая препинтеграция →
вероятный сопричинный фактор несходимости VINS. Куда копать: на какой частоте
картинка реально входит в VINS (`sim.yaml`/feature_tracker) — если режется до
~10 Гц, ratio 5× лучше. Связано с [[bootstrap-excite-tuning]]. Детали —
`docker/sim/todo2.txt`.
