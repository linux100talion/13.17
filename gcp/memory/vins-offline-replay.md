---
name: vins-offline-replay
description: "Офлайн-стенд VINS — реплей бэга (/feature+IMU) в изолированном ROS-домене, итерации по конфигу без полётов"
metadata: 
  node_type: memory
  type: project
  originSessionId: 6125abe1-a7e9-4d25-9a1a-fa9d7a6490c9
  modified: 2026-08-19T12:34:49.124Z
---

`src/lab/vins_offline_replay.sh <имя> [BAG] [key value ...]` — гоняет ТОЛЬКО
vins_estimator по записанному бэгу в `ROS_DOMAIN_ID=42` внутри контейнера
p1317_nav (живой стек не видит реплей — дисциплина прогонов не нарушается).
Один прогон ≈ длительность бэга / RATE (env, default 2).

**Why:** правка конфига VINS → полный полёт = 10-15 мин; реплей = 30 с на тех же
данных, детерминированно. Именно так найден корень «солвер-пустышки»
([[vins-solver-fix]]): три вариации шумов дали идентичный развал → параметры
не влияют → оптимизатор мёртв.

**How to apply:** бэг должен содержать `/feature` + `/gz_imu/data_flu`
(+ `/model/iris_cam/odometry` для сравнения с истиной) — снимать полёт с
`TOPICS_EXTRA="/odometry /feature /model/iris_cam/odometry /gz_imu/data_flu"`.
Выход VINS форка — `/odometry` (НЕ /vins_estimator/odometry). Результаты:
`/tmp/offline/est_<имя>.log`, `dump_<имя>.csv` (vins/truth построчно).
База конфига — `/tmp/sim_960x540.yaml` (генерится живым launch; после правки
sim.yaml пересоздать вручную или перезапустить стек). Эталонные бэги:
`docker/sim/output/VINSBASE_bag` (грязные штампы), `VINSFIX_bag` (честные
штампы), `VINSEXT_bag` (штампы+экстринсики — основной для реплея).
