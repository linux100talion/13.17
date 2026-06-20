---
name: gce-gpu-plan
description: Soon spinning up a real Linux + NVIDIA GPU on Google Compute Engine for torch/CUDA/ROS runs
metadata: 
  node_type: memory
  type: project
  originSessionId: 28871731-7c51-479b-a8d3-486a20f78a67
---

Пользователь планирует **скоро** поднять нормальный Linux с видеокартой NVIDIA на
**Google Compute Engine** (GCE). Текущая рабочая среда — Termux ARM, где НЕ ставятся
torch/faiss/cv2/rosbag2 (только numpy + py_compile), поэтому вся реализация (c)-
основного навигации проверяется numpy-зеркалами, а не на реальных сетях/bag'ах.

**Что переедет на GCE-GPU-машину** (hardware-gated хвосты, подробности в репе:
`src/nav/tools/nn2_route/c3_TODO.txt`, раздел B):
- извлечение φ (DINOv2) из реальных bag'ов 100 облётов;
- обучение топографа (`train_topograph.py`) и голов C (`fit_route_heads`) → `train_route_coords.pt`;
- загрузка `RouteHeads.load(...)` + прогон `route_heads_node` на живом `/image_color`;
- слияние φ в `nn2_scene` (один прогон DINOv2 на метрику+головы);
- вшивка нод в setup.py/launch;
- при необходимости — внешний солвер pose-graph (ШАГ 6, GTSAM/g2o/ceres).

**Why:** объём работ (c)-основного спланирован так, что чистая логика готова
и проверена сейчас, а тяжёлые прогоны откладываются до появления GPU.
**How to apply:** когда пользователь заведёт GCE-инстанс, идти по чек-листу
`src/nav/tools/nn2_route/c3_gce_setup.txt` (инстанс/зависимости/порядок прогона
B1–B6/что перенести); не предлагать запускать torch/ROS в текущей среде.

Готовы numpy-зеркала ШАГ 1–6 (c)-основного на ветке `nn2_c3`: register_flights,
build_global_dataset, cross_flight_correspond, c3_route_pipeline, c3_qc_report,
route_heads(+node), c3_pose_graph. Ждёт GPU только B-раздел c3_TODO.txt.

**Статус на 2026-06-20:** заведён GCE-инстанс `dev-workspace-1317`
(`europe-west4-a`, проект `drone-13-17-workspace-2026`) — но пока **CPU-only**
(`n1-standard-8`, без `--accelerator`). Причина: (1) T4 в дефиците
(`ZONE_RESOURCE_POOL_EXHAUSTED` ловили в us-central1-a и во всём europe-west4),
(2) для проверки кода GPU не нужен — `nvcc` собирает CUDA без видеокарты.
Это **build-box для проверки кода** (compile/colcon/линт/сборка docker-образов),
а НЕ замена GPU-машины для B-раздела: тяжёлые прогоны torch/DINOv2/обучение
по-прежнему ждут реального GPU. Создаётся тогглом `GPU=0 ./01_create_workspace.sh`;
`GPU=1` (дефолт) поднимет с T4, когда ёмкость будет. Управление — скрипты `gcp/`
(01..07). Правило репы: работаем только существующими скриптами, нет — пишем,
коммитим, пушим, потом юзаем. Граница build-box (что можно/нельзя проверять,
анти-лок-ин по арх GPU/CUDA/OpenGL) — раздел «Build-box» в `gcp/CLAUDE.md`.

Связано: [[c3-primary-pipeline]] (если заведётся), концепт-ветки nn2_c3 /
nn2-fusion-notes / nn2-dataset-registration.
