---
name: always-upload-video
description: Прогоны в симе всегда заливать видео (scene.mp4) на Google Drive — GDRIVE_UP=1 MP4=1
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3e4cb44f-d2bd-4acf-889e-3a7656f65ac8
  modified: 2026-07-26T23:12:14.934Z
---

Любой прогон симуляции через `capture_scene.sh` запускать с **`GDRIVE_UP=1 MP4=1`** —
видео (`scene.mp4`) должно писаться и заливаться на Google Drive ВСЕГДА.

**Почему:** пользователь работает с телефона и смотрит результат прогона по видео на
Drive; без ролика прогон для него «слепой».

**How to apply:** не использовать `SKIP_CAM=1`/`GDRIVE_UP=0`/`MP4=0` даже для
диагностических/ID-прогонов. Если для анализа нужны доп. топики (напр. `/image_mono`,
`/gz_imu/data_flu` для [[control-refactor-arch]] pitch_flow_check) — добавлять их через
`TOPICS_EXTRA`, а НЕ отключать запись камеры. Bag будет крупнее — это осознанная плата.
