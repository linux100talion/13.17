---
name: gce-gpu-plan
description: "GCE build-box dev-workspace-1317 — статус инфраструктуры, Docker-образы собраны, ждём GPU"
metadata: 
  node_type: memory
  type: project
  originSessionId: e91c61b2-3d22-4338-af7d-8631b0040ec7
---

## Инстанс `dev-workspace-1317` (europe-west4-a)

CPU-only build-box (n1-standard-8, 30 GB RAM, 120 GB диск). Поднят после того,
как первая попытка апгрейда до GPU упала на глобальной квоте `GPUS_ALL_REGIONS=0`.

**Текущий статус (2026-06-20):**
- Docker 29.6.0 установлен и запущен
- SSH-ключ (ed25519) настроен для GitHub; remote переключён на SSH
- Оба симуляционных образа **собраны и готовы**:
  - `sim-simulator:latest` — 7.6 GB (Gazebo Harmonic + ArduPilot SITL + ardupilot_gazebo)
  - `sim-nav:latest` — 27.2 GB (CUDA 12.2 + OpenCV+CUDA + ROS2 Humble + MAVROS + PyTorch + FAISS)
- Dockerfile-фиксы закоммичены: `pexpect+future` (waf), `gstreamer-dev` (ardupilot_gazebo cmake)
- Headless EGL рендеринг добавлен в simulator: `gz sim --headless-rendering`

**Запустить пайплайн и собирать bag'и нельзя без GPU** — Gazebo камера рендерится
через EGL (без GPU чёрные кадры), camera_node использует CUDA-дебайер,
`runtime: nvidia` в docker-compose требует nvidia-container-toolkit.

## Путь к GPU

**Блокер решён:** глобальная квота `GPUS_ALL_REGIONS` была 0 → поднята через
`09_quota_manager.sh ensure`. Региональные T4-квоты (`NVIDIA_T4_GPUS`,
`PREEMPTIBLE_NVIDIA_T4_GPUS`) есть.

**Апгрейд:** `./08_add_gpu.sh` (запускать с ЛОКАЛЬНОЙ машины, не с инстанса!):
стоп → `delete --keep-disks=boot` → `create` с тем же диском + T4.
Boot-диск с образами переживает пересоздание.
Тогглы: `SPOT=1` (дешевле), `MACHINE_TYPE=n1-standard-4` (реже EXHAUSTED).

После апгрейда внутри инстанса:
1. `sudo apt-get install -y nvidia-driver-535 && sudo reboot`
2. Установить nvidia-container-toolkit (команды печатает `08_add_gpu.sh`)
3. `docker compose up` → пайплайн готов

## Скрипты gcp/

01 создание · 02 питание · 03 resize · 04 статус · 05 удаление ·
06 размеры · 07 список · **08 add-gpu** · **09 quota-manager**

Конвенция: новое действие = новый скрипт в репе, коммит, пуш, потом использование.
Детали — `gcp/CLAUDE.md`.

## Что ждёт GPU (c3_TODO.txt раздел B)

B1 извлечение φ из bag'ов (DINOv2+rosbag2) · B2 обучение топографа ·
B3 обучение голов C · B4 загрузка в рантайм · B5 слияние φ в nn2_scene ·
B6 вшивка нод в setup.py/launch.

Раздел A (два провода до FCU) — код писать можно уже сейчас (не требует GPU).
