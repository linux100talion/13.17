═══════════════════════════════════════════════════════════════════════════
  docker/sim/scripts/ — entrypoint- и host-скрипты симуляционного стека
═══════════════════════════════════════════════════════════════════════════

СКРИПТЫ
  host_setup.sh   — подготовка ХОСТА перед запуском (root/sudo). Запуск:
                    make host-setup. Не в контейнере — работает с ядром хоста.
  sim_up.sh       — entrypoint контейнера simulator: Gazebo + SITL + ros_gz_bridge.
                    Монтируется как /scripts/sim_up.sh, стартует автоматически
                    (command: в docker-compose.yml). Также: make sim.
  nav_up.sh       — entrypoint контейнера nav: colcon build + bayerizer + VINS +
                    MAVROS (+ камера-нода через sim_nav.launch.py). make nav.
  capture_frames.sh — снять N кадров из мира Gazebo в worlds/preview/*.png.

═══════════════════════════════════════════════════════════════════════════
  HOST_SETUP.SH — что и зачем (важно для свежего/облачного бокса)
═══════════════════════════════════════════════════════════════════════════

Делает три вещи, все идемпотентно:

1. X11 для GUI Gazebo
   xhost +local:root — только если есть xhost и задан DISPLAY. На headless-
   боксе (GCE без дисплея) пропускается: gz sim идёт --headless-rendering (EGL),
   дисплей не нужен.

2. videodev (ядро V4L2) — ЗАВИСИМОСТЬ v4l2loopback
   v4l2loopback depends: videodev. На минимальных/облачных образах (GCE и др.)
   videodev НЕ предустановлен — он в пакете linux-modules-extra-$(uname -r).
   Без него modprobe v4l2loopback падает:
       v4l2loopback: Unknown symbol v4l2_event_subscribe (err -2)
       v4l2loopback: Unknown symbol video_device_alloc (err -2)  ...
   Скрипт ставит linux-modules-extra-<kernel>, если modinfo videodev пуст,
   и грузит videodev перед v4l2loopback.

3. v4l2loopback — виртуальная камера /dev/rawbayer
   modprobe v4l2loopback devices=1 video_nr=9 card_label=rawbayer
            exclusive_caps=0
   Симлинк /dev/video9 -> /dev/rawbayer (docker-compose пробрасывает именно это
   фиксированное имя в контейнер nav через devices:). chmod 666 — чтобы нода в
   контейнере имела доступ.

   ⚠️ ПАРАМЕТРЫ width=/height= НЕ ПЕРЕДАЁМ. В v4l2loopback >= 0.13 (на боксе —
      0.15.0) таких параметров НЕТ (есть только max_width/max_height, дефолт
      8192). Передача width=/height= ломает загрузку ("unknown parameter").
      Реальное разрешение 1280×720 задаёт camera_node через VIDIOC_S_FMT —
      модулю его знать не нужно.

   ⚠️ Гонка с udev: udev переустанавливает права на video-устройствах
      асинхронно ПОСЛЕ modprobe. Поэтому перед chmod зовём udevadm settle —
      иначе 666 может не «прилипнуть» (вернётся к 660 root:video).

═══════════════════════════════════════════════════════════════════════════
  ПОРЯДОК НА СВЕЖЕМ БОКСЕ
═══════════════════════════════════════════════════════════════════════════

  1. Docker + compose-плагин (см. gcp/ или официальный apt-репозиторий Docker).
  2. make host-setup            # этот скрипт: videodev + v4l2loopback
  3. GPU-режим:  make build && make up
     CPU-режим:  make CPU=1 build && make CPU=1 up   (GPU-less, ветка nn2_c3_cpu)
  4. make wait && make logs

  host_setup.sh от режима (GPU/CPU) НЕ зависит — v4l2loopback нужен в обоих.
═══════════════════════════════════════════════════════════════════════════
