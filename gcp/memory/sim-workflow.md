---
name: sim-workflow
description: "Рабочий процесс симуляции — entrypoint скрипты, make-цели, что теряется при fresh-start"
metadata: 
  node_type: memory
  type: project
  originSessionId: e91c61b2-3d22-4338-af7d-8631b0040ec7
---

## Типичная итерация

```bash
cd docker/sim/
make restart-all   # docker compose stop → start (быстро, ephemeral state жив)
make wait          # ждёт "nav: готово" в логах (до 5 мин)
make status        # ✓/✗ по каждому процессу + последние ошибки из логов
make logs          # непрерывный хвост output/*.log
```

## Полный сброс (теряет ephemeral state)

```bash
make fresh-start   # docker compose down → up (пересоздаёт контейнеры)
make wait          # colcon build ~5-7 мин при первом запуске
make status
```

## Что выживает при restart-all, теряется при fresh-start

**Выживает** (bind mounts → данные на хосте):
- `src/`, `worlds/`, `output/`, `scripts/`, `config/sitl-extra.parm`

**Теряется** (внутри контейнера):
- `vins_oss/` — auto-клонируется + патчится в nav_up.sh
- `build/install/` colcon — пересобирается (nav_up.sh проверяет `install/setup.bash`)
- `ros-humble-image-transport` — нужно в Dockerfile (TODO)
- `numpy<2` — теперь auto-устанавливается в nav_up.sh

## Патчи при клонировании vins_oss (nav_up.sh)

1. `rclcpp::Duration(0)` → `rclcpp::Duration(0, 0)` во ВСЕХ файлах vins_oss (find+xargs)
2. IMU QoS: `.best_effort()` на строке 357 estimator_node.cpp
3. IMU монотонный патч в imu_callback: вместо drop — t = last_imu_t + 1e-6 + const_cast stamp

## Армирование

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "GUIDED"}'
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL '{altitude: 10.0, ...}'
```

ARMING_CHECK 0 в `config/sitl-extra.parm` → force-arm не нужен.

## make status — что показывает

```
── simulator ──────
  ✓ gz sim (Gazebo)
  ✓ SITL (arducopter)
  ✓ ros_gz_bridge
── nav ────────────
  ✓ bayerizer
  ✓ camera_node
  ✓ feature_tracker
  ✓ vins_estimator
  ✓ MAVROS
  -- последние ошибки sim_nav.log --
  -- mavros --
```

## Известные падения (некритично для VINS)

- `nn1_anchor`, `nn2_scene`, `openhd_streamer` — NumPy 2.x ABI (nav_up.sh теперь ставит numpy<2)
- `relocalizer`, `ray_tracer` — зависят от nn1/nn2, заглушки
- `ar_demo` — вспомогательный пакет vins_oss, не нужен для VINS core
