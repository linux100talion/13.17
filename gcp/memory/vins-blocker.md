---
name: vins-blocker
description: "Статус VINS пайплайна — всё запущено, но Initialization finish! зацикливается без перехода в NON_LINEAR"
metadata: 
  node_type: memory
  type: project
  originSessionId: e91c61b2-3d22-4338-af7d-8631b0040ec7
---

**Статус на 2026-06-22:** диагностирована и исправлена первопричина #1 — dt=1e-6 патч. Сборка идёт.

## Что работает

- Все контейнеры: p1317_simulator, p1317_mavlink_router, p1317_nav
- Gazebo + SITL (с sitl-extra.parm: SCHED_LOOP_RATE 100, ARMING_CHECK 0)
- ros_gz_bridge → bayerizer → /dev/rawbayer → camera_node (~16fps) → /image_mono
- feature_tracker ✓, vins_estimator ✓, MAVROS ✓
- Арминг работает без force-arm (ARMING_CHECK 0 в .parm)
- Взлёт по `ros2 service call /mavros/cmd/takeoff` — успешно
- `Initialization finish!` появляется при движении дрона

## Текущий блокер

`Initialization finish!` печатается каждые ~10 с, но VINS не переходит в NON_LINEAR:

```
[vins_estimator-8]: Initialization finish!
position: 0.01..., orientation: -0.10...
# ... и через 10с снова:
[vins_estimator-8]: Initialization finish!
```

`failureDetection()` триггерится сразу после init и сбрасывает solver_flag → INITIAL.
`/odometry` топик есть в `ros2 topic list`, но сообщений нет.

## Диагностика 2026-06-22

**Наблюдения в полёте:**
- `numerical unstable in preintegration` — нарастающие предупреждения
- orientation вышел за [-π, π]: `5.83 -7.85 -3.72`
- `throw img` продолжает появляться во время полёта
- VINS падает обратно в инициализацию через ~10-15с

**Подтверждённая причина:** IMU патч `t = last_imu_t + 1e-6` создаёт цепочку N×1μs шагов (sim /clock ~155Hz < IMU 250Hz → несколько msg с одинаковым stamp). В preintegration: dt=1e-6, dt²=1e-12 ≈ near machine-epsilon → ковариационная матрица деградирует → VINS отвергает IMU измерения → failureDetection().

**Применённый фикс (2026-06-22):**
- `estimator_node.cpp` imu_callback: заменено `t = last_imu_t + 1e-6` на `return` (skip duplicate)
- `nav_up.sh` обновлён: новый паттерн skip + fallback-паттерн на старый 1e-6 для nav-rebuild
- Сборка запущена, результат будет виден после перезапуска

**Ожидаемый результат:** `numerical unstable in preintegration` должны исчезнуть; VINS должен удержать NON_LINEAR.

**Если после фикса всё равно не держит:**
- Проверить noise params в sim.yaml (acc_n, gyr_n, acc_w, gyr_w) — возможно слишком жёсткие
- Проверить MIN_PARALLAX (текстура mili_fortress на высоте ~10м)
- Отключить failure detection в estimator.cpp для изоляции проблемы

**Why:** Без стабильного /odometry нет входа для ray_tracer → нет навигации.

**How to apply:** После `make nav-rebuild` армировать и взлетать, смотреть sim_nav.log — должны появиться строки position: без `numerical unstable`.
