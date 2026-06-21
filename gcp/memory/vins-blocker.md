---
name: vins-blocker
description: "Статус VINS пайплайна — всё запущено, но Initialization finish! зацикливается без перехода в NON_LINEAR"
metadata: 
  node_type: memory
  type: project
  originSessionId: e91c61b2-3d22-4338-af7d-8631b0040ec7
---

**Статус на 2026-06-21 (вечер):** весь критичный стек запущен, но VINS не удерживает трекинг.

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

**Возможные причины:**
1. IMU монотонный патч создаёт dt=1e-6 между дублированными timestamp → IMU preintegration даёт residuals несовместимые с визуальным решением → failureDetection() срабатывает
2. Недостаточная визуальная текстура в mili_fortress на высоте ~10м → плохое SfM-решение
3. Параметры шума IMU в sim.yaml (заниженные) создают слишком жёсткие ограничения

**Что пробовали:** полёт по квадрату 3 круга (5 сек каждая сторона), полёт кругами 120с с угловой скоростью 0.25 рад/с.

**Why:** Без стабильного /odometry нет входа для ray_tracer → нет поправки дрейфа VINS → нет навигации.

**How to apply:** Следующий шаг — диагностировать failureDetection(). Варианты:
- Отключить failure detection в estimator.cpp (закомментировать clearState())
- Проверить IMU noise params в sim.yaml (acc_n, gyr_n, acc_w, gyr_w)
- Увеличить параметр MIN_PARALLAX в sim.yaml
- Посмотреть что за residuals возникают после init (добавить отладочный вывод)
