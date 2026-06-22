---
name: vins-init
description: "Статус отладки VINS инициализации: check2 исправлен, extrinsic исправлен, но scale=0.02 и failure detection каждые ~30с"
metadata: 
  node_type: memory
  type: project
  originSessionId: 8d76a2f4-dd5b-4b31-9d8f-7e6500548bcb
---

## Что было исправлено (2026-06-22)

### 1. check2 / numerical unstable — РЕШЕНО
- Причина: ковариационная матрица IMU preintegration теряла PD-свойство (tiny negative eigenvalue ~-1e-18) → LLT давала sqrt_info~1e25 → Ceres расходился.
- Фикс: в `/root/VINS-MONO-ROS2/vins_estimator/src/factor/imu_factor.h` добавлено `cov += 1e-10 * I` перед LLT.
- Дополнительно: skip-патч в `estimator_node.cpp` imu_callback: дубликаты IMU (одинаковый timestamp) → `return` вместо `t = last_imu_t + 1e-6`.
- Оба патча зафиксированы в форке ветка `1317_debug` (bind-mounted из `/root/VINS-MONO-ROS2`).

### 2. estimate_extrinsic — РЕШЕНО (конвенция форка перевёрнута)
- В этом форке VINS-MONO-ROS2: `0=fix, 1=optimize around initial, 2=no prior (calibrate from scratch)` — ПРОТИВОПОЛОЖНО стандарту VINS.
- Раньше стояло `estimate_extrinsic: 1` → оптимизировал онлайн → сходился к identity.
- Раньше `estimate_extrinsic: 2` → игнорировал yaml-матрицу полностью, ставил identity.
- Исправлено: `estimate_extrinsic: 0` (fixed) в `sim.yaml`.
- Проверка: лог показывает `fix extrinsic param` и печатает правильную матрицу.

### 3. estimate_td — ИСПРАВЛЕНО
- В симуляции оба источника (camera + IMU) штампуются sim-clock → временная задержка = 0, оценку отключили: `estimate_td: 0`, `td: 0.0`.

### 4. extrinsicRotation — ИСПРАВЛЕНО
Конвенция RIC: матрица поворота тело (FLU) → camera optical (Z вперёд, X вправо, Y вниз).
Камера в Gazebo смотрит вдоль +X_link, pitch=0.26 рад из SDF:
```
X_cam (право)  = -Y_link = [0, -1, 0]
Y_cam (вниз)   = -Z_link = [-0.257, 0, -0.966]
Z_cam (вперёд) = +X_link = [0.966, 0, -0.257]

RIC = [[0, -1, 0], [-0.257, 0, -0.966], [0.966, 0, -0.257]]
```
Теоретически: RIC * [0,0,-9.81] = [0, 9.48, 2.52] (гравитация → вниз в кадре ✓).

## Текущий блокер (2026-06-22): scale=0.02, failure detection каждые ~30с

### Симптом
```
visualInitialAlign: scale=0.0188  g=[9.502 0.071 -2.454]
Initialization finish!
# через ~30с:
failure detection!
system reboot!
```

### Проблема 1: gravity direction не совпадает с расчётным
- Ожидалось: g ≈ [0, 9.48, 2.52] (по расчёту RIC)
- Фактически: g ≈ [9.5, 0, -2.5] — гравитация указывает в +X_cam (право в кадре), а не +Y_cam (вниз).
- Это СИСТЕМАТИЧЕСКАЯ ошибка (воспроизводится каждый запуск), не случайная.
- Гипотеза: реальная конвенция Gazebo Harmonic / ros_gz_bridge отличается от предполагаемой. Возможно изображение повёрнуто на 90° или камера смотрит в другом направлении. **НЕ РЕШЕНО.**

### Проблема 2: scale ≈ 0.019–0.030 (нужен ~1.0)
- scale = IMU_displacement / SFM_displacement ≈ 0.02 → SFM в 50 раз завышает displacement.
- Предполагаемая причина: features слишком далеко (стены крепости 20–50 м) → большой apparent depth в SFM → маленький scale.
- Тест: `make fly` сразу после арминга (движение во время инициализации) → scale=0.0303, не помогло.
- При scale~0.02 VINS после init экстраполирует скорость ~6 м/с из ничего → pos drift → `big z translation` (порог 1м) → failure detection.

### Что надо попробовать дальше
1. **Проверить реальную ориентацию изображения**: сохранить кадр с `/image_mono` или `/camera/image_raw` в Gazebo и визуально определить где право/лево/верх/низ → пересчитать RIC.
2. **Уменьшить threshold failureDetection**: изменить `> 1` на `> 5` в `estimator.cpp` (big z), посмотреть сходится ли VINS при большем времени.
3. **Начальная позиция дрона ближе к стенам**: feature depth 2–5 м → scale близок к 1.
4. **Проверить `extrinsicRotation` через ric-принт**: в `visualization.cpp:84` печатается Euler ric каждый кадр при NON_LINEAR — сравнить с yaml.
5. **GPU апгрейд GCE** (08_add_gpu.sh): RTF=0.5x → RTF=1.0x → нормальный IMU rate → лучше scale.

**Why:** Без стабильного NON_LINEAR `/vins_estimator/odometry` не публикуется → ray_tracer не получает вход → навигация не работает.

**How to apply:** При следующей сессии начать с п.1 (визуальная проверка ориентации кадра) + п.2 (relaxed failureDetection threshold).
