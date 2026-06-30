---
name: flow-damper
description: FLOW-DAMP — scale-free velocity-демпфер от ветра по одной форвард-камере; ветка nn2_c3_vins_althold_4, код-скелет написан, продолжение по handoff в спеке
metadata:
  type: project
---

Новое направление (начато 2026-06-30): **FLOW-DAMP** — scale-free velocity-демпфер БОКОВОГО
сноса по ОДНОЙ форвард-камере. Включается у трудного места, ИГНОРИРУЕТ VINS/одометрию,
гасит снос (ветер), пропускает стики. Выживает там, где VINS срывается (малое движение).

**Зачем:** монокулярный VINS теряет метрическую трансляцию без возбуждения IMU и срывается
при малом движении/hover (см. [[vins-init]]). Демпфер — регулятор, а не оценщик состояния:
не дивергирует, а простаивает при отсутствии потока. Путь (B): прямой регулятор поверх
ALT_HOLD через RC override, НЕ через EKF3 (тот хочет метрику, мы scale-free).

**Ветка:** `nn2_c3_vins_althold_4` (из nn2_c3_vins_althold_3). Концепт целиком —
`docker/sim/FAQ_vins.md` (разделы 6–15). Спека — `docker/sim/FLOW_DAMP_spec.md`.

**Решено (фазировка):** v0 = демпфер ТОЛЬКО бокового сноса (ROLL); продольный (looming) +
yaw сквозные пилоту; sparse LK + аффинный фит; сценарий A (пролёт между препятствиями).
Фаза 2 = looming→PITCH + визуальный yaw-hold поверх доказанного ядра.

**Код написан (на телефоне, py_compile OK, рантайм только в контейнере nav):**
- `src/lab/flow_estimator.py` — FlowEstimator (numpy+cv2, без ROS): sparse LK → derotate
  (ω_cam=R·ω_imu) → боковой поток; параметры R_cam_imu + rotflow_sign.
- `src/lab/flow_damp_node.py` — v0-нода: стик=setpoint → PID → ROLL-override.
- `src/lab/flow_derotation_check.py` — ОФФЛАЙН-валидация знака derotation по bag (без
  сим-стека), берёт {R,R^T}×{±} с минимумом остатка на вращательных кадрах.

**ПРОДОЛЖЕНИЕ:** раздел «ПРОДОЛЖЕНИЕ В РАБОЧЕМ БОКСЕ (handoff)» в `FLOW_DAMP_spec.md` —
самодостаточен. Порядок: Шаг1 derotation-check (снять TODO[sign]) → Шаг2 конфликт override
(свернуть в alt_hold_bootstrap --flow-hold) → Шаг3 тюнинг (оракул = истинная поза gz, ветер
SIM_WIND) → Шаг4 опц. estimate_extrinsic=2.

**Связь с [[vins-init]]:** там зафиксирована систематич. ошибка гравитации (g в +X_cam
вместо +Y_cam) — тот же вопрос ориентации/знака extrinsic камера↔IMU. derotation-check
(Шаг1) и estimate_extrinsic (Шаг4) его и разрешают. Боевой dummy_13_7.yaml = identity-
ротация (placeholder, нужна Kalibr перед боевым). Связано: [[sim-workflow]], [[user-setup]].
