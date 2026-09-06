---
name: sim-truth-twist-body-frame
description: "Twist истины Gazebo /model/iris_cam/odometry задан в ОСЯХ ТЕЛА (child_frame), не мира — ловушка разбора: модуль скорости верен, компоненты по курсу крутить нельзя дважды; stick_lateral/vins_twist_check исправлены 2026-09-06, числа сноса 113224 (cmd/5) были с ошибкой"
metadata:
  type: reference
---

`/model/iris_cam/odometry` (ros_gz_bridge, gz OdometryPublisher): pose — мир, twist.linear — В ОСЯХ
ТЕЛА (x вперёд, y влево). Проверка по bag lv2_joy_20260906_150448 (плечи на курсах −10° и 170°):
|twist − Δpos/dt| = 0.024 м/с в теле против 3.07 в мире (n 6666). Симптом ошибки: на плечах с
курсом ~180° «скорость вперёд» отрицательна при стике вперёд у ОБОИХ стабилизаторов, «трек» всех
плеч ≈ 0°.

**Why:** модуль |v| (gust_hold_compare, brake_phase, pk/vmax) от рамы не зависит и верен; всё, что
раскладывает по курсу (боковой снос на плечах, сравнение с мировым twist VINS, «тело» в scratch
att_ipm), обязано знать раму. Первая версия `stick_lateral.py` и scratch-разбор 113224 крутили
тело ещё раз — на курсе у 0° почти верно, у 180° знак вперёд переворачивается, боковая мешается с
продольной; числа сноса 0.56–0.76 м/с в cmd/5/README — с этой ошибкой (bag исчез, не пересчитать),
честные плечи — 150448 в cmd/6/README (DpVins vlat_mean 0.0–0.35, RMS 0.7–0.97; DpHold 0.05/0.49).

**How to apply:** vx/vy истины брать как тело; для мира поворачивать по курсу истины
(`vins_twist_check.py` так делает). Twist VINS `/odometry` — в раме ПОЗЫ (мир VINS), проверено.
Связано: [[dphold-vs-dpvins-gusts]], [[bridge-poisons-ekf-on-vins-divergence]].
