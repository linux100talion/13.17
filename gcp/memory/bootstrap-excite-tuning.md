---
name: bootstrap-excite-tuning
description: "ALT_HOLD bootstrap EXCITE — почему дрон улетал в дом, формула размаха, текущая цель «пятачок 60с»"
metadata: 
  node_type: memory
  type: project
  originSessionId: 23847c91-aec3-4a50-9af7-539d49f00c31
---

Ветка `nn2_c3_vins_althold_2`. Отладка `src/lab/alt_hold_bootstrap.py` (команда
`bootstrap`): взлёт в ALT_HOLD + раскачка EXCITE для init VINS без GPS.

**Две причины, почему дрон улетал за сцену / врезался в дом:**
1. Непрерывный yaw во время translate-цикла проворачивал ось → импульсы +τ/−2τ/+τ
   не гасились в мировой системе → снос. ПОФИКШЕНО: yaw теперь импульсом между
   translate-циклами (`--yaw-dur`/`BS_YAW_DUR`), translate при фикс. курсе.
2. **Главное — амплитуда слишком большая.** Один взмах профиля уводит на
   `peak ≈ a·τ²`, где `a = g·tan(BS_EXCITE/500·45°)` (PWM→наклон, ANGLE_MAX≈45°).
   При `BS_EXCITE=40, τ=3`: наклон 3.6°, a≈0.62 м/с² → **peak ≈ 5.6 м за ОДИН
   взмах** → на первом же взмахе влетает в дом. Мотание камеры в кадрах = кувырки
   ПОСЛЕ удара, не раскачка.

**Формула для тюнинга:** peak[м] ≈ g·tan(BS_EXCITE/500·45°)·BS_EXCITE_PERIOD².
Хочешь «пятачок» D метров → peak ≤ D/2. Уменьшать и `BS_EXCITE`, и период
(peak ∝ τ²). Пример мягких: `BS_EXCITE=12 BS_EXCITE_PERIOD=2` → peak≈0.7 м.

**Текущая descoped-цель (важно):** СНАЧАЛА научить дрон «крутиться на пятачке»
3–5 м и не улетать ≥60 сим-сек «с закрытыми глазами» (open-loop, без позиционной
обратной связи) — VINS-сходимость пока вторична. `BS_VINS_TO` НЕ поднимаем, пока
не закрыт этот пункт + не разобран IMU sim freq (см. [[imu-sim-freq-sim]]).
Детали и чек-лист — `docker/sim/todo2.txt`.

**ДВЕ независимые причины «дрон ровно летит и улетает за край» (диагностика по bag):**
1. **Мёртвая зона RC** — excite ±12 PWM целиком в `RCx_DZ` (дефолт ~30) →
   `get_pilot_desired_lean_angles` видел «стик в центре» → нулевой наклон → excite
   НИЧЕГО не делал (поэтому смена BS_EXCITE/PERIOD не меняла поведение; pitch std
   во весь EXCITE 0.06°). Фикс: `RC1_DZ=RC2_DZ=RC4_DZ=0` в `sitl-extra.parm`.
2. **Смещение уровня AHRS** — EKF-наклон в EXCITE средне ~0°, НО в видео дрон
   ускоряется за край («небольшое постоянное ускорение») → реальный наклон ~1°,
   AHRS считает его горизонтом (вероятно level-cal в `nav_up.sh` снялся на неровной
   земле → `AHRS_TRIM_X/Y` ≠ 0). RC_DZ=0 это НЕ лечит. Проверять `AHRS_TRIM_X/Y`.

**Сценарий `liftland`** (новый, `src/lab/liftland.sh` + `--hold-only`/`--hold-sec`
в alt_hold_bootstrap.py + команда в capture_scene.sh): ALT_HOLD взлёт→держать
уровень БЕЗ раскачки→посадка. Изолирует дрейф (причина 2) от excite. Запуск:
`bash src/lab/capture_scene.sh 960x540 liftland` (`BS_HOLD_SEC` — сколько висеть).

**КОРЕНЬ runaway (подтверждён истиной Gazebo, НЕ SITL-SIMSTATE — та врёт по
позиции при внешней физике!):** в GPS-denied EKF не отличает наклон от
горизонт-ускорения (tilt/accel ambiguity, нет опорной скорости) → небольшой
bias интегрируется в растущий наклон → дрон сам себя разгоняет → уезжает на
десятки/сотни метров (карта `grass_plane` всего 150×150 м). Лечится ТОЛЬКО
референсом скорости/позиции (VINS/flow). `AHRS_TRIM=0`, `SIM_WIND=0` — не они.

**✅ РЕШЕНО (СИМ-костыль `gz-position-hold`, коммиты до b7916ac):** на время
бутстрапа подменяем «идеальным VINS» = истинной позой Gazebo. Реализация:
`worlds/iris_cam/model.sdf` (gz-sim-odometry-publisher → `/model/iris_cam/odometry`)
+ мост в `sim_up.sh` + PID в `alt_hold_bootstrap.py` (`--gz-hold`, env `BS_GZHOLD=1`).
Ошибка+скорость world → тело (по yaw) → PWM roll/pitch override. **Знаки коррекции
оба +1** (выверено отладочным логом `gz:`: pitch_off<0 = ускорение ВПЕРЁД, конвенция
RC противоположна excite-комментарию). PID гейны: kp40 kd120 ki8 (imax100).
Результат: дрон держится в **~0.2 м** (истина Gazebo). Костыль ТОЛЬКО для sim — на
Orin референс даст реальный VINS. Запуск: `BS_GZHOLD=1 ... capture_scene ... liftland`.
Осталось: вернуть камеру + мягкая раскачка для параллакса → реальный VINS-init.
