# HowTo — синтетические тесты флоу-демпфера (A + B + C)

Детерминированная оффлайн-стенд-обвязка для тюнинга и верификации бокового
FLOW-DAMP (`src/lab/alt_hold_bootstrap.py`, ветка `--flow-hold`) **без** дорогого и
недетерминированного полного прогона SITL+Gazebo.

**Зачем.** Полный гибридный прогон — RTF 0.07 (~5 мин), а боковой замкнутый контур
демпфера **недетерминирован**: тайминг обработки кадров потока привязан к wall-clock,
поэтому ОДИН и тот же конфиг даёт разброс (H2=0.41 vs H4=2.40 — см. `tune_results.md`).
Тюнить такой метрикой нельзя. Три синтетических теста убирают и стоимость, и
недетерминизм: закон управления настраивается в surrogate за миллисекунды, а полной
симуляцией только **подтверждается победитель** (1 прогон вместо слепого свипа).

Теория PID (почему `ki=0`, физика членов) — `FAQ_pid.md`. Журнал прогонов —
`tune_results.md`.

```
A  flow_synth_test.py  геометрия/знак FlowEstimator     (cv2 → в контейнере)
B  flow_loop_sim.py    ОДУ замкнутого контура + PID      (numpy → где угодно/контейнер)
C  flow_calib.py       system-ID планта B из бэга        (rosbag2 → в контейнере)
```

Пайплайн демпфера, который они покрывают:
```
кадр ─LK→ поток ─derotate(ω,R)→ lateral(∝v) ─PID(kp,ki,kd,osign)→ roll_off ─руль→ a_боков
        └── A: этот блок ──┘   └────── B: этот замкнутый контур ──────┘
        C: фитит s,bias,k,d,τ из реального бэга → делает B предсказательным
```

---

## A — геометрия/знак FlowEstimator (`flow_synth_test.py`)

Юнит-тест: гоняет реальный `FlowEstimator` на СИНТЕТИЧЕСКИХ парах кадров с ИЗВЕСТНЫМ
сдвигом/поворотом → assert на знак и величину. Пиннит то, что «Шаг 1» вымучивал на
бэгах (rsign=+1, матрица R экстринсиков), чтобы будущий рефактор не перевернул знак
молча.

```bash
docker exec -i p1317_nav bash -lc \
  'source /opt/ros/humble/setup.bash; python3 /lab/flow_synth_test.py'
```

Проверки (exit 1 при провале):
- **trans_x**: сдвиг по X → `lateral` = сдвиг, верный знак (обе полярности);
- **trans_y**: сдвиг по Y → `lateral`≈0 (оси не путаются);
- **derot**: равномерный гориз. сдвиг ⇔ рыскание камеры вокруг Y. `rsign=0` →
  рыскание течёт ложным сносом (`lateral≈+S`); `rsign=+1` → **гасится** (≈0);
  `rsign=−1` → **удваивается** (`≈+2S`, складывает вместо вычитания).

> Ограничение теста derot: warp = равномерный сдвиг, что физически = поворот вокруг Y
> лишь в ПРИБЛИЖЕНИИ центра кадра (доминирующий член Longuet-Higgins). Тест проверяет
> СВЯЗКУ/ЗНАК derotation (R + вычитание + rsign), а не переисследует формулу `_rot_flow`.

Ожидаемый хвост: `РЕЗУЛЬТАТ: все проверки пройдены ✓ (rsign=+1 и R подтверждены синтетикой)`.

---

## B — симулятор замкнутого контура (`flow_loop_sim.py`)

Чистая numpy-ОДУ боковой оси + **тот же** закон PID, что в
`alt_hold_bootstrap.py:_on_flow_image` (строки 260-271):
```
плант:   v̇ = k·roll_off + d(t)     ẏ = v            (roll PWM → боковое ускорение)
сенсор:  flow = s·v(t−τ) + bias + шум,  на f_cam (кадры)
PID:     err=flow; i=clip(i+ki·err·dt,±imax); dterm=kd·(err−prev)/dt
         u=clip(kp·err+i+dterm,±max); roll_off=osign·blend·u    ← 1:1 с нодой
```

Запуск (нужен только numpy; на голом хосте — через контейнер):
```bash
# один прогон (дефолт kp8 ki0 osign+1)
python3 /lab/flow_loop_sim.py
# свип kp×ki → карта RMS_v / устойчивость
python3 /lab/flow_loop_sim.py --sweep
# воспроизвести знаковую катастрофу O1
python3 /lab/flow_loop_sim.py --osign -1
```

Читается вывод так:
- `RMS_v` — боковая RMS скорости в окне (главная метрика, ниже лучше);
- `v_терм` — терминальная |v| (что демпфер оставляет by-design);
- `max|y|` — насколько унесло (позиция дрейфует линейно при любой ненулевой v_терм — это
  НЕ неустойчивость, а свойство velocity-демпфера, см. `FAQ_pid.md`);
- вердикт `[устойч./затухает/РАСХОДИТСЯ]` — по **терминальной vs транзиентной** |v|
  (растёт ли скорость в конце относительно начального выброса; робастно к осцилляции
  через ноль и шумовому джиттеру при малом RMS_v).

Знаковое соглашение планта: `k<0` (дефолт) → положительный `roll_off` ТОРМОЗИТ
положительную скорость, т.е. `osign=+1` = демпфер (подтверждено реальностью), а
`osign=−1` → положительная ОС → runaway (воспроизводит O1).

Ключевые ручки: `--kp --ki --kd --osign` (регулятор); `--k --s --tau --dist --flow-bias
--noise` (плант — калибруются слоем C); `--sweep-kp --sweep-ki` (сетка свипа).

`--flow-bias` — DC-биас потока LK: `∫flow = s·y + bias·t`, член `bias·t` разгоняет
интеграл (windup) — механизм вреда `ki>0` из `FAQ_pid.md`.

---

## C — калибровка планта из реального бэга (`flow_calib.py`)

Делает B **предсказательным**: вытаскивает `k, s, bias, noise, d, τ` из уже снятого
прогона демпфера (system-ID), чтобы B называл `kp` под целевую скорость, а не гадал.
Мост «реальность→синтетика»: реальные кадры+гиро гоняются оффлайн через ТОТ ЖЕ
`FlowEstimator` (детерминированно), истинная поза Gazebo даёт скорость/ускорение.

Нужен bag с топиками: `/image_color`, `/gz_imu/data_flu`, `/model/iris_cam/odometry`
и (желательно) `/flow_dbg` — sim-штампованный **фактический `roll_off`** от ноды
(Vector3Stamped: x=roll_off, y=flow, z=conf). С `/flow_dbg` `k` и `d` фитятся
**из данных** (в open-loop excite `roll_off` экзогенный → чистая регрессия
`a_right=k·roll_off+d`); без него — `k` из физики руля, `d` из стационара, а `roll_off`
реконструируется как `osign·kp·flow` (годно только для замкнутого прогона).

```bash
docker exec -i -e SAFE_SEC=30 -e R_SAFE=40 -e CAL_KP=4 p1317_nav \
  bash -lc 'source /opt/ros/humble/setup.bash; python3 /lab/flow_calib.py'
```
(`CAL_KP` — kp, с которым СНЯТ бэг; `CAL_OSIGN` — его osign; `SAFE_SEC/R_SAFE/Z_TO` —
как в `drift_check.py`.)

Что фитит:
- **s, bias**: `flow ≈ s·v_right + bias` (реплей flow vs истинная боковая скорость) —
  прямое сенсорное отношение + реальный DC LK-снос;
- **noise + структура**: СКО остатка фита (реалистичный `--noise` для B) + автокорреляция
  лаг-1 (белый шум vs коррелированный — коррелированный объясняет, почему высокий kp
  реально хуже, чем в модели с белым шумом);
- **τ**: лаг кросс-корреляции `flow ↔ v_right`;
- **k**: из ДАННЫХ (`a_right=k·roll_off+d`, если есть `/flow_dbg` с экзогенным roll_off),
  иначе из физики руля (`roll_off`→угол→`a=g·tanθ`), знак из `osign`;
- **d**: из той же регрессии (данные) либо из стационара.

Валидация в конце: `v_term_pred = d/(|k|·kp·s)` vs наблюдаемая `RMS(v_right)`. Внизу
печатается готовая команда `flow_loop_sim.py --sweep …` с зафикшенными параметрами.

> ⚠ **R²(flow↔v)** в выводе — доля дисперсии боковой скорости, объяснённая потоком.
> Низкий R² (у O2 = 0.19) значит: поток — слабый прокси скорости, 81% — шум. Это
> ПОТОЛОК тюнинга — гейном не чинится, нужен рост SNR потока (медиана по кадрам /
> RANSAC вместо raw-median / отбор фич).

---

## Кампания калибровочных бэгов (open-loop system-ID)

Замкнутый прогон демпфера — плохой вход для калибровки (`roll_off∝v_right` → k/d не
разделяются, `flow↔v` меряется лишь вдоль одной траектории; у O2 R²=0.19). Целевые
бэги с ЗАДАННЫМ возбуждением развязывают идентификацию. Все три — под gz-hold (дрон
на сцене), с `/flow_dbg` в bag.

**1. roll-chirp (главный: k, s, τ, частотная характеристика).** Pitch держит gz, на
roll — заданный чирп `roll_off` (f0→f1) + ступени, демпфер ВЫКЛ. `roll_off` экзогенный
→ чистые k/s/τ из данных.
```bash
CPU=1 BS_GZHOLD=1 BS_ROLL_EXCITE=1 BS_HOLD_SEC=40 \
  BS_RE_AMP=50 BS_RE_F0=0.15 BS_RE_F1=1.5 BS_RE_CHIRP=25 BS_RE_STEP=3 \
  TOPIC="/image_color" \
  TOPICS_EXTRA="/gz_imu/data_flu /model/iris_cam/odometry /flow_dbg" \
  BS_THROTTLE_CLIMB=1800 BS_MODE_BUDGET=80 BS_ARM_BUDGET=80 \
  BS_CLIMB_BUDGET=120 BS_LAND_BUDGET=180 GDRIVE_UP=1 MP4=1 N_FRAMES=0 \
  bash src/lab/capture_scene.sh 960x540 liftland
# калибровка: docker exec -i -e SAFE_SEC=35 -e R_SAFE=40 p1317_nav \
#   bash -lc 'source /opt/ros/humble/setup.bash; python3 /lab/flow_calib.py'
# (roll_off экзогенный → k из данных; если дрон уходит за R_SAFE — поднять BS_RE_F0/уменьшить AMP)
```

**2. hover-baseline (d + шумовой пол потока в покое).** gz-hold ОБЕ оси, демпфер и
excite ВЫКЛ. v_right≈0 → flow = чистый шум/биас (пол), а gz-`roll_off` удержания
выявляет возмущение d.
```bash
CPU=1 BS_GZHOLD=1 BS_HOLD_SEC=40 \
  TOPIC="/image_color" TOPICS_EXTRA="/gz_imu/data_flu /model/iris_cam/odometry /flow_dbg" \
  ... GDRIVE_UP=1 bash src/lab/capture_scene.sh 960x540 liftland
```

**3. yaw-derot (проверка rsign/derotation на РЕАЛЬНОЙ сцене).** gz-hold + импульсы yaw
(`BS_GZ_YAW`), трансляции нет → поток rotation-доминирован. Анализ — `flow_derotation_check.py`.
```bash
CPU=1 BS_GZHOLD=1 BS_GZ_YAW=80 BS_GZ_YAW_PERIOD=6 BS_HOLD_SEC=40 \
  TOPIC="/image_color" TOPICS_EXTRA="/gz_imu/data_flu /model/iris_cam/odometry" \
  ... GDRIVE_UP=1 bash src/lab/capture_scene.sh 960x540 liftland
```

`BS_ROLL_EXCITE` env → `--roll-excite*`: `BS_RE_AMP` (PWM), `BS_RE_F0/F1` (Гц чирпа),
`BS_RE_CHIRP` (сек чирпа), `BS_RE_STEP` (полупериод ступени). Реализация —
`alt_hold_bootstrap.py:_roll_excite_cmd`.

## Рабочий цикл тюнинга

```
1. Снять эталонный гибрид-bag (реальный прогон, любой разумный kp):
   CPU=1 BS_GZHOLD=1 BS_FLOWHOLD=1 BS_HOLD_SEC=40 \
     BS_FLOW_KP=4 BS_FLOW_KI=0 BS_FLOW_MAX=150 \
     TOPIC="/image_color" TOPICS_EXTRA="/gz_imu/data_flu /model/iris_cam/odometry" \
     BS_THROTTLE_CLIMB=1800 BS_MODE_BUDGET=80 BS_ARM_BUDGET=80 \
     BS_CLIMB_BUDGET=120 BS_LAND_BUDGET=180 GDRIVE_UP=1 MP4=1 N_FRAMES=0 \
     bash src/lab/capture_scene.sh 960x540 liftland
2. A: подтвердить, что знак/геометрия не сломаны   → flow_synth_test.py
3. C: откалибровать плант из этого bag             → flow_calib.py (CAL_KP=…)
4. B: свипнуть kp на калиброванных параметрах       → flow_loop_sim.py --sweep --k … --s …
5. Взять kp из B → ОДИН реальный гибрид на подтверждение → drift_check.py
   (RMS_v совпал с предсказанием B ⇒ калибровка надёжна, гейн найден)
```

Текущий результат цикла (O2-калибровка): **kp16-24, ki0, osign+1** → предсказано
RMS_v ~0.2-0.5 м/с. Осталось подтвердить одним реальным прогоном. Детали — `tune_results.md`.
```
python3 /lab/flow_loop_sim.py --sweep --k -0.0196 --s 0.807 --tau 0.00 \
        --dist 0.266 --flow-bias 0.500 --noise 1.648
```
