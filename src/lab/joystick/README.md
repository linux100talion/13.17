# src/lab/joystick — запись и реплей пульта (freefly без человека)

Цель: пилот один раз летает freefly руками (TX12 → `/joy`), мы разбираем bag
«что и когда он делал», превращаем в сценарий — и дальше стенд гоняет тот же
полёт сам, без пульта. Инжекция — в топик `/joy` (не в ядро Linux): для
`JoyPilot` (`control_pkg/infrastructure/ros_pilot.py`) реплей неотличим от
живого `joy_linux_node`, весь стек ниже `/joy` идентичен ручному полёту.

## Цикл работы

```
1. РУЧНОЙ ПОЛЁТ (как обычно; /joy уже пишется в bag через TOPICS_EXTRA):
     bash src/lab/freefly_lv.sh
   Каждый прогон freefly_lv АРХИВИРУЕТСЯ: docker/sim/output/joystick/<NAME>/
   (scene.mp4, <NAME>.env — вся мета BS_*/ветер/commit, bag/, joy.log;
   для реплея — ещё joy_replay.log и копия сценария). JPEG-кадры не делаются
   вовсе (FRAMES=0 по умолчанию; вернуть — FRAMES=1). NAME=… задаёт имя,
   дефолт lv<LV>_<пилот>_<дата_время>; KEEP_BAG=0 — не забирать bag (2+ ГБ).
   Без архива scene.mp4/scene_bag живут до СЛЕДУЮЩЕГО прогона (capture_scene
   чистит их на старте). Архив копится — старые прогоны чистить руками.
   Повторный запуск поверх летящего прогона блокируется (наслоение стоило
   bag'а 2026-08-22): freefly ждёт ДИЗАРМ пилота — газ min + yaw ВЛЕВО 2–3 с.

2. РАЗБОР bag (стек не трогает):
     RUN=<NAME> bash src/lab/joystick/analyze.sh     # bag из архива прогона
     bash src/lab/joystick/analyze.sh                # или свежий output/scene_bag
   → в каталог прогона (или output/joystick/ без RUN):
       report.txt          — лента: CH6, жесты арм/дизарм, высоты, сегменты стиков
       scenario_draft.json — черновик сценария (жесты уже заменены якорями)
       raw.jsonl           — сырой таймлайн /joy (для валидации канала)

3. ПРАВКА черновика → src/lab/joystick/scenarios/<имя>.json (в репу):
   заменить сырые тайминги набора/снижения высоты на wait_alt,
   ожидание латча режима — на wait_mode (см. «Формат сценария»).

4. РЕПЛЕЙ (тот же атомарный прогон, что ручной, только пилот виртуальный):
     BS_PILOT=replay BS_REPLAY_SCENARIO=/lab/joystick/scenarios/<имя>.json \
       bash src/lab/freefly_lv.sh
   Валидация канала (сырой повтор, траекторию НЕ повторяет — разомкнут):
     BS_PILOT=replay BS_REPLAY_RAW=/root/sim_ws/output/joystick/<RUN>/raw.jsonl \
       bash src/lab/freefly_lv.sh
```

Сравнение ручного и реплейного прогонов — по их архивам: в bag обоих есть
`/joy` и gt-одометрия, видео лежат рядом (`.../joystick/<ручной>/scene.mp4` vs
`.../joystick/<реплей>/scene.mp4`).

`BS_REPLAY_*` доезжают до контейнера автопробросом `BS_*` в `capture_scene.sh`;
`bootstrap_arch2.sh` при `BS_PILOT=replay` поднимает `joy_replay.py` вместо
`joy_linux_node` и отдаёт ноде `--pilot joy`. Лог реплея — `output/joy_replay.log`.

## Формат сценария (joy_replay.py)

Значения стиков — СЕМАНТИЧЕСКИЕ (PWM-конвенция, v=(PWM−1500)/400), знаки
JOY_SIGNS накладывает сам реплей (те же, что у ноды; при кастомном
`BS_JOY_SIGNS` он передаётся реплею автоматически):

- `roll`: +1 = вправо; `pitch`: −1 = вперёд («от себя»), +1 = назад;
- `thr`: −1 = газ min (для руддер-арма), +1 = max;
- `yaw`: +1 = вправо;
- `sw` (CH6): −1 = наш стек (тумблер ВВЕРХ), 0 = центр (ALT_HOLD; при
  `BS_FF_LOITER=1` — штатный LOITER-на-VINS), +1 = MANUAL (ВНИЗ).

```json
{
  "version": 1,
  "name": "lv_takeoff_loiter_land",
  "init": {"sticks": {"thr": -1}, "sw": -1},
  "steps": [
    {"arm":  {"timeout": 120}, "note": "руддер-арм, замкнуто по /mavros/state"},
    {"sticks": {"thr": 0.4},   "note": "взлёт"},
    {"wait_alt": {"gte": 2.5, "timeout": 60}},
    {"sticks": {"thr": 0.0}},
    {"hold": 3.0},
    {"sw": 0,                  "note": "центр CH6 → LOITER-на-VINS"},
    {"wait_mode": {"is": "LOITER", "timeout": 30}},
    {"hold": 10.0},
    {"sticks": {"pitch": -0.3}, "note": "немного вперёд"},
    {"hold": 3.0},
    {"sticks": {"pitch": 0.0}},
    {"sticks": {"thr": -0.4},  "note": "снижение"},
    {"wait_alt": {"lte": 0.15, "timeout": 90}},
    {"sticks": {"thr": -1.0}},
    {"hold": 2.0},
    {"disarm": {"timeout": 30}}
  ]
}
```

Шаги (один вид действия на шаг; `note` — к любому):

| Шаг | Что делает |
|---|---|
| `{"sticks": {...}}` | мгновенно выставить оси (частично можно); держатся до смены |
| `{"sw": -1\|0\|1}` | положение CH6 |
| `{"hold": сек}` | держать состояние N sim-секунд |
| `{"ramp": {"sticks": {...}, "dur": с}}` | линейный ход осей за dur |
| `{"arm": {"timeout": с}}` | жест thr=−1,yaw=+1 до `armed` (/mavros/state), потом yaw=0 |
| `{"disarm": {"timeout": с}}` | жест thr=−1,yaw=−1 до дизарма |
| `{"wait_alt": {"gte"/"lte": м, "timeout": с}}` | ждать высоту (ground truth gz) |
| `{"wait_mode": {"is": "LOITER", "timeout": с}}` | ждать режим FCU |

Таймауты и `hold` — в sim-секундах (RTF-независимо). `/mavros/state` идёт 1 Гц —
якоря arm/wait_mode дискретны ±1 с. Высота — gt `/model/iris_cam/odometry`
(жива в GPS-denied; rel_alt без GPS замерзает). Таймаут любого якоря →
аварийная посадка (thr=−0.5 до земли → газ min → дизарм) — стенд не виснет.

## Почему якоря, а не тайминги

Сырой повтор стиков — разомкнутый контур: ветер (`WIND_SPD=10`), тайминг EKF
warmup и сходимости VINS гуляют между прогонами, точная траектория не
повторится. Поэтому боевые сценарии — семантические, с замыканием по
armed/mode/alt; `raw.jsonl` — только проверка, что канал реплея честный
(в bag реплейного прогона `/joy` пишется так же — можно сравнить с ручным).

Известные упрощения v1: анализатор берёт ПЕРВЫЙ жест арма (неудачные попытки
до него в черновик не попадают — смотреть report.txt); сегментация стиков
квантует по `--eps` (0.1) — микрокоррекции пилота осознанно сглаживаются.
