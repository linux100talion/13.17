# Как делался реплей-A/B (на примере ab_loiteryaw, 2026-08-31)

Памятка по циклу «ручной полёт → сценарий → два реплея → метрики → видео»,
снятая с реальной кампании: A/B фикса нырка LOITER при yaw+вперёд (разбор
ручного полёта `lv2_joy_20260831_074358`, сценарий `../ab_loiteryaw.json`,
прогоны `docker/sim/output/joystick/ab_loiteryaw_base|fix`). Общая механика
реплея — `src/lab/joystick/README.md`; здесь — рабочая последовательность
и грабли, на которые реально наступили.

## Цикл целиком

```bash
# 1. Разбор ручного полёта (стек не трогает, только чтение bag):
RUN=lv2_joy_20260831_074358 bash src/lab/joystick/analyze.sh
#    → report.txt (лента событий + сегменты стиков), scenario_draft.json, raw.jsonl

# 2. Сценарий пишется РУКАМИ по report.txt, не из scenario_draft.json:
#    для A/B нужен не повтор всего полёта (330 шагов кванта), а ДИСТИЛЛЯЦИЯ
#    дефектных эпизодов. Из report.txt берутся уровни стиков и длительности
#    конкретных эпизодов (тут: t=97/180/195-198 — «разгон → yaw на ходу»),
#    плюс чистый замер (пируэт на месте) для изоляции одной ручки.
#    Якоря wait_alt/wait_mode вместо таймингов — полёт замкнут по факту.
#    → src/lab/joystick/scenarios/ab_loiteryaw.json (в репо)

# 3. Прогон A (база): фикс временно убирается из рабочего дерева
git stash push -m "fix (A/B: временно убран для базового прогона)"
NAME=ab_loiteryaw_base BS_PILOT=replay \
  BS_REPLAY_SCENARIO=/lab/joystick/scenarios/ab_loiteryaw.json \
  MP4=0 HUD_MP4=0 IPM_MP4=0 bash src/lab/freefly_lv.sh > /tmp/run_a.log 2>&1

# 4. Прогон B (фикс): вернуть дерево и повторить ТОТ ЖЕ сценарий
git stash pop
NAME=ab_loiteryaw_fix BS_PILOT=replay \
  BS_REPLAY_SCENARIO=/lab/joystick/scenarios/ab_loiteryaw.json \
  MP4=0 HUD_MP4=0 IPM_MP4=0 bash src/lab/freefly_lv.sh > /tmp/run_b.log 2>&1

# 5. Метрики по bag обоих прогонов (скрипт лежит РЯДОМ: memory/ab_metrics.py):
#    эпизоды |yaw-стика|>0.2 (кластеризация с паузой 0.8 с) × режим FCU ×
#    наклоны по ИСТИНЕ Gazebo (/model/iris_cam/odometry) + окно-хвост 2.5 с
#    (реакция контура ПОСЛЕ отпускания стика — нырок живёт там);
#    авторитет yaw = Δкурс_за_нажатие / длительность / уровень стика.
docker cp src/lab/joystick/scenarios/memory/ab_metrics.py p1317_nav:/tmp/ && \
  docker exec p1317_nav bash -lc \
  'source /opt/ros/humble/setup.bash; python3 /tmp/ab_metrics.py \
   /root/sim_ws/output/joystick/ab_loiteryaw_base/bag/scene_bag_0.db3'

# 6. Видео — ПОСТФАКТУМ из bag (на прогонах MP4=0 ради скорости цикла):
for RUN in ab_loiteryaw_base ab_loiteryaw_fix; do
  D=/root/sim_ws/output/joystick/$RUN
  docker exec -e SCENE_BAG=$D/bag -e SCENE_MP4=$D/scene.mp4 p1317_nav bash -lc \
    'source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash;
     source /root/sim_ws/install/setup.bash; python3 /lab/make_video.py'
  docker exec -e SCENE_BAG=$D/bag -e SCENE_HUD_MP4=$D/scene_hud.mp4 p1317_nav bash -lc \
    'source /opt/ros/humble/setup.bash; source /opt/overlay/install/setup.bash;
     source /root/sim_ws/install/setup.bash; python3 /lab/hud_video.py'
done
```

Наблюдение за летящим прогоном — ТОЛЬКО чтением (дисциплина прогона):
`tail -F docker/sim/output/joy_replay.log` — вехи шагов сценария и сбои
(`ТАЙМАУТ ЯКОРЯ`, `ГЕОЗАБОР`, аварийная посадка); `-F` по имени обязателен —
файл пересоздаётся каждым прогоном, `-f` останется на старом inode.
Проверка, что в B летал именно новый код, — по логу ноды: строка яруса
(здесь «ЛЕСЕНКА: ярус LOITER — … yaw у демпфера») + `>>> ИТОГ: FREEFLY_DONE`.

## Что важно в самом сценарии (SF-мастер)

- `init`: `{"sticks": {…, "thr": 0}, "sw": 1, "sf": 0}` — CH6 заранее в
  потолок LOITER, SF не-вверх до арма (руддер-арму нужны сырые оси).
- Первым шагом — `{"hold": 3.0}` с газом В ЦЕНТРЕ, и только потом `{"arm"}`
  (жест thr=−1 + yaw=+1 ведёт сам якорь). Почему — см. грабли №1.
- После арма `{"sf": 1}` → лесенка ведёт; взлёт `wait_alt`, латч —
  `wait_mode LOITER` (не таймингом).
- Посадка — снижение + руддер-дизарм, НЕ кнопка SA: `--land-btn` реплея не
  синхронизирован с `BS_LAND_JOY` бокса (реплей жмёт b0, нода слушала b1).
- `fence` + `station_keeping` оставить: разомкнутые эпизоды разгона уносят
  борт по-разному в A и B (разный темп разворота на тот же стик) — «руки
  пилота» держат обе ветки на сцене, машинерия у веток одинаковая.

## Грабли (все — реальные, с этой кампании)

1. **Защёлка газа Арбитра съедает руддер-арм.** Под SF-мастером `sf=0` =
   MANUAL → газ идёт через ThrottleLatch Арбитра, а защёлка открывается
   только ВИЗИТОМ стика в центр. `thr: -1` с нулевой секунды в init =
   защёлка вечно закрыта → FCU видит газ в центре → руддер-арм не проходит →
   «ТАЙМАУТ ЯКОРЯ: arm» через 180 с. У живого пилота 074358 газ первые
   10.8 с стоял в центре — потому у него и работало. Лечение: init thr=0 +
   hold перед якорем arm.

2. **Аварийный pkill ноды осиротил рекордер bag — 90 ГБ земли.** Первый
   бракованный прогон сворачивали через `docker exec p1317_nav pkill -f
   bootstrap_arch2` (подсказка самого freefly_lv). Секвенсор при этом НЕ
   остановил `ros2 bag record`; хуже — архивация успела сделать `mv` bag'а,
   а рекордер продолжал писать в тот же inode уже внутри архива. Поймано на
   90 ГБ (диск 55%). После ЛЮБОГО аварийного сворачивания проверять:
   `docker exec p1317_nav bash -lc "ps -eo cmd | grep '[r]os2 bag'"` и
   добивать `pkill -INT -f 'ros2 bag record'`.

3. **Пайп на выводе freefly_lv не закрывается при орфанах.** Запуск
   `bash freefly_lv.sh | tail -40` в фоне: когда скрипт умер, а осиротевший
   рекордер унаследовал write-конец пайпа, `tail` не видит EOF — фоновая
   команда «висит» бесконечно и уведомление о завершении не приходит.
   Запускать с редиректом в файл (`> run.log 2>&1`), без пайпа.

4. **fresh-start пересоздаёт контейнер — /tmp контейнера пустеет.** Скрипт
   метрик, закинутый `docker cp` в `p1317_nav:/tmp/`, после следующего
   прогона (freefly_lv делает fresh-start) пропадает — копировать заново
   перед каждым разбором (или держать в `/lab/…`, он bind-mount).

5. **A/B кода — через git stash, и это хрупко.** База = `git stash push`,
   фикс = `git stash pop`; между ними летает прогон. Сценарий и прочие
   НЕтрекнутые файлы stash не трогает (это и нужно). После каждого шага —
   `git status` глазами: упавший посреди цикла прогон оставляет дерево в
   «не той» половине A/B. Параметры FCU из `sitl-extra.parm` подхватываются
   на fresh-start (freefly_lv делает его сам), но только если их нет в
   eeprom (проверить `docker/sim/scripts/sitl_lv_profile.py` — что он
   param_set'ит, то из .parm не переопределить).

## Результат кампании (для калибровки ожиданий)

Один прогон ≈ 5–8 мин wall (fresh-start + eeprom + полёт ~90–125 сим-с +
архив; bag 3–4 ГБ при MP4=0). Реплей воспроизвёл дефект ЯРЧЕ ручного полёта
(наклон 54° против 30° у пилота — виртуальный пилот жмёт стик без страха),
фикс тем же сценарием: наклон 54.2→20.8°, авторитет yaw 67–104→37–43 °/с на
единицу стика, скорость LOITER 10.7→4.7 м/с. Детали — память бокса
(`loiter-yaw-dive.md`) и `docker/sim/output/joystick/ab_loiteryaw_*/`.
