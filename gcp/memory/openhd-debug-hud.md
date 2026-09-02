---
name: openhd-debug-hud
description: "Debug-HUD в OpenHD-оверлее ДОКАЗАН живым прогоном (2026-08-23): /mission/status от лётной ноды + баннеры в openhd_streamer; с 2026-08-30 баннер = ЯРУС лесенки SF-мастера (не «VINS READY»), живость VINS — только ODO; HUD только в FPV :5600, в scene.mp4 его НЕТ по построению"
metadata: 
  node_type: memory
  type: project
  originSessionId: 96a84f1e-b60f-4417-9634-e6dcd52bb66e
  modified: 2026-08-25T17:41:03.154Z
---

Debug-HUD (договорённость ночи 2026-08-22/23) РЕАЛИЗОВАН 2026-08-23. Мотив:
полёт lv1_joy_20260822_232043 — VINS init после посадки, пилот щёлкал CH6
вслепую, гейт молча держал.

**Как сделано (ключевые решения):**
- Топик статуса — **`/mission/status`** (String, "k=v k=v"), а не /freefly/status:
  консистентно с /mission/pilot_done, нода не только freefly.
- Единый источник правды: строку собирает чистая функция
  `control_pkg/application/hud.py::hud_status(s, fresh_sec)` — ровно гейт
  Freefly/LoiterHold: `st=READY` = extnav_ready + age<fresh + alt>1.5;
  `WAIT` (why: extnav/stale/ground); `DEAD` (why: no_odom / stale>3×fresh —
  гистерезис выхода Freefly). Офлайн-тест `src/control/test/test_hud_status.py`
  (все 16 офлайн-тестов зелёные).
- Публикует `bootstrap_node._tick` каждый тик через `RosDebugSink.publish_status`
  (ros_io.py); переход st= дополнительно логируется строкой «HUD: …».
- `openhd_streamer`: параметр `hud` (default true). Баннер гейта (зел/жёлт/крас)
  рисуется ТОЛЬКО при свежем (<3 с) /mission/status — на Orin без лётной ноды
  гаснет сам. Остальные строки: ODO Гц+возраст /odometry (стример меряет сам,
  зелёный = VINS init жив), FEAT из /feature И /feature_tracker/feature (сим/борт),
  режим+armed /mavros/state (import mavros_msgs под try/except), CMD = PWM-смещения
  из /flow_dbg(.x)+/flow_dbg2(.x), DRIFT = норма /nn1/drift (+возраст, если >10 с).
  Возрасты — часами ноды (sim в симе) → RTF-независимо.
- bag: `/mission/status` добавлен в TOPICS_EXTRA freefly_lv.sh; joy_timeline.py
  читает его и кладёт переходы «HUD: VINS READY (odom=N)» в ленту событий
  (read_bag теперь возвращает 5 значений).

**ДОКАЗАН живым прогоном** lv1_replay_20260823_141554 (реплей lv_flight2,
ветер 5): FREEFLY_DONE, полный цикл. Лента report.txt: 53.2 HUD WAIT(extnav) →
75.3 HUD VINS READY (odom=644) → 100.7 CH6-центр + LOITER → 127.9 WAIT(ground)
→ 128.5 касание. READY пришёл за 25 с ДО щелчка — ровно устраняет слепоту 232043.

⚠️ **В scene.mp4 HUD НЕТ по построению**: mp4 собирается из /image_color
(чистая камера из bag), HUD живёт только в FPV-потоке :5600. Для архива есть
**scene_hud.mp4** — пост-рендер из bag (2026-08-23): отрисовка вынесена в
`nav_pkg/hud_renderer.py` (без ROS, масштаб k=w/1280), стример и
`src/lab/hud_video.py` рисуют ОДНИМ кодом; hud_video импортирует рендерер из
bind-mounted исходников (не из colcon-install — не протухает), потоковая
запись (make_video.py держит весь полёт в RAM ~7 ГБ — не повторять), fps по
первым 90 кадрам; /mission/status без header → sim-время последнего
стемпованного сообщения. freefly_lv.sh рендерит сам (HUD_MP4=0 — выкл) и
кладёт в архив; /feature добавлен в TOPICS_EXTRA (строка FEAT в пост-рендере).
Проверен на bag lv1_replay_20260823_141554 — баннеры совпали с лентой событий;
прогон lv1_replay_20260823_150316 подтвердил FEAT в пост-рендере (132 на земле
ДО init VINS — трекер жив раньше одометрии, 150 = потолок max_cnt в полёте).
Поверх счётчика — ЗЕЛЁНЫЕ ТОЧКИ самих фич (2026-08-23): каналы /feature =
[id, u, v, vx, vy], u/v — пиксели кадра камеры (подтверждено по
feature_tracker_node.cpp:160-173 форка); рисует HudRenderer (протухание
0.5 с — быстрее строки FEAT), в стримере параметр hud_features, в hud_video
env SCENE_FEAT_DOTS (масштабирует u,v при MAXW-даунскейле). Точки в ЖИВОМ
FPV доказаны прогоном lv1_replay_20260823_165709 (кадры с :5600 на земле и
в READY-полёте; «ловец» — фоновый скрипт ждёт «HUD: st=READY» в логе прогона
и пассивно снимает gst-launch'ем, стек не трогается).
Проверка живого потока с хоста (стек не трогает): `timeout 10 gst-launch-1.0
udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,
payload=96" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! jpegenc !
multifilesink location=f_%04d.jpg` — кадры подтвердили отрисовку (баннер
гейта корректно гаснет после завершения лётной ноды).

Попутно: дефолт WIND_SPD в freefly_lv.sh снижен 10 → 5 (2026-08-23, просьба
Андрея; 10 был стрессом LV-серии).

**Постоянный баннер статуса борта** (2026-08-25, ещё НЕ доказан живым
прогоном): в строку /mission/status добавлено поле `ekf=` (hud.py,
EKF_FRESH_SEC=2.0 — зеркало WaitEkfPos: свежий /mavros/local_position =
«EKF держит позицию, взлетать можно»). HudRenderer рисует машину состояний:
«EKF WARMUP» (жёлт) → «EKF READY - TAKEOFF OK» (зел) → после арма «ARMED»
(зел) → после дизарма снова READY/WARMUP по ekf=. В полёте показывается
ARMED, а не ekf: после GPS-kill local_position молчит штатно — WARMUP в
воздухе врал бы. Старые bag без ekf= — баннера нет. Текст HUD только ASCII
(Hershey-шрифты cv2 кириллицу не рисуют). Разборщики (joy_timeline,
phase_stats) k=v-совместимы.

**Строка ALT двумя источниками** (2026-08-25, ещё НЕ доказана живым прогоном):
в /mission/status добавлено `zekf=` — z /mavros/local_position/pose «глазами
EKF3» (DroneState.ekf_z, пишется в _on_lpos ros_telemetry.py рядом с пульсом
свежести). Пара к `alt=` (rel_alt миссии: баро при BS_ALT_SRC=baro).
HudRenderer рисует «ALT baro X.Xm ekf X.Xm»; расхождение >0.5 м — ЖЁЛТЫМ:
занижение вертикали EKF — механизм удушения IPM-гейта демпфера у земли
(прогон 174603: EKF z −0.27 м → гейт alt<0.5 на истинных 0.7 м). Протухший
local_position (после GPS-kill) → честное `zekf=--`, не последнее значение;
старые bag без поля → «ekf --». 16 офлайн-тестов зелёные.

**Баннер яруса + блок лесенки вместо «VINS READY/WAIT»** (2026-08-30, ветка
nn2_c3_laptop_yaw, ещё НЕ доказан живым прогоном; синтетика 8 состояний +
офлайн-тесты зелёные). Мотив (Андрей, разбор ab_noise 00:45–01:00): st/why —
гейт ОДНОГО яруса (LOITER), а назывался именем VINS: по bag VINS шёл 10 Гц
(odom 105→314, age 0.1), «WAIT (extnav)» = очередь зрелости EK3_SRC1_POSXY=6
(ripe_sec 30 + odom>vins_min 300 → переключилась на 320-й), «WAIT (ground)» =
баро 0.3–0.4 < loiter_alt 0.5. В /mission/status добавлены lvl/tier/lat
(правда Freefly.ladder_state() — LadderState в hud.py; None без sf_master),
t1/w1/vmin (ярус 1 = зеркало VinsHandover.vins_ready, vinshold_gate),
lalt/ripe/rsec/rcnt (пороги для ПРОГРЕССА «extnav 250/300 28/30s», «ground
0.3<0.5m»). HudRenderer: баннер ТОЛЬКО «TIER n NAME» + цвет (просьба Андрея:
зел = ярус == потолок SC; жёлт = ждёт следующий / гистерезис; красн = следующий
мёртв или FCU REFUSES >5 с латча; белый = MANUAL); причины — ТОЛЬКО в блоке
под строкой режима «<MODE> ARM SC n TIER n» (шапка красная «MANUAL (SF)»):
три строки ярусов, активный — заливка, выше потолка — белый, «>» = ярус,
которого лесенка ждёт (его текст = «почему не выше»). Строка ALT (три
высоты) вынесена из левой стопки и заякорена по ЦЕНТРУ НИЗА кадра
(_line_bottom_center, отступ 14 px @1280) — просьба Андрея 2026-08-30. Блок
режима+ярусов (_draw_mode_block) стоит СРАЗУ под баннером яруса, res/rat и
IPM — после него (та же просьба); порядок на экране ≠ номера разделов hud.md.
ИТОГОВАЯ РАСКЛАДКА 2026-08-30 (три зоны, hud.md §2): ЛЕВАЯ стопка — режимы
(ARMED/EKF → TIER → режим FCU + ярусы → DRIFT → scene); ПРАВАЯ стопка
(_line_right, якорь по правому краю сверху) — датчики: IPM → res/rat → FEAT →
ODO → CMD; НИЗ по центру — ALT. Шрифты всего HUD ×0.7 (FONT_K в
hud_renderer.py — глифы, толщина, подложка, шаг; просьба Андрея «на ~30 %
мельче»; 1.0 = прежний размер). FPV читается хуже scene_hud.mp4 по
построению: стример 640×360 (k=0.5) + openh264 4 Мбит/с, mp4 — 960×540
(RES профиля, k=0.75); фикс 2026-08-30 — стример рисует HUD ПОСЛЕ resize
(нативная растеризация, точки фич ×out_width/width), рамки NN1 — до. Не
сделано (по выбору Андрея): отдельный hud_font стримера, out_width 960. Без tier= (старый bag, легаси) —
«LOITER READY/WAIT (why)» (прежний баннер под честным именем). Живость VINS
в HUD — ТОЛЬКО ODO. joy_timeline: «HUD: LOITER READY» + «HUD: ярус n …»;
phase_stats по st=READY не сломан. Живой стример берёт рендерер из
colcon-install → подхватит на следующем restart-all (nav_up.sh собирает).

В очереди: калибровка порогов joy_timeline (GESTURE_LVL/MIN, --eps);
доказать EKF-баннер, строку ALT и баннер яруса живым прогоном.
Связано: [[joystick-replay-series]], [[lv-loiter-series]].

Урок пилоту (подтверждён сравнением 222539 vs 232043): раскачка для init VINS —
трансляции при фиксированном курсе, yaw короткими импульсами МЕЖДУ ними;
непрерывный yaw = init не наступает вовсе.
