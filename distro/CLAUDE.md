# distro/ — деплой на реальный борт (Jetson Orin Nano)

Локальный контекст каталога. Архитектура бортового стека — в корневом `CLAUDE.md`
(раздел «Бортовой стек — `docker/orin/`»). Здесь — как работаем с бортом, что лежит,
в каком он состоянии и чем бортовая запись bag отличается от симуляционной.

## Как работаем с бортом (с 2026-09-16, ветка `laptop_drone`)

`distro/` — **источник правды для конфигурации Jetson** (юниты systemd, профили
NetworkManager, скрипты, бинарники, всё вне контейнера). Цикл всегда один:

```
правка ЛОКАЛЬНО в distro/  →  ./deploy.sh -n (что изменится)  →  ./deploy.sh [секции]
   →  проверка по ssh (read-only)  →  коммит
```

- **На борту руками НИЧЕГО не менять** — ни файлы, ни `nmcli`, ни `systemctl
  enable`, ни `apt`. Иначе `distro/` перестаёт совпадать с бортом, а «снимок ≠ борт»
  = ложные диагнозы (тот же принцип, что дисциплина прогона в симе). Исключение
  сделал — сразу перенеси в `distro/` и задеплой, чтобы `./deploy.sh -n` был чист.
- По ssh на борту можно: смотреть (логи, `systemctl status`, `ls`, `docker ps`),
  запускать/останавливать сервисы, снимать bag. Нельзя: писать в `/etc`, `/usr`,
  `~/` мимо deploy.sh.
- Обратная сверка (борт → ноут): `rsync` покрываемых путей в scratch + `diff -rq`
  с `distro/`. Делали 2026-09-16: расхождений 0, борт на уровне 2 июня.
- `deploy.sh` без `--delete`: лишнее на борту не трогает, удаление файла — руками
  через `-X 'rm …'` и из `distro/` одновременно.

**Доступ.** Юзер `andriy`. С 2026-09-21 **основной путь — Wi-Fi через встроенный
адаптер** `wlP1p1s0` (MAC `f8:3d:c6:57:34:65`; в TP-Link_6611 — `192.168.0.104`, DHCP):
на ноуте `ssh jetson` (алиас в `~/.ssh/config` → `192.168.0.104`), `deploy.sh` без `-H`
сам пробует `192.168.0.104` → `jetson.local` → `192.168.55.1`. USB-линк `192.168.55.1`
(l4tbr0) — резерв, когда кабель воткнут. IP по DHCP может уехать — на роутере сделать
**Address Reservation** для `f8:3d:c6:57:34:65` → `.104` (и Alfa `00:c0:ca:b9:55:0c`
→ `.105`); `jetson.local` (avahi на борту, nss-mdns на ноуте) работает как запасное
имя. Ключ `doc/ssh-keys/jetson` (в `.gitignore`). **Пароль sudo на борту: `ok`**
(ценности не представляет — борт доступен только по USB/локалке). `rsync` в sudoers
NOPASSWD (секции etc/usr), остальное root'ом deploy.sh делает через `sudo -S`
(`-r`, `-X`; пароль — `$JETSON_SUDO`, дефолт `ok`).

**Wi-Fi.** Профили — `etc/NetworkManager/system-connections/*.nmconnection`
(с паролями, решение оставить в репо). Новая сеть = новый файл по образцу
(свой uuid, `autoconnect-priority` ниже домашних), `./deploy.sh -r etc` (reload
NM — иначе новый файл не подхватится), `-X 'nmcli connection up <id>'`.
Файлы едут строго `600 root` (`--chmod=F600`; git 600 не хранит, NM с 644 профиль
молча игнорирует).

**Wi-Fi Alfa AWUS036ACH (RTL8812AU) — `doc/HW/alfa.md`.** Драйвера в ядре борта нет —
ставится `usr/local/sbin/setup-rtl8812au.sh` (DKMS, `./deploy.sh -X … usr etc`, после
обновления ядра повторить); опции `etc/modprobe.d/8812au.conf`. Там же: почему
micro-USB 2.0 кабель в USB 3.0 micro-B гнезде даёт `error -71`, как отличить кабель,
почему адаптер сидит на USB 2.0 (норма), причуды драйвера (пустой `iw scan`, диод, `txpower`)
профиль NM `TP-Link_6611_alfa` (Alfa `wlx00c0cab9550c`; **метрика 700 — резерв**, встроенный 600 основной: Alfa молча терял линк без события disconnect и, будучи основным маршрутом, глотал ответы на `.104`), `rtw_power_mgnt=0`, `rtw_ips_mode=0`, NM `powersave=2`; бустер EDUP EP-AB025 — **только 5.8 ГГц** (макс. вход 20 dBm, порог 3), мощность Alfa 15 dBm держит dispatcher `etc/NetworkManager/dispatcher.d/50-alfa-txpower`; TODO — кабель, обрывы (тест мимо бустера), 5.8 ГГц на обеих сторонах.

**Что на борту устарело (снимок 2026-09-16, всё на уровне май–июнь):**

| Есть на борту | Должно быть (репо) |
|---|---|
| контейнер `vins_project_13_7` из образа `vins_ws-vins_core` — старый `home/andriy/vins_ws/Dockerfile` (`arm64v8/ros:humble-ros-base-jammy`, **без CUDA**), монтирует только `~/vins_ws/src` | `docker/orin/` — `dustynv/ros:humble-ros-base-l4t-r36.3.0`, `runtime: nvidia`, монтирует `src/vins`, `src/camera`, `src/nav` (+ добавить `src/control`, `src/mission`) |
| VINS: апстрим `dongbo19` ветка `main` (4c3cf08) с ручными незакоммиченными правками в `~/vins_ws/src/VINS-MONO-ROS2` | форк `linux100talion`, ветка `1317_debug` |
| камера: python `cam_node.py` через `vins_service.sh` | C++ `camera_node` (`src/camera/`) + `openhd_streamer` (`vins_service.sh` из `634119d`) |
| `control_pkg`, `mission_pkg`, `nav_pkg` — нет вообще | `src/control`, `src/mission`, `src/nav` |
| `~/workspaces/isaac_ros-dev` (166 МБ) — остатки; образы isaac_ros 40 ГБ **удалены 2026-09-16** | — |

Шаги деплоя кода (TODO, по порядку): секция `src/` в deploy.sh под bind mount →
`docker/orin/` вместо `vins_ws/{Dockerfile,compose}` + пересборка образа → клон форка
`1317_debug` → актуальные `vins_service*.sh` → `colcon build` в контейнере.

## Что где лежит

```
deploy.sh                    — rsync секций home/ etc/ usr/ на Jetson: -n dry-run, -r reload
                               NM/systemd, -x/-X команда после деплоя (andriy/root); шапка = usage
etc/NetworkManager/system-connections/ — Wi-Fi-профили (600 root, см. «Wi-Fi» выше)
etc/systemd/system/          — юниты: mavros, vins / vins_m, auto-bag / auto-bag-m, orin-shutdown
home/andriy/hw_check.sh      — проверка железа через полётник (MAVLink, только чтение):
                               секции baro (оба барометра I2C2), compass (QMC5883L: найден,
                               поле, шум, калибровка), gps (M9N: фикс, спутники); шапка = usage
home/andriy/mavlogs/         — auto_bag.sh / auto_bag_m.sh — запись bag (см. ниже)
home/andriy/vins_ws/         — vins_service*.sh (старые: python cam_node.py, без стримера),
                               Dockerfile/compose, конфиги VINS, древний camera_node.cpp
home/andriy/simple_cam/      — стримеры и профили камеры, tuner plus/cuda
home/andriy/workspaces/      — остатки isaac_ros (драйвер камеры Argus не поддерживает)
usr/local/bin/               — start_mavros.sh (MAVROS + запрос HIGHRES_IMU/RAW_IMU 200 Гц)
usr/local/sbin/              — setup-rtl8812au.sh: DKMS-драйвер Alfa AWUS036ACH (см. «Wi-Fi Alfa»)
etc/modprobe.d/8812au.conf   — опции модуля 8812au (сны выключены)
etc/NetworkManager/dispatcher.d/50-alfa-txpower — txpower Alfa 15 dBm под бустер на каждом up
doc/                         — заметки: cmd.txt, cam.txt, wifi, ssh config
doc/HW/                      — ЖЕЛЕЗО: hw.txt (паспорт борта), фото/мануалы (TX12, M9N);
                               Ardupilot_Params/ — параметры полётника (MP/stellar_cld.txt) и плата;
                               alfa.md — Wi-Fi Alfa AWUS036ACH: кабель/USB, драйвер, причуды, TODO;
                               esc/ — настройки ESC Flycolor X-Cross HV3 (xcross_hv3.ixi) + как снять/восстановить
doc/ssh-keys/jetson, doc/wifi.txt — СЕКРЕТЫ, в .gitignore (репо публичный)
```

Суффикс `_m` у юнитов/скриптов = ручной режим без ожидания арминга.

## Запись bag на борту (auto-bag) vs в симуляции (freefly_lv → capture_scene)

Разбор 2026-09-07. Две пары «юнит + скрипт» на хосте Jetson (не в контейнере),
обе вызывают `ros2 bag record`; во всём остальном подходы расходятся.

**Борт, `auto_bag.sh` (`auto-bag.service`):** ждёт `/mavros/state` поллингом
`ros2 topic list`, затем читает `ros2 topic echo /mavros/state | grep armed:`
через process substitution (PID рекордера остаётся в основном шелле). На
`armed: true` — `ros2 bag record -o /home/andriy/mavlogs/bag_YYYYmmdd_HHMMSS …`,
на `false` — SIGINT рекордеру и wait. Каждый цикл арм/дизарм = отдельный bag.
Trap на `systemctl stop`: SIGINT рекордеру, wait, `pkill -9` зависающему
`ros2 topic echo`. **`auto_bag_m.sh`:** без гейта арминга — пишет сразу, как
появился MAVROS, один bag на запуск сервиса. Окружение задаётся явно (systemd
не читает `.bashrc`): `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=0`, CycloneDDS,
только `/opt/ros/humble` — воркспейс VINS не нужен, фичи едут в стандартном
`sensor_msgs/PointCloud`. Юниты: `User=andriy`, `KillSignal=SIGINT`,
`TimeoutStopSec=3`, `Restart=on-failure`, зависимость только `network.target`
(порядок с mavros/vins не задан — потому и поллинг).

| Аспект | Борт (distro) | Сим (`src/lab/freefly_lv.sh` → `capture_scene.sh`) |
|---|---|---|
| Кто запускает | systemd, юзер andriy, на хосте | `capture_scene.sh` с ноута через `docker exec nav`, root в контейнере |
| Старт | `armed: true` (в `-m` — сразу при MAVROS) | Перед всей последовательностью команд, ещё до арма, +3 с форы |
| Стоп | `armed: false` → SIGINT; `systemctl stop` → trap | После последовательности: `pkill -INT -f "ros2 bag record"`, 2 с |
| Bag на прогон | По одному на цикл арм/дизарм | Ровно один `scene_bag`, прошлый чистится на старте |
| Куда | `/home/andriy/mavlogs/bag_<дата>`, навсегда | `output/scene_bag` → `output/joystick/<имя>/bag` (mv изнутри контейнера, `KEEP_BAG=1`) |
| Окружение | humble, CycloneDDS, `LOCALHOST_ONLY=1` | humble + overlay cv_bridge + `sim_ws/install` |
| Топики | 7: `/mavros/imu/data_raw`, `/mavros/imu/data`, `/image_mono`, `/camera_info`, `/odometry`, `/path`, `/feature` | ~19: `/image_color`, `/mavros/local_position/pose`, `/joy`, `/mavros/state`, `/mission/status`, `/feature`, `/odometry`, `/model/iris_cam/odometry` (истина), `/flow_dbg{,2,6,7,8,9,10}`, `/mavros/imu/data`, `/nn1/bridge`, `/vins/sane` (`TOPICS_EXTRA`) |
| Картинка | `/image_mono` (вход VINS) | `/image_color` (полный BGR) → scene.mp4 / scene_hud.mp4 / scene_ipm.mp4 |
| Пост-обработка | Нет | mp4, HUD-рендер, IPM-видео, кадры, Google Drive, `analyze.sh`, мета `.env`, лог порывов |
| Ручки | Нет | `RECORD`, `SKIP_CAM`, `TOPICS_EXTRA`, `KEEP_BAG` |
| Владелец файлов | andriy | root → `chown` в trap EXIT |
| Штампы времени | wall-time | sim-time из `/clock` |

### Что это значит для этапа деплоя

- **Разбор несовместим.** Инструменты `src/lab/` (joy_timeline, hud_video,
  ipm_video, analyze.sh, phase_stats) кормятся `/image_color`, `/mission/status`,
  `/flow_dbg*`, `/mavros/state`, `/joy` — в бортовых bag'ах их нет, ни один
  разбор на них не запустится. Обратно: сим не пишет `/image_mono`,
  `/camera_info`, `/path`, `/mavros/imu/data_raw`. Список топиков надо свести
  к одному (`TOPICS_EXTRA` сима — отправная точка).
- **Земля до арма теряется.** `auto_bag.sh` пишет только между армом и
  дизармом, сим — весь прогрев и стояние на земле. Предармовое поведение VINS
  на земле было корнем проблемы `vins-ground-stand-init` (память) — борт его не
  зафиксирует. `auto_bag_m.sh` пишет всё, но без привязки к полёту.
- **Объём.** `/image_color` bgr8 1280×720 @30 fps ≈ 9 ГБ за прогон (базовый
  датасет joystick/base: 24 прогона ≈ 217 ГБ). На Jetson писали `/image_mono`
  втрое компактнее осознанно. Выбор при переносе: mono + HUD-топики, либо color
  с лимитом по времени/сжатием.
- **Нет меты прогона.** Сим кладёт рядом с bag'ом `.env` со всеми `BS_*` и
  `joy.log`; на борту — только timestamp в имени. Нужен дамп окружения
  лётной ноды рядом с bag'ом.
- **Остановка — гонка.** На борту дизарм одновременно гасит запись (SIGINT из
  `auto_bag.sh`) и через `orin_shutdown` делает `shutdown -h now`; systemd даёт
  3 с на закрытие bag'а — на большом sqlite может не успеть, гарантии
  сохранности последнего bag'а нет. В симе стоп явный и упорядоченный, но если
  прогон упал между стартом и стопом записи, trap EXIT рекордер не трогает —
  зачищает следующий `restart-all`.
- **Мелочь:** в `auto_bag.sh` строка `BAG_PID=$!` продублирована (83–84),
  безвредно. `doc/cmd.txt` советует добавить `/tf /tf_static` — нужны RViz.

## ЗАДАЧА ПЕРЕД ДЕПЛОЕМ: чем заменить зелёную линию (истину Gazebo)

Разбор 2026-09-09 по кампании возврата (`cmd/rth_track`, `src/mission/rth.md`). В симе
у нас ТРИ линии: зелёная — истина Gazebo, красная — VINS, голубая — EKF полётника. На
борту **зелёной нет**, и это не косметика.

**Что на борту чем является.** Красная (VINS) — единственный ИСТОЧНИК позиции: врёт она
→ врёт всё. Но летит борт по ГОЛУБОЙ: уставки GUIDED идут в раме EKF, трек возврата
записан в ней же, а голубая = красная через мост (`FrameAnchor`) и EKF (фильтр + ИНС
между кадрами 10 Гц → 400 Гц). Зелёной — оракула, по которому мы всё это судили, — не
будет.

**Чем это опасно.** Все числа последних разборов («промах по истине», «дрейф EKF»,
«масштаб VINS 1.00», «поворот рамы 0.4°») на борту НЕИЗМЕРИМЫ в полёте. Останутся только
самосогласованные метрики: след возврата (совпадение с уходом ПО НАШИМ ЖЕ цифрам), гейт
здоровья (VINS против IPM), баро против `zekf`. Систематическая ошибка — уплывший
масштаб или повёрнутая рама — на борту выглядит ИДЕАЛЬНО: красивый след, `rth=ready`,
борт «вернулся домой» в своих координатах, а физически он в другом месте. Ровно этого мы
не заметили бы в 073004 без зелёной линии.

**Что делать (в порядке цены):**

1. **Пассивный GPS как молчаливый оракул.** Сейчас на борту `GPS1_TYPE=0` — приёмника
   нет вовсе. Поставить и НЕ включать в EKF (`EK3_SRC*` не трогаем, GPS только в лог/bag)
   — получим ту самую зелёную линию для разбора после полёта, не меняя навигацию ни на
   грамм. Для доводки VINS и возврата окупается сразу.
2. **Рулетка**: расстояние «точка взлёта → точка посадки» как прямой аналог «промаха по
   истине». Грубо, но независимо.
3. **NN1** — по проекту именно она даёт абсолютную привязку В ПОЛЁТЕ; пока её нет, дрейф
   в воздухе не наблюдаем в принципе.

**Что ещё добавится на борту (проверить при деплое):**

| | в симе | на борту |
|---|---|---|
| курс рамы EKF | идеальный | **компас** (`EK3_SRC1_YAW=1`, `COMPASS_USE=1`): его уход поворачивает и трек, и возврат — а увидеть это нечем |
| IMU | заниженные шумы `sim.yaml` | вибрации рамы, бортовой `dummy_13_7.yaml` — init VINS капризнее |
| гейн канала IPM | измерен 0.5–0.77 | **перемерять**: от него зависят круг латча RTH и чек занижения |
| пороги гейтов | калиброваны в симе (`BS_VINS_HOVER_V` 3, `_V_MAX` 12, `_SCALE_ALT_MAX` 25) | проверить на реальных скоростях |
| режимы | `FLTMODE_CH=0` | **включён (CH5)**: пилот вырывает режим сам — шаг возврата это уважает (`RTH_EJECT`) |
| потеря компаньона | — | нет позиции → ни GUIDED, ни RTL; остаётся ручной полёт по углам (`src/control/gates.md`) |
