# RViz2: просмотр bag'а прогона (путь, стрелка носа, видео камеры)

## Команды

```bash
# один прогон, скорость 1 (с хоста, ROS jazzy; стек сима гасить не нужно)
bash src/lab/bag_rviz.sh docker/sim/output/joystick/lv2_joy_20260906_231055

# то же по каталогу bag / по .db3
bash src/lab/bag_rviz.sh docker/sim/output/joystick/<RUN>/bag
bash src/lab/bag_rviz.sh docker/sim/output/joystick/<RUN>/bag/scene_bag_0.db3

# скорость ×2, с 30-й секунды, по кругу
RATE=2 START=30 LOOP=1 bash src/lab/bag_rviz.sh docker/sim/output/joystick/<RUN>

# свой конфиг rviz / другой домен ROS / хвост аргументов → ros2 bag play
RVIZ_CFG=~/my.rviz DOMAIN=7 bash src/lab/bag_rviz.sh <RUN> --topics /image_color /model/iris_cam/odometry
```

В окне `ros2 bag play`: `SPACE` — пауза, `→` — шаг, `↑`/`↓` — скорость ±10 %.
`Ctrl+C` в терминале гасит реплей, RViz и помощников. Вид сверху — панель
Views → «Top-down».

Вручную, без скрипта (эквивалент):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42                      # НЕ 0 — там живой Gazebo
$(ros2 pkg prefix tf2_ros)/lib/tf2_ros/static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id world --child-frame-id map &
python3 src/lab/bag_path_pub.py &            # Odometry/PoseStamped → /truth/path /vins/path /ekf/path
rviz2 -d src/lab/bag_view.rviz &
ros2 bag play docker/sim/output/joystick/<RUN>/bag     # БЕЗ --clock
```

Опционально, чтобы player не ругался на `/mavros/state`:
`sudo apt install ros-jazzy-mavros-msgs` (RViz этот топик не нужен).

## Что рисуется

| Дисплей | Топик | Цвет | Примечание |
|---|---|---|---|
| Image | `/image_color` | — | полный BGR-кадр; TF не нужен (2D-окно); QoS Reliable, как записано |
| Odometry «Truth (Gazebo)» | `/model/iris_cam/odometry` | зелёный | стрелка = нос (ось x тела), шлейф Keep 3000, шаг 0.3 м |
| Path «Truth path» | `/truth/path` | зелёный | линия из `bag_path_pub.py` |
| Odometry «VINS» | `/odometry` | красный | СВОЙ `world`: начало = точка init, курс первого кадра |
| Path «VINS path» | `/vins/path` | красный | |
| Pose «EKF pose» | `/mavros/local_position/pose` | голубой | frame `map` → статический TF `world→map`; QoS **Best Effort** (так публикует MAVROS, с Reliable молчит) |
| Path «EKF path» | `/ekf/path` | голубой | |

Fixed Frame — `world`. Сетка 5 м, оси в начале мира. TF-дисплей выключен
(включить в списке — увидеть `world`/`map`).

## Разбор: почему был «Detected jump back in time. Resetting RViz»

Исходная попытка: `rviz2 --ros-args -p use_sim_time:=true` + `ros2 bag play ./ --clock`.
Причины две, обе про время.

**1. Два источника `/clock`.** Стек симуляции работает на host-сети
(`network_mode: host`, `ROS_DOMAIN_ID=0`), поэтому живой Gazebo через
`ros_gz_bridge` светит на хост `/clock` — sim-секунды с запуска (в момент
разбора ≈ 6300 с, ~250 Гц) — и живые `/model/iris_cam/odometry`, `/odometry`.
`ros2 bag play --clock` добавляет второй `/clock` с эпохой записи
(1788725217…). RViz с `use_sim_time` берёт оба вперемешку → на каждом кадре
время «назад» → предупреждение и сброс. Заодно живой борт из Gazebo рисуется
поверх реплея. Лечение — свой `ROS_DOMAIN_ID` для реплея (скрипт: `DOMAIN`,
default 42); стек гасить не обязательно.

**2. Два времени в самом bag'е.** Штамп ЗАПИСИ сообщения (по нему играет
player и по нему считает `--clock`) — wall-эпоха ноута (1788725217…341,
124 с). Заголовки сообщений (`header.stamp`) — sim-время Gazebo (0…124 с,
ноды в `use_sim_time`). Для просмотра ни то ни другое не нужно: TF только
статический (без времени), Image без TF, Fixed Frame = frame сообщений
(`world`) — поэтому ни `--clock`, ни `use_sim_time` не передаём, RViz живёт
на wall-time.

## Оговорки

- **`/image_mono`, `/path`, `/goal_pose` в bag'е нет** — исходный конфиг
  `rviz.conf.rviz` смотрел в них и молчал. В bag'е freefly: `/image_color`,
  `/model/iris_cam/odometry` (истина), `/odometry` (VINS),
  `/mavros/local_position/pose` (EKF), `/feature`, `/flow_dbg*`, `/joy`,
  `/mavros/state`, `/mission/status`, `/mavros/imu/data`, `/nn1/bridge`,
  `/vins/sane` (`TOPICS_EXTRA` в `freefly_lv.sh`). Линию пути RViz строит только
  из `nav_msgs/Path`, из Odometry — лишь стрелки; отсюда `bag_path_pub.py`.
- **VINS и истина в разных «world»** при одном имени frame: `/odometry` VINS —
  от точки init с курсом первого кадра, `/model/iris_cam/odometry` — мир Gazebo.
  `FrameAnchor` в `ray_tracer` выравнивает только то, что идёт в EKF; сырой
  `/odometry` в bag'е не выровнен. Красная линия смещена/повёрнута относительно
  зелёной по построению (в 231055 почти совпали — спавн у начала мира на восток).
- **`/feature` не показывать как PointCloud в мире**: точки там — нормированные
  координаты изображения (z = 1), не 3D; в конфиг не включён.
- **QoS:** `/mavros/local_position/pose` и `/mavros/imu/data` записаны Best
  Effort — подписка Reliable не сматчится (дисплей молчит без ошибки). Остальные
  топики Reliable. `bag_path_pub.py` подписан Best Effort на всё (совместимо с
  обоими).
- **`/mavros/state`** player в jazzy пропускает (`mavros_msgs` на хосте нет) —
  для RViz не нужен.
- **Path тяжёлый на высоком темпе:** 20k поз × 45 Гц — десятки МБ/с, поэтому
  `bag_path_pub.py` публикует не чаще 5 Гц, прореживает по 5 см и обнуляет
  траекторию при скачке штампа назад > 1 с (`LOOP=1`, перезапуск реплея).
- **`set -u` и `setup.bash` ROS** не дружат (`AMENT_TRACE_SETUP_FILES` не
  задана) — в скрипте сорсинг обёрнут `set +u … set -u`.
- **`ros2 run` и Ctrl+C:** TERM python-обёртке `ros2 run` до её ребёнка не
  доходит — `static_transform_publisher` оставался жить. Скрипт зовёт бинарник
  напрямую (`$(ros2 pkg prefix tf2_ros)/lib/tf2_ros/static_transform_publisher`).
- **Бортовые bag'и Jetson** (`distro/`, auto-bag) пишут `/image_mono`,
  `/camera_info`, `/path`, IMU raw — другой набор, для них конфиг нужен свой
  (см. `distro/CLAUDE.md`; в `distro/doc/cmd.txt` — старые заметки RViz: Fixed
  Frame = `camera_frame`, Best Effort на Image).

Файлы: `src/lab/bag_rviz.sh` (запуск), `src/lab/bag_path_pub.py` (Path),
`src/lab/bag_view.rviz` (конфиг). Описание — строка в таблице
`src/lab/CLAUDE.md` («Диагностические инструменты»).
