# Калибровка бортовой камеры и связки камера–IMU (Kalibr → VINS-Mono)

Бортовой `config_pkg/config/dummy_13_7.yaml` до сих пор несёт **заглушки**: интринсики
сима (`fx=fy=640, cx=640, cy=360`, дисторсия 0), `extrinsicRotation` = единичная,
трансляция «на глаз», шумы IMU «×10 от чипа». Всё это надо ИЗМЕРИТЬ, прежде чем VINS
полетит на реальном борте. Инструмент — **Kalibr**; VINS-Mono съедает его выход
напрямую (модель `pinhole-radtan` = наш `PINHOLE` с `k1 k2 p1 p2`).

## Версии — закреплены, чтобы мишень, калибратор и потребитель совпадали

| Что | Где | Версия |
|---|---|---|
| Kalibr (исходники) | `/home/andriy/kalibr` (клон на хосте, вне репо — как форк VINS) | `ethz-asl/kalibr` @ **`1f60227`** (master, 2024-03-08) |
| Kalibr (образ) | `docker images kalibr:1f60227` | собран из того же клона, `Dockerfile_ros1_20_04` (ROS1 noetic) |
| Мишень | `aprilgrid_a4.pdf` + `aprilgrid_a4.yaml` (этот каталог) | сгенерирована `kalibr_create_target_pdf` того же коммита |
| Потребитель | `src/vins/VINS-MONO-ROS2/config_pkg/config/dummy_13_7.yaml` | форк `1317_debug` |

Пересборка образа / регенерация мишени:
```bash
cd /home/andriy/kalibr && git checkout 1f60227
docker build -t kalibr:1f60227 -f Dockerfile_ros1_20_04 .
docker run --rm -v $PWD:/out --entrypoint bash kalibr:1f60227 -c \
  "source /catkin_ws/devel/setup.bash && cd /out && \
   rosrun kalibr kalibr_create_target_pdf --type apriltag --nx 6 --ny 8 --tsize 0.023 --tspace 0.3 aprilgrid"
```
(ENTRYPOINT образа — shell-форма, аргументы глотает → всегда `--entrypoint bash`;
утилиты лежат в `devel/lib/kalibr/`, зовутся через `rosrun kalibr …`. PDF генератора —
размером с сетку, 191×251 мм; в репо он положен на настоящий лист A4 через
ghostscript `PageOffset [26.5 65]`, геометрия тегов не меняется. Сверено 2026-09-16:
растр 150 dpi из образа и из локального pyx — 0 отличных пикселей.)

## Мишень: печать

- `aprilgrid_a4.pdf` — AprilGrid **6×8**, тег 23 мм, промежуток 6.9 мм (0.3 стороны),
  семейство t36h11, на листе A4 портрет.
- Печатать **в масштабе 100 % / «фактический размер»**, НЕ «вписать в страницу».
- Наклеить на жёсткое ровное (стекло, МДФ, пенокартон) — прогиб листа = ошибка.
- **Измерить линейкой** сторону чёрного квадрата тега (номинал 23.0 мм) и вписать в
  `aprilgrid_a4.yaml` → `tagSize` (в метрах). `tagSpacing` — отношение, от масштаба
  печати не зависит.
- A4 хватает для интринсиков и для связки камера–IMU на дистанции 0.4–1 м. Если
  есть A3/плоттер — та же команда с `--tsize` побольше, всё остальное без изменений.

## Пайплайн (по шагам, ниже — по мере прохождения)

1. **Bag на Jetson** (ROS2): `/image_mono` от `camera_node` (реальная камера) +
   `/mavros/imu/data_raw` (200 Гц — `start_mavros.sh` запрашивает HIGHRES/RAW_IMU).
   Два прогона: (а) только камера, мишень неподвижна, камера медленно обходит все
   ракурсы и углы кадра; (б) камера+IMU, 60–90 с, возбуждение всех 6 осей, без
   смаза, хороший свет.
2. **ROS2 → ROS1 bag**: `pip install rosbags` → `rosbags-convert --src <bag_dir> --dst
   <file>.bag` (Kalibr — ROS1).
3. **Интринсики**: `kalibr_calibrate_cameras --bag cam.bag --topics /image_mono
   --models pinhole-radtan --target aprilgrid_a4.yaml` → `camchain-*.yaml`:
   `intrinsics [fu fv pu pv]` → `fx fy cx cy`, `distortion_coeffs [k1 k2 p1 p2]`.
4. **Шумы IMU** (Аллан): статичный bag IMU 2–3 ч → `allan_variance_ros` →
   `acc_n gyr_n acc_w gyr_w` (в `imu.yaml` Kalibr — те же, ×5–10 по их рекомендации
   для шага 5; в `dummy_13_7.yaml` — измеренные).
5. **Камера–IMU**: `kalibr_calibrate_imu_camera --bag imucam.bag --cam camchain.yaml
   --imu imu.yaml --target aprilgrid_a4.yaml` → `T_cam_imu` и `timeshift_cam_imu`.
6. **Перенос в `dummy_13_7.yaml`**: Kalibr даёт `T_cam_imu` (точки IMU → камера), VINS
   ждёт обратное — `extrinsicRotation/Translation` = **`inv(T_cam_imu)`** (камера →
   IMU/body). `td` = `timeshift_cam_imu` (обе стороны: `t_imu = t_cam + shift`).
   ⚠️ Транспонирование матрицы здесь уже ловили в симе (память `vins-solver-fix`) —
   проверить по знаку: камера смотрит вперёд-вниз, ось Z камеры ≈ +X борта.

Собственный калибратор VINS-Mono (`camera_model/Calibration`, шахматка через OpenCV)
даёт только интринсики — при Kalibr не нужен; для перекрёстной проверки шахматку
печатает тот же `kalibr_create_target_pdf --type checkerboard`.
