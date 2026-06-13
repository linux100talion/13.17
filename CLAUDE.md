# Проект 13.17 — Автономный дрон без GPS

Бортовой компьютер: **NVIDIA Jetson Orin Nano**. Навигация без GPS на основе компьютерного зрения и VINS-Mono.

## Архитектура (Docker)

Три контейнера в `docker-compose.yml`:

- **simulator** — Gazebo Harmonic + ArduPilot SITL + ROS 2 Humble. Рендер мира, физика, виртуальная камера 1920×1200 (BAYER_GBRG8).
- **mavlink_router** — раздаёт MAVLink телеметрию без конфликтов портов: QGC на :14550, MAVROS на :14540, C++ код на :14541.
- **ai_brain** — NVIDIA CUDA + ROS 2 Humble (без графики). Весь интеллект: VINS-Mono, нейросети, MAVROS, камерная нода.

## Навигация (два слоя)

**Нейросеть №1 — якорная локализация** (абсолютная точность, замена GPS):
- Модель: YOLOv8 или SuperPoint + LightGlue
- Находит известные ориентиры (столбы, здания, перекрёстки)
- Ray Tracing через intrinsics камеры + барометр/IMU → абсолютная позиция
- Сбрасывает дрейф VINS-Mono

**Нейросеть №2 — топологическая карта** (семантическое понимание):
- Модель: DINOv2 + AnyLoc + FAISS
- Сжимает сцену в глобальный дескриптор, сравнивает с базой разведывательного облёта
- Управляет полётом по смыслу («лети правее леса»)

Нейросеть №2 ведёт дрон, Нейросеть №1 периодически сбрасывает накопленный дрейф.

## Камера — AR0234 (1280×720 / 1920×1200, Bayer GBRG)

### C++ код

**`home/andriy/vins_ws/src/plus/cuda/camera_node.cpp`** — боевая ROS2 нода:
- V4L2 захват → CUDA дебайер (`BayerGB2RGB`) → публикует `mono8` в `/image_mono` для VINS-Mono
- Стримит H.264 через GStreamer на порт 5600 (OpenHD)
- Параметры gain/r/g/b меняются через `ros2 param set` на лету
- Интринсики камеры пока захардкожены (заглушка) — калибровка отложена

**`home/andriy/simple_cam/plus/cuda/main.cpp`** — автономный тюнер без ROS:
- То же V4L2+CUDA, но с HTTP сервером (httplib.h)
- Веб-интерфейс на `:8080` со слайдерами (gain, экспозиция, баланс белого)
- MJPEG: `:5000` (mono), `:5001` (color)

> Разные коды Байера в CUDA (`BayerGB2RGB`) и CPU (`BayerGR2BGR`) — это норма. OpenCV CUDA модуль имеет сдвиг в именовании паттернов относительно CPU версии, оба файла обрабатывают один физический паттерн.

## VINS-Mono

Уже работает при армировании дрона. Калибровка отложена на потом.

Конфиги: `home/andriy/vins_ws/src/VINS-MONO-ROS2/config_pkg/config/`

## Systemd сервисы (на Orin Nano)

- `mavros.service` — MAVROS
- `auto-bag.service` / `auto-bag-m.service` — автозапись ROS bag
- `vins.service` / `vins_m.service` — запуск VINS-Mono
- `orin-shutdown.service` — управление выключением (Go-утилита)

## Сети (NetworkManager)

Home34, Home34_5G, Starlink, Tenda_34 — конфиги в `etc/NetworkManager/system-connections/`

## Репозиторий

- GitHub: `https://github.com/linux100talion/13.17`
- Ветка: `main`
- Git user: Andriy Kutsevol `andriykutsevol@gmail.com`
