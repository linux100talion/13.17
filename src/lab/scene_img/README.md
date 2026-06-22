# scene_img — кадры камеры дрона из симуляции

30 кадров с `/image_color` (camera_node, bgr8, 1280×720), снятых при облёте
квадрата 5×5м на высоте 4м (`fly_square.py`). Шаг ~1с между кадрами.

Назначение: диагностика инициализации VINS — посмотреть, что реально видит
камера в мире `mili_fortress`.

Извлечены из rosbag `/image_color` скриптом `docker/sim/output/extract_frames.py`.
