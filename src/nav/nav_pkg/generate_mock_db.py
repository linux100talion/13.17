#!/usr/bin/env python3
import json
import os
import numpy as np

# Настройки нашей фейковой базы
DB_DIR = "reference_db"
os.makedirs(DB_DIR, exist_ok=True)

# Формируем словарь как в ТЗ
database_dict = {
    "crossroad_view_001.npz": {"id": "crossroad_1", "lat": 47.111, "lon": 37.222, "alt": 150.0},
    "crossroad_view_002.npz": {"id": "crossroad_1", "lat": 47.111, "lon": 37.222, "alt": 150.0},
    "crossroad_view_003.npz": {"id": "crossroad_1", "lat": 47.111, "lon": 37.222, "alt": 150.0},
    "bunker_view_001.npz": {"id": "bunker_1", "lat": 47.333, "lon": 37.444, "alt": 145.5},
    "bunker_view_002.npz": {"id": "bunker_1", "lat": 47.333, "lon": 37.444, "alt": 145.5},
    "bunker_view_003.npz": {"id": "bunker_1", "lat": 47.333, "lon": 37.444, "alt": 145.5},
    "bush1_view_001.npz": {"id": "bush_1", "lat": 47.555, "lon": 37.666, "alt": 160.2},
    "bush1_view_002.npz": {"id": "bush_1", "lat": 47.555, "lon": 37.666, "alt": 160.2},
    "bush1_view_003.npz": {"id": "bush_1", "lat": 47.555, "lon": 37.666, "alt": 160.2}
}

# Сохраняем database.json
with open(os.path.join(DB_DIR, "database.json"), "w", encoding="utf-8") as f:
    json.dump(database_dict, f, indent=4)

# Генерируем фейковые файлы .npz (имитация выхлопа SuperPoint)
# Реальный SuperPoint выдает: 
# - keypoints: координаты точек (X, Y)
# - descriptors: векторы признаков (256 мерные)
# - scores: уверенность сети
for filename in database_dict.keys():
    filepath = os.path.join(DB_DIR, filename)
    
    # Имитируем, что на картинке найдено 500 уникальных точек
    num_keypoints = 500
    
    # Фейковые пиксельные координаты (X, Y) на картинке 640x480
    keypoints = np.random.rand(num_keypoints, 2) * [640, 480]
    
    # Фейковые дескрипторы (массив 256 признаков для каждой из 500 точек)
    descriptors = np.random.rand(256, num_keypoints).astype(np.float32)
    
    # Уверенность сети для каждой точки
    scores = np.random.rand(num_keypoints).astype(np.float32)
    
    # Сохраняем в сжатый numpy-архив
    np.savez(
        filepath, 
        keypoints=keypoints, 
        descriptors=descriptors, 
        scores=scores
    )

print(f"✅ База успешно сгенерирована в папке '{DB_DIR}'")
print(f"Сгенерировано файлов .npz: {len(database_dict)}")
print("Готово к интеграции с LightGlue и ROS 2!")
