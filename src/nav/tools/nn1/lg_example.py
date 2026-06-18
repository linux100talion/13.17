import torch
import numpy as np
from lightglue import LightGlue, SuperPoint
from lightglue.utils import rbd # Вспомогательная функция для удаления batch-измерения

# 1. Инициализация нейросети LightGlue (режим матчинга признаков SuperPoint)
# .eval() отключает обучение, .cuda() переносит модель на GPU Jetson Orin
matcher = LightGlue(features='superpoint').eval().cuda()

# ---------------------------------------------------------
# ЭТАП 1: Подготовка данных из базы (image0 - Эталон)
# ---------------------------------------------------------
# Загружаем файл эталона, который мы сгенерировали ранее
base_data = np.load('reference_db/crossroad_view_001.npz')

# Переводим numpy-массивы в PyTorch тензоры и переносим на видеокарту
# LightGlue ожидает наличие "Batch Dimension" (размера пакета), поэтому мы добавляем .unsqueeze(0)
# Формат координат: (1, N, 2), где N - количество точек (500)
kpts0 = torch.from_numpy(base_data['keypoints']).float().unsqueeze(0).cuda()

# В нашей базе векторы лежат как (256, N). Трансформерам нужен формат (N, 256).
# Поэтому мы транспонируем матрицу (.T) перед добавлением Batch Dimension
desc0 = torch.from_numpy(base_data['descriptors']).float().T.unsqueeze(0).cuda()


# ---------------------------------------------------------
# ЭТАП 2: Подготовка данных с камеры дрона (image1 - Текущий кадр)
# ---------------------------------------------------------
# В реальном полете здесь будет вызов бортового SuperPoint. 
# Для примера сымитируем, что мы уже извлекли 600 точек с текущего кадра.
current_kpts = np.random.rand(600, 2) * [640, 480]
current_desc = np.random.rand(600, 256).astype(np.float32)

kpts1 = torch.from_numpy(current_kpts).float().unsqueeze(0).cuda()
desc1 = torch.from_numpy(current_desc).float().unsqueeze(0).cuda()


# ---------------------------------------------------------
# ЭТАП 3: Формируем входной словарь для LightGlue
# ---------------------------------------------------------
input_dict = {
    'image0': {
        'keypoints': kpts0,     # Тензор (1, 500, 2)
        'descriptors': desc0    # Тензор (1, 500, 256)
    },
    'image1': {
        'keypoints': kpts1,     # Тензор (1, 600, 2)
        'descriptors': desc1    # Тензор (1, 600, 256)
    }
}

# ---------------------------------------------------------
# ЭТАП 4: ИНФЕРЕНС (Поиск совпадений)
# ---------------------------------------------------------
with torch.no_grad(): # Обязательно отключаем расчет градиентов для скорости!
    out = matcher(input_dict)

# Очищаем вывод от Batch-измерения для удобства работы
out = rbd(out) 

# Получаем индексы совпавших точек
# matches0 - это массив пар индексов [индекс_в_image0, индекс_в_image1]
matches = out['matches'] 
scores = out['scores'] # Уверенность нейросети для каждого совпадения

print(f"Найдено совпадений: {len(matches)}")

# ---------------------------------------------------------
# ЭТАП 5: Интеграция в ROS 2 (Формирование Bounding Box)
# ---------------------------------------------------------
if len(matches) > 15: # Порог уверенности: если совпало больше 15 точек, считаем, что мы нашли объект
    # Вытаскиваем пиксельные координаты совпавших точек именно на текущем кадре (image1)
    matched_kpts1 = kpts1[0, matches[:, 1]] # Формат: (Кол-во_совпадений, 2)
    
    # Конвертируем обратно в CPU / Numpy для питоновских расчетов
    matched_kpts_np = matched_kpts1.cpu().numpy()
    
    # Находим границы (Bounding Box) для вашей ноды
    min_x = np.min(matched_kpts_np[:, 0])
    max_x = np.max(matched_kpts_np[:, 0])
    min_y = np.min(matched_kpts_np[:, 1])
    max_y = np.max(matched_kpts_np[:, 1])
    
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    
    print(f"Объект найден! Центр на кадре: X={center_x:.1f}, Y={center_y:.1f}")
else:
    print("Совпадений слишком мало. Объект не обнаружен.")
