#!/usr/bin/env python3
# bag_frames.py — извлечь кадры /image_color из rosbag по WALL-моментам (та же
# эпоха, что в логах VINS/ноды → удобно ловить init/reboot), сохранить PNG +
# монтаж-сетку + метрики (ORB-фичи, резкость, средний цвет → детект «оранжевого
# фриза»). «Что видела камера» в ключевые моменты. Запуск внутри nav (overlay):
#   python3 /lab/bag_frames.py "name:wall,name:wall,..."   # явные моменты
#   python3 /lab/bag_frames.py 8                            # N кадров равномерно
# env: SCENE_BAG (default …/output/scene_bag), SCENE_OUT (…/output/bag_frames)
import os, sys, cv2, numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
BAG = os.environ.get('SCENE_BAG', '/root/sim_ws/output/scene_bag')
OUT = os.environ.get('SCENE_OUT', '/root/sim_ws/output/bag_frames')
os.makedirs(OUT, exist_ok=True)
br = CvBridge(); r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('', ''))
frames = []
while r.has_next():
    t, d, ns = r.read_next()
    if t == '/image_color': frames.append((ns/1e9, d))
if not frames:
    print('нет /image_color в bag'); raise SystemExit(1)
t0, t1 = frames[0][0], frames[-1][0]
print(f'кадров: {len(frames)}; wall {t0:.0f}..{t1:.0f} ({t1-t0:.0f}s)')
arg = sys.argv[1] if len(sys.argv) > 1 else '8'
if ':' in arg:
    EV = [(p.split(':')[0], float(p.split(':')[1])) for p in arg.split(',')]
else:
    k = int(arg); EV = [(f'f{i+1}', t0 + (t1-t0)*i/(k-1)) for i in range(k)]
orb = cv2.ORB_create(2000); tiles = []
print(f'{"name":14} {"wall":>11} {"ORB":>5} {"sharp":>7} {"B":>4}{"G":>4}{"R":>4} note')
for nm, tw in EV:
    j = min(range(len(frames)), key=lambda k: abs(frames[k][0]-tw))
    rs, d = frames[j]; img = br.imgmsg_to_cv2(deserialize_message(d, Image), 'bgr8')
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY); n = len(orb.detect(g, None))
    lap = cv2.Laplacian(g, cv2.CV_64F).var(); b, gg, rr = [float(img[:, :, i].mean()) for i in range(3)]
    note = 'ОРАНЖ-ФРИЗ' if (rr > gg > b and lap < 10) else ('норм' if n > 80 else '?')
    cv2.imwrite(f'{OUT}/{nm}_{rs:.0f}.png', img)
    print(f'{nm:14} {rs:11.0f} {n:5d} {lap:7.1f} {b:4.0f}{gg:4.0f}{rr:4.0f} {note}')
    tile = cv2.resize(img, (480, 270))
    cv2.putText(tile, f'{nm} ORB={n}', (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    tiles.append(tile)
while len(tiles) % 3:
    tiles.append(np.zeros((270, 480, 3), np.uint8))
rows = [np.hstack(tiles[i:i+3]) for i in range(0, len(tiles), 3)]
cv2.imwrite(f'{OUT}/montage.png', np.vstack(rows))
print('монтаж:', f'{OUT}/montage.png')
