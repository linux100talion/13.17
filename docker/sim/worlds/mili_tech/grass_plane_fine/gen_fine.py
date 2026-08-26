import cv2, numpy as np, os
base = '/usr/local/DATA/Calude/13.17/docker/sim/worlds/mili_tech'
out = f'{base}/grass_plane_fine'   # генератор лежит в этом же каталоге
os.makedirs(f'{out}/materials/textures', exist_ok=True)

# --- текстура: grass_dry 512 → 4096 + многооктавный шум яркости.
# 4096 px на тайл 15 м → texel 3.7 мм — предел, который вообще нужен каналу:
# варп IPM у пола (alt_floor 0.5 м) сэмплирует 3.3 мм/px, камера с 0.5 м
# разрешает ~2 мм. Октавы (σ px → мм при 3.66 мм/px): 1.5→5, 6→22, 24→88,
# 96→350 — сигнал на всех масштабах LK-окна. JPEG q92: шум в PNG не жмётся
# (десятки МБ в репо), а jpeg-артефакты трекингу не мешают.
g = cv2.imread(f'{base}/grass_plane/materials/textures/grass_dry.png').astype(np.float32)
N = 4096
g = cv2.resize(g, (N, N), interpolation=cv2.INTER_CUBIC)
rng = np.random.default_rng(1317)
def octave(k, amp):
    n = rng.standard_normal((N, N)).astype(np.float32)
    n = cv2.GaussianBlur(n, (0, 0), k)
    return amp * n / max(1e-6, n.std())
lum = octave(1.5, 10.0) + octave(6, 8.0) + octave(24, 7.0) + octave(96, 6.0)
fine = np.clip(g + lum[..., None], 0, 255).astype(np.uint8)
tex = f'{out}/materials/textures/grass_fine.jpg'
cv2.imwrite(tex, fine, [cv2.IMWRITE_JPEG_QUALITY, 92])
print('texture:', fine.shape, f"{os.path.getsize(tex)//1024} KiB")

# --- model.sdf: коллизия одним боксом, визуалы сеткой 10x10 по 15 м (texel 3.7 мм)
head = '''<?xml version="1.0" ?>
<!-- СГЕНЕРИРОВАНО gen_fine.py (лежит рядом; сид 1317) — вариант grass_plane с
     МЕЛКОЙ фактурой земли для полётов у земли. Мотив (прогоны lv2 040737/041255):
     gz-sim не тайлит текстуру по грани бокса, у grass_plane один grass_dry.png
     512px растянут на 150 м (texel 29 см) - ближе ~3 м земля бесфактурная каша,
     полнокадровый LK и IPM-канал слепнут по построению мира, а не алгоритма.
     Здесь текстура тайлится ГЕОМЕТРИЕЙ: сетка визуалов 15х15 м (texel 3.7 мм,
     grass_fine.jpg 4096px = grass_dry + многооктавная яркость 5мм/2/9/35 см -
     предел нужного: варп IPM у пола 0.5 м сэмплирует 3.3 мм/px), йав
     тайлов 0/90/180/270 псевдослучайно - рвёт периодичность. Коллизия прежним
     одним боксом, верх на z=0.05 - спавны/оси мира не тронуты.
     Опт-ин: мир mili_fortress_fine.sdf (env WORLD, см. docker-compose). -->
<sdf version="1.5">
  <model name="grass_plane_fine">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>150 150 .1</size>
          </box>
        </geometry>
      </collision>
'''
vis = []
rng2 = np.random.default_rng(13)
for i in range(10):
    for j in range(10):
        x = -67.5 + 15.0 * i
        y = -67.5 + 15.0 * j
        yaw = float(rng2.integers(0, 4)) * 1.5707963267948966
        vis.append(f'''      <visual name="v{i}_{j}">
        <cast_shadows>false</cast_shadows>
        <pose>{x} {y} 0 0 0 {yaw:.10f}</pose>
        <geometry>
          <box>
            <size>15 15 .1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.4 0.45 0.3 1</ambient>
          <diffuse>0.55 0.6 0.4 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <pbr>
            <metal>
              <albedo_map>materials/textures/grass_fine.jpg</albedo_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
''')
tail = '''    </link>
  </model>
</sdf>
'''
with open(f'{out}/model.sdf', 'w') as f:
    f.write(head + ''.join(vis) + tail)
with open(f'{out}/model.config', 'w') as f:
    f.write('''<?xml version="1.0"?>

<model>
  <name>Grass Plane Fine</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>

  <description>
    grass_plane с мелкой фактурой: сетка визуалов 15x15 м тайлит текстуру
    геометрией (texel 3.7 мм) - земля трекается вблизи (полёты у земли).
  </description>

</model>
''')
print('model.sdf lines:', sum(1 for _ in open(f'{out}/model.sdf')))
