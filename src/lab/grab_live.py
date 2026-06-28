#!/usr/bin/env python3
# grab_live.py — снять ОДИН живой кадр /image_color, сохранить PNG + метрики
# (ORB-фичи, резкость=Laplacian var, средние BGR — для детекта «оранжевого фриза»
# софт-рендера). Запуск внутри nav (нужен overlay для cv_bridge):
#   python3 /lab/grab_live.py [out.png]
import sys, time, cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
OUT = sys.argv[1] if len(sys.argv) > 1 else '/root/sim_ws/output/live.png'
rclpy.init(); n = rclpy.create_node('grab_live'); br = CvBridge(); got = {}
n.create_subscription(Image, '/image_color', lambda m: got.setdefault('m', m), 10)
t0 = time.time()
while 'm' not in got and time.time()-t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.5)
if 'm' not in got:
    print('НЕТ кадра /image_color за 20с'); raise SystemExit(1)
img = br.imgmsg_to_cv2(got['m'], 'bgr8'); h, w = img.shape[:2]
g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
n_kp = len(cv2.ORB_create(2000).detect(g, None))
b, gg, rr = [float(img[:, :, i].mean()) for i in range(3)]
note = 'ОРАНЖ-ФРИЗ?' if (rr > gg > b and cv2.Laplacian(g, cv2.CV_64F).var() < 10) else 'норм'
print(f'{w}x{h}  ORB={n_kp}  sharp={cv2.Laplacian(g,cv2.CV_64F).var():.1f}  '
      f'BGR=({b:.0f},{gg:.0f},{rr:.0f})  {note}')
cv2.imwrite(OUT, img); print('saved', OUT)
