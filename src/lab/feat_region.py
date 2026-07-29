#!/usr/bin/env python3
"""feat_region — ГДЕ детектор ищет точки и что это стоит каналу скорости.

Повод. S_lat по L1_scale2ax вышла 0.48 px/(м/с) при расчётных 1.3 (земля в центре
кадра) и 3.3 (земля у нижнего края). Причина не в сцене: 93% точек детектор ставит в
полосу горизонта, где поток почти нулевой, а до ближней земли не доходит вовсе —
бюджет в 200 углов съедает контрастная линия горизонта.

Скрипт считает по уже снятому bag'у одну и ту же медиану бокового потока при разных
масках детекта и регрессирует её на ИСТИННУЮ боковую скорость (одометрия gz). Прогон
обязан быть открыто-контурным по зрению (оракул ведёт, зрение смотрит) — иначе
меряется не перцепт, а собственная петля.

Запуск ВНУТРИ nav (нужен cv_bridge из overlay):
  docker exec p1317_nav bash -lc 'source /opt/ros/humble/setup.bash; \
    source /opt/overlay/install/setup.bash; source /root/sim_ws/install/setup.bash; \
    python3 /lab/feat_region.py'

Замер по L1_scale2ax (960x540, высота 5 м, наклон камеры 15°, горизонт в строке 141):
  весь кадр   S_lat +0.40 px/(м/с), шум 3.10 м/с, R² 0.02, точек 109
  ниже 200    +1.17,                1.04 м/с,     0.14,     71
  ниже 270    +1.65,                0.76 м/с,     0.23,    200
  ниже 360    +2.42,                0.59 м/с,     0.32,    200
Понижение qualityLevel (0.01 → 0.001) не меняет ничего: под маской детектор упирается
в лимит 200 точек, а не в качество — углов на земле в избытке.
"""
import math, numpy as np, cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
br=CvBridge(); r=SequentialReader()
r.open(StorageOptions(uri="/root/sim_ws/output/L1_scale2ax_bag",storage_id="sqlite3"),ConverterOptions("cdr","cdr"))
fr=[]; od=[]
while r.has_next():
    tp,raw,_=r.read_next()
    if tp=="/image_color":
        m=deserialize_message(raw,Image); fr.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9, cv2.cvtColor(br.imgmsg_to_cv2(m,"bgr8"),cv2.COLOR_BGR2GRAY)))
    elif tp=="/model/iris_cam/odometry":
        m=deserialize_message(raw,Odometry); p=m.pose.pose.position; q=m.pose.pose.orientation
        yw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        od.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,yw))
od=np.array(od); fr.sort(key=lambda f:f[0]); t0=od[0,0]
tf=np.array([f[0] for f in fr])-t0; t=od[:,0]-t0
x,y,z,yw=od[:,1],od[:,2],od[:,3],od[:,4]
vx,vy=np.gradient(x,t),np.gradient(y,t)
vr=-vx*np.sin(yw)+vy*np.cos(yw)
h,w=fr[0][1].shape
LK=dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))
sel=[i for i in range(len(fr)-1) if tf[i]>=12 and np.interp(tf[i],t,z)>3.0]
def run(name, mask_from, qual):
    FEAT=dict(maxCorners=200,qualityLevel=qual,minDistance=8,blockSize=7)
    mask=None
    if mask_from is not None:
        mask=np.zeros((h,w),np.uint8); mask[mask_from:,:]=255
    ts_,med,npts=[],[],[]
    for i in sel:
        p0=cv2.goodFeaturesToTrack(fr[i][1],mask=mask,**FEAT)
        if p0 is None or len(p0)<8: continue
        nx,st,_=cv2.calcOpticalFlowPyrLK(fr[i][1],fr[i+1][1],p0,None,**LK)
        st=st.reshape(-1).astype(bool)
        a=p0.reshape(-1,2)[st]; b=nx.reshape(-1,2)[st]
        if len(a)<8: continue
        ts_.append(tf[i]); med.append(float(np.median((b-a)[:,0]))); npts.append(len(a))
    if len(ts_)<20: print("%-28s точек не хватило (кадров %d)"%(name,len(ts_))); return
    s=np.array(med); vv=np.interp(np.array(ts_),t,vr)
    A=np.column_stack([vv,np.ones(len(s))]); cf,*_=np.linalg.lstsq(A,s,rcond=None)
    res=s-A@cf; se=np.sqrt(np.diag(np.linalg.pinv(A.T@A))*np.sum(res**2)/(len(s)-2))
    print("%-28s S_lat %+6.2f ± %.2f px/(м/с) | шум %.2f px = %5.2f м/с | R² %.2f | точек %.0f"
          % (name, cf[0], se[0], np.std(res), np.std(res)/max(1e-9,abs(cf[0])), 1-np.var(res)/np.var(s), np.median(npts)))
run("весь кадр (как сейчас)", None, 0.01)
run("ниже горизонта (>200)", 200, 0.01)
run("нижняя половина (>270)", 270, 0.01)
run("нижняя треть (>360)", 360, 0.01)
run("нижняя треть, качество 0.003", 360, 0.003)
run("нижняя треть, качество 0.001", 360, 0.001)
