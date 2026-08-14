#!/usr/bin/env python3
"""ЧЕСТНОСТЬ канала курса: какую долю истинного разворота видит `flow_yaw`.

Мимо контура и мимо накопителя с утечкой: сравниваем истинный разворот за висение с
`∫flow_yaw·dt / S`, где S = 0.324 px/кадр на °/с (паспорт Y4). Доля 1.0 = сигнал меряет
курс один в один, 0 = слеп, отрицательная = ещё и знак не тот.

Зачем. В E2 борт разворачивало на 23…360° за висение, а контур этого не видел (ошибка не
выходила за ±3 единицы, PWM ни разу не в потолке). Вопрос «сигнал врёт или контур слаб»
решается только замером сигнала против истины — по замкнутому контуру его не получить,
контур нулит то, что измеряет.

Замер по трём стендам:
  ось курса одна, соседи на оракулах (Y5s, Y2_kp0) → +0.96 ± 0.12 — датчик честен;
  три оси на Dp (E2, борт идёт 1-4 м/с)            → −0.09 ± 0.02 — канал залит трансляцией;
  без стабилизации вовсе (nostab_run5)             → −10        — залит ещё сильнее.
Точки выстраиваются по СКОРОСТИ борта, а не по числу включённых осей: `flow_yaw` считается
в предположении «в дальней сцене трансляция ≈0» (см. комментарий в flow_estimator.py), и
предположение ломается движением.

Запуск:
  REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
  docker run --rm -v $REPO/src/lab:/lab:ro \
    -v $REPO/docker/sim/output:/out:ro ros:humble-ros-base bash -lc \
    'source /opt/ros/humble/setup.bash; python3 /lab/yaw_fidelity.py /out/E2s1_bag ...'
"""
import math,sys
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
st=lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9
yaw_of=lambda q: math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2))
print("сравнение НАКОПЛЕННОГО: истинный разворот против ∫flow_yaw·dt (сглажено интегралом)")
print(f"{'прогон':8s} | {'истина°':>8s} | {'∫flow px':>9s} | {'если S=0.324 →°':>15s} | {'доля':>5s}")
R=[]
for bag in sys.argv[1:]:
    r=SequentialReader(); r.open(StorageOptions(uri=bag,storage_id='sqlite3'),ConverterOptions('cdr','cdr'))
    od,d2=[],[]
    while r.has_next():
        t,raw,_=r.read_next()
        if t=='/model/iris_cam/odometry':
            m=deserialize_message(raw,Odometry); od.append((st(m),m.pose.pose.position.z,yaw_of(m.pose.pose.orientation)))
        elif t=='/flow_dbg2':
            m=deserialize_message(raw,Vector3Stamped); d2.append((st(m),m.vector.z))
    od=np.array(od); d2=np.array(d2); h=od[od[:,1]>2.0]
    n=bag.rstrip('/').split('/')[-1].replace('_bag','')
    if len(h)<30 or len(d2)<30: print(f"{n:8s} | нет"); continue
    t0,t1=h[0,0],h[-1,0]; w=d2[(d2[:,0]>=t0)&(d2[:,0]<=t1)]
    if len(w)<10: print(f"{n:8s} | нет сигнала"); continue
    true=math.degrees(np.unwrap(h[:,2])[-1]-np.unwrap(h[:,2])[0])
    integ=np.trapz(w[:,1],w[:,0])            # px·с = единицы визуального курса
    deg=integ/0.324
    frac=deg/true if abs(true)>3 else float('nan')
    print(f"{n:8s} | {true:+8.0f} | {integ:+9.2f} | {deg:+15.0f} | {frac:5.2f}")
    if np.isfinite(frac): R.append(frac)
if R: print(f"\nдоля увиденного разворота: {np.mean(R):+.2f} ± {np.std(R,ddof=1):.2f} (n={len(R)})")
