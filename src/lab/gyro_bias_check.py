#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""СМЕЩЕНИЕ НУЛЯ ГИРОСКОПА ПО РЫСКАНИЮ — ω_z из MAVROS против истины по одометрии.

ЗАЧЕМ. Пока зрение гироскоп не использовало, смещение ω_z никого не трогало. Как только
канал вида сверху стал ВЫЧИТАТЬ вращательное поле (`ipm_derot`), смещение умножилось на
плечо полосы (≈4.5 м) и полилось прямо в измерение боковой скорости. Демпфер СКОРОСТИ
зануляет то, что видит, — значит гонит борт ровно с этой скоростью, ровно и односторонне.

Замер, которым это поймали (серия L2s против J2s):
    прогон |  ω_z гиро |  ω_z истина |  смещение |  × 4.5 м → м/с
    I1s1   |  -0.0181  |   +0.0055   |  -0.0236  |    -0.106
    L2s1   |  -0.0205  |   +0.0077   |  -0.0282  |    -0.127
    L2s2   |  -0.0209  |   +0.0033   |  -0.0242  |    -0.109
Смещение СВОЁ У КАЖДОГО ПРОГОНА (-0.006…-0.028), поэтому и уход по серии разошёлся
1.4…13.7 м. Последняя колонка — цена смещения для канала вида сверху.

⚠️ Мерить ДО того, как включать что-либо, использующее гироскоп в измерении положения
или скорости. Бэг подойдёт лёгкий (после strip_bags): нужны только одометрия и IMU.

Запуск (контейнер одноразовый, монтировать АБСОЛЮТНЫМИ путями):
  REPO=$(git rev-parse --show-toplevel)   # корень репы (из любого места внутри)
  docker run --rm -v $REPO/src/lab:/lab:ro \
    -v $REPO/docker/sim/output:/out:ro ros:humble-ros-base bash -lc \
    'python3 /lab/gyro_bias_check.py J2s1 L2s1 L2s2'
"""
import numpy as np, math, sys, os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
st=lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
# ⚠️ ЧЕМ СНИМАТЬ СМЕЩЕНИЕ. Скользящее среднее по ω_z В ПОЛЁТЕ не годится: СКО настоящего
# разворота 0.13 рад/с против смещения 0.02, и за окно висения (~24 с) среднее оценивает
# ноль с ошибкой того же порядка, что и сам ноль. Стенд это и показал — числа по τ прыгали
# немонотонно (τ=3 сдвиг −0.01, τ=5 +0.03, τ=10 +0.17) и менялись от прогрева, то есть
# выбирался бы шум. Поэтому колонки ниже сравнивают полётное смещение с оценкой НА ЗЕМЛЕ,
# где истинный разворот РОВНО ноль и усредняется чистый датчик:
#   земля  — среднее ω_z до отрыва (z < 0.5 м), это и есть кандидат в поправку;
#   ост.   — что осталось бы после вычитания земляной оценки (цель — ноль);
#   пол1/2 — смещение по половинам висения: расходятся — значит ноль ПЛЫВЁТ и статическая
#            калибровка на земле принципиально не спасает.
print('%-7s| %11s | %11s | %10s | %9s | %9s | %8s | %8s | %8s'
      %('прогон','ω_z гиро','ω_z истина','смещение','→ м/с','земля','ост.','пол1','пол2'))
for b in sys.argv[1:]:
    u='/out/%s_bag'%b
    if not os.path.isdir(u): continue
    r=SequentialReader(); r.open(StorageOptions(uri=u,storage_id='sqlite3'),ConverterOptions('cdr','cdr'))
    od=[];im=[]
    while r.has_next():
        t,raw,_=r.read_next()
        if t=='/model/iris_cam/odometry':
            m=deserialize_message(raw,Odometry)
            od.append((st(m),m.pose.pose.position.z,yw(m.pose.pose.orientation)))
        elif t=='/mavros/imu/data':
            m=deserialize_message(raw,Imu); im.append((st(m),m.angular_velocity.z))
    od=np.array(od); im=np.array(im)
    g=np.arange(od[0,0],od[-1,0],0.05)
    z=np.interp(g,od[:,0],od[:,1]); hd=np.interp(g,od[:,0],np.unwrap(od[:,2]))
    idx=np.nonzero(z>2.5)[0]; i0,i1=idx[0]+40,idx[-1]-40
    S=slice(i0,i1)
    wt=np.gradient(hd,0.05)[S]                       # истинная скорость разворота
    wg=np.interp(g[S],im[:,0],im[:,1])               # то, что отдаёт гироскоп
    bias=wg.mean()-wt.mean()
    # оценка НА ЗЕМЛЕ: всё, что до отрыва (истинный разворот там ровно ноль)
    gr=np.nonzero(z<0.5)[0]; gr=gr[gr<idx[0]]
    gb=float(np.interp(g[gr],im[:,0],im[:,1]).mean()) if len(gr)>10 else float('nan')
    h=(i1-i0)//2
    b1=np.interp(g[i0:i0+h],im[:,0],im[:,1]).mean()-np.gradient(hd,0.05)[i0:i0+h].mean()
    b2=np.interp(g[i0+h:i1],im[:,0],im[:,1]).mean()-np.gradient(hd,0.05)[i0+h:i1].mean()
    print('%-7s| %+9.4f   | %+9.4f   | %+8.4f   | %+7.3f | %+9.4f | %+8.4f | %+8.4f | %+8.4f'
          %(b,wg.mean(),wt.mean(),bias,4.5*bias,gb,bias-gb,b1,b2))
