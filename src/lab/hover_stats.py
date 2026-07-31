import math, os, sys
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from geometry_msgs.msg import Vector3Stamped
st=lambda m: m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def load(bag):
    r=SequentialReader(); r.open(StorageOptions(uri=bag,storage_id='sqlite3'),ConverterOptions('cdr','cdr'))
    od=[];d2=[]
    while r.has_next():
        t,raw,ts=r.read_next()
        if t=='/model/iris_cam/odometry':
            m=deserialize_message(raw,Odometry); p=m.pose.pose.position
            od.append((st(m),p.x,p.y,p.z,yw(m.pose.pose.orientation)))
        elif t=='/flow_dbg2':
            m=deserialize_message(raw,Vector3Stamped); d2.append((st(m),m.vector.x))
    return np.array(od),np.array(d2)
def period(t,x):
    x=x-np.mean(x); s=np.sign(x); z=np.where(np.diff(s)!=0)[0]
    if len(z)<3: return float('nan')
    return float(2*np.mean(np.diff(t[z])))
print(f'{"прогон":10} | {"уход":>6} | {"размах прод":>11} | {"период":>7} | {"в потолке":>9} | {"|команда|":>9}')
for bag in sys.argv[1:]:
    try: od,d2=load(bag)
    except Exception as e: print(f'{os.path.basename(bag)}: {e}'); continue
    if not len(od): continue
    z=od[:,3]; hi=z>0.9*np.percentile(z,90); i0=int(np.argmax(hi))
    t0=od[i0,0]; sel=(od[:,0]>=t0)&(od[:,0]<=t0+40)
    t=od[sel,0]; x=od[sel,1]-od[sel,1][0]; y=od[sel,2]-od[sel,2][0]
    d=np.hypot(x,y); y0=float(np.interp(t[0],od[:,0],np.unwrap(od[:,4])))
    f=x*math.cos(y0)+y*math.sin(y0)
    if len(d2):
        p=d2[(d2[:,0]>=t0)&(d2[:,0]<=t0+40),1]
        sat=100.0*np.mean(np.abs(p)>=149) if len(p) else float('nan')
        amp=np.mean(np.abs(p)) if len(p) else float('nan')
    else: sat=amp=float('nan')
    print(f'{os.path.basename(bag).replace("_bag",""):10} | {d.max():5.1f}м | {f.max()-f.min():10.1f}м | '
          f'{period(t,f):6.1f}с | {sat:8.0f}% | {amp:8.0f}')
