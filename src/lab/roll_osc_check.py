import numpy as np, math, sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
st=lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
print('%-7s|%5s| %8s | %9s | %9s | %8s | %9s'%('прогон','kp','в потолке','период PWM','размах v','|v| ср','период v'))
def per(t,a):
    a=a-np.mean(a); s=np.sign(a); z=np.nonzero(np.diff(s))[0]
    return 2*np.mean(np.diff(t[z])) if len(z)>2 else float('nan')
for arg in sys.argv[1:]:
    b,kp=arg.split(':')
    r=SequentialReader(); r.open(StorageOptions(uri='/out/'+b+'_bag',storage_id='sqlite3'),ConverterOptions('cdr','cdr'))
    od=[];d7=[]
    while r.has_next():
        t,raw,ts=r.read_next()
        if t=='/model/iris_cam/odometry':
            m=deserialize_message(raw,Odometry); p=m.pose.pose.position
            od.append((st(m),p.x,p.y,p.z,yw(m.pose.pose.orientation)))
        elif t=='/flow_dbg7':
            m=deserialize_message(raw,Vector3Stamped); d7.append((st(m),m.vector.x,m.vector.z))
    od=np.array(od); d7=np.array(d7)
    g=np.arange(od[0,0],od[-1,0],0.05)
    x=np.interp(g,od[:,0],od[:,1]); y=np.interp(g,od[:,0],od[:,2])
    hd=np.interp(g,od[:,0],np.unwrap(od[:,4]))
    vl=-np.gradient(x,0.05)*np.sin(hd)+np.gradient(y,0.05)*np.cos(hd)
    tg=np.interp(g,d7[:,0],d7[:,1]); pw=np.interp(g,d7[:,0],d7[:,2])
    free=np.abs(tg)<=1e-6
    p=pw[free]; v=vl[free]; tt=g[free]
    print('%-7s|%5s| %7.0f%% | %8.1f с | %8.2f м/с | %6.2f м/с | %8.1f с'
          %(b,kp,100*np.mean(np.abs(p)>=149),per(tt,p),v.max()-v.min(),np.mean(np.abs(v)),per(tt,v)))
