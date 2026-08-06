import numpy as np, math, sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
st=lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
print('%-7s|%5s| %-11s | %-11s | %8s | %7s | %8s'%('прогон','kp','ВЫБЕГ впр','ВЫБЕГ влев','отдача','PWM СКО','смен знака'))
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
    if not len(d7): print('%-7s| нет /flow_dbg7'%b); continue
    g=np.arange(od[0,0],od[-1,0],0.05)
    x=np.interp(g,od[:,0],od[:,1]); y=np.interp(g,od[:,0],od[:,2])
    hd=np.interp(g,od[:,0],np.unwrap(od[:,4]))
    vl=-np.gradient(x,0.05)*np.sin(hd)+np.gradient(y,0.05)*np.cos(hd)
    tg=np.interp(g,d7[:,0],d7[:,1]); pw=np.interp(g,d7[:,0],d7[:,2])
    on=np.abs(tg)>1e-6
    e=np.nonzero(np.diff(on.astype(int)))[0]
    starts=[k+1 for k in e if on[k+1]]; ends=[k for k in e if not on[k+1]]
    run=[]
    for k in ends[:2]:
        w=slice(k+1,k+1+120)
        if w.stop>=len(vl): break
        run.append((x[w.stop-1]-x[w.start])*(-math.sin(hd[k]))+(y[w.stop-1]-y[w.start])*math.cos(hd[k]))
    dv=[abs(np.mean(vl[s:s+120])) for s in starts[:2] if s+120<len(vl)]
    p=pw[~on]; sgn=np.sign(p-np.mean(p))
    print('%-7s|%5s| %+10.2f м | %+10.2f м | %6.2f м/с | %7.0f | %6.1f /с'
          %(b,kp,run[0] if run else float('nan'),run[1] if len(run)>1 else float('nan'),
            np.mean(dv) if dv else float('nan'),np.std(p),np.sum(np.diff(sgn)!=0)/(len(p)*0.05)))
