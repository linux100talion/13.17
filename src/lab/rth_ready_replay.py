#!/usr/bin/env python3
"""rth_ready_replay.py — ПРОКРУТИТЬ ЛАТЧ ВОЗВРАТА (RthReadiness) по готовому bag.

Зачем: латч решает, разрешён ли возврат домой (rth= в статусе, зелёный/красный
баннер FPV) и когда открывать мост в EKF (гейт зрелости). Проверять его в
воздухе дорого — здесь настоящий класс кормится тем же, что видела нода:

  armed, поза EKF   → /mavros/state, /mavros/local_position/pose
  путь по IPM       → интеграл /flow_dbg8|9 (vfwd/vlat, тело) — как у ноды,
                      только восстановленный из скоростей
  курс              → /mavros/imu/data
  зрелость/здоровье → odom=/age=/reb= из /mission/status + записанный /vins/sane
  мост              → /nn1/bridge

Печатает ленту переходов (heal → ready → lost) с причинами и итог: когда
залатчился дом, сколько к тому моменту отъехали, что порвало раму.

  docker exec p1317_nav bash -lc "source /opt/ros/humble/setup.bash;
      source /root/sim_ws/install/setup.bash;
      python3 /lab/rth_ready_replay.py /root/sim_ws/output/joystick/<RUN>/bag
      [--radius 5 --heal-sec 30 --ripe-sec 5 --min-count 300]"
"""
import argparse
import bisect
import math
import sys

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import State
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

sys.path.insert(0, '/root/sim_ws/src/control')
sys.path.insert(0, 'src/control')
from control_pkg.application.rth_ready import RthReadiness     # noqa: E402
from control_pkg.domain.state import DroneState                # noqa: E402

TOPICS = ['/mission/status', '/mavros/state', '/mavros/local_position/pose',
          '/mavros/imu/data', '/flow_dbg8', '/flow_dbg9', '/nn1/bridge', '/vins/sane']


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    r.set_filter(StorageFilter(topics=TOPICS))
    out = {k: [] for k in TOPICS}
    while r.has_next():
        topic, raw, rec = r.read_next()
        t = rec * 1e-9                      # ВРЕМЯ ЗАПИСИ — одна шкала для всех
        if topic == '/mission/status':
            d = dict(kv.partition('=')[::2] for kv in
                     deserialize_message(raw, String).data.split())
            out[topic].append((t, d))
        elif topic == '/mavros/state':
            m = deserialize_message(raw, State)
            out[topic].append((t, bool(m.armed)))
        elif topic == '/mavros/local_position/pose':
            m = deserialize_message(raw, PoseStamped)
            p = m.pose.position
            out[topic].append((t, (p.x, p.y, p.z)))
        elif topic == '/mavros/imu/data':
            out[topic].append((t, quat_yaw(deserialize_message(raw, Imu).orientation)))
        elif topic in ('/flow_dbg8', '/flow_dbg9'):
            m = deserialize_message(raw, Vector3Stamped)
            out[topic].append((t, m.vector.y if topic == '/flow_dbg8' else m.vector.x))
        elif topic == '/nn1/bridge':
            out[topic].append((t, deserialize_message(raw, String).data))
        elif topic == '/vins/sane':
            out[topic].append((t, bool(deserialize_message(raw, Bool).data)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--radius', type=float, default=5.0)
    ap.add_argument('--heal-sec', type=float, default=30.0)
    ap.add_argument('--ripe-sec', type=float, default=5.0)
    ap.add_argument('--min-count', type=int, default=100)
    ap.add_argument('--track-m', type=float, default=3.0)
    ap.add_argument('--jump-m', type=float, default=2.0)
    ap.add_argument('--home-settle', type=float, default=3.0)
    a = ap.parse_args()

    d = load(a.bag)
    st = d['/mission/status']
    if not st:
        sys.exit('в bag нет /mission/status')
    idx = {k: [x[0] for x in v] for k, v in d.items()}

    def near(topic, t, default=None):
        arr, times = d[topic], idx[topic]
        if not arr:
            return default
        i = min(bisect.bisect_right(times, t), len(arr)) - 1
        return arr[i][1] if i >= 0 else default

    rth = RthReadiness(radius=a.radius, heal_sec=a.heal_sec, ripe_sec=a.ripe_sec,
                       min_count=a.min_count, track_m=a.track_m, jump_m=a.jump_m,
                       home_settle=a.home_settle)
    t0 = st[0][0]
    fwd = lat = 0.0                 # интеграл скоростей IPM (тело) = путь
    prev_t = None
    prev_state, prev_ripe = rth.state, rth.ripe
    last = (rth.state, '', 0, 0.0, 0.0, None)
    print(f"bag: {a.bag}")
    print(f"  ручки: круг {a.radius:g} м, лечение {a.heal_sec:g} с, зрелость "
          f"{a.ripe_sec:g} с при odom ≥ {a.min_count}")
    print("  ⚠️ вердикт здоровья берётся ЗАПИСАННЫЙ (/vins/sane из bag). В bag'ах до "
          "2026-09-09\n     он ложно срабатывал в RTL/SMART_RTL (фикс modes.navigates) "
          "— на таких прогонах\n     'lost:insane' может быть артефактом; сверять "
          "vins_sane_replay.py")
    print("   t(с)  событие")
    for seq, (t, sd) in enumerate(st):
        if prev_t is not None:
            dt = t - prev_t
            fwd += (near('/flow_dbg8', t, 0.0) or 0.0) * dt
            lat += (near('/flow_dbg9', t, 0.0) or 0.0) * dt
        prev_t = t
        pos = near('/mavros/local_position/pose', t)
        brg = near('/nn1/bridge', t)
        s = DroneState(
            now_sim=t - t0, armed=bool(near('/mavros/state', t, False)),
            ipm_fwd=fwd, ipm_lat=lat, flow_seq=seq,
            att_yaw=float(near('/mavros/imu/data', t, 0.0) or 0.0),
            vins_odom_count=int(float(sd.get('odom', 0))),
            vins_last_sim=(t - t0) - float(sd.get('age', 999.0)),
            vins_rebirths=int(float(sd.get('reb', 0))),
            bridge_seen=brg is not None,
            bridge_open=(brg.split()[0] == 'open') if brg else True,
            ekf_x=pos[0] if pos else None, ekf_y=pos[1] if pos else None,
            ekf_z=pos[2] if pos else None)
        state = rth.update(s, sane=bool(near('/vins/sane', t, True)))
        if s.armed:                       # итог берём с ПОСЛЕДНЕГО тика в воздухе:
            last = (state, rth.why, len(rth.track), rth.path_m, rth.dist, rth.home)
        if rth.ripe != prev_ripe:
            prev_ripe = rth.ripe
            print(f"  {t - t0:6.1f}  зрелость VINS → {'ДА (мост можно открывать)' if rth.ripe else 'нет'}")
        if state != prev_state:
            prev_state = state
            if state == 'ready':
                h = rth.home
                print(f"  {t - t0:6.1f}  ЛАТЧ: дом ({h[0]:+.1f},{h[1]:+.1f}) EKF, "
                      f"отъехали по IPM {rth.dist:.1f} м → RTH РАЗРЕШЁН")
            elif state == 'lost':
                print(f"  {t - t0:6.1f}  ЗАПРЕТ: {rth.why} → RTH нет на весь полёт")
            else:
                print(f"  {t - t0:6.1f}  дизарм — латч сброшен (следующий полёт чистый)")
    st_, why_, ntr, pm, dist, home = last
    hm = f"({home[0]:+.1f},{home[1]:+.1f})" if home else "--"
    print(f"  ИТОГ (последний тик в воздухе): {st_}{(':' + why_) if why_ else ''}; "
          f"дом {hm} EKF; трек {ntr} точек / {pm:.0f} м; смещение по IPM {dist:.1f} м")


if __name__ == '__main__':
    main()
