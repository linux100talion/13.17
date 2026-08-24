#!/usr/bin/env python3
# ============================================================================
# spawn_pose.py — ГДЕ СЕЛ ДРОН → строка спавна для следующего прогона.
#
# Достаёт из bag'а прогона финальную (посадочную) позу борта в МИРОВЫХ осях
# Gazebo и печатает готовую строку env:
#
#   SPAWN_POSE="x y z 0 0 yaw"
#
# Её понимает scripts/sim_up.sh: подставляет в <pose> модели iris_cam в копии
# мира (/tmp, репозиторный SDF не трогается) → следующий прогон СТАРТУЕТ ТАМ,
# ГДЕ ЗАКОНЧИЛСЯ ПРЕДЫДУЩИЙ, с тем же курсом.
#
# Запуск С ХОСТА (ROS не нужен — читаем sqlite bag'а напрямую):
#   python3 src/lab/spawn_pose.py docker/sim/output/joystick/lv1_joy_20260824_140447
#   python3 src/lab/spawn_pose.py <...>/bag/scene_bag_0.db3 --keep-rp
#
# И сразу в полёт (freefly_lv умеет то же одним env — SPAWN_FROM=<прогон>):
#   eval "$(python3 src/lab/spawn_pose.py <прогон>)" && bash src/lab/freefly_lv.sh
#
# Источник позы — /model/iris_cam/odometry (истинная поза Gazebo, odom_frame
# world, БЕЗ вычитания начальной — проверено по gz-sim OdometryPublisher).
# Топик пишется в bag при TOPICS_EXTRA (дефолт freefly_lv его содержит).
# Fallback — /mavros/local_position/pose (ENU от origin EKF, т.е. от точки
# СТАРТА того прогона): к нему прибавляется --origin (спавн того прогона).
#
# roll/pitch по умолчанию ЗАНУЛЯЮТСЯ (--keep-rp — оставить): спавн с креном
# = переходный процесс осадки на старте, EKF трясёт, арм задерживается.
# ============================================================================
import argparse
import math
import os
import sqlite3
import struct
import sys

GT = '/model/iris_cam/odometry'
LP = '/mavros/local_position/pose'


class Cdr:
    """Минимальный CDR-ридер (ROS2 rmw): 4 байта заголовка + выравнивание."""

    def __init__(self, buf):
        self.b = buf
        self.o = 4                      # пропускаем encapsulation header
        self.le = buf[1] == 1

    def _align(self, n):
        r = (self.o - 4) % n
        if r:
            self.o += n - r

    def u32(self):
        self._align(4)
        v = struct.unpack_from('<I' if self.le else '>I', self.b, self.o)[0]
        self.o += 4
        return v

    def i32(self):
        self._align(4)
        v = struct.unpack_from('<i' if self.le else '>i', self.b, self.o)[0]
        self.o += 4
        return v

    def f64(self):
        self._align(8)
        v = struct.unpack_from('<d' if self.le else '>d', self.b, self.o)[0]
        self.o += 8
        return v

    def string(self):
        n = self.u32()
        v = self.b[self.o:self.o + n - 1].decode('utf-8', 'replace')
        self.o += n
        return v

    def header(self):
        sec = self.i32()
        nsec = self.u32()
        self.string()                   # frame_id
        return sec + nsec * 1e-9


def read_odom(buf):
    """nav_msgs/Odometry → (t, [x,y,z], [qx,qy,qz,qw])."""
    r = Cdr(buf)
    t = r.header()
    r.string()                          # child_frame_id
    p = [r.f64() for _ in range(3)]
    q = [r.f64() for _ in range(4)]
    return t, p, q


def read_pose_stamped(buf):
    """geometry_msgs/PoseStamped → (t, [x,y,z], [qx,qy,qz,qw])."""
    r = Cdr(buf)
    t = r.header()
    p = [r.f64() for _ in range(3)]
    q = [r.f64() for _ in range(4)]
    return t, p, q


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rp_of(q):
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(s)


def find_db3(path):
    """Принимает каталог прогона, каталог bag'а или сам .db3."""
    if not os.path.exists(path) and not os.path.isabs(path):
        # относительный путь от КОРНЯ РЕПЫ — чтобы работало из любого каталога
        root = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        if os.path.exists(os.path.join(root, path)):
            path = os.path.join(root, path)
    if os.path.isfile(path):
        return path
    for sub in ('', 'bag', 'scene_bag'):
        d = os.path.join(path, sub) if sub else path
        if os.path.isdir(d):
            hits = sorted(f for f in os.listdir(d) if f.endswith('.db3'))
            if hits:
                return os.path.join(d, hits[0])
    raise SystemExit(f"ОШИБКА: не нашёл .db3 в '{path}'")


def tail(db, topic, n):
    """Последние n сообщений топика (в хронологическом порядке)."""
    cur = db.cursor()
    row = cur.execute("select id from topics where name=?", (topic,)).fetchone()
    if row is None:
        return []
    rows = cur.execute(
        "select timestamp,data from messages where topic_id=? "
        "order by timestamp desc limit ?", (row[0], n)).fetchall()
    return rows[::-1]


def main():
    ap = argparse.ArgumentParser(
        description="финальная поза борта из bag'а → SPAWN_POSE для sim_up.sh")
    ap.add_argument('run', help='каталог прогона, каталог bag или файл .db3')
    ap.add_argument('--keep-rp', action='store_true',
                    help='НЕ занулять roll/pitch (по умолчанию зануляются)')
    ap.add_argument('--dz', type=float, default=0.0,
                    help='добавка к z, м (например +0.02 если сел в яму)')
    ap.add_argument('--origin', default='0 0 0.245 0 0 0',
                    help='спавн ТОГО прогона — нужен только для fallback '
                         'по /mavros/local_position/pose')
    ap.add_argument('--quiet', action='store_true', help='только строка env')
    args = ap.parse_args()

    db3 = find_db3(args.run)
    db = sqlite3.connect(db3)

    rows = tail(db, GT, 200)
    src = GT
    if rows:
        samples = [read_odom(d) for _, d in rows]
    else:                                # fallback: локальная поза MAVROS
        rows = tail(db, LP, 200)
        if not rows:
            raise SystemExit(
                f"ОШИБКА: в bag'е нет ни {GT}, ни {LP} — позу взять неоткуда.\n"
                f"  (добавь {GT} в TOPICS_EXTRA прогона)")
        src = LP
        ox, oy, oz = [float(v) for v in args.origin.split()[:3]]
        samples = []
        for _, d in rows:
            t, p, q = read_pose_stamped(d)
            # ENU MAVROS (x=восток, y=север) совпадает по осям с миром Gazebo,
            # начало — origin EKF того прогона (= точка его старта).
            samples.append((t, [ox + p[0], oy + p[1], oz + p[2]], q))

    t_end, p_end, q_end = samples[-1]
    yaw = yaw_of(q_end)
    roll, pitch = rp_of(q_end)

    # успокоился ли борт: разброс позиции по хвосту (последние ~2 с выборки)
    tailn = [s for s in samples if t_end - s[0] <= 2.0] or samples[-20:]
    move = max(math.dist(s[1], p_end) for s in tailn)
    dyaw = max(abs(math.degrees(yaw_of(s[2]) - yaw)) for s in tailn)

    if not args.keep_rp:
        roll = pitch = 0.0
    z = p_end[2] + args.dz

    pose = (f"{p_end[0]:.4f} {p_end[1]:.4f} {z:.4f} "
            f"{roll:.4f} {pitch:.4f} {yaw:.5f}")

    if not args.quiet:
        print(f"# bag:   {db3}", file=sys.stderr)
        print(f"# поза:  {src}", file=sys.stderr)
        print(f"# xyz:   ({p_end[0]:.3f}, {p_end[1]:.3f}, {p_end[2]:.3f}) м, "
              f"курс {math.degrees(yaw):.2f}° (ENU: 0 = нос на восток)",
              file=sys.stderr)
        print(f"# покой: сдвиг за хвост {move:.3f} м, курс ±{dyaw:.2f}°",
              file=sys.stderr)
        if move > 0.20 or dyaw > 5.0:
            print("# ⚠️ борт в конце bag'а ЕЩЁ ДВИГАЛСЯ — поза не «место посадки», "
                  "а последний кадр записи", file=sys.stderr)
        if not args.keep_rp and (abs(math.degrees(rp_of(q_end)[0])) > 3
                                 or abs(math.degrees(rp_of(q_end)[1])) > 3):
            print("# ⚠️ борт лежал с креном/тангажом >3° — занулено (--keep-rp "
                  "оставит как есть)", file=sys.stderr)
    print(f'SPAWN_POSE="{pose}"')


if __name__ == '__main__':
    main()
