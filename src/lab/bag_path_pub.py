#!/usr/bin/env python3
# ============================================================================
# bag_path_pub.py — ТРАЕКТОРИЯ ДЛЯ RVIZ ИЗ РЕПЛЕЯ BAG'а (Odometry/PoseStamped → Path).
#
# В bag'ах freefly_lv нет nav_msgs/Path: /path VINS не пишется. RViz умеет
# рисовать линию только из Path, а из Odometry — лишь стрелки (Keep N). Эта нода
# слушает позовые топики реплея и публикует накопленный Path для каждого:
#
#   /model/iris_cam/odometry (истина Gazebo)  → /truth/path
#   /odometry                (VINS)           → /vins/path
#   /mavros/local_position/pose (EKF)         → /ekf/path
#
# Path публикуется не чаще 5 Гц (иначе 20k поз × 45 Гц = десятки МБ/с), точки
# прореживаются по расстоянию (--min-dist), длина ограничена (--max-poses).
# Скачок штампа назад > 1 с (рестарт/--loop реплея) обнуляет траекторию.
#
# Подписки BEST_EFFORT: совместимы и с RELIABLE-паблишерами реплея (/odometry,
# истина) и с BEST_EFFORT (/mavros/local_position/pose) — иначе pose молчит.
# frame_id Path = frame_id источника (истина/VINS — 'world', EKF — 'map';
# map→world даёт статический TF из bag_rviz.sh).
#
# Запускается из src/lab/bag_rviz.sh; отдельно (хост, ROS jazzy):
#   source /opt/ros/jazzy/setup.bash
#   python3 src/lab/bag_path_pub.py [--odom СРЦ:ПУТЬ ...] [--pose СРЦ:ПУТЬ ...]
# ============================================================================
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

DEFAULT_ODOM = ['/model/iris_cam/odometry:/truth/path', '/odometry:/vins/path']
DEFAULT_POSE = ['/mavros/local_position/pose:/ekf/path']


class Track:
    """Накопитель одной траектории: прореживание, кап, сброс на скачке назад."""

    def __init__(self, node, out_topic, min_dist, max_poses):
        self.path = Path()
        self.pub = node.create_publisher(Path, out_topic, 1)
        self.min_dist = min_dist
        self.max_poses = max_poses
        self.last_t = None
        self.dirty = False
        self.n_in = 0

    def add(self, header, pose):
        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        if self.last_t is not None and t < self.last_t - 1.0:
            self.path.poses.clear()          # реплей перезапущен — с чистого листа
        self.last_t = t
        self.n_in += 1
        if self.path.poses:
            p0 = self.path.poses[-1].pose.position
            d = math.dist((p0.x, p0.y, p0.z),
                          (pose.position.x, pose.position.y, pose.position.z))
            if d < self.min_dist:
                return
        ps = PoseStamped()
        ps.header = header
        ps.pose = pose
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_poses:
            del self.path.poses[0]
        self.path.header = header
        self.dirty = True

    def flush(self):
        if self.dirty:
            self.pub.publish(self.path)
            self.dirty = False


class BagPathPub(Node):
    def __init__(self, odom_pairs, pose_pairs, min_dist, max_poses, hz):
        super().__init__('bag_path_pub')
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.tracks = {}
        for src, dst in odom_pairs:
            tr = Track(self, dst, min_dist, max_poses)
            self.tracks[src] = tr
            self.create_subscription(
                Odometry, src, lambda m, tr=tr: tr.add(m.header, m.pose.pose), qos)
        for src, dst in pose_pairs:
            tr = Track(self, dst, min_dist, max_poses)
            self.tracks[src] = tr
            self.create_subscription(
                PoseStamped, src, lambda m, tr=tr: tr.add(m.header, m.pose), qos)
        self.create_timer(1.0 / hz, self.tick)
        self.create_timer(5.0, self.report)
        for src, dst in list(odom_pairs) + list(pose_pairs):
            self.get_logger().info(f'{src} → {dst}')

    def tick(self):
        for tr in self.tracks.values():
            tr.flush()

    def report(self):
        s = ', '.join(f'{src}: {tr.n_in} in / {len(tr.path.poses)} pts'
                      for src, tr in self.tracks.items())
        self.get_logger().info(s)


def parse_pairs(items):
    out = []
    for it in items:
        if ':' not in it:
            raise SystemExit(f'ожидаю ИСТОЧНИК:ВЫХОД, получил {it!r}')
        src, dst = it.split(':', 1)
        out.append((src, dst))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--odom', action='append', metavar='СРЦ:ПУТЬ',
                    help=f'nav_msgs/Odometry → Path (default: {" ".join(DEFAULT_ODOM)})')
    ap.add_argument('--pose', action='append', metavar='СРЦ:ПУТЬ',
                    help=f'geometry_msgs/PoseStamped → Path (default: {" ".join(DEFAULT_POSE)})')
    ap.add_argument('--min-dist', type=float, default=0.05, help='шаг прореживания, м')
    ap.add_argument('--max-poses', type=int, default=50000, help='кап длины Path')
    ap.add_argument('--hz', type=float, default=5.0, help='темп публикации Path')
    a = ap.parse_args()
    odom = parse_pairs(a.odom if a.odom is not None else DEFAULT_ODOM)
    pose = parse_pairs(a.pose if a.pose is not None else DEFAULT_POSE)
    rclpy.init()
    node = BagPathPub(odom, pose, a.min_dist, a.max_poses, a.hz)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                 # Ctrl+C / kill из bag_rviz.sh — штатный выход
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
