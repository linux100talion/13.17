#!/usr/bin/env python3
"""
Непрерывный облёт квадрата для прогрева VINS.
Публикует setpoint_position/local на 10 Гц.
Каждую сторону квадрата проходит за SIDE_TIME секунд, затем переходит к следующей.

Запуск внутри nav-контейнера:
    python3 /lab/fly_square.py [--size M] [--alt M] [--side-time S]
Или через make: make fly
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

WAYPOINTS_REL = [
    ( 1,  0),
    ( 1,  1),
    ( 0,  1),
    ( 0,  0),
]


class SquareFlight(Node):
    def __init__(self, size: float, alt: float, side_time: float):
        super().__init__('fly_square')
        self.pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.size = size
        self.alt = alt
        self.side_time = side_time
        self.waypoints = [(x * size, y * size) for x, y in WAYPOINTS_REL]
        self.wp_idx = 0
        self.wp_start = time.time()
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info(
            f"fly_square: size={size}m alt={alt}m side_time={side_time}s"
        )

    def step(self):
        now = time.time()
        if now - self.wp_start >= self.side_time:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self.wp_start = now
            x, y = self.waypoints[self.wp_idx]
            self.get_logger().info(f"  -> waypoint {self.wp_idx}: x={x:.1f} y={y:.1f} z={self.alt:.1f}")

        x, y = self.waypoints[self.wp_idx]
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(self.alt)
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--size', type=float, default=5.0, help='сторона квадрата, м (default 5)')
    parser.add_argument('--alt', type=float, default=3.0, help='высота полёта, м (default 3)')
    parser.add_argument('--side-time', type=float, default=8.0, help='время на каждую сторону, с (default 8)')
    args = parser.parse_args()

    rclpy.init()
    node = SquareFlight(args.size, args.alt, args.side_time)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Остановлено.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
