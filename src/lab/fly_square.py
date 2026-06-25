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
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

WAYPOINTS_REL = [
    ( 1,  0),
    ( 1,  1),
    ( 0,  1),
    ( 0,  0),
]


class SquareFlight(Node):
    def __init__(self, size: float, alt: float, side_time: float, loops: int = 0):
        super().__init__('fly_square')
        # use_sim_time: иначе нода считает по РЕАЛЬНЫМ часам — при низком RTF
        # (симуляция ~13× медленнее, ветка nn2_c3_cpu) сторона квадрата прошла бы
        # за доли sim-секунды, дрон не успевал бы лететь, а штамп setpoint
        # разъезжался бы с sim-временем FCU. Сажаем ноду на /clock (sim-время):
        # таймер, отсчёт сторон и штамп — всё в sim-времени, как у VINS/MAVROS.
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.size = size
        self.alt = alt
        self.side_time = side_time   # секунды SIM-времени на сторону
        self.waypoints = [(x * size, y * size) for x, y in WAYPOINTS_REL]
        self.wp_idx = 0
        self.max_loops = loops    # 0 = бесконечно (до Ctrl+C); >0 = выйти после N кругов
        self.loops_done = 0
        self.done = False
        self.wp_start = self.get_clock().now()
        # 0.1 с sim-времени → 10 sim-Гц паблиша (для GUIDED-таргета достаточно).
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info(
            f"fly_square: size={size}m alt={alt}m side_time={side_time}s (sim-время)"
        )

    def step(self):
        now = self.get_clock().now()
        if (now - self.wp_start).nanoseconds * 1e-9 >= self.side_time:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self.wp_start = now
            if self.wp_idx == 0:   # вернулись к (0,0) = круг завершён (дрон у старта)
                self.loops_done += 1
                if self.max_loops > 0 and self.loops_done >= self.max_loops:
                    self.get_logger().info(f"Облёт завершён ({self.loops_done} кругов).")
                    self.done = True
                    return
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
    parser.add_argument('--loops', type=int, default=0, help='число полных кругов; 0 = бесконечно до Ctrl+C (default 0)')
    args = parser.parse_args()

    rclpy.init()
    node = SquareFlight(args.size, args.alt, args.side_time, args.loops)
    try:
        # При --loops>0 крутимся, пока нода не выставит done (N кругов пройдено).
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Остановлено.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
