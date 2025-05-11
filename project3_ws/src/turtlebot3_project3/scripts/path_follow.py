#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pathlib import Path
import time

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 3.0  # seconds

        # Load RPM path
        self.rpm_path = self.load_rpm_path('path_rpms_final.txt')
        self.current_index = 0

        # Start publishing commands
        self.timer = self.create_timer(self.timer_period, self.publish_velocity)

    def load_rpm_path(self, filename):
        path = Path(filename)
        if not path.exists():
            self.get_logger().error(f"File {filename} not found.")
            return []
        with open(filename, 'r') as file:
            lines = file.readlines()
            rpm_values = [tuple(map(float, line.strip().split(','))) for line in lines if line.strip()]
        return rpm_values

    def rpm_to_mps(self, rpm):
        # Convert RPM to m/s (approximation for TurtleBot3 Waffle)
        WHEEL_RADIUS = 0.033  # meters
        return (rpm * WHEEL_RADIUS)

    def publish_velocity(self):
        if self.current_index >= len(self.rpm_path):
            self.get_logger().info('Path complete.')
            self.timer.cancel()
            return

        left_rpm, right_rpm = self.rpm_path[self.current_index]
        self.get_logger().info(f"Publishing RPMs - Left: {left_rpm}, Right: {right_rpm}")

        left_mps = self.rpm_to_mps(left_rpm)
        right_mps = self.rpm_to_mps(right_rpm)

        # Basic differential drive model
        WHEEL_BASE = 0.287  # meters

        linear_x = (right_mps + left_mps) / 2.0
        angular_z = (right_mps - left_mps) / WHEEL_BASE

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        self.publisher_.publish(twist)
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
