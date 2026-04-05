#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from wheelchair_interfaces.msg import UwbRange
from geometry_msgs.msg import Twist

class UwbFollower(Node):

    def __init__(self):
        super().__init__('uwb_follower')

        self.desired_distance = 2.0
        self.kp = 0.8

        self.sub = self.create_subscription(
            UwbRange,
            '/uwb/range',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def callback(self, msg):
        dist = msg.distance

        error = self.desired_distance - dist

        cmd = Twist()
        cmd.linear.x = self.kp * error

        # Clamp velocity
        cmd.linear.x = max(min(cmd.linear.x, 0.5), -0.5)

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = UwbFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()