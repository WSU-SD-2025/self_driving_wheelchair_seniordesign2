import serial
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RoboClawWheelBridge(Node):
    def __init__(self):
        super().__init__('roboclaw_wheel_bridge')

        # parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_separation', 0.575)

        # ROS2 side Limits
        self.declare_parameter('max_linear_x', 1.5)
        self.declare_parameter('max_angular_z', 1.0)

        # ROS2 side deadzone
        self.declare_parameter('linear_deadzone', 0.01)
        self.declare_parameter('angular_deadzone', 0.01)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.max_linear_x = self.get_parameter('max_linear_x').get_parameter_value().double_value
        self.max_angular_z = self.get_parameter('max_angular_z').get_parameter_value().double_value
        self.linear_deadzone = self.get_parameter('linear_deadzone').get_parameter_value().double_value
        self.angular_deadzone = self.get_parameter('angular_deadzone').get_parameter_value().double_value

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f'Serial opened: {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            raise

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    
    def clamp(self, x, mn, mx):
        return max(mn, min(mx, x))
    
    def deadzone(self, v, w):
        if abs(v) < self.linear_deadzone:
            v = 0.0
        if abs(w) < self.angular_deadzone:
            w = 0.0
        return v, w
    
    def cmd_vel_to_wheels(self, v, w):
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)
        return v_left, v_right
    

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        v, w = self.deadzone(v,w)

        v = self.clamp(v, -self.max_linear_x, self.max_linear_x)
        w = self.clamp(w, -self.max_angular_z, self.max_angular_z)

        v_left, v_right = self.cmd_vel_to_wheels(v, w)

        line = f'WHEEL,{v_left:.4f},{v_right:.4f}\n'

        try:
            self.ser.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    
    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboClawWheelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()