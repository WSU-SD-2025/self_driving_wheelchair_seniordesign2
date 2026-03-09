import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

def is_float(s):
    s = s.strip()

    if s == "": return False
    if s.count('.') > 1: return False
    if s.count('-') > 1: return False

    s = s.replace('.', '').replace('-', '')
    return s.isdigit()


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(Imu, 'imu/data', 10)

        self.ser = serial.Serial(port, baud, timeout = 0.1)
        self.timer = self.create_timer(0.01, self.read_serial)

        self.get_logger().info(f'Listening on {port} @ {baud}')

    
    def read_serial(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()

        if not line: return

        parts = line.split(',')

        if len(parts) != 10: return

        # check numeric
        for p in parts:
            if not is_float(p):
                return
            
        qx = float(parts[0])
        qy = float(parts[1])
        qz = float(parts[2])
        qw = float(parts[3])

        wx = float(parts[4])
        wy = float(parts[5])
        wz = float(parts[6])

        ax = float(parts[7])
        ay = float(parts[8])
        az = float(parts[9])

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]

        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

        