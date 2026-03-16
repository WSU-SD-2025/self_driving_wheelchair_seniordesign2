import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import serial

class EncoderOdomBridge(Node):
    def __init__(self):
        super().__init__('encoder_odom_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        self.declare_parameter('left_cpr', 715.0)
        self.declare_parameter('right_cpr', 1200.0)
        self.declare_parameter('wheel_radius', 0.171) # 171mm
        self.declare_parameter('wheel_separation', 0.575) # 575mm

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        self.left_cpr = self.get_parameter('left_cpr').value
        self.right_cpr = self.get_parameter('right_cpr').value

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_left = None
        self.prev_right = None
        self.prev_time_ms = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Covariance Values
        # x, y, yaw are meaningful in planar odom
        # z, r, p are not observed -> large covariance
        self.pose_covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.twist_covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 99999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'Serial opened: {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.timer = self.create_timer(0.01, self.read_serial)

    
    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warning(f'Serial read error: {e}')
            return
        
        if not line:
            return
        
        if line == 'ENCODER_READY':
            self.get_logger().info('ESP32 encoder ready')
            return
        
        if not line.startswith('ENCODER,'):
            return
        
        parts = line.split(',')

        if len(parts) != 4:
            self.get_logger().warning(f'Bad serial line: {line}')
            return
        
        try:
            left_count = int(parts[1])
            right_count = int(parts[2])
            time_ms = int(parts[3])
        except ValueError:
            self.get_logger().warning(f'Invalid encoder data: {line}')
            return
        
        if self.prev_left is None:
            self.prev_left = left_count
            self.prev_right = right_count
            self.prev_time_ms = time_ms
            return
        
        dt = (time_ms - self.prev_time_ms) / 1000.0

        if dt <= 0.0:
            self.prev_left = left_count
            self.prev_right = right_count
            self.prev_time_ms = time_ms
            return
        
        left_delta = left_count - self.prev_left
        right_delta = right_count - self.prev_right

        self.prev_left = left_count
        self.prev_right = right_count
        self.prev_time_ms = time_ms

        left_dist = (left_delta / self.left_cpr) * (2.0 * math.pi * self.wheel_radius)
        right_dist = (right_delta / self.right_cpr) * (2.0 * math.pi * self.wheel_radius)

        ds = (left_dist + right_dist) / 2.0
        dtheta = (right_dist - left_dist) / self.wheel_separation

        self.x += ds * math.cos(self.theta + dtheta / 2.0)
        self.y += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        vx = ds / dt
        wz = dtheta / dt

        now = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = wz

        odom_msg.pose.covariance = self.pose_covariance
        odom_msg.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0

            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = math.sin(self.theta / 2.0)
            tf_msg.transform.rotation.w = math.cos(self.theta / 2.0)

            self.tf_broadcaster.sendTransform(tf_msg)



def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
