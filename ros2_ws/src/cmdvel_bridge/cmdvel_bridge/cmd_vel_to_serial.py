import rclpy
import time
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        self.declare_parameter('max_linear', 1.5)
        self.declare_parameter('max_angular', 3.0)

        self.declare_parameter('send_hz', 30.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.send_hz = float(self.get_parameter('send_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.latest_linear = 0.0
        self.latest_angular = 0.0

        self.last_cmd_time = self.get_clock().now()

        self.serial = None
        self.connect_serial()

        self.sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 10)
        self.timer = self.create_timer(1.0 / self.send_hz, self.timer_callback)

        self.get_logger().info(f"Subscribed to {self.cmd_vel_topic}")
        self.get_logger().info(f"Serial: {self.serial_port} @ {self.baud_rate}")
        self.get_logger().info(f"Send rate: {self.send_hz} Hz")
        self.get_logger().info(f"Cmd timeout: {self.cmd_timeout} sec")


    def connect_serial(self):
        try:
            self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2.0)
            self.get_logger().info("Serial connected")
        except Exception as e:
            self.serial = None
            self.get_logger().error(f"Serial connection failed: {e}")

    
    def clamp(self, x, lo, hi):
        return max(lo, min(x, hi))
    

    def cmd_callback(self, msg: Twist):
        self.latest_linear = msg.linear.x
        self.latest_angular = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    
    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.cmd_timeout:
            v_cmd = 0.0
            w_cmd = 0.0
        else:
            v_cmd = self.latest_linear
            w_cmd = self.latest_angular

        v = self.clamp(v_cmd, -self.max_linear, self.max_linear)
        w = self.clamp(w_cmd, -self.max_angular, self.max_angular)

        packet = f"<{v:.3f},{w:.3f}>\n"

        if self.serial is None:
            self.connect_serial()
            return
        
        try:
            self.serial.write(packet.encode('utf-8'))
            self.get_logger().info(f"Sent: linear={v:.3f}, angular={w:.3f}", throttle_duration_sec=0.5)
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node.serial is not None:
        try:
            node.serial.close()
        except Exception:
            pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()