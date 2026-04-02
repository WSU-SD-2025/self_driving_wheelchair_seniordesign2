import rclpy
import time
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv
import threading
from pathlib import Path
from datetime import datetime

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 1.0)

        self.declare_parameter('send_hz', 20.0)
        self.declare_parameter('cmd_timeout', 1.0)

        self.declare_parameter('log_dir', str(Path.home() / 'self_driving_wheelchair' / 'log'))
        self.declare_parameter('csv_prefix', 'wheelchair_log')

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.send_hz = float(self.get_parameter('send_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.log_dir = Path(self.get_parameter('log_dir').value)
        self.csv_prefix = self.get_parameter('csv_prefix').value

        self.latest_linear = 0.0
        self.latest_angular = 0.0

        self.last_cmd_time = self.get_clock().now()

        self.serial = None
        self.csv_file = None
        self.csv_writer = None
        self.header_written = False
        self.reader_thread = None
        self.reader_running = False

        self.prepare_log_file()
        self.connect_serial()

        self.sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 10)
        self.timer = self.create_timer(1.0 / self.send_hz, self.timer_callback)

        self.get_logger().info(f"Subscribed to {self.cmd_vel_topic}")
        self.get_logger().info(f"Serial: {self.serial_port} @ {self.baud_rate}")
        self.get_logger().info(f"Send rate: {self.send_hz} Hz")
        self.get_logger().info(f"Cmd timeout: {self.cmd_timeout} sec")

    
    def prepare_log_file(self):
        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.log_dir / f"{self.csv_prefix}_{timestamp}.csv"
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)


    def connect_serial(self):
        try:
            self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2.0)
            self.get_logger().info("Serial connected")

            self.reader_running = True
            self.reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.reader_thread.start()

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

    
    def serial_reader(self):
        while self.reader_running and self.serial is not None:
            try:
                raw = self.serial.readline()
                if not raw:
                    continue

                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                self.handle_serial_line(line)

            except Exception as e:
                self.get_logger().error(f"Serial read failed: {e}")
                self.safe_close_serial()
                break

    
    def handle_serial_line(self, line: str):
        if line.startswith("time_ms,"):
            if not self.header_written:
                self.csv_writer.writerow([x.strip() for x in line.split(",")])
                self.csv_file.flush()
                self.header_written = True
                self.get_logger().info("CSV header captured from ESP32")
            return
        
        if "," not in line:
            return
        
        parts = [x.strip() for x in line.split(",")]

        if len(parts) == 9:
            if not self.header_written:
                self.csv_writer.writerow(["time_ms", "ref_v", "ref_w", "y_voltage", "x_voltage", "vL", "vR", "v", "w"])
                self.header_written = True
            self.csv_writer.writerow(parts)
            self.csv_file.flush()

    
    def safe_close_serial(self):
        self.reader_running = False
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None

    
    def destroy_node(self):
        self.reader_running = False

        if self.reader_thread is not None and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        
        self.safe_close_serial()

        if self.csv_file is not None:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()