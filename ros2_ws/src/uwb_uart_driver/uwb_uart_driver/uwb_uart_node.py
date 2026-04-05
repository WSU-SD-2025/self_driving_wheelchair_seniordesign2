#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from wheelchair_interfaces.msg import UwbRange
import serial
import threading
import re
import time

class UwbUartNode(Node):

    def __init__(self):
        super().__init__('uwb_uart_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600) # ros2 run uwb_uart_driver uwb_uart_node   --ros-args   -p port:=/dev/ttyUSB2   -p baudrate:=921600   -p role:=tag   -p id:=1
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('role', 'tag')      # tag or anchor
        self.declare_parameter('id', 1)
        self.declare_parameter('dest_id', 1)

        role = self.get_parameter('role').value
        node_id = self.get_parameter('id').value
        dest_id = self.get_parameter('dest_id').value

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publishers
        self.range_pub = self.create_publisher(
            UwbRange,
            '/uwb/range',
            10
        )

        self.raw_pub = self.create_publisher(
            String,
            '/uwb/raw',
            10
        )

        # Open serial
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1
            )
            self.get_logger().info(f'Opened UART on {port}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Failed to open UART: {e}')
            raise e

        self.configure_device(role, node_id, dest_id)

        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        buffer = ''

        while rclpy.ok() and self.running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1).decode(errors='ignore')
                data = data.replace('\r', '\n')
                buffer += data

                while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self.process_line(line.strip())
            except Exception as e:
                self.get_logger().warn(f'UART read error: {e}')

    def process_line(self, line: str):
        if not line:
            return

        # Publish raw data
        raw_msg = String()
        raw_msg.data = line
        self.raw_pub.publish(raw_msg)

        # Example expected frame:
        # POS,ANCHOR=1,DIST=2.45


        try:
            match = re.search(r',([A-Z0-9]+),(\d+)cm,(\d+)dB', line)             
            if match:
                anchor = match.group(1)
                dist_cm = float(match.group(2))
                msg = UwbRange()
                msg.anchor_id = anchor
                msg.distance = dist_cm / 100.0   # convert cm → meters
                self.range_pub.publish(msg)
            

        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

    def configure_device(self, role, node_id, dest_id):

        cmds = []

        if role.lower() == "anchor":
            cmds = [
                f"AT+ROLE=ANCHOR\n",
                f"AT+ID={node_id}\n",
                "AT+SAVE\n",
                "AT+RST\n"
        ]

        elif role == "tag":
            cmds = [
                f"AT+ROLE=TAG\n",
                f"AT+ID={node_id}\n",
                f"AT+DEST={dest_id}\n",
                "AT+SAVE\n",
                "AT+RST\n"
                "AT+START\n"
            ]

        for cmd in cmds:
            self.ser.write(cmd.encode())
            self.get_logger().info(f"Sent: {cmd.strip()}")
            time.sleep(0.2)

def main(args=None):
    rclpy.init(args=args)
    node = UwbUartNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()