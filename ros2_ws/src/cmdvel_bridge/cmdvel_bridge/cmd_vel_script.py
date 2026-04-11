import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StepCmdPublisher(Node):
    def __init__(self):
        super().__init__('step_cmd_publisher')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Setting
        self.send_hz = 20.0
        self.step_hold_sec = 5.0
        self.zero_hold_sec = 3.0

        # Mode
        # Linear, Angular, Combined
        self.mode = 'linear'

        self.max_linear = 1.0
        self.max_angular = 1.0

        # Linear
        self.linear_levels = [
            0.0,
            0.02, 0.04, 0.06, 0.08, 0.10,
            0.15, 0.20, 0.30, 0.40, 0.50,
            0.60, 0.70, 0.80, 0.90, 1.00,
            0.0,
            -0.02, -0.04, -0.06, -0.08, -0.10,
            -0.15, -0.20, -0.30, -0.40, -0.50,
            -0.60, -0.70, -0.80, -0.90, -1.00,
            0.0
        ]

        # Angular
        self.angular_levels = [
            0.0,
            0.02, 0.04, 0.06, 0.08, 0.10,
            0.15, 0.20, 0.30, 0.40, 0.50,
            0.60, 0.70, 0.80, 0.90, 1.00,
            0.0,
            -0.02, -0.04, -0.06, -0.08, -0.10,
            -0.15, -0.20, -0.30, -0.40, -0.50,
            -0.60, -0.70, -0.80, -0.90, -1.00,
            0.0
        ]

        self.sequence = self.build_sequence()

        self.current_step_idx = 0
        self.step_start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.send_hz, self.timer_callback)

        self.get_logger().info('==========================================')
        self.get_logger().info(f'Mode           : {self.mode}')
        self.get_logger().info(f'Send rate      : {self.send_hz:.1f} Hz')
        self.get_logger().info(f'Step hold      : {self.step_hold_sec:.2f} s')
        self.get_logger().info(f'Zero hold      : {self.zero_hold_sec:.2f} s')
        self.get_logger().info(f'Total steps    : {len(self.sequence)}')
        self.get_logger().info('==========================================')

    
    def build_sequence(self):
        sequence = []

        if self.mode == 'linear':
            for v in self.linear_levels:
                v_cmd = max(min(v, self.max_linear), -self.max_linear)
                w_cmd = 0.0
                hold = self.zero_hold_sec if abs(v_cmd) < 1e-9 else self.step_hold_sec
                sequence.append((v_cmd, w_cmd, hold))

        elif self.mode == 'angular':
            for w in self.angular_levels:
                v_cmd = 0.0
                w_cmd = max(min(w, self.max_angular), -self.max_angular)
                hold = self.zero_hold_sec if abs(w_cmd) < 1e-9 else self.step_hold_sec
                sequence.append((v_cmd, w_cmd, hold))

        else:
            raise ValueError("Invalid mode")
        
        return sequence
    

    def publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


    def timer_callback(self):
        if self.current_step_idx >= len(self.sequence):
            self.publish_zero_and_stop()
            return
        
        now = self.get_clock().now()
        elapsed = (now - self.step_start_time).nanoseconds / 1e9
        v_cmd, w_cmd, hold_sec = self.sequence[self.current_step_idx]
        self.publish_cmd(v_cmd, w_cmd)

        if elapsed >= hold_sec:
            self.get_logger().info(
                f'Step {self.current_step_idx + 1:02d}/{len(self.sequence):02d} done | '
                f'hold = {hold_sec:.1f}s | v={v_cmd:.3f} | w={w_cmd:.3f}'
            )

            self.current_step_idx += 1
            self.step_start_time = now

            if self.current_step_idx < len(self.sequence):
                next_v, next_w, next_hold = self.sequence[self.current_step_idx]
                self.get_logger().info(
                    f'Next step {self.current_step_idx + 1:02d}/{len(self.sequence):02d} | '
                    f'hold={next_hold:.1f}s | v={next_v:.3f} | w={next_w:.3f}'
                )

    def publish_zero_and_stop(self):
        self.get_logger().info('All steps completed.')
        for _ in range(10):
            self.publish_cmd(0.0, 0.0)
        self.timer.cancel()
        self.destroy_node()
        rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    node = StepCmdPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    finally:
        try:
            zero = Twist()
            for _ in range(10):
                node.pub.publish(zero)
        except Exception:
            pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()




if __name__ == '__main__':
    main()