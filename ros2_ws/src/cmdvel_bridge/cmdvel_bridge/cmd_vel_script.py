import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StepCmdPublisher(Node):
    def __init__(self):
        super().__init__('step_cmd_publisher')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Setting
        self.send_hz = 20.0
        self.step_hold_sec = 3.0

        # Mode
        # Linear, Angular, Combined
        self.mode = 'linear'

        # Linear
        self.linear_steps = [
            (0.0, 0.0),
            (0.1, 0.0),
            (0.2, 0.0),
            (0.3, 0.0),
            (0.4, 0.0),
            (0.5, 0.0),
            (0.6, 0.0),
            (0.7, 0.0),
            (0.8, 0.0),
            (0.9, 0.0),
            (1.0, 0.0),
            (0.0, 0.0),

            (-0.1, 0.0),
            (-0.2, 0.0),
            (-0.3, 0.0),
            (-0.4, 0.0),
            (-0.5, 0.0),
            (-0.6, 0.0),
            (-0.7, 0.0),
            (-0.8, 0.0),
            (-0.9, 0.0),
            (-1.0, 0.0),
            (0.0, 0.0)
        ]

        # Angular
        self.angular_steps = [
            (0.0, 0.0),
            (0.0, 0.1),
            (0.0, 0.2),
            (0.0, 0.3),
            (0.0, 0.4),
            (0.0, 0.5),
            (0.0, 0.6),
            (0.0, 0.7),
            (0.0, 0.8),
            (0.0, 0.9),
            (0.0, 1.0),
            (0.0, 0.0),

            (0.0, -0.1),
            (0.0, -0.2),
            (0.0, -0.3),
            (0.0, -0.4),
            (0.0, -0.5),
            (0.0, -0.6),
            (0.0, -0.7),
            (0.0, -0.8),
            (0.0, -0.9),
            (0.0, -1.0),
            (0.0, 0.0)
        ]

        # Combined
        self.combined_steps = [
            (0.0, 0.0),
            (0.1, 0.1),
            (0.2, 0.2),
            (0.3, 0.3),
            (0.4, 0.4),
            (0.5, 0.5),
            (0.6, 0.6),
            (0.7, 0.7),
            (0.8, 0.8),
            (0.9, 0.9),
            (1.0, 1.0),
            (0.0, 0.0),

            (-0.1, -0.1),
            (-0.2, -0.2),
            (-0.3, -0.3),
            (-0.4, -0.4),
            (-0.5, -0.5),
            (-0.6, -0.6),
            (-0.7, -0.7),
            (-0.8, -0.8),
            (-0.9, -0.9),
            (-1.0, -1.0),
            (0.0, 0.0),

            (0.1, -0.1),
            (0.2, -0.2),
            (0.3, -0.3),
            (0.4, -0.4),
            (0.5, -0.5),
            (0.6, -0.6),
            (0.7, -0.7),
            (0.8, -0.8),
            (0.9, -0.9),
            (1.0, -1.0),
            (0.0, 0.0),

            (-0.1, 0.1),
            (-0.2, 0.2),
            (-0.3, 0.3),
            (-0.4, 0.4),
            (-0.5, 0.5),
            (-0.6, 0.6),
            (-0.7, 0.7),
            (-0.8, 0.8),
            (-0.9, 0.9),
            (-1.0, 1.0),
            (0.0, 0.0)
        ]

        if self.mode == 'linear':
            self.steps = self.linear_steps
        elif self.mode == 'angular':
            self.steps = self.angular_steps
        elif self.mode == 'combined':
            self.steps = self.combined_steps
        else:
            raise ValueError("Invalid mode")
        
        self.current_step_idx = 0
        self.step_start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.send_hz, self.timer_callback)

        self.get_logger().info(f'Mode: {self.mode}')
    
    def publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def timer_callback(self):
        if self.current_step_idx >= len(self.steps):
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('All steps completed. Stopping.')
            self.timer.cancel()
            rclpy.shutdown()
            return
        
        elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
        v_cmd, w_cmd = self.steps[self.current_step_idx]

        self.publish_cmd(v_cmd, w_cmd)

        if elapsed >= self.step_hold_sec:
            self.get_logger().info(f'Step {self.current_step_idx + 1}/{len(self.steps)} done: '
                                   f'v={v_cmd:.3f}, w={w_cmd:.3f}')
            
            self.current_step_idx += 1
            self.step_start_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = StepCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero = Twist()
        node.pub.publish(zero)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
