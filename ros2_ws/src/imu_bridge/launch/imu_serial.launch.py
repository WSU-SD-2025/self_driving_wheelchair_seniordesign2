from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='imu_bridge',
            executable='imu_serial_node',
            name='imu_serial_node',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 115200,
                'frame_id': 'imu_link'
            }]
        )

    ])