from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[
                '/home/sejunmoon/self_driving_wheelchair/ros2_ws/src/wheelchair_bringup/config/xbox_teleop.yaml'
            ]
        ),
    ])