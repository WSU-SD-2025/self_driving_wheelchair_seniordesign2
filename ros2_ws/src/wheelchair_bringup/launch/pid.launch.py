import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    view_wheelchair_launch = os.path.join(
        get_package_share_directory('wheelchair_description'),
        'launch',
        'view_wheelchair.launch.py'
    )

    sensor_bridge_yaml = os.path.join(
        get_package_share_directory('wheelchair_bringup'),
        'config',
        'sensor_bridge.yaml'
    )

    wheel_odom_yaml = os.path.join(
        get_package_share_directory('wheelchair_bringup'),
        'config',
        'wheel_odom.yaml'
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(view_wheelchair_launch)
    )

    sensor_bridge_node = Node(
        package='wheelchair_bringup',
        executable='sensor_bridge',
        name='sensor_bridge',
        output='screen',
        parameters=[sensor_bridge_yaml]
    )

    wheel_odom_node = Node(
        package='wheelchair_bringup',
        executable='wheel_odom_node',
        name='wheel_odom_node',
        output='screen',
        parameters=[wheel_odom_yaml]
    )

    return LaunchDescription([
        robot_state_publisher_launch,
        sensor_bridge_node,
        wheel_odom_node
    ])