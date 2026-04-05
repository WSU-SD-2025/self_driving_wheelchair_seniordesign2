from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ekf_yaml = PathJoinSubstitution(
        [FindPackageShare('navigation'), 'config', 'ekf.yaml']
    )

    sensor_bridge_yaml = PathJoinSubstitution(
        [FindPackageShare('wheelchair_bringup'), 'config', 'sensor_bridge.yaml']
    )

    wheel_odom_yaml = PathJoinSubstitution(
        [FindPackageShare('wheelchair_bringup'), 'config', 'wheel_odom.yaml']
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

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_odom',
        output='screen',
        parameters=[ekf_yaml]
    )

    return LaunchDescription([
        sensor_bridge_node,
        wheel_odom_node,
        ekf_node
    ])