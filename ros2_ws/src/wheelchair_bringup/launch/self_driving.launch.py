import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_state_publisher_launch = os.path.join(
        get_package_share_directory('wheelchair_description'),
        'launch',
        'view_wheelchair.launch.py'
    )
    
    localization_launch = os.path.join(
        get_package_share_directory('navigation'),
        'launch',
        'localization.launch.py'
    )

    navigation_launch = os.path.join(
        get_package_share_directory('navigation'),
        'launch',
        'navigation.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_publisher_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch)
        ),

        TimerAction(
            period = 1.0,
            actions = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch)
                )
            ]
        )
    ])