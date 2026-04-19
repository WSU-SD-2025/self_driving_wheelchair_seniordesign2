import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes

def generate_launch_description():
    nav2_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'nav2_params.yaml'
    )

    ldlidar_param_file = os.path.join(
        get_package_share_directory('wheelchair_bringup'),
        'config',
        'ldlidar.yaml'
    )

    twist_mux_yaml = os.path.join(
        get_package_share_directory('wheelchair_bringup'),
        'config',
        'twist_mux.yaml'
    )

    # LD LiDAR container
    ldlidar_container = ComposableNodeContainer(
        name='ldlidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
    )

    # LD LiDAR component
    ldlidar_component = ComposableNode(
        package='ldlidar_component',
        plugin='ldlidar::LdLidarComponent',
        name='ldlidar_node',
        parameters=[ldlidar_param_file],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Load component into container
    load_ldlidar = LoadComposableNodes(
        target_container='ldlidar_container',
        composable_node_descriptions=[ldlidar_component]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_yaml],
        remappings=[
            ('cmd_vel', '/cmd_vel/nav'),
            ('/cmd_vel', '/cmd_vel/nav'),
        ],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_yaml],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_yaml],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_yaml],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_yaml],
    )

    # twist_mux = Node(
    #     package='twist_mux',
    #     executable='twist_mux',
    #     name='twist_mux',
    #     output='screen',
    #     parameters=[twist_mux_yaml],
    #     remappings=[
    #         ('cmd_vel_out', '/cmd_vel/raw'),
    #         ('/cmd_vel_out', '/cmd_vel/raw'),
    #     ],
    # )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_yaml],
        remappings=[
            ('cmd_vel', '/cmd_vel/nav'),
            ('/cmd_vel', '/cmd_vel/nav'),
            ('cmd_vel_smoothed', '/cmd_vel/smooth'),
            ('/cmd_vel_smoothed', '/cmd_vel/smooth'),
        ],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_yaml],
        remappings=[
            ('cmd_vel_in', '/cmd_vel/smooth'),
            ('/cmd_vel_in', '/cmd_vel/smooth'),
            ('cmd_vel_out', '/cmd_vel'),
            ('/cmd_vel_out', '/cmd_vel'),
        ],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_yaml]
    )

    return LaunchDescription([
        ldlidar_container,
        load_ldlidar,

        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        # twist_mux,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager_navigation,
    ])