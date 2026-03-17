import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


# TODO: Replace all placeholder values with real measured dimensions after hardware mounting

# TODO: Update base_length, base_width, base_height using real wheelchair measurements

# TODO: Update sensor positions (camera, lidar, imu) with real measured offsets from base_link
# Do not use arbitary coordinates
# All sensor positions must be relative to base_link (wheel midpoint)

# TODO: Add sensor orientation (rpy) after mounting
# Especially important for IMU and LiDAR



def load_yaml(path: str):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    pkg_share = get_package_share_directory('wheelchair_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'wheelchair.urdf.xacro')
    dim_file   = os.path.join(pkg_share, 'config', 'dimensions.yaml')
    #rviz_file  = os.path.join(pkg_share, 'rviz', 'wheelchair.rviz')

    data = load_yaml(dim_file)

    wc = data.get('wheelchair', {})
    sensors = data.get('sensors', {})

    # YAML uses realsense_xyz, but URDF mounts camera_link
    realsense_xyz = sensors.get('realsense_xyz', [0.50, 0.50, 0.50])
    lidar_xyz     = sensors.get('lidar_xyz',     [0.50, 0.50, 0.50])
    imu_xyz       = sensors.get('imu_xyz',       [0.50, 0.50, 0.50])

    # base_z: keep 0 for now; later you can set to wheel radius, etc.
    base_z = wc.get('base_z', 0.171) 

    xacro_cmd = [
        'xacro ', xacro_file,

        ' base_length:=', str(wc.get('base_length', 0.50)),
        ' base_width:=',  str(wc.get('base_width',  0.50)),
        ' base_height:=', str(wc.get('base_height', 0.50)),
        ' base_z:=',      str(base_z),

        # RealSense mount (from realsense_xyz)
        ' camera_x:=', str(realsense_xyz[0]),
        ' camera_y:=', str(realsense_xyz[1]),
        ' camera_z:=', str(realsense_xyz[2]),

        # LiDAR mount
        ' lidar_x:=', str(lidar_xyz[0]),
        ' lidar_y:=', str(lidar_xyz[1]),
        ' lidar_z:=', str(lidar_xyz[2]),

        # IMU mount
        ' imu_x:=', str(imu_xyz[0]),
        ' imu_y:=', str(imu_xyz[1]),
        ' imu_z:=', str(imu_xyz[2]),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_cmd)
        }]
    )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_file]
    # )

    return LaunchDescription([
        robot_state_publisher,
        #rviz
    ])
