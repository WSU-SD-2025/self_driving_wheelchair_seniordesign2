# Installation Instructions

1. Clone the github respository
1. Within the repository, navigate to /ros2_ws/src/wheelchair_description/
1. Ensure the folders "/meshes", "/rviz", and "/urdf" exist and create them if they do not. The folders do not need anything in them.
1. Navigate to /ros2_ws/src/ and then delete the folder /ldrobot-lidar-ros2/.
1. execute the command "git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git" to ensure you have the latest copy of the libraries.
1. Navigate to /ros2_ws
1. Execute the command "colcon build"

Assuming there are no errors, the system can be started by executing the following commands
```bash
source install/setup.bash
ros2 launch wheelchair_bringup self_driving.launch.py
```

Rviz2 can be used to visualize the wheelchair's sensors and to give it goals. In Rviz2, set the "Fixed Frame" to "odom", then add visualizations by topic.

# Troubleshooting

## AssertionError: could not find template: idl__type_support.hpp.em

If you get this error when trying to build the package with the colcon build command, it is probably because you have an older rosidl package that colcon build is using. In our experience, this happens when using ros2 inside a docker container running ubuntu. Check if there is more than one place that ros packages are sourced and if so, remove the older rosidl packages from those sourced folders. We removed all duplicate, older rosidl packages.

Example:

Docker container with ros2 humble installed sources packages from two different folders. Each folder contains a duplicate rosidl package.

```bash
/opt/ros/humble/share/rosidl_default_generators/
/opt/ros/humble/install/share/rosidl_default_generators/
```

The package in "/opt/ros/humble/install/share/" is older so it is removed from that folder (highly recommend moving out of the folder instead of deleting it).

## Could not find /meshes folder

See step 3 in installation instructions. Some folders (like /meshes) must be added manually by the user.

## Package 'wheelchair_bringup' not found, searching: ...

Most likely cause is not sourcing the installation before launch. Execute the command "source install/setup.bash" while inside the /ros2_ws directory.

## Could not open serial port /dev/ttyACM0: No such file or directory

If you encounter this error after launch, this means the system is not detecting the ESP32 hardware. Ensure you have a good usb connection to the ESP32 board.
