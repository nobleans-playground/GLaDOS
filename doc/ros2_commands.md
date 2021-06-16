ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_python <package_name>

# build and run
colcon build
colcon build --packages-select cpp_pubsub
. install/setup.bash
ros2 run cpp_pubsub talker
ros2 run cpp_pubsub listener

# debug
ros2 run tf2_ros static_transform_publisher 1 2 3 0.5 0.1 -1.0 foo bar
ros2 run tf2_ros tf2_echo foo bar
ros2 run tf2_tools view_frames.py

# colcon_cd
source first?
colcon_cd --set
colcon list
colcon_cd

# TODO
currently working on:
- box on wheels
    - python launch files
    - ament_cmake (https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html)
    - tutorial source files: https://github.com/cyberbotics/webots_ros2/tree/foxy/webots_ros2_turtlebot
    - pubsub tutorial -> launch
