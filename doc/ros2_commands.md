# install webots
sudo apt install ros-$ROS_DISTRO-webots-ros2
# make sure you are in point 4. of https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install
# Now, if Webots is not installed (or not the right version), when launching the node, a pop-up window will ask the user if he wants to install Webots, if he says yes, the tarball is downloaded and installed in $HOME/.ros/webots$VERSION, I have checked but did not found any recommended location to save such files (some other packages such RVIz2 are creating their own hidden folder in the home directory).
ros2 launch webots_ros2_demos armed_robots.launch.py

# create
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
    - ament_cmake (https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html) or colcon build?
    - pubsub tutorial -> python launch file
