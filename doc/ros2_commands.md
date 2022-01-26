# ROS2 Foxy -> Galactic
`sudo apt purge ros-foxy* && sudo apt autoremove`

https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html

`sudo apt install ros-galactic-webots-ros2`

# Install Webots
make sure you are in point 4. of https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install and install.

Now, if Webots is not installed (or not the right version), when launching the node, a pop-up window will ask the user if he wants to install Webots, if he says yes, the tarball is downloaded and installed in $HOME/.ros/webots$VERSION, I have checked but did not found any recommended location to save such files (some other packages such RVIz2 are creating their own hidden folder in the home directory). [Source](https://github.com/cyberbotics/webots_ros2/pull/141#issuecomment-694158287).

More resources: https://www.cyberbotics.com/doc/guide/installation-procedure

`ros2 launch webots_ros2_demos armed_robots.launch.py`

# Tutorials
- See https://github.com/ros2/examples (or locally: `~/ros2_example_ws/src/examples`) for a whole elaborate set of examples from the https://index.ros.org/doc/ros2/Tutorials/ page.
- Webots ROS2 library: https://github.com/cyberbotics/webots_ros2/tree/foxy

# Cheat Sheets
- https://github.com/ubuntu-robotics/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf
- https://gist.github.com/kscottz/ef8a62700ef6ae5e04a0e91af5453ccd

# Create ROS2 Packages
`ros2 pkg create --build-type ament_cmake <package_name>`\
`ros2 pkg create --build-type ament_python <package_name>`

# Build and Run
`colcon` is a ROS build tool aiming to ease building a set of packages, testing them, etc. `ament_cmake` is the underlying build system for a single package.

`colcon build`\
`colcon build --packages-select cpp_pubsub`\
`. install/setup.bash`\
`ros2 run cpp_pubsub talker`\
`ros2 run cpp_pubsub listener`\
`ros2 launch my_package main.launch.py`
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

# Debug
`ros2 daemon status`

`ros2 node list`\
`ros2 topic list`\
`ros2 topic <command> <topic>`\

`ros2 run rqt_gui rqt_gui`\
`ros2 run rqt_graph rqt_graph`\

tf_tree or tf2_tree doesn't seem to be available for ROS2 foxy (https://index.ros.org/r/rqt_tf_tree/#foxy and https://answers.ros.org/question/375754/ros2-foxy-rqt_tf_tree/). https://github.com/clydemcqueen/tf_monitor could be a solution.\
`ros2 run tf2_ros static_transform_publisher 1 2 3 0.5 0.1 -1.0 foo bar`\
`ros2 run tf2_ros tf2_echo foo bar`\
`ros2 run tf2_tools view_frames.py`\

`ros2 run rviz2 rviz2`\

`ros2 interface list`\
`ros2 doctor`

# colcon_cd
No auto-complete even with supporting package. (source first?)\
`colcon_cd --set`\
`colcon list`\
`colcon_cd`

# Limit ROS2 to localhost only (turns off automatic node network discovery)
Add to ~/.bashrc: `export ROS_LOCALHOST_ONLY=1`

# Silver searcher
sudo apt install silversearcher-ag

# Fancy Ctrl+R
git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
~/.fzf/install