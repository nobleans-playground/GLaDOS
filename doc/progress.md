# TODO
currently working on:
 ## Box on wheels
- working glados_simulation package: `ros2 launch glados_simulation simulation.py`
- create own URDF in `glados_description`: use rviz to visualize
- load `glados_description` proto (https://www.cyberbotics.com/doc/reference/proto) into webots -> edit world file to refer to glados.proto instead of turtlebot3, however, the protos folder needs to be in the share directory of `glados_simulation` (!currently just copied!). How do I point to a different directory for the protos? (Bram?)
- connect urdf to webots simulation using a `diff_drive` controller? How to connect other tf components? -> working on understanding `robot_state_publisher` in conjuction with `glados_driver.py`
- clean-up launch files, example: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/lgsvl_interface/launch/lgsvl_vehicle_control_command.launch.py
- Split `glados_simulation` over `glados`, `glados_description` and `glados_simulation` packages.

## ROS2 Controller in CPP for webots
- understand ROS2 control framework in combination with webots: `driver_node` and `robot state_publisher`
    - https://ros-controls.github.io/control.ros.org/index.html
    - http://wiki.ros.org/ros_control
    - `webots_ros2_core` package contains the drivers and API link to webots. Look here for how the control is configured in `simulation.py` and `glados_driver`.