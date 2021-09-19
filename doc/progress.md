# TODO
currently working on:
 ## Box on wheels
- working glados_simulation package: `ros2 launch glados_simulation simulation.py`
- create own URDF in `glados_description`: use rviz to visualize
- load `glados_description` proto (https://www.cyberbotics.com/doc/reference/proto) into webots -> edit world file to refer to glados.proto instead of turtlebot3, however, the protos folder needs to be in the share directory of `glados_simulation` **!currently just copied!**. How do I point to a different directory for the protos? (Bram?)
- connect urdf to webots simulation using a `diff_drive` controller? How to connect other tf components? -> working on understanding `robot_state_publisher` in conjuction with `glados_driver.py`
- Set correct wheel distance parameters etc of glados controller
- clean-up launch files, example: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/lgsvl_interface/launch/lgsvl_vehicle_control_command.launch.py
- Split `glados_simulation` over `glados`, `glados_description` and `glados_simulation` packages.
- Split urdf parts

## upgrade to webots R2021b

## ROS2 Controller in CPP for webots
- understand ROS2 control framework in combination with webots: `driver_node` and `robot state_publisher`
    - https://ros-controls.github.io/control.ros.org/index.html
    - http://wiki.ros.org/ros_control
    - `webots_ros2_core` package contains the drivers and API link to webots. Look here for how the control is configured in `simulation.py` and `glados_driver`.

## Model Robot in CAD, for mesh (and textures), and export to ROS2 and Webots
- Requirements:
    - Research what formats are supported. I.e. the inputs for ROS2 and Webots (see saved firefox session)
    - Conversion/Export should support: several types of joints, links, sensors and actuators in order to create a single source of truth. Otherwise, both urdf.xacro and CAD model need to be kept aligned in order to export correct meshes.
    - Conversion to xacro useful or use it to add sensors/actuators?
- Design: Describe the pipeline steps
- Implementation: