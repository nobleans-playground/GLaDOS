# TODO
currently working on:
## Box on wheels
- Fixing `sudo apt upgrade` (webots 2022a + webots driver changes), fixed almost, but seems like I have two joint_state_publishers (see rviz). So perhaps webots also publishing using a different coordinate frame?
- Ros2 control: `glados_driver.py` seems to be the only controller needed to make everything function, so why is there a mirrored ros diff driver controller in the yaml file? -> see firefox open tabs
- Add a caster.
- clean-up launch files, example: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/lgsvl_interface/launch/lgsvl_vehicle_control_command.launch.py
- Split `glados_simulation` over `glados`, `glados_description` and `glados_simulation` packages.
- Split urdf.xacro parts

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