# TODO
currently working on:
- box on wheels
    - working glados_simulation package: `ros2 launch glados_simulation simulation.py`
    - understand ROS2 control framework in combination with webots: `driver_node` and `robot state_publisher`
        - http://wiki.ros.org/ros_control
        - `webots_ros2_core` package contains the drivers and API link to webots. Look here for how the control is configured
    - create own URDF in `glados_description`: use rviz to visualize
    - Split `glados_simulation` over `glados`, `glados_description` and `glados_simulation` packages.