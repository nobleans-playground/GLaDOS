## Package layout
| Package name       | Contents
| -----------------  | -----
| `glados`             | The main package which typically holds its launchfiles and some executables for operation
| `glados_description` | The robots description is in a seperate directory holding the xacro files and meshes to generate the urdf
| `glados_msgs`        | Optional package which **only** contains interfaces external to the robot
| `glados_simulation`  | Seperate package for all simulation related content. This package typically does not end up on the robot itself.



- How does the `robot_state_publisher` with an empty robot fill the `/robot_description` topic with the webots URDF? -> `robot_device.py` is responsible for this, -> `device_manager.py`, -> `webots_node.py` -> `webots_differential_drive_node.py`
- webots somehow publishes to `/robot_description`, which is not what you want, because it has bugs in it. PR (https://github.com/cyberbotics/webots_ros2/pull/378) -> so don't use this feature and publish it yourself.
    - Bug: Body isn't visualized in rviz
        - cilinder works, still looking for the bug in URDF/rviz why it isn't visualized. Locale issue for size<1 is not the problem.
        - URDF -> proto -> webots -> URDF -> description topic. description topic shows that after conversion back to URDF the baselink is empty when using box and functioning normally for cylinder.