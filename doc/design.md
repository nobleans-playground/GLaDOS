## Package layout
| Package name       | Contents
| -----------------  | -----
| `glados`             | The main package which typically holds its launchfiles and some executables for operation
| `glados_description` | The robots description is in a seperate directory holding the xacro files and meshes to generate the urdf
| `glados_msgs`        | Optional package which **only** contains interfaces external to the robot
| `glados_simulation`  | Seperate package for all simulation related content. This package typically does not end up on the robot itself.



- How does the `robot_state_publisher` with an empty robot fill the `/robot_description` topic with the webots URDF? Via parameter server, which it then publishes as a topic once? -> `robot_device.py` is responsible for this, -> `device_manager.py`, -> `webots_node.py` -> `webots_differential_drive_node.py`