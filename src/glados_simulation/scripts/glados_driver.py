#!/usr/bin/env python3

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 Glados driver."""

import rclpy
from webots_ros2_core.webots_differential_drive_node import WebotsDifferentialDriveNode


class GladosDriver(WebotsDifferentialDriveNode):
    def __init__(self, args):
        super().__init__(
            'glados_driver',
            args,
            left_encoder='jointL_sensor',
            left_joint='jointL',
            right_encoder='jointR_sensor',
            right_joint='jointR',
            robot_base_frame='base_link',
            wheel_distance=0.070,
            wheel_radius=0.034
        )
        self.start_device_manager({
            # 'robot': {'publish_base_footprint': True},
            # 'LDS-01': {'topic_name': '/scan'},
            # 'inertial_unit+accelerometer+gyro': {'frame_id': 'imu_link', 'topic_name': '/imu'}
        })


def main(args=None):
    rclpy.init(args=args)
    driver = GladosDriver(args=args)
    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
