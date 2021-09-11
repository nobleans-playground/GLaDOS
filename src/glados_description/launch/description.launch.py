#!/usr/bin/env python

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node

from urdf2webots.importer import convert2urdf
# from webots_ros2_importer import urdf2proto

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

import xacro

ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='glados',
        description=''
    )
]

def generate_launch_description():

    urdf_folder_path = os.path.join(get_package_share_directory('glados_description'), 'urdf')
    xacro_file = os.path.join(urdf_folder_path, 'glados.urdf.xacro')
    urdf_file = os.path.join(urdf_folder_path, 'glados.urdf')
    proto_file = os.path.join(urdf_folder_path, 'glados.proto')
    assert os.path.exists(xacro_file), "glados.urdf.xacro doesn't exist in "+str(urdf_folder_path)

    robot_description_xacro = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_xacro.toxml()

    # print(robot_description_xml)
    with open(urdf_file, 'w') as f:
            f.write(robot_description_xml)

    convert2urdf(urdf_file, proto_file, False, False, False, False, False, None, '0 1 0 0')
    # urdf2proto(--input=urdf_file)




    namespace = LaunchConfiguration('namespace')

    return LaunchDescription(ARGUMENTS + [

    ])
