#!/usr/bin/env python3

"""
Script used to start controllers for racecar_gazebo
Code Modified from
https://github.com/ros-simulation/gazebo_ros2_control/blob/master/gazebo_ros2_control_demos/launch/cart_example_effort.launch.py
"""


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Get URDF via xacro
    racecar_description_share_dir = os.path.join(
        get_package_share_directory('racecar_description'))

    urdf_file_name = 'racecar.urdf.xml'
    xacro_file_name = 'racecar.xacro.xml'

    xacro_file = os.path.join(racecar_description_share_dir,
                              'urdf',
                               urdf_file_name)

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml()}

    racecar_controller = os.path.join(
        get_package_share_directory("racecar_control"),
        "config",
        "racecar_control.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, racecar_controller],
                output={"stdout": "screen", "stderr": "screen"},
            )
        ]
    )