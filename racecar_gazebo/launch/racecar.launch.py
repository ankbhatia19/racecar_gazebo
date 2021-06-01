#!/usr/bin/env python3

"""
Script used to spawn a racecar in a generic position
Code Modified from
https://github.com/ros-simulation/gazebo_ros2_control/blob/master/gazebo_ros2_control_demos/launch/cart_example_effort.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from .xml_launch_description_source import XMLLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    #racecar_control = IncludeLaunchDescription(([os.path.join(
    #                get_package_share_directory('racecar_control'), 'launch'), '/racecar_control.launch.xml']),
    #        )

    racecar_description_share_dir = os.path.join(
        get_package_share_directory('racecar_description'))

    xacro_file = os.path.join(racecar_description_share_dir,
                              'urdf',
                              'racecar.xacro.xml')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity','racecar'],
                        output='screen')

    #load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
    #     output='screen'
    # )

    # load_effort_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller', 'effort_controllers'],
    #     output='screen'
    # )

    return LaunchDescription([
        #RegisterEventHandler(
        #    event_handler=OnProcessExit(
        #        target_action=spawn_entity,
        #        on_exit=[],
        #        #on_exit=[],
        #    )
        #),
        #RegisterEventHandler(
        #    event_handler=OnProcessExit(
        #        target_action=load_joint_state_controller,
        #        on_exit=[load_effort_controller],
        #    )
        #),
        gazebo,
        #racecar_control,
        node_robot_state_publisher,
        spawn_entity,
    ])
