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
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    robot_description_value = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("racecar_description"), "urdf", "racecar.xacro"])
    ])
    params = {"robot_description": robot_description_value}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        remappings=[('/joint_states', '/racecar/joint_states')],
        parameters=[params]
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity','racecar', 
            '-x', '0', 
            '-y', '0', 
            '-z', '0.5'
        ],
        output='screen'
    )

    controllers = ['left_rear_wheel_velocity_controller', 'right_rear_wheel_velocity_controller', 'left_front_wheel_velocity_controller', 'right_front_wheel_velocity_controller', 'left_steering_hinge_position_controller', 'right_steering_hinge_position_controller']
    load_controllers = [ ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', state],
                output='screen'
                )
        for state in controllers]  



    racecar_controller = os.path.join(
        get_package_share_directory("racecar_control"),
        "config",
        "racecar_control.yaml",
        )

    return LaunchDescription([
        RegisterEventHandler(
           event_handler=OnProcessExit(
               target_action=spawn_entity,
               on_exit=[load_joint_state_controller],
           )
        ),
        RegisterEventHandler(
           event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=load_controllers,
           )
        ),
        gazebo,
        #racecar_control,
        
        node_robot_state_publisher,
        spawn_entity
    ])